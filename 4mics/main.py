"""
main.py — Punto de entrada del sistema DOA (4 micrófonos INMP441 + MUSIC).

Configuración POR DEFECTO (sin ningún flag):
    seguimiento + motor MUSIC + audio serial del ESP32 + gate espectral armónico.
    Es decir, `python3 main.py` arranca a rastrear un dron por su firma acústica
    sobre el puerto serial por defecto. Cualquier otra opción debe explicitarse.

Modos de servo (mutuamente excluyentes), modelados sobre el sistema 2mics:

    --seguimiento : (DEFAULT) el servo SIGUE la fuente en tiempo real (tracking
                    suave). Cuando además se detecta un evento fuerte, salta a
                    esa dirección y la MANTIENE fija unos segundos
                    (snap-and-hold), luego retoma el seguimiento. Tras silencio
                    prolongado vuelve gradualmente al centro. Es el modo por
                    defecto si no se pasa ningún flag de modo.

    --evento      : el servo NO sigue nada. Solo apunta —de inmediato— al
                    detectar un evento, y queda fijo en esa dirección hasta
                    el próximo evento. Pensado para localización puntual
                    (disparos, palmadas, golpes). Optimizado para reaccionar
                    rápido: dispara en el primer frame confirmado, sin promedios
                    ni reintentos diferidos.

    --sin-servo   : no mueve el servo; corre solo DOA + detección + registro +
                    display. Útil para validar la cadena sin hardware de servos.

Gate espectral armónico (confirmar dron por su huella acústica):
    SOLO aplica en los modos --seguimiento y --sin-servo (los pensados para
    drones): tras el gate de energía, confirma que la fuente tiene la
    estructura armónica (peine de BPF) de un multirrotor antes de localizar/
    mover el servo. El snap-and-hold del seguimiento queda ARMADO al
    confirmarse el evento y se dispara en el PRIMER frame activo con la firma
    confirmada (el gate necesita llenar su ventana de análisis, ~9 hops, más
    que los EVENT_MIN_FRAMES del detector). Se desactiva con --sin-espectral.
    El modo --evento lo SALTEA SIEMPRE: está pensado para fuentes impulsivas
    (aplausos, golpes), no drones — dispara por energía sola.

Uso:
    # Default: seguimiento + MUSIC + serial + gate espectral (rastrear dron):
    python3 main.py
    python3 main.py --serial /dev/ttyUSB0

    # Probar la cadena con palmadas/otros sonidos (el modo evento nunca
    # usa el gate espectral, no hace falta --sin-espectral):
    python3 main.py --evento

    # Solo DOA + registro, sin servo:
    python3 main.py --sin-servo

    # Simulación (sin hardware; conviene --sin-espectral, los tonos sim no son
    # un peine armónico de dron):
    python3 main.py --simulate --sim-az 45 --sim-el 20 --sin-espectral

    # Ajustar sensibilidad del detector de energía:
    python3 main.py --evento --k 1.0

    # Subir ganancia digital +6 dB (micrófonos débiles):
    python3 main.py --gain 2.0

Argumentos que sobreescriben config.py:
    --serial PORT          Puerto serial del ESP32
    --k FLOAT              Factor de umbral del detector (umbral_ruido)
    --gain FLOAT           Ganancia digital aplicada a cada frame (1.0 = sin cambio)
    --silence-ratio FLOAT  Histéresis: umbral_silencio = ruido_piso*(1+k*ratio)
    --simulate             Usar audio simulado en lugar del ESP32
    --sim-az FLOAT         Azimut de la fuente simulada (grados)
    --sim-el FLOAT         Elevación de la fuente simulada (grados)
    --seguimiento          (DEFAULT) Servo sigue la fuente en tiempo real
    --evento               Servo apunta una vez por evento (reacción rápida)
    --sin-servo            No mover el servo (solo DOA + registro)
    --sin-espectral        Desactiva el gate espectral armónico (dispara por
                           energía). Solo afecta a seguimiento/sin-servo: el
                           modo evento nunca usa el gate.
    --engine {music,srp}   Motor de DOA (default: MUSIC)
    --servo-lock-time SEC  Segundos que el servo queda fijo tras un evento
                           en modo seguimiento (default: config.py)
    --no-log               No guardar eventos en CSV
    --verbosity INT        Nivel de verbosidad (0-3)
"""

import sys
import time
import argparse
import numpy as np

# Agregar la raíz del proyecto al path para importar los módulos de src/
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as cfg

from src.acquisition.audio_input import SerialAudioInput, SimulatedAudioInput
from src.processing.doa_engine    import MUSICEngine, DOAResult
from src.processing.srp_doa_engine import SRPDoaEngine
from src.processing.detector      import EventDetector
from src.processing.spectral_gate import HarmonicDroneGate
from src.utils.display           import render
from src.utils.logger            import EventLogger
from src.utils.servo_control     import ServoController


class DOATracker:
    """
    Suaviza la salida frame-a-frame del motor MUSIC con dos mecanismos:
      1. Confidence gating: ignora estimaciones con confianza < min_conf.
      2. EMA en el ángulo: ángulo_nuevo = a*ángulo_prev + (1-a)*estimado.

    Esto resuelve el problema de saltos bruscos causados por:
      - Frames de silencio donde el subespacio de ruido está mal definido.
      - Alta varianza frame-a-frame con COV_ALPHA bajo.

    El tracker NO modifica el DOAResult original; retorna valores separados
    para no contaminar el log (que siempre registra el estimado crudo).
    """

    def __init__(self, smooth_alpha=0.6, min_conf=2.0):
        self.alpha    = smooth_alpha
        self.min_conf = min_conf
        self._az      = None   # None hasta la primera estimación válida
        self._el      = None

    def update(self, result):
        """
        Actualiza el tracker con un nuevo DOAResult.

        Retorna (az_smooth, el_smooth) — ángulos suavizados para display/servos.
        Si todavía no hay ninguna estimación válida, retorna (0.0, 0.0).
        """
        if result.valid and result.confidence >= self.min_conf:
            if self._az is None:
                # Primera estimación válida: inicializar sin inercia
                self._az = result.azimuth
                self._el = result.elevation
            else:
                self._az = self.alpha * self._az + (1 - self.alpha) * result.azimuth
                self._el = self.alpha * self._el + (1 - self.alpha) * result.elevation

        # Si nunca hubo estimación válida, retornar neutro
        az = self._az if self._az is not None else 0.0
        el = self._el if self._el is not None else 0.0
        return az, el

    def value(self):
        """Ángulo suavizado actual SIN incorporar una nueva estimación.
        Se usa en frames de silencio (gating por detector) para no contaminar
        el tracker con direcciones de ruido — relevante sobre todo en SRP-PHAT,
        cuyo PHAT blanquea el ruido y produce confianza no despreciable aun sin
        fuente; MUSIC en cambio da confianza ~0 en silencio."""
        az = self._az if self._az is not None else 0.0
        el = self._el if self._el is not None else 0.0
        return az, el

    @property
    def initialized(self):
        return self._az is not None


def parse_args():
    p = argparse.ArgumentParser(description='Sistema DOA 4 micrófonos INMP441')
    p.add_argument('--serial',        default=cfg.SERIAL_PORT)
    # default=None para distinguir "el usuario NO pasó el flag" (usar perfil/base)
    # de un valor explícito. La resolución de precedencia se hace en main().
    p.add_argument('--k',             type=float, default=None,
                   help=f"Factor de umbral del detector (default: perfil del modo "
                        f"o {cfg.DETECTOR_K}).")
    p.add_argument('--gain',          type=float, default=cfg.DIGITAL_GAIN)
    p.add_argument('--silence-ratio', type=float, default=None,
                   dest='silence_ratio',
                   help="Histéresis del detector (default: perfil del modo o "
                        f"{cfg.DETECTOR_SILENCE_RATIO}).")
    p.add_argument('--srp-mode', choices=['onset', 'accum'], default=None,
                   dest='srp_mode',
                   help="Override del modo SRP ('onset' impulsivos / 'accum' "
                        "sostenidas). Por defecto lo fija el perfil del modo de "
                        "servo (--evento→onset, --seguimiento→accum).")
    p.add_argument('--simulate',      action='store_true')
    p.add_argument('--sim-az',        type=float, default=45.0)
    p.add_argument('--sim-el',        type=float, default=0.0)

    # Modos de servo (mutuamente excluyentes), espejo del sistema 2mics.
    # DEFAULT = seguimiento: sin ningún flag, el sistema corre en modo
    # seguimiento (tracking de dron con MUSIC sobre el audio serial). Las demás
    # opciones deben explicitarse: --evento (apuntado puntual) o --sin-servo
    # (solo DOA + registro, sin mover el servo).
    modo = p.add_mutually_exclusive_group()
    modo.add_argument('--seguimiento', action='store_true',
                      help="(DEFAULT) El servo sigue la fuente en tiempo real; "
                           "ante un evento fuerte salta a esa dirección y la "
                           "mantiene fija unos segundos, luego retoma el "
                           "seguimiento. Es el modo por defecto si no se pasa "
                           "ningún flag de modo.")
    modo.add_argument('--evento', action='store_true',
                      help="El servo apunta una sola vez por evento, de "
                           "inmediato, y queda fijo hasta el próximo evento "
                           "(localización puntual, reacción rápida).")
    modo.add_argument('--sin-servo', action='store_true', dest='sin_servo',
                      help="No mover el servo: corre solo DOA + detección + "
                           "registro + display (útil para validar la cadena "
                           "sin hardware de servos).")

    p.add_argument('--servo-lock-time', type=float,
                   default=cfg.SERVO_EVENT_LOCK_DURATION, dest='servo_lock_time',
                   help="Segundos que el servo queda fijo tras un evento en "
                        "modo seguimiento (default: %(default)s).")
    p.add_argument('--engine', choices=['srp', 'music'], default=cfg.DOA_ENGINE,
                   help="Motor de DOA: 'srp' (SRP-PHAT, robusto, recomendado) o "
                        "'music' (subespacio). Default: config.py (%(default)s).")
    p.add_argument('--sin-espectral', action='store_true', dest='sin_espectral',
                   help="Desactiva el gate espectral armónico. El sistema "
                        "dispara por ENERGÍA sola (sin confirmar la firma de "
                        "dron), para probar la cadena con palmadas/voz/tonos.")
    p.add_argument('--no-log',        action='store_true')
    p.add_argument('--verbosity',     type=int, default=cfg.VERBOSITY)
    return p.parse_args()


def main():
    args = parse_args()

    # --- Resolver modo de servo y su perfil ---
    # El modo elegido (si hay) selecciona un perfil de config que afina TODA la
    # cadena (motor DOA + detector) para su caso de uso. Precedencia de cada
    # parámetro: flag explícito de CLI > perfil del modo > default base.
    # DEFAULT = seguimiento. Sin ningún flag de modo se corre seguimiento
    # (tracking de dron con MUSIC). --evento y --sin-servo deben explicitarse.
    if args.evento:
        servo_mode = 'evento'
    elif args.sin_servo:
        servo_mode = None          # solo DOA + registro, sin mover el servo
    else:
        servo_mode = 'seguimiento'  # default (con o sin --seguimiento explícito)
    profile    = cfg.MODE_PROFILES.get(servo_mode, {})

    srp_mode         = (args.srp_mode
                        or profile.get('SRP_MODE', cfg.SRP_MODE))
    k_val            = (args.k if args.k is not None
                        else profile.get('DETECTOR_K', cfg.DETECTOR_K))
    silence_ratio    = (args.silence_ratio if args.silence_ratio is not None
                        else profile.get('DETECTOR_SILENCE_RATIO', cfg.DETECTOR_SILENCE_RATIO))
    # COV_ALPHA: el perfil 'evento' lo baja a 0.5 para mayor reactividad.
    # El perfil 'seguimiento' y los defaults usan cfg.COV_ALPHA (0.85 para drone).
    cov_alpha        = profile.get('COV_ALPHA', cfg.COV_ALPHA)
    # EVENT_MIN_FRAMES: el perfil 'evento' lo baja a 3 (confirmación rápida).
    event_min_frames = profile.get('EVENT_MIN_FRAMES', cfg.EVENT_MIN_FRAMES)

    # --- Inicializar entrada de audio ---
    if args.simulate:
        # --sim-az/--sim-el están en coordenadas REALES (mundo). El motor
        # trabaja en el marco del array (inclinado ARRAY_TILT_DEG) y el
        # pipeline convierte a real sumando el tilt a la salida; para que el
        # lazo cierre, la fuente simulada se genera RESTANDO el tilt
        # (real → marco del array). Así, --sim-el 20 se reporta como el≈20°.
        audio = SimulatedAudioInput(
            mic_positions  = cfg.MIC_POSITIONS,
            sample_rate    = cfg.SAMPLE_RATE,
            hop_size       = cfg.HOP_SIZE,
            azimuth_deg    = args.sim_az,
            elevation_deg  = args.sim_el - cfg.ARRAY_TILT_DEG,
            speed_of_sound = cfg.SPEED_OF_SOUND,
        )
        print(f"[main] Modo simulado — fuente en az={args.sim_az}° el={args.sim_el}°")
    else:
        audio = SerialAudioInput(
            port            = args.serial,
            baud            = cfg.SERIAL_BAUD,
            hop_size        = cfg.HOP_SIZE,
            bits_per_sample = cfg.BYTES_PER_SAMPLE * 8,
        )
        print(f"[main] Puerto serial: {args.serial}")

    # --- Inicializar motor de DOA (SRP-PHAT o MUSIC, segun --engine) ---
    # El motor escanea la elevacion en el MARCO DEL ARRAY (restando el tilt al
    # rango real); el resultado se convierte a elevacion real sumando el tilt.
    el_scan = (cfg.ELEVATION_MIN - cfg.ARRAY_TILT_DEG,
               cfg.ELEVATION_MAX - cfg.ARRAY_TILT_DEG)
    if args.engine == 'srp':
        print("[main] Pre-calculando retardos de grilla (SRP-PHAT)...")
        engine = SRPDoaEngine(
            mic_positions    = cfg.MIC_POSITIONS,
            sample_rate      = cfg.SAMPLE_RATE,
            frame_size       = cfg.HOP_SIZE,
            freq_min         = cfg.FREQ_MIN,
            freq_max         = cfg.FREQ_MAX,
            speed_of_sound   = cfg.SPEED_OF_SOUND,
            angle_resolution = cfg.ANGLE_RESOLUTION,
            az_range         = (cfg.AZIMUTH_MIN, cfg.AZIMUTH_MAX),
            el_range         = el_scan,
            mode             = srp_mode,
            accum_alpha      = cfg.SRP_ACCUM_ALPHA,
        )
        doa_min_conf   = cfg.SRP_MIN_CONF_UPDATE
        event_min_conf = cfg.SRP_EVENT_MIN_CONFIDENCE
        print(f"[main] Motor SRP-PHAT listo (modo={srp_mode}).")
    else:
        print("[main] Pre-calculando steering vectors (MUSIC)...")
        engine = MUSICEngine(
            mic_positions    = cfg.MIC_POSITIONS,
            sample_rate      = cfg.SAMPLE_RATE,
            frame_size       = cfg.HOP_SIZE,
            freq_min         = cfg.FREQ_MIN,
            freq_max         = cfg.FREQ_MAX,
            speed_of_sound   = cfg.SPEED_OF_SOUND,
            angle_resolution = cfg.ANGLE_RESOLUTION,
            num_sources      = cfg.NUM_SOURCES,
            cov_alpha        = cov_alpha,   # resuelto desde el perfil del modo
            diag_loading     = cfg.DIAGONAL_LOADING,
            az_range         = (cfg.AZIMUTH_MIN, cfg.AZIMUTH_MAX),
            el_range         = el_scan,
        )
        doa_min_conf   = cfg.DOA_MIN_CONF_UPDATE
        event_min_conf = cfg.EVENT_MIN_CONFIDENCE
        print("[main] Motor MUSIC listo.")

    # --- Inicializar detector ---
    if not (0.0 < silence_ratio < 1.0):
        print(f"[main] ADVERTENCIA: silence_ratio={silence_ratio} fuera del rango "
              f"(0.0, 1.0). Usando valor por defecto 0.5.", flush=True)
        silence_ratio = 0.5

    # En simulación la fuente suena desde el frame 0: la calibración de piso
    # (que asume SILENCIO inicial) absorbería la señal y el detector nunca
    # dispararía — y como el escaneo DOA se saltea en IDLE, no habría ninguna
    # estimación. Se fija un piso bajo para entrar en ACTIVE de inmediato.
    noise_floor_cfg = getattr(cfg, 'DETECTOR_NOISE_FLOOR', None)
    if args.simulate and noise_floor_cfg is None:
        noise_floor_cfg = 1e-6

    detector = EventDetector(
        k               = k_val,
        min_frames      = event_min_frames,   # resuelto desde el perfil del modo
        cooldown_frames = cfg.COOLDOWN_FRAMES,
        silence_ratio   = silence_ratio,
        calib_frames    = cfg.DETECTOR_CALIB_FRAMES,
        noise_floor     = noise_floor_cfg,
    )
    if args.verbosity >= 1:
        perfil_txt = f"perfil={servo_mode}" if servo_mode else "sin perfil (base)"
        print(f"[main] Detector — k={k_val}, silence_ratio={silence_ratio}, "
              f"piso FIJO  [{perfil_txt}]")
        if detector.calibrating:
            calib_s = cfg.DETECTOR_CALIB_FRAMES * cfg.HOP_SIZE / cfg.SAMPLE_RATE
            print(f"[main] Calibrando piso de ruido durante ~{calib_s:.1f}s "
                  f"({cfg.DETECTOR_CALIB_FRAMES} frames) — MANTENÉ SILENCIO "
                  f"(sin la fuente a detectar).")

    # --- Gate espectral armónico (confirmación de dron por huella acústica) ---
    # Segundo gate en cascada DESPUÉS del de energía: una vez que hay energía,
    # confirma que la fuente tiene la estructura armónica (peine de BPF) de un
    # multirrotor antes de localizar/mover el servo.
    # SOLO aplica en seguimiento/sin-servo. El modo EVENTO lo saltea SIEMPRE:
    # está pensado para fuentes impulsivas (no drones) y además su disparo
    # rápido (EVENT_MIN_FRAMES=3) ocurre antes de que la ventana del gate
    # (2048 muestras ≈ 9 hops) pueda llenarse. Se desactiva con --sin-espectral
    # (o SPECTRAL_ENABLED=False en config).
    spectral_enabled = (getattr(cfg, 'SPECTRAL_ENABLED', True)
                        and not args.sin_espectral
                        and servo_mode != 'evento')
    spectral_gate = None
    if spectral_enabled:
        spectral_gate = HarmonicDroneGate(
            sample_rate           = cfg.SAMPLE_RATE,
            window_size           = cfg.SPECTRAL_WINDOW,
            hop_size              = cfg.HOP_SIZE,
            bpf_min               = cfg.SPECTRAL_BPF_MIN,
            bpf_max               = cfg.SPECTRAL_BPF_MAX,
            n_harmonics           = cfg.SPECTRAL_N_HARMONICS,
            hps_downsample        = cfg.SPECTRAL_HPS_DOWNSAMPLE,
            music_band            = (cfg.SPECTRAL_MUSIC_BAND_LO, cfg.SPECTRAL_MUSIC_BAND_HI),
            harmonic_snr_db       = cfg.SPECTRAL_HARMONIC_SNR_DB,
            min_harmonics         = cfg.SPECTRAL_MIN_HARMONICS,
            min_harmonics_in_band = cfg.SPECTRAL_MIN_HARMONICS_IN_BAND,
            score_min             = cfg.SPECTRAL_SCORE_MIN,
            harmonic_tol_hz       = cfg.SPECTRAL_HARMONIC_TOL_HZ,
            hold_frames           = cfg.SPECTRAL_HOLD_FRAMES,
            harmonic_fraction_min = cfg.SPECTRAL_HARMONIC_FRACTION_MIN,
            confirm_windows       = cfg.SPECTRAL_CONFIRM_WINDOWS,
        )
    if args.verbosity >= 1:
        if spectral_enabled:
            print(f"[main] Gate espectral: ACTIVO "
                  f"(BPF {cfg.SPECTRAL_BPF_MIN:.0f}-{cfg.SPECTRAL_BPF_MAX:.0f} Hz, "
                  f">={cfg.SPECTRAL_MIN_HARMONICS} armónicos, "
                  f">={cfg.SPECTRAL_MIN_HARMONICS_IN_BAND} en banda MUSIC).")
        else:
            if servo_mode == 'evento':
                motivo = "modo evento: impulsivos, sin firma de dron"
            elif args.sin_espectral:
                motivo = "--sin-espectral"
            else:
                motivo = "SPECTRAL_ENABLED=False"
            print(f"[main] Gate espectral: DESACTIVADO ({motivo}) — "
                  f"se dispara por energía sola.")

    # --- Logger ---
    logger = None if args.no_log else EventLogger(cfg.LOG_FILE)

    # --- Servo y modo de operación ---
    # Espejo del 2mics: el modo elegido habilita el servo. Ningún flag = sin
    # servo (el resto del pipeline corre igual).
    servo = None
    if servo_mode is not None:
        cfg.SERVO_ENABLED = True
        servo = ServoController(cfg)

    lock_duration = args.servo_lock_time

    # --- Loop principal ---
    if args.gain != 1.0 and args.verbosity >= 1:
        gain_db = 20 * np.log10(abs(args.gain))
        print(f"[main] Ganancia digital: x{args.gain} ({gain_db:+.1f} dB)")

    # Tracker de salida: suaviza el ángulo y aplica confidence gating
    tracker = DOATracker(
        smooth_alpha = cfg.DOA_SMOOTH_ALPHA,
        min_conf     = doa_min_conf,
    )

    if args.verbosity >= 1:
        print(f"[main] DOA: {args.engine.upper()} "
              f"(conf min update={doa_min_conf}, evento={event_min_conf})")
        if servo_mode == 'seguimiento':
            print(f"[main] Modo: SEGUIMIENTO (snap-and-hold {lock_duration:.1f}s por evento)")
        elif servo_mode == 'evento':
            print(f"[main] Modo: EVENTO (apuntado puntual, reacción rápida)")
        else:
            print(f"[main] Modo: sin servo (solo DOA + registro)")

    audio.start()
    last_display = 0
    last_result  = None
    _idle_frames = 0   # contador de frames IDLE consecutivos para reset diferido de R
    _spectral_prev = False  # estado previo del gate espectral (para log de transición)
    _snap_pending  = False  # seguimiento: snap-and-hold armado, espera firma espectral
    _event_logged  = False  # evento: primera estimación del evento ya registrada

    # lock_until — timestamp hasta el cual, en modo SEGUIMIENTO, el servo
    # está fijado al último evento. Mientras time.time() < lock_until el
    # seguimiento continuo no mueve el servo (deja la dirección del evento
    # quieta). Espeja el lock_to/is_locked del ServoController del 2mics.
    lock_until = 0.0

    try:
        while True:
            frame = audio.read_frame(timeout=1.0)
            if frame is None:
                continue

            # Aplicar ganancia digital (no modifica el array original)
            if args.gain != 1.0:
                frame = frame * args.gain

            energy = float(np.mean(frame ** 2))
            det_signal = detector.update(frame)

            # Estimar DOA solo con actividad (onset/event/active). En IDLE se
            # SALTEA el escaneo completo: el pseudoespectro del silencio no se
            # usa para nada (el tracker no se actualiza en idle) y en la RPi
            # 3A+ ese cómputo por frame es lo que define si el sistema
            # sostiene el tiempo real. La covarianza de MUSIC queda congelada
            # durante el silencio; el reset diferido de abajo la limpia tras
            # MUSIC_RESET_IDLE_FRAMES.
            if det_signal != 'idle':
                result = engine.process(frame)
                # Convertir la elevación del marco del array a elevación REAL
                # (el array está inclinado ARRAY_TILT_DEG): real = array + tilt.
                if result.valid and cfg.ARRAY_TILT_DEG:
                    result.elevation += cfg.ARRAY_TILT_DEG
            else:
                result = DOAResult()   # inválido: sin escaneo en silencio

            # Actualizar tracker SOLO si el detector ve señal (evento/activo).
            # En silencio no se incorpora la estimación: SRP-PHAT (PHAT) produce
            # confianza no despreciable aun sobre ruido, y sin este gate el
            # tracker derivaría hacia direcciones espurias entre eventos.
            if det_signal in ('event', 'active'):
                _idle_frames = 0
                az_smooth, el_smooth = tracker.update(result)
            elif det_signal == 'onset':
                # Flanco de subida (camino directo). No se actualiza el tracker
                # (evento aún no confirmado), pero el motor acumula estos frames.
                _idle_frames = 0
                az_smooth, el_smooth = tracker.value()
            else:  # 'idle': silencio / cooldown
                az_smooth, el_smooth = tracker.value()
                _idle_frames += 1
                # Reset diferido de R: solo se limpia la covarianza MUSIC tras
                # silencio SOSTENIDO (>= MUSIC_RESET_IDLE_FRAMES frames consecutivos).
                # Evita el parpadeo del tracking: los bajones breves de amplitud
                # del drone (efecto de pala, multipath) no disparan el reset, así
                # no hay que reconstruir R desde cero en cada bajón.
                # Para SRP el reset es inmediato (no tiene estado interno relevante).
                reset_threshold = getattr(cfg, 'MUSIC_RESET_IDLE_FRAMES', 0)
                if _idle_frames >= max(reset_threshold, 1):
                    if hasattr(engine, 'reset'):
                        engine.reset()
                    # Limpiar también la ventana del gate espectral tras silencio
                    # sostenido: el próximo evento se evalúa desde cero.
                    if spectral_gate is not None:
                        spectral_gate.reset()
                    _idle_frames = 0   # reiniciar contador para no llamar reset() cada frame
            if result.valid:
                last_result = result   # el display conserva la última estimación real

            # =================================================================
            # GATE ESPECTRAL ARMÓNICO (confirmación de dron por huella acústica)
            # =================================================================
            # Cascada: el de energía ya marcó actividad; acá se confirma que la
            # fuente es un dron (peine de BPF) antes de habilitar localización +
            # servo + registro. `spectral_pass` condiciona TODO lo que sigue.
            # Con el gate desactivado (--sin-espectral) es siempre True → el
            # sistema dispara por energía sola (modo de prueba con otros sonidos).
            if spectral_gate is not None:
                if det_signal in ('event', 'active', 'onset'):
                    sres = spectral_gate.update(frame)
                    spectral_pass = sres.is_drone
                    if args.verbosity >= 1 and spectral_pass and not _spectral_prev:
                        print(f"\n[espectral] dron CONFIRMADO  BPF~{sres.bpf:.0f} Hz  "
                              f"{sres.n_harmonics} armónicos ({sres.n_in_band} en "
                              f"banda MUSIC)  HNR={sres.hnr_db:.1f} dB")
                    elif args.verbosity >= 2 and det_signal == 'event' and not spectral_pass:
                        print(f"\n[espectral] energía sin firma de dron — "
                              f"NO se localiza (BPF~{sres.bpf:.0f} Hz, "
                              f"{sres.n_harmonics} armónicos, HNR={sres.hnr_db:.1f} dB)")
                    _spectral_prev = spectral_pass
                else:
                    # idle / cooldown: no hay señal que evaluar, pero el latch
                    # DECAE igual — el veredicto confirmado no sobrevive más de
                    # hold_frames sin evidencia (misma semántica que en activo).
                    spectral_gate.idle_tick()
                    spectral_pass = spectral_gate.confirmed
                    _spectral_prev = spectral_pass
            else:
                spectral_pass = True   # gate desactivado: pasa todo

            # =================================================================
            # REGISTRO DE EVENTOS — UNA fila por EVENTO (no por frame)
            # =================================================================
            #   modo evento           → fila inmediata con la PRIMERA estimación
            #                           válida del evento (el ángulo del apuntado).
            #   seguimiento/sin-servo → track() acumula las estimaciones del
            #                           evento en curso; al volver a idle,
            #                           end_event() escribe UNA fila con el RANGO
            #                           de ángulos ocupado, duración y máximos.
            # Se registra el estimado CRUDO (fiel para análisis post-hoc).
            if logger:
                if servo_mode == 'evento':
                    if det_signal in ('event', 'active') and result.valid \
                       and not _event_logged:
                        logger.log_single(result, energy)
                        _event_logged = True
                    elif det_signal == 'idle':
                        _event_logged = False
                else:
                    if det_signal in ('event', 'active') and result.valid \
                       and spectral_pass:
                        logger.track(result, energy)
                    elif det_signal == 'idle':
                        logger.end_event()

            # =================================================================
            # MODO SEGUIMIENTO: tracking continuo + snap-and-hold en evento
            # =================================================================
            # Espeja el modo --seguimiento del 2mics:
            #   - En cada frame con señal, el servo sigue la fuente con
            #     movimiento suave (update(): batching, zona muerta, paso máx).
            #   - Al confirmarse un evento fuerte, SALTA de inmediato a esa
            #     dirección (point_to crudo) y la mantiene fija lock_duration
            #     segundos; durante ese lock el seguimiento continuo no mueve
            #     el servo. Vencido el lock, retoma el seguimiento.
            #   - Tras silencio prolongado, tick() lo devuelve al centro.
            if servo_mode == 'seguimiento':
                now_t  = time.time()
                locked = now_t < lock_until

                # Snap-and-hold DIFERIDO: se ARMA al confirmarse el evento
                # ('event') y se DISPARA en el primer frame (event o active)
                # con la firma espectral confirmada. El gate necesita llenar
                # su ventana (2048 muestras ≈ 9 hops, más que EVENT_MIN_FRAMES)
                # y 2 ventanas de confirmación, así que en el frame 'event'
                # todavía no puede haber confirmado; sin este diferido, el
                # snap no se dispararía nunca con el gate activo. Con
                # --sin-espectral (spectral_pass=True) dispara en 'event'.
                if det_signal == 'event':
                    _snap_pending = True
                elif det_signal == 'idle':
                    _snap_pending = False   # el evento terminó sin confirmar

                if _snap_pending and det_signal in ('event', 'active') and \
                   spectral_pass and result.valid and \
                   result.confidence >= event_min_conf:
                    # Snap inmediato al evento (override del seguimiento).
                    # Crudo y forzado: queremos reactividad, no suavizado.
                    if servo:
                        servo.point_to(result, force=True)
                    lock_until = now_t + lock_duration
                    locked = True
                    _snap_pending = False
                    if args.verbosity >= 1:
                        print(f"\n[evento->fija] az={result.azimuth:+.1f}° "
                              f"el={result.elevation:+.1f}° "
                              f"conf={result.confidence:.1f} dB  "
                              f"(fija {lock_duration:.1f}s)")

                elif servo and not locked and spectral_pass and \
                        det_signal in ('event', 'active'):
                    # Seguimiento continuo con ángulo suavizado (evita saltos).
                    smooth_result = DOAResult(
                        az=az_smooth, el=el_smooth,
                        conf=result.confidence,
                        spectrum=result.spectrum,
                        valid=result.valid,
                    )
                    servo.update(smooth_result)

                if servo:
                    servo.tick()  # retorno gradual al centro tras silencio

            # =================================================================
            # MODO EVENTO: apuntado puntual con reacción rápida
            # =================================================================
            # SIN gate espectral: este modo está pensado para fuentes
            # IMPULSIVAS (aplausos, golpes, disparos), no drones — dispara por
            # energía sola (spectral_enabled lo excluye más arriba).
            # Espeja el modo --evento del 2mics (lock_to en event_started),
            # priorizando la VELOCIDAD de reacción:
            #   - Dispara en el PRIMER frame confirmado del evento
            #     (det_signal == 'event', tras EVENT_MIN_FRAMES de validación
            #     del detector). No hay acumulación, promedios ni reintentos
            #     diferidos: se apunta YA, al estimado crudo del frame.
            #   - No hay seguimiento continuo ni retorno al centro: el servo
            #     queda fijo apuntando al último evento hasta el próximo
            #     (el detach asincrónico del PWM apaga la señal; el SG90
            #     mantiene la posición mecánicamente).
            #
            # Por qué el frame 'event' y no antes: el detector recién lo emite
            # tras EVENT_MIN_FRAMES sobre umbral, momento en que la covarianza
            # de MUSIC ya absorbió ~3 hops de la señal nueva (perfil evento:
            # COV_ALPHA=0.5 -> 0.5^3 ≈ 0.13 de peso residual del silencio). En
            # frames más tempranos el ángulo puede estar fuera por 20°+.
            elif servo_mode == 'evento':
                if det_signal == 'event' and result.valid and \
                   result.confidence >= event_min_conf:
                    if servo:
                        servo.point_to(result, force=True)
                    if args.verbosity >= 1:
                        servo_tag = "" if servo else " [sin servo]"
                        print(f"\n[evento->apunta] az={result.azimuth:+.1f}° "
                              f"el={result.elevation:+.1f}° "
                              f"conf={result.confidence:.1f} dB{servo_tag}")
                elif args.verbosity >= 2 and det_signal == 'event':
                    print(f"\n[evento descartado] conf={result.confidence:.1f} dB "
                          f"< {event_min_conf} dB (o inválido)")

            # Actualizar display — usa ángulos suavizados
            now = time.time()
            if now - last_display >= cfg.DISPLAY_INTERVAL:
                serial_stats = None
                if hasattr(audio, 'pkts_received'):
                    serial_stats = {
                        'received': audio.pkts_received,
                        'lost':     audio.pkts_lost,
                        'corrupt':  audio.pkts_corrupt,
                    }
                servo_az, servo_el = (servo.position if servo else (None, None))

                # Construir resultado display con ángulos suavizados
                if last_result is not None and last_result.valid:
                    display_result = DOAResult(
                        az       = az_smooth,
                        el       = el_smooth,
                        conf     = last_result.confidence,
                        spectrum = last_result.spectrum,
                        valid    = tracker.initialized,
                    )
                else:
                    display_result = last_result

                render(
                    doa_result        = display_result,
                    det_state         = detector.state,
                    energy            = energy,
                    threshold_event   = detector.threshold_event,
                    threshold_silence = detector.threshold_silence,
                    noise_floor       = detector.noise_floor,
                    serial_stats      = serial_stats,
                    servo_az          = servo_az,
                    servo_el          = servo_el,
                )
                last_display = now

    except KeyboardInterrupt:
        print("\n[main] Detenido por el usuario.")
    finally:
        audio.stop()
        if logger:
            logger.close()
        if servo:
            servo.close()


if __name__ == '__main__':
    main()
