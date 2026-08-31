"""
main.py — Sistema de localización de fuente sonora con 4 micrófonos y ESP32

Sin flags: seguimiento + MUSIC + audio serial del ESP32 + gate espectral

Modos de servomotor, mutuamente excluyentes:
    --seguimiento  (DEFAULT) el servo sigue la fuente luego de confirmarla. Tras
                   unos segundos de silencio vuelve al centro.
    --evento       apunta de inmediato al detectar un evento y queda fijo hasta
                   el próximo. Para fuentes impulsivas.
    --sin-servo    solo DOA + detección + registro + display.

El gate espectral solo aplica en --seguimiento y --sin-servo.

Diagnóstico con archivo .wav(--wav): el pipeline completo corre sobre un
WAV de capture_wav.py, con y sin restricción temporal para comprobar tiempo real.

    python3 main.py --wav captura.wav                  
    python3 main.py --wav captura.wav --wav-realtime  

Otros ejemplos:
    python3 main.py --serial /dev/ttyUSB0      # para datos del ESP32 por USB, con conf default
    python3 main.py --evento --k 1.0                   # más sensible
    python3 main.py --simulate --sim-az 45 --sim-el 20 --sin-espectral
    python3 main.py --gain 2.0                         # +6 dB

Todos los flags están documentados en --help.
"""

import sys
import time
import argparse
import numpy as np

# Raíz del proyecto en el path para importar src/
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as cfg

from src.acquisition.audio_input import (SerialAudioInput, SimulatedAudioInput, WavAudioInput)
from src.processing.doa_engine    import MUSICEngine, DOAResult
from src.processing.srp_doa_engine import SRPDoaEngine
from src.processing.detector      import EventDetector
from src.processing.spectral_gate import HarmonicDroneGate
from src.utils.display           import render
from src.utils.logger            import EventLogger
from src.utils.perf              import PerfMeter
from src.utils.servo_control     import ServoController


class DOATracker:
    """
    Suaviza la salida para seguirla con el motor: descarta estimaciones con confianza <
    min_conf y aplica un EMA al ángulo. No modifica el DOAResult original, así
    el log siempre registra el estimado crudo.
    """

    def __init__(self, smooth_alpha=0.6, min_conf=2.0):
        self.alpha    = smooth_alpha
        self.min_conf = min_conf
        self._az      = None   # None hasta la primera estimación válida
        self._el      = None

    def update(self, result):
        """Devuelve (az, el) suavizados; (0.0, 0.0) si aún no hubo ninguna
        estimación válida."""
        if result.valid and result.confidence >= self.min_conf:
            if self._az is None:
                # Primera estimación válida: inicializar sin inercia
                self._az = result.azimuth
                self._el = result.elevation
            else:
                self._az = self.alpha * self._az + (1 - self.alpha) * result.azimuth
                self._el = self.alpha * self._el + (1 - self.alpha) * result.elevation

        az = self._az if self._az is not None else 0.0
        el = self._el if self._el is not None else 0.0
        return az, el

    def value(self):
        """Ángulo actual sin incorporar una estimación nueva."""
        az = self._az if self._az is not None else 0.0
        el = self._el if self._el is not None else 0.0
        return az, el

    @property
    def initialized(self):
        return self._az is not None


def parse_args():
    p = argparse.ArgumentParser(description='Sistema DOA 4 micrófonos')
    p.add_argument('--serial',        default=cfg.SERIAL_PORT)
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
    p.add_argument('--noise-floor', type=float, default=None, dest='noise_floor',
                   help="Fija el piso de ruido del detector a mano (energía "
                        "media por muestra) y SALTEA la calibración de arranque. "
                        "Útil cuando la grabación no empieza en silencio: la "
                        "calibración absorbería la fuente y el umbral quedaría "
                        "inalcanzable (el piso es fijo, no se recupera).")
    p.add_argument('--simulate',      action='store_true')
    p.add_argument('--sim-az',        type=float, default=45.0)
    p.add_argument('--sim-el',        type=float, default=0.0)

    # --- Entrada desde WAV (diagnóstico algoritmo vs plataforma) ---
    p.add_argument('--wav', default=None,
                   help="Correr el pipeline completo sobre un WAV de "
                        "capture_wav.py en vez del serial")
    p.add_argument('--wav-realtime', action='store_true', dest='wav_realtime',
                   help="Con --wav: entregar los frames al ritmo real.")
    p.add_argument('--drop-policy', choices=['newest', 'oldest'], default='newest',
                   dest='drop_policy',
                   help="Qué frame se tira cuando la cola se llena.")

    # Modos de servo, mutuamente excluyentes. Sin ningún flag = seguimiento.
    modo = p.add_mutually_exclusive_group()
    modo.add_argument('--seguimiento', action='store_true',
                      help="(DEFAULT) El servo sigue la fuente en tiempo real; "
                           "ante un evento fuerte salta a esa dirección y la "
                           "mantiene fija unos segundos.")
    modo.add_argument('--evento', action='store_true',
                      help="Apunta una sola vez por evento, de "
                           "inmediato, y queda fijo hasta el próximo evento.")
    modo.add_argument('--sin-servo', action='store_true', dest='sin_servo',
                      help="No mover el servo: correr el resto del pipeline.")

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
    p.add_argument('--sin-display', action='store_true', dest='sin_display',
                   help="No renderizar el display de terminal.")
    p.add_argument('--verbosity',     type=int, default=cfg.VERBOSITY)
    return p.parse_args()


def _print_summary(args, cfg, audio, detector, stat,
                   meter_loop, meter_doa, meter_gate, meter_disp,
                   frame_period_ms, wall_s, spectral_enabled, servo_mode):
    """Resumen de diagnóstico: si la señal superó el umbral (DETECTOR), dónde
    se cortó la cadena de gates (CADENA) y si sostiene el tiempo real (TIEMPOS,
    INTEGRIDAD)."""
    if stat['frames'] == 0:
        print("\n[resumen] Sin frames procesados.")
        return

    audio_s = stat['frames'] * cfg.HOP_SIZE / cfg.SAMPLE_RATE
    W = 72
    print()
    print("=" * W)
    print("  RESUMEN DE DIAGNÓSTICO")
    print("=" * W)

    # --- entrada ---
    if args.wav:
        fuente = f"WAV {args.wav}" + (" [ritmo real]" if args.wav_realtime
                                      else " [offline, sin restricción temporal]")
    elif args.simulate:
        fuente = "simulada"
    else:
        fuente = f"serial {args.serial}"
    print(f"  Fuente        : {fuente}")
    print(f"  Motor / modo  : {args.engine.upper()}  /  "
          f"{servo_mode or 'sin-servo'}  "
          f"(gate espectral {'ON' if spectral_enabled else 'OFF'})")
    print(f"  Frames        : {stat['frames']}  "
          f"({audio_s:.2f} s de audio en {wall_s:.2f} s de reloj)")

    # --- detector: ¿la señal llegó al umbral? ---
    print("-" * W)
    def _db(x):
        return 10 * np.log10(max(float(x), 1e-12))
    if detector.calibrating:
        print("  DETECTOR      : la corrida TERMINÓ EN CALIBRACIÓN — la señal "
              "nunca se evaluó.")
        print(f"                  Se necesitan {cfg.DETECTOR_CALIB_FRAMES} frames "
              f"y solo hubo {stat['frames']}.")
    else:
        margen_db = _db(stat['energy_max']) - _db(detector.threshold_event)
        print(f"  Piso de ruido : {detector.noise_floor:.3e} ({_db(detector.noise_floor):+6.1f} dB)")
        print(f"  Umbral evento : {detector.threshold_event:.3e} ({_db(detector.threshold_event):+6.1f} dB)"
              f"   [k={detector.k}]")
        print(f"  Energía máx   : {stat['energy_max']:.3e} ({_db(stat['energy_max']):+6.1f} dB)"
              f"   -> margen {margen_db:+.1f} dB sobre el umbral")
        if margen_db < 0:
            print("                  *** LA SEÑAL NUNCA SUPERÓ EL UMBRAL. No es "
                  "un problema de DOA:")
            print("                      o el piso se calibró con ruido/fuente "
                  "adentro, o k es muy alto.")
            print("                      Probá --noise-floor con el piso real, "
                  "o bajá --k.")
        elif margen_db < 3:
            print("                  *** Margen muy justo (<3 dB): cualquier "
                  "fluctuación del dron")
            print("                      hace que el evento se caiga contra "
                  "threshold_silence.")

    # --- cadena de gates: dónde se cortó ---
    print("-" * W)
    print("  CADENA        frames -> onset -> evento -> activo -> espectral -> acción")
    print(f"                {stat['frames']:6d}    {stat['onsets']:5d}    "
          f"{stat['events']:6d}    {stat['active']:6d}    "
          f"{stat['spectral_confirms']:9d}    "
          f"{stat['snaps'] + stat['points']:6d}")
    if stat['events'] == 0 and stat['onsets'] > 0:
        print(f"                Hubo onsets pero ninguno llegó a "
              f"EVENT_MIN_FRAMES: la energía no se")
        print( "                sostuvo lo suficiente (o llegó entrecortada por "
               "frames descartados).")
    if spectral_enabled and stat['events'] > 0 and stat['spectral_confirms'] == 0:
        print( "                Energía OK pero el gate armónico NUNCA confirmó. "
               "Ojo: ese gate")
        print(f"                necesita {cfg.SPECTRAL_WINDOW} muestras CONTIGUAS; "
              f"con frames descartados el")
        print( "                peine se emborrona. Repetí con --sin-espectral "
               "para aislarlo.")

    # --- snap-and-hold: por qué el servo no reapuntó ---
    if servo_mode == 'seguimiento' and stat['snap_armado'] > 0:
        print("-" * W)
        print(f"  SNAP-AND-HOLD : armado {stat['snap_armado']}x  ->  "
              f"disparado {stat['snaps']}x  |  PERDIDO {stat['snap_perdido']}x")
        for etiqueta, muestras in (('hasta disparar', stat['snap_espera']),
                                   ('hasta perderse', stat['snap_espera_perdido'])):
            if muestras:
                med_e = float(np.median(muestras))
                print(f"  {'':<14}  espera {etiqueta}: mediana {med_e:.0f} frames "
                      f"({med_e * frame_period_ms:.0f} ms)")
        if stat['snap_perdido'] > 0:
            b = stat['snap_bloq']
            tot = sum(b.values()) or 1
            dom = max(b, key=b.get)
            print(f"  {'':<14}  frames bloqueados por: "
                  f"gate espectral={b['espectral']} ({100*b['espectral']/tot:.0f}%)  "
                  f"DOA inválido={b['invalido']}  "
                  f"confianza<{cfg.EVENT_MIN_CONFIDENCE}={b['confianza']}")
            print(f"  {'':<14}  *** el servo se perdió "
                  f"{100.0 * stat['snap_perdido'] / stat['snap_armado']:.0f}% de "
                  f"los reapuntados.")
            if dom == 'espectral':
                # Dos causas que se ven igual en el contador pero piden acciones
                # opuestas. Discriminante: cuánto duró la espera comparada con lo
                # que el gate necesita para poder confirmar.
                #   espera ~= need  -> el evento terminó antes; llegó TARDE
                #   espera >> need  -> tuvo tiempo de sobra; RECHAZÓ
                need = (cfg.SPECTRAL_WINDOW // cfg.HOP_SIZE) + cfg.SPECTRAL_CONFIRM_WINDOWS
                # Solo los PERDIDOS: los que dispararon no dicen nada sobre por
                # qué el gate bloqueó a los otros.
                perdidos = stat['snap_espera_perdido']
                med = float(np.median(perdidos)) if perdidos else 0.0
                if med > 3 * need:
                    print(f"  {'':<14}      El gate RECHAZÓ la señal: la espera "
                          f"mediana fue {med:.0f} frames")
                    print(f"  {'':<14}      ({med * frame_period_ms / 1000:.1f} s) contra los "
                          f"~{need} hops ({need * frame_period_ms:.0f} ms) que")
                    print(f"  {'':<14}      necesita para poder confirmar. Tiempo "
                          f"le sobró: NO es latencia,")
                    print(f"  {'':<14}      es veredicto. Mirá qué está midiendo "
                          f"con --verbosity 2, y")
                    print(f"  {'':<14}      revisá SPECTRAL_MIN_HARMONICS / "
                          f"SCORE_MIN / HARMONIC_FRACTION_MIN.")
                    if stat['spectral_confirms'] == 0:
                        print(f"  {'':<14}      OJO: 0 confirmaciones en toda la "
                              f"corrida. Si la fuente es una")
                        print(f"  {'':<14}      GRABACIÓN reproducida por parlante, "
                              f"es esperable: el peine")
                        print(f"  {'':<14}      necesita la fundamental de BPF "
                              f"({cfg.SPECTRAL_BPF_MIN:.0f}-{cfg.SPECTRAL_BPF_MAX:.0f} Hz) y un")
                        print(f"  {'':<14}      parlante chico la corta. Validá el "
                              f"gate con --wav sobre la")
                        print(f"  {'':<14}      grabación original, sin cadena de "
                              f"reproducción de por medio.")
                else:
                    print(f"  {'':<14}      El gate LLEGÓ TARDE: necesita ~{need} "
                          f"hops ({need * frame_period_ms:.0f} ms) para")
                    print(f"  {'':<14}      confirmar (llenar {cfg.SPECTRAL_WINDOW} "
                          f"muestras + {cfg.SPECTRAL_CONFIRM_WINDOWS} ventanas)")
                    print(f"  {'':<14}      y el evento duró {med:.0f} frames. "
                          f"Comparalo con --sin-espectral.")

    # --- DOA ---
    print("-" * W)
    if stat['conf']:
        c = np.asarray(stat['conf'])
        print(f"  DOA           : {stat['scans']} escaneos, {stat['valid']} válidos  "
              f"| confianza p50={np.percentile(c, 50):.1f}  "
              f"p95={np.percentile(c, 95):.1f}  max={c.max():.1f} dB")
    else:
        print(f"  DOA           : {stat['scans']} escaneos, ninguna estimación válida")

    # --- tiempos ---
    print("-" * W)
    print("  TIEMPOS (presupuesto por frame = "
          f"{frame_period_ms:.1f} ms)")
    print(meter_loop.summary(audio_seconds=audio_s))
    print(meter_doa.summary())
    if meter_gate.n:
        print(meter_gate.summary())
    if meter_disp.n:
        print(meter_disp.summary())
        # Es I/O de terminal y encima intermitente: se reporta qué fracción de
        # frames lo pagan.
        frac = 100.0 * meter_disp.n / stat['frames']
        d = meter_disp.stats()
        print(f"  {'':<22} corre en {frac:.1f}% de los frames "
              f"(DISPLAY_INTERVAL={cfg.DISPLAY_INTERVAL}s) y les suma "
              f"{d['p50']:.1f} ms")
        if d['p50'] > 1.0:
            print(f"  {'':<22} *** está EN EL CAMINO CRÍTICO. Es I/O de "
                  f"terminal (por SSH bloquea")
            print(f"  {'':<22}     contra la red), no trabajo del algoritmo. "
                  f"Correr con --sin-display.")

    # --- carga condicionada a actividad ---
    # El promedio sobre TODOS los frames miente: la carga es bimodal y en IDLE el
    # frame cuesta ~0. Un sistema a 2.4x durante los eventos da 0.2x global si
    # estuvo callado el 93% del tiempo, y los frames que se pierden son los del
    # evento.
    d = meter_doa.stats()
    rt_activo = None
    if d and stat['scans'] > 0:
        rt_activo = d['p50'] / frame_period_ms
        print("-" * W)
        print(f"  CARGA CON SEÑAL (el promedio global de arriba diluye esto con "
              f"el silencio)")
        print(f"  {'':<12}  frames con escaneo: {stat['scans']}/{stat['frames']} "
              f"({100.0 * stat['scans'] / stat['frames']:.0f}%)")
        print(f"  {'':<12}  costo por frame activo (p50): {d['p50']:.1f} ms  "
              f"-> {rt_activo:.2f}x el presupuesto")

    # --- integridad de la entrada ---
    st = audio.stats() if hasattr(audio, 'stats') else None
    if st:
        print("-" * W)
        print(f"  INTEGRIDAD    : rx={st['received']}  lost={st['lost']}  "
              f"corrupt={st['corrupt']}")
        print(f"  Descartados   : {st['dropped']} ({100 * st['drop_ratio']:.1f}% "
              f"del total)  [cola pico {st['q_high']}/{st['q_max']}]")
        if st['dropped'] > 0:
            # La cola solo se llena durante el evento: el denominador son los
            # frames con señal, no la corrida entera.
            act = stat['scans'] + st['dropped']
            if act > 0:
                print(f"                  Pero la cola solo se llena CON SEÑAL: "
                      f"{st['dropped']} descartados sobre")
                print(f"                  {act} frames de actividad = "
                      f"{100.0 * st['dropped'] / act:.0f}% DEL AUDIO DEL EVENTO "
                      f"nunca llegó al pipeline.")
            print("                  *** Frames LEÍDOS BIEN y tirados por cola "
                  "llena: el procesamiento")
            print("                      no sostiene el tiempo real. Este es el "
                  "modo de falla que")
            print("                      `lost` NO puede ver (el contador del "
                  "ESP32 queda consecutivo).")

    # --- veredicto ---
    # Por el costo CON SEÑAL y los descartes reales, no por el promedio global
    # (que en un sistema que no escanea en silencio mide cuánto silencio hubo).
    print("-" * W)
    rt_global = meter_loop.realtime_factor(audio_s)
    dropped = st['dropped'] if st else 0
    if rt_activo is not None and rt_activo >= 1.0:
        print(f"  VEREDICTO     : NO sostiene tiempo real CON SEÑAL "
              f"({rt_activo:.2f}x el presupuesto).")
        print(f"                  El promedio global ({rt_global:.2f}x) no "
              f"contradice esto: solo dice que")
        print( "                  el sistema estuvo callado la mayor parte del "
               "tiempo. Los frames que")
        print( "                  se pierden son los del evento.")
        print( "                  Ningún tamaño de cola arregla esto; solo corre "
               "la falla hacia adelante.")
        print( "                  Palancas: MUSIC_BIN_STRIDE=2, steering en "
               "complex64, proyección")
        print( "                  sobre el subespacio de SEÑAL, y desacoplar la "
               "tasa de DOA de la del")
        print( "                  detector (el detector y el gate tienen que ver "
               "TODOS los frames).")
    elif dropped > 0:
        print(f"  VEREDICTO     : hubo {dropped} frames descartados pese a un "
              f"costo aparentemente")
        print( "                  holgado — revisá los PICOS (p99/max), no la "
               "mediana: alcanza con")
        print( "                  ráfagas cortas para llenar la cola.")
    elif rt_activo is not None and rt_activo >= 0.85:
        # Mismo corte que perf.summary(): si divergen, el reporte se contradice.
        print(f"  VEREDICTO     : al límite con señal ({rt_activo:.2f}x). Poco "
              f"margen para picos de")
        print( "                  latencia del SO.")
    else:
        margen = f"{rt_activo:.2f}x con señal" if rt_activo is not None else f"{rt_global:.2f}x"
        print(f"  VEREDICTO     : el cómputo entra en presupuesto ({margen}).")
    if args.wav and not args.wav_realtime:
        print( "                  NOTA: corrida OFFLINE. Los tiempos son reales, "
               "pero acá no hubo")
        print( "                  contrapresión posible. Repetí con "
               "--wav-realtime para ver los descartes.")
    print("=" * W)


def main():
    args = parse_args()

    # El modo selecciona un perfil que afina motor DOA + detector.
    # Precedencia por parámetro: flag CLI > perfil del modo > default base.
    if args.evento:
        servo_mode = 'evento'
    elif args.sin_servo:
        servo_mode = None          # solo DOA + registro
    else:
        servo_mode = 'seguimiento'  # default, con o sin flag explícito
    profile    = cfg.MODE_PROFILES.get(servo_mode, {})

    srp_mode         = (args.srp_mode
                        or profile.get('SRP_MODE', cfg.SRP_MODE))
    k_val            = (args.k if args.k is not None
                        else profile.get('DETECTOR_K', cfg.DETECTOR_K))
    silence_ratio    = (args.silence_ratio if args.silence_ratio is not None
                        else profile.get('DETECTOR_SILENCE_RATIO', cfg.DETECTOR_SILENCE_RATIO))
    cov_alpha        = profile.get('COV_ALPHA', cfg.COV_ALPHA)
    event_min_frames = profile.get('EVENT_MIN_FRAMES', cfg.EVENT_MIN_FRAMES)

    # --- Entrada de audio ---
    if args.simulate:
        # --sim-az/--sim-el van en coordenadas REALES; el motor trabaja en el
        # marco del array y el pipeline suma el tilt a la salida.
        audio = SimulatedAudioInput(
            mic_positions  = cfg.MIC_POSITIONS,
            sample_rate    = cfg.SAMPLE_RATE,
            hop_size       = cfg.HOP_SIZE,
            azimuth_deg    = args.sim_az,
            elevation_deg  = args.sim_el - cfg.ARRAY_TILT_DEG,
            speed_of_sound = cfg.SPEED_OF_SOUND,
        )
        print(f"[main] Modo simulado — fuente en az={args.sim_az}° el={args.sim_el}°")
    elif args.wav:
        # Pipeline idéntico al de vivo: solo cambia de dónde salen las
        # muestras y si hay un reloj imponiendo el ritmo.
        audio = WavAudioInput(
            path         = args.wav,
            hop_size     = cfg.HOP_SIZE,
            num_channels = len(cfg.MIC_POSITIONS),
            expected_fs  = cfg.SAMPLE_RATE,
            realtime     = args.wav_realtime,
            drop_policy  = args.drop_policy,
        )
        modo_txt = ("TIEMPO REAL simulado (43 fps + cola acotada)"
                    if args.wav_realtime else
                    "OFFLINE (sin restricción temporal — no se puede perder un frame)")
        print(f"[main] Entrada WAV: {args.wav}")
        print(f"[main]   {audio.total_frames} frames  "
              f"({audio.duration_s:.2f} s @ {audio.sample_rate} Hz)  —  {modo_txt}")
    else:
        audio = SerialAudioInput(
            port            = args.serial,
            baud            = cfg.SERIAL_BAUD,
            hop_size        = cfg.HOP_SIZE,
            bits_per_sample = cfg.BYTES_PER_SAMPLE * 8,
            drop_policy     = args.drop_policy,
        )
        print(f"[main] Puerto serial: {args.serial}")

    # El motor escanea la elevación en el MARCO DEL ARRAY (restando el tilt) y
    # la salida vuelve a coordenadas reales sumándolo. Se satura a ±90°: más
    # allá, _precompute_steering reconstruye cos(el) = sqrt(1-u²), siempre
    # positivo, y el punto de grilla apuntaría a otro lado sin ningún aviso.
    el_scan = (max(-90.0, cfg.ELEVATION_MIN - cfg.ARRAY_TILT_DEG),
               min( 90.0, cfg.ELEVATION_MAX - cfg.ARRAY_TILT_DEG))
    if args.engine == 'srp':
        print("[main] Pre-calculando retardos de grilla (SRP-PHAT)...")
        engine = SRPDoaEngine(
            mic_positions    = cfg.MIC_POSITIONS,
            sample_rate      = cfg.SAMPLE_RATE,
            frame_size       = cfg.HOP_SIZE,
            freq_min         = cfg.FREQ_MIN,
            freq_max         = cfg.FREQ_MAX,
            speed_of_sound   = cfg.SPEED_OF_SOUND,
            az_resolution    = cfg.AZIMUTH_RESOLUTION,
            el_resolution    = getattr(cfg, 'ELEVATION_RESOLUTION', None),
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
            az_resolution    = cfg.AZIMUTH_RESOLUTION,
            el_resolution    = getattr(cfg, 'ELEVATION_RESOLUTION', None),
            num_sources      = cfg.NUM_SOURCES,
            cov_alpha        = cov_alpha,   # resuelto desde el perfil del modo
            diag_loading     = cfg.DIAGONAL_LOADING,
            az_range         = (cfg.AZIMUTH_MIN, cfg.AZIMUTH_MAX),
            el_range         = el_scan,
            # FFT desacoplada del hop: el motor bufferea hasta MUSIC_FFT_SIZE.
            fft_size         = getattr(cfg, 'MUSIC_FFT_SIZE', None),
            bin_stride       = getattr(cfg, 'MUSIC_BIN_STRIDE', 1),
        )
        doa_min_conf   = cfg.DOA_MIN_CONF_UPDATE
        event_min_conf = cfg.EVENT_MIN_CONFIDENCE
        print(f"[main] Motor MUSIC listo "
              f"(hop {engine.hop}, FFT {engine.Nfft} → solape "
              f"{100 * (1 - 1 / engine.n_hops):.0f}%, df {engine.df:.1f} Hz, "
              f"{engine.freqs.size} bins en [{cfg.FREQ_MIN}, {cfg.FREQ_MAX}] Hz).")

    # --- Calibración diagonal de ganancia por canal ---
    _cg = getattr(cfg, 'CHANNEL_GAINS', None)
    if _cg is None:
        _channel_gains = None
    else:
        _channel_gains = np.asarray(_cg, dtype=np.float64)
        if _channel_gains.shape != (len(cfg.MIC_POSITIONS),):
            raise SystemExit(
                f"[main] CHANNEL_GAINS debe tener {len(cfg.MIC_POSITIONS)} "
                f"valores, tiene {_channel_gains.shape}")
        if args.verbosity >= 1:
            print(f"[main] Calibración de ganancia por canal: "
                  + ", ".join(f"{g:.3f}" for g in _channel_gains))
            print( "       (corrige el sesgo del DOA, NO la relación S/R de los "
                   "canales débiles)")

    # --- Detector ---
    if not (0.0 < silence_ratio < 1.0):
        print(f"[main] ADVERTENCIA: silence_ratio={silence_ratio} fuera del rango "
              f"(0.0, 1.0). Usando valor por defecto 0.5.", flush=True)
        silence_ratio = 0.5

    # En simulación la fuente suena desde el frame 0: la calibración absorbería
    # la señal y el detector nunca dispararía. Piso bajo para entrar en ACTIVE.
    noise_floor_cfg = getattr(cfg, 'DETECTOR_NOISE_FLOOR', None)
    if args.simulate and noise_floor_cfg is None:
        noise_floor_cfg = 1e-6
    if args.noise_floor is not None:
        noise_floor_cfg = args.noise_floor

    detector = EventDetector(
        k               = k_val,
        min_frames      = event_min_frames,  
        cooldown_frames = cfg.COOLDOWN_FRAMES,
        silence_ratio   = silence_ratio,
        calib_frames    = cfg.DETECTOR_CALIB_FRAMES,
        noise_floor     = noise_floor_cfg,
        calib_percentile = getattr(cfg, 'DETECTOR_CALIB_PERCENTILE', 20.0),
        # El gate de energía mide en la MISMA banda que enmascara el motor DOA.
        band = ((cfg.HOP_SIZE, len(cfg.MIC_POSITIONS), cfg.SAMPLE_RATE,
                 cfg.FREQ_MIN, cfg.FREQ_MAX)
                if getattr(cfg, 'DETECTOR_BAND_LIMITED', True) else None),
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

    # Gate 2. Solo en seguimiento/sin-servo: el modo EVENTO lo saltea siempre
    # (impulsivas, y su disparo precede al llenado de la ventana del gate).
    spectral_enabled = (getattr(cfg, 'SPECTRAL_ENABLED', True)
                        and not args.sin_espectral
                        and servo_mode != 'evento')

    # El gate no le pasa nada al motor: MUSIC calcula igual y el veredicto
    # solo condiciona si el resultado se usa (servo, registro).
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

    # Con --wav no hay servo: pigpiod contaminaría la medición de tiempos. El
    # servo_mode —y con él el perfil y el snap-and-hold— se conserva intacto.
    servo = None
    if servo_mode is not None and not args.wav:
        cfg.SERVO_ENABLED = True
        servo = ServoController(cfg)
    elif servo_mode is not None and args.wav:
        print(f"[main] Servo DESHABILITADO (entrada WAV) — el perfil "
              f"'{servo_mode}' se aplica igual.")

    lock_duration = args.servo_lock_time

    # --- Loop principal ---
    if args.gain != 1.0 and args.verbosity >= 1:
        gain_db = 20 * np.log10(abs(args.gain))
        print(f"[main] Ganancia digital: x{args.gain} ({gain_db:+.1f} dB)")

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

    # =====================================================================
    # INSTRUMENTACIÓN
    # =====================================================================
    # PRESUPUESTO DURO del lazo: el firmware entrega un hop cada
    # HOP_SIZE/SAMPLE_RATE s. Si una iteración tarda más, la cola del lector se
    # llena y se descartan frames sin que el contador del ESP32 lo note.
    frame_period_ms = 1000.0 * cfg.HOP_SIZE / cfg.SAMPLE_RATE
    meter_loop = PerfMeter("lazo completo",  budget_ms=frame_period_ms)
    meter_doa  = PerfMeter(f"DOA ({args.engine})", budget_ms=frame_period_ms)
    meter_gate = PerfMeter("gate espectral")
    # El display se mide aparte: es I/O de terminal, no cómputo, y solo corre
    # 1 de cada DISPLAY_INTERVAL*fps frames.
    meter_disp = PerfMeter("display (I/O terminal)")

    stat = {
        'frames': 0, 'scans': 0, 'valid': 0, 'events': 0, 'active': 0,
        'onsets': 0, 'spectral_confirms': 0, 'snaps': 0, 'points': 0,
        'energy_max': 0.0, 'conf': [],
        # Traza del snap-and-hold (modo seguimiento):
        #   snap_armado  : veces que un evento armó el snap
        #   snap_perdido : veces que el evento terminó con el snap aún armado
        #   snap_bloq    : desglose de por qué no llegó a dispararse
        'snap_armado': 0, 'snap_perdido': 0,
        'snap_bloq': {'espectral': 0, 'invalido': 0, 'confianza': 0},
        # Esperas SEPARADAS: mezclarlas contamina el discriminante del resumen,
        # que necesita cuánto esperaron los snaps que NO llegaron a disparar.
        'snap_espera': [],           # frames armado -> disparo
        'snap_espera_perdido': [],   # frames armado -> fin del evento
    }

    # =====================================================================
    # PRECALENTAMIENTO
    # =====================================================================
    # La primera llamada a cada bloque reserva sus workspaces (planes de FFT,
    # temporales de numpy): decenas de ms de costo único que dentro del lazo se
    # comerían el margen para picos de latencia del SO. Se paga acá con frames
    # de ceros, y después se resetea todo para que no queden en el buffer del
    # motor, la ventana del gate ni el latch.
    _warm = np.zeros((cfg.HOP_SIZE, len(cfg.MIC_POSITIONS)))
    for _ in range(max(2, getattr(engine, 'n_hops', 1) + 1)):
        engine.process(_warm)
    if hasattr(engine, 'reset'):
        engine.reset()
    if spectral_gate is not None:
        for _ in range(cfg.SPECTRAL_WINDOW // cfg.HOP_SIZE + cfg.SPECTRAL_CONFIRM_WINDOWS + 1):
            spectral_gate.update(_warm)
        spectral_gate.reset()
    if args.verbosity >= 1:
        print("[main] Precalentamiento hecho (workspaces de FFT reservados).")

    audio.start()
    t_wall0 = time.time()
    last_display = 0
    last_result  = None
    _idle_frames = 0   # frames IDLE consecutivos, para el reset diferido de R
    _spectral_prev = False  # estado previo del gate (log de transición)
    _snap_pending  = False  # seguimiento: snap armado, espera firma espectral
    _event_logged  = False  # evento: primera estimación ya registrada
    _was_calibrating = detector.calibrating

    # Con entrada WAV se saltea el display: hace _clear() en cada refresco y
    # borraría las líneas de evento que interesan. Se fuerza con --verbosity 3.
    show_display = not (args.wav and args.verbosity < 3) and not args.sin_display

    # Traza del snap: en qué frame se armó y qué lo bloqueó.
    _snap_arm_frame = None
    _snap_why = {'espectral': 0, 'invalido': 0, 'confianza': 0}

    # Timestamp hasta el cual, en SEGUIMIENTO, el servo queda fijado al último
    # evento: mientras dura, el seguimiento continuo no lo mueve.
    lock_until = 0.0

    try:
        while True:
            frame = audio.read_frame(timeout=1.0)
            if frame is None:
                # Solo WavAudioInput señala EOF; en serial/simulado un None es
                # simplemente un timeout sin datos.
                if getattr(audio, 'eof', False):
                    break
                continue

            # El cronómetro arranca DESPUÉS de read_frame: esperar no es
            # trabajo, es tiempo ocioso.
            t_iter = time.perf_counter()
            stat['frames'] += 1

            # Ganancia digital (no modifica el array original)
            if args.gain != 1.0:
                frame = frame * args.gain

            # Antes del detector y del motor: el steering asume ganancias
            # iguales y el piso debe medirse sobre señal ya calibrada.
            if _channel_gains is not None:
                frame = frame * _channel_gains

            # MISMA medida que usa el detector para su umbral (banda útil).
            energy = detector.energy(frame)
            if energy > stat['energy_max']:
                stat['energy_max'] = energy
            det_signal = detector.update(frame, energy=energy)

            # Fin de la calibración: si el piso quedó alto (la grabación no
            # empezaba en silencio) el umbral es inalcanzable, y el piso es FIJO.
            if _was_calibrating and not detector.calibrating:
                _was_calibrating = False
                if args.verbosity >= 1:
                    print(f"[detector] Piso calibrado: {detector.noise_floor:.3e} "
                          f"({10 * np.log10(max(detector.noise_floor, 1e-12)):+.1f} dB)  "
                          f"-> umbral evento {detector.threshold_event:.3e} "
                          f"({10 * np.log10(max(detector.threshold_event, 1e-12)):+.1f} dB), "
                          f"silencio {detector.threshold_silence:.3e}")

            # Solo se escanea con actividad: nadie consume el pseudoespectro
            # del silencio y en la RPi ese cómputo define si el sistema sostiene
            # el tiempo real. R queda congelada hasta el reset diferido.
            if det_signal != 'idle':
                with meter_doa:
                    result = engine.process(frame)
                stat['scans'] += 1
                if result.valid:
                    stat['valid'] += 1
                    stat['conf'].append(result.confidence)
                # Marco del array -> elevación real: real = array + tilt.
                if result.valid and cfg.ARRAY_TILT_DEG:
                    result.elevation += cfg.ARRAY_TILT_DEG
            else:
                result = DOAResult()   # inválido: sin escaneo en silencio

            if det_signal == 'event':
                stat['events'] += 1
            elif det_signal == 'active':
                stat['active'] += 1
            elif det_signal == 'onset':
                stat['onsets'] += 1

            # Solo se incorporan estimaciones con señal confirmada: en silencio
            # SRP-PHAT da confianza sobre ruido y el tracker derivaría.
            if det_signal in ('event', 'active'):
                _idle_frames = 0
                az_smooth, el_smooth = tracker.update(result)
            elif det_signal == 'onset':
                # Flanco de subida: el motor acumula estos frames pero el
                # evento no está confirmado, así que el tracker no se toca.
                _idle_frames = 0
                az_smooth, el_smooth = tracker.value()
            else:  # 'idle': silencio / cooldown
                az_smooth, el_smooth = tracker.value()
                _idle_frames += 1
                # R y la ventana del gate solo se limpian tras silencio
                # SOSTENIDO: resetear en cada bajón breve del dron obligaría a
                # reconstruir R y haría parpadear el tracking.
                reset_threshold = getattr(cfg, 'MUSIC_RESET_IDLE_FRAMES', 0)
                if _idle_frames >= max(reset_threshold, 1):
                    if hasattr(engine, 'reset'):
                        engine.reset()
                    if spectral_gate is not None:
                        spectral_gate.reset()
                    _idle_frames = 0   # no llamar reset() en cada frame
            if result.valid:
                last_result = result   # el display conserva la última real

            # =================================================================
            # GATE ESPECTRAL ARMÓNICO
            # =================================================================
            # El de energía ya marcó actividad; acá se confirma el peine de
            # BPF. `spectral_pass` condiciona todo lo que sigue; con
            # --sin-espectral es siempre True.
            if spectral_gate is not None:
                if det_signal in ('event', 'active', 'onset'):
                    with meter_gate:
                        sres = spectral_gate.update(frame)
                    spectral_pass = sres.is_drone
                    if spectral_pass and not _spectral_prev:
                        stat['spectral_confirms'] += 1
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
                    # idle/cooldown: nada que evaluar, pero el latch decae
                    # igual (no sobrevive más de hold_frames sin evidencia).
                    spectral_gate.idle_tick()
                    spectral_pass = spectral_gate.confirmed
                    _spectral_prev = spectral_pass
            else:
                spectral_pass = True   # gate desactivado: pasa todo

            # =================================================================
            # REGISTRO — UNA fila por EVENTO
            # =================================================================
            #   evento     → fila inmediata con la primera estimación válida.
            #   seguimiento → track() acumula y end_event() escribe una fila con
            #                 el RANGO de ángulos al volver a idle.
            # Se registra el estimado CRUDO, no el suavizado.
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
            # MODO SEGUIMIENTO: tracking continuo + snap-and-hold
            # =================================================================
            # Con señal el servo sigue la fuente suavemente (batching, zona
            # muerta, paso máximo). Al confirmarse un evento salta crudo a esa
            # dirección y la mantiene lock_duration segundos, durante los cuales
            # el seguimiento continuo no lo mueve. tick() lo devuelve al centro.
            if servo_mode == 'seguimiento':
                now_t  = time.time()
                locked = now_t < lock_until

                # Snap DIFERIDO: se ARMA en 'event' y dispara en el primer
                # frame con la firma confirmada. El gate necesita ~9 hops más las
                # confirmaciones, así que en 'event' todavía no pudo confirmar;
                # sin el diferido nunca dispararía con el gate activo.
                if det_signal == 'event':
                    _snap_pending = True
                    stat['snap_armado'] += 1
                    _snap_arm_frame = stat['frames']
                    _snap_why = {'espectral': 0, 'invalido': 0, 'confianza': 0}
                elif det_signal == 'idle':
                    if _snap_pending:
                        # Terminó sin reapuntar: se registra cuántos frames
                        # estuvo bloqueado por cada causa.
                        stat['snap_perdido'] += 1
                        for kk in _snap_why:
                            stat['snap_bloq'][kk] += _snap_why[kk]
                        if _snap_arm_frame is not None:
                            stat['snap_espera_perdido'].append(
                                stat['frames'] - _snap_arm_frame)
                        if args.verbosity >= 1:
                            dom = max(_snap_why, key=_snap_why.get)
                            print(f"\n[snap PERDIDO] evento de "
                                  f"{stat['frames'] - (_snap_arm_frame or 0)} frames "
                                  f"terminó sin reapuntar — bloqueado por: "
                                  f"espectral={_snap_why['espectral']} "
                                  f"invalido={_snap_why['invalido']} "
                                  f"confianza={_snap_why['confianza']} "
                                  f"(dominante: {dom})")
                    _snap_pending = False
                    _snap_arm_frame = None

                # Qué condición bloquea el snap mientras espera. Mismo orden que
                # la guarda de abajo.
                if _snap_pending and det_signal in ('event', 'active'):
                    if not spectral_pass:
                        _snap_why['espectral'] += 1
                    elif not result.valid:
                        _snap_why['invalido'] += 1
                    elif result.confidence < event_min_conf:
                        _snap_why['confianza'] += 1

                if _snap_pending and det_signal in ('event', 'active') and \
                   spectral_pass and result.valid and \
                   result.confidence >= event_min_conf:
                    # Snap crudo y forzado: se busca reactividad, no suavizado.
                    if servo:
                        servo.point_to(result, force=True)
                    lock_until = now_t + lock_duration
                    locked = True
                    _snap_pending = False
                    stat['snaps'] += 1
                    if _snap_arm_frame is not None:
                        stat['snap_espera'].append(stat['frames'] - _snap_arm_frame)
                    _snap_arm_frame = None
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
            # Sin gate espectral: dispara por energía sola, en el PRIMER frame
            # confirmado y al estimado crudo, sin promedios ni reintentos. No hay
            # seguimiento ni retorno al centro: el servo queda fijo hasta el
            # próximo evento. Se espera al frame 'event' porque recién ahí la
            # covarianza absorbió ~3 hops de señal nueva; antes erra 20°+.
            elif servo_mode == 'evento':
                if det_signal == 'event' and result.valid and \
                   result.confidence >= event_min_conf:
                    if servo:
                        servo.point_to(result, force=True)
                    stat['points'] += 1
                    if args.verbosity >= 1:
                        servo_tag = "" if servo else " [sin servo]"
                        print(f"\n[evento->apunta] az={result.azimuth:+.1f}° "
                              f"el={result.elevation:+.1f}° "
                              f"conf={result.confidence:.1f} dB{servo_tag}")
                elif args.verbosity >= 2 and det_signal == 'event':
                    print(f"\n[evento descartado] conf={result.confidence:.1f} dB "
                          f"< {event_min_conf} dB (o inválido)")

            # Display — usa ángulos suavizados
            now = time.time()
            if show_display and now - last_display >= cfg.DISPLAY_INTERVAL:
                servo_az, servo_el = (servo.position if servo else (None, None))

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

                # Todo el refresco va adentro del medidor, incluida la
                # agregación de estadísticas: si no, el instrumento no se mide a
                # sí mismo. La ventana acotada acota su costo.
                with meter_disp:
                    serial_stats = audio.stats() if hasattr(audio, 'stats') else None
                    perf_stats = meter_loop.stats(
                        limit=getattr(cfg, 'DISPLAY_STATS_WINDOW', 300))
                    if perf_stats:
                        perf_stats['budget_ms'] = frame_period_ms
                    render(
                        doa_result        = display_result,
                        det_state         = detector.state,
                        energy            = energy,
                        threshold_event   = detector.threshold_event,
                        threshold_silence = detector.threshold_silence,
                        noise_floor       = detector.noise_floor,
                        az_range          = (cfg.AZIMUTH_MIN, cfg.AZIMUTH_MAX),
                        el_range          = (cfg.ELEVATION_MIN, cfg.ELEVATION_MAX),
                        serial_stats      = serial_stats,
                        servo_az          = servo_az,
                        servo_el          = servo_el,
                        perf_stats        = perf_stats,
                    )
                last_display = now

            # Incluye TODO el trabajo por frame (detector, DOA, gate, servo,
            # display), que es lo que compite por el presupuesto.
            meter_loop.add(time.perf_counter() - t_iter)

    except KeyboardInterrupt:
        print("\n[main] Detenido por el usuario.")
    finally:
        audio.stop()
        if logger:
            logger.close()
        if servo:
            servo.close()
        _print_summary(args, cfg, audio, detector, stat,
                       meter_loop, meter_doa, meter_gate, meter_disp,
                       frame_period_ms, time.time() - t_wall0,
                       spectral_enabled, servo_mode)


if __name__ == '__main__':
    main()
