"""
Estimación de Dirección de Arribo (DOA) acústico utilizando algoritmo GCC-PHAT
para Rapsberry Pi y dos micrófonos MEMS INMP441
con seguimiento con servomotor

autor: Rodrigo Boyé
Proyecto Final Integrador - Ingeniería en Telecomunicaciones
Universidad Nacional de Río Negro
Trabajo realizado con Invap
Bariloche, Argentina, 2026

Conexiones:
cableado I2S (ambos micrófonos comparten bus)
| INMP441 Pin | RPi GPIO (BCM) | Notas |
|-------------|----------------|-------|
| SCK         | GPIO 18        | Bit Clock (BCLK) |
| WS          | GPIO 19        | Word Select (LRCLK) |
| SD          | GPIO 20        | Data In |
| VDD         | 3.3V           | Alimentación |
| GND         | GND            | Tierra |
| L/R (mic L) | GND            | Canal izquierdo |
| L/R (mic R) | 3.3V           | Canal derecho |
IMPORTANTE: Para configurar la grabación I2S en la RPi, ejecutar el script setup_rpi_i2s.sh (reboot requerido)

cableado del servo (especifico para RPi 3A+)
| Servo Cable  | RPi GPIO (BCM) | Notas |
|--------------|----------------|-------|
| Señal        | GPIO 12        | PWM0 (default) o GPIO 13 (PWM1) |
| Alimentación | 5V             | para servos de baja corriente, sino usar fuente externa|
| Tierra       | GND            | Tierra común con la RPi |

"""

import argparse
import os
import sys
import time
import signal
import numpy as np
from datetime import datetime

# configuración de parámetros
k = 100  # factor de multiplicación para el umbral de evento, ajustable según el entorno y la sensibilidad deseada

DEFAULT_CONFIG = {
    'sample_rate': 48000,       # frecuencia de muestreo de los microfonos en Hz
    'block_size': 128,         # tamaño del bloque para procesamiento, mas pequeño -> mas rapido, grande -> preciso
    'mic_distance': 0.035,       # distancia entre los micrófonos en metros
    'speed_of_sound': 343.0,    # velocidad del sonido en m/s, ajustar en temperaturas muy altas o bajas
    'umbral_ruido': 1e-5,       # umbral de energia para considerar que hay una señal valida y procesar el DOA
    'umbral_evento': 1e-5 * k,  # umbral de energia para considerar que hay un evento significativo y registrarlo
}

# ------------------------------------------------------------------------------------------------

# ALGORITMO GCC-PHAT (correlación generalizada con transformación de fase)
#
# Referencia: Knapp & Carter, 1976, "The Generalized Correlation Method for Estimation of Time Delay"
#
# señales x1, x2 -> ventana -> FFT -> cross-spectrum -> normalización PHAT -> IFFT -> pico -> TDOA -> DOA
#
# retorna
#   tau (float): el retardo en muestras entre las señales
#   confidence (float rango [0 1]): la confianza de la estimación, basada en la magnitud del pico 
#
# cache de ventanas para no recalcular en cada frame
_window_cache = {}

def gcc_phat(x1, x2, max_delay_samples):
    n = len(x1)
    nfft = 2*n
    
    if n not in _window_cache:
        _window_cache[n] = np.hanning(n)
    window = _window_cache[n]
    
    X1 = np.fft.rfft(x1 * window, n=nfft)
    X2 = np.fft.rfft(x2 * window, n=nfft)
    
    cross_spectrum = X1 * np.conj(X2)
    magnitude = np.abs(cross_spectrum)
    eps = 1e-10
    return cross_spectrum / (magnitude + eps)


def gcc_phat_resolve(phat_avg, n, max_delay_samples):
    """
    toma el cross-spectrum PHAT promediado (de uno o más bloques) y
    calcula el TDOA y la confianza.
    separado de gcc_phat para permitir acumular varios bloques antes de resolver.
    """
    nfft = 2 * n
    eps = 1e-10
    
    gcc = np.fft.fftshift(np.fft.irfft(phat_avg, n=nfft))
    center = nfft // 2
    
    search_start = center - max_delay_samples
    search_end   = center + max_delay_samples + 1
    gcc_search = gcc[search_start:search_end]
    
    peak_local  = np.argmax(gcc_search)
    peak_global = search_start + peak_local
    confidence  = float(gcc[peak_global])
    tau         = -(peak_global - center)
    
    # interpolación parabólica sobre valores reales
    if 0 < peak_local < len(gcc_search) - 1:
        alpha = gcc_search[peak_local - 1]
        beta  = gcc_search[peak_local]
        gamma = gcc_search[peak_local + 1]
        denom = alpha - 2*beta + gamma
        if abs(denom) > eps:
            tau = tau - float(np.clip(0.5 * (alpha - gamma) / denom, -0.5, 0.5))
    
    return float(tau), confidence


# conversión de TDOA a DOA
# cos(theta) = tau[s] * c / d
#
# retorna 
#   ang_deg (float): el DOA en grados
#   valid (bool): indica si el valor de cos(theta) está dentro del rango válido [-1, 1]
#
def tdoa_to_angle(tau, sample_rate, mic_distance, speed_of_sound):
    # convertir retardo en muestras a segundos
    tau_seconds = tau / sample_rate
    
    # calcular el ángulo de llegada
    cos_theta = tau_seconds * speed_of_sound / mic_distance
    
    # verificar que el rango sea valido
    valid = True
    if abs(cos_theta) > 1.0:
        valid = False
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
    # si el rango no es valido se clipea pero se advierte error de medicion
        
    angle_rad = np.arccos(cos_theta)
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg, valid

#------------------------------------------------------------------------------------------------
# CAPTURA DE AUDIO
#
# utiliza sounddevice para capturar audio desde ALSA (Advanced Linux Sound Architecture)
#
# captura audio stereo en bloques y separa en canal izquierdo y derecho
# referencia: https://python-sounddevice.readthedocs.io/en/0.3.14/api.html
class AudioCapture:
    def __init__(self, sample_rate, block_size, device=None):
        self.sample_rate = sample_rate
        self.block_size = block_size
        self.device = device
        self.stream = None
        self.overflow_count = 0
        
    def start(self):
        import sounddevice as sd
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            blocksize=self.block_size,
            channels=2,
            dtype='float32',
            device=self.device,
            latency='high',  # pide buffer más grande a PortAudio, reduce overflows
        )
        self.stream.start()
        print(f"Stream abierto: {self.sample_rate} Hz, "
          f"{self.block_size} samples por bloque")
    
    def read_block(self):
        data, overflowed = self.stream.read(self.block_size)
        if overflowed:
            self.overflow_count += 1
        ch_left = data[:, 0]
        ch_right = data[:, 1]
        return ch_left, ch_right
    
    def stop(self):
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
            if self.overflow_count > 0:
                print(f"Stream cerrado ({self.overflow_count} overflows durante la sesión)")
            else:
                print("Stream cerrado")

# simula una señal con ruido para probar el algoritmo sin hardware            
class SimulatedCapture:
    def __init__(self, sample_rate, block_size, mic_distance, speed_of_sound,
                 source_angle_deg=60.0, source_freq=1000.0, snr_db=20.0):
        self.sample_rate = sample_rate
        self.block_size = block_size
        self.mic_distance = mic_distance
        self.speed_of_sound = speed_of_sound
        self.source_angle_deg = source_angle_deg
        self.source_freq = source_freq
        self.snr_db = snr_db
        self.t_offset = 0  # Para continuidad entre bloques
        print(f"[SIM] Simulando fuente a {source_angle_deg}° | "
              f"f={source_freq} Hz | SNR={snr_db} dB")

    def start(self):
        pass  # Nada que inicializar

    def read_block(self):
        n = self.block_size
        t = (self.t_offset + np.arange(n)) / self.sample_rate
        self.t_offset += n

        # Señal fuente (tono puro + algo de banda ancha para realismo)
        signal_tone = np.sin(2 * np.pi * self.source_freq * t)
        broadband = np.random.randn(n) * 0.3
        source = signal_tone + broadband

        # Calcular retardo en muestras
        angle_rad = np.radians(self.source_angle_deg)
        tau_seconds = (self.mic_distance / self.speed_of_sound) * np.cos(angle_rad)

        # Aplicar retardo al canal derecho usando interpolación en frecuencia
        # (método exacto, sin artefactos de interpolación temporal)
        S = np.fft.rfft(source)
        freqs = np.fft.rfftfreq(n, d=1.0 / self.sample_rate)
        phase_shift = np.exp(-1j * 2 * np.pi * freqs * tau_seconds)
        ch_right = np.fft.irfft(S * phase_shift, n=n)
        ch_left = source.copy()

        # Agregar ruido
        noise_power = np.var(source) / (10 ** (self.snr_db / 10))
        ch_left += np.random.randn(n) * np.sqrt(noise_power)
        ch_right += np.random.randn(n) * np.sqrt(noise_power)

        return ch_left.astype(np.float32), ch_right.astype(np.float32)

    def stop(self):
        pass
    
    
# ------------------------------------------------------------------------------------------------
# VISUALIZACION EN LA TERMINAL
#
# muestra el DOA estimado en una barra horizontal ASCII
#
class TerminalDisplay:
    
    BAR_WIDTH = 60  # caracteres de ancho de la barra

    def __init__(self):
        self.angle_history = []
        self.max_history = 10  # Para promedio móvil (suavizado)

    def show(self, angle_deg, confidence, valid, energy, is_silent,
             servo_angle=None, buf_fill=None):
        """
        Dibuja una línea en terminal mostrando el ángulo estimado.

        180° ←──────────────|──────────────→ 0°
        (izquierda)      (frente)       (derecha)
        """
        if is_silent:
            status = "\r[SILENCIO]  --- sin señal suficiente ---"
            sys.stdout.write(status.ljust(120))
            sys.stdout.flush()
            return

        # Promedio móvil para suavizar la visualización
        self.angle_history.append(angle_deg)
        if len(self.angle_history) > self.max_history:
            self.angle_history.pop(0)
        smoothed = np.mean(self.angle_history)

        # Mapear ángulo [0, 180] a posición en la barra [0, BAR_WIDTH-1]
        # 180° = izquierda (posición 0), 0° = derecha (posición BAR_WIDTH-1)
        pos = int((1.0 - smoothed / 180.0) * (self.BAR_WIDTH - 1))
        pos = np.clip(pos, 0, self.BAR_WIDTH - 1)

        # Construir barra
        bar = list("─" * self.BAR_WIDTH)
        # Marcar el centro (90° = broadside)
        center = self.BAR_WIDTH // 2
        bar[center] = "│"
        # Marcar posición estimada
        bar[pos] = "●"

        bar_str = "".join(bar)

        # Indicador de validez y confianza
        flag = "✓" if valid else "⚠"
        conf_bar = "█" * int(confidence * 10) + "░" * (10 - int(confidence * 10))

        servo_str = f" srv={servo_angle:5.1f}°" if servo_angle is not None else ""
        buf_str = ""
        if buf_fill is not None:
            cur, total = buf_fill
            filled = int(cur / total * 5)
            buf_str = f" buf[{'▮' * filled}{'▯' * (5 - filled)}]"

        line = (f"\r 180°[{bar_str}]0°  "
                f"θ={smoothed:6.1f}°{servo_str}{buf_str} {flag} "
                f"conf[{conf_bar}]")
        sys.stdout.write(line)
        sys.stdout.flush()

    def clear_history(self):
        self.angle_history.clear()
        
        
# ------------------------------------------------------------------------------------------------
# CONTROL DE SERVOMOTOR
#
# controla un servo para apuntar hacia el DOA estimado
# utiliza pigpio para PWM por hardware (DMA) — sin jitter
# requiere: sudo pigpiod -t 0 (iniciar daemon antes de ejecutar, -t 0 evita conflicto con I2S)
# referencia: https://abyz.me.uk/rpi/pigpio/python.html
#
# dos modos de operación (se eligen con --seguimiento o --evento):
#   seguimiento: el servo sigue el DOA en tiempo real (batching + dead zone + movimiento adaptativo)
#   evento: el servo solo se mueve cuando se detecta un evento, y se queda fijo lock_duration segundos

class ServoController:
    
    #invertidos para el sentido que uso en el montaje físico, ajustar si el servo se mueve en dirección contraria
    MIN_PULSE = 2500   # microsegundos para 0°
    MAX_PULSE = 700  # microsegundos para 180°
    
    def __init__(self, gpio_pin=12, min_angle=0, max_angle=180,
                 batch_size=8, min_confidence=0.2, dead_zone=5.0,
                 max_step=20.0, lock_duration=3.0,
                 invert=False, simulate=False):
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.batch_size = batch_size
        self.min_confidence = min_confidence
        self.dead_zone = dead_zone
        self.max_step = max_step
        self.invert = invert
        self.simulate = simulate
        self.gpio_pin = gpio_pin
        
        self.angle_buffer = []
        self.last_sent_angle = 90.0
        self.silence_count = 0
        self.silence_threshold = 30
        
        self.locked = False
        self.lock_until = 0
        self.lock_duration = lock_duration
        
        self.pi = None
        
        if not simulate:
            self._init_hardware(gpio_pin)
    
    def _angle_to_pulsewidth(self, angle):
        """convierte grados [0, 180] a microsegundos [500, 2500]"""
        angle = max(self.min_angle, min(self.max_angle, angle))
        ratio = (angle - self.min_angle) / (self.max_angle - self.min_angle)
        return int(self.MIN_PULSE + ratio * (self.MAX_PULSE - self.MIN_PULSE))
    
    def _init_hardware(self, gpio_pin):
        try:
            import pigpio
            self.pi = pigpio.pi()
            if not self.pi.connected:
                print("[SERVO] ERROR: no se puede conectar a pigpiod.")
                print("        Ejecutar: sudo pigpiod -t 0")
                self.simulate = True
                return
            
            self._move_to(90)
            time.sleep(1)
            self._stop_pwm()
            print(f"[SERVO] pigpio conectado | GPIO {gpio_pin} | "
                  f"batch={self.batch_size} | dead_zone={self.dead_zone}°")
            
        except ImportError:
            print("[SERVO] ERROR: pigpio no disponible. Instalar: pip install pigpio")
            self.simulate = True
        except Exception as e:
            print(f"[SERVO] ERROR: {e}")
            self.simulate = True
    
    def _move_to(self, angle):
        if not self.simulate and self.pi is not None:
            self.pi.set_servo_pulsewidth(self.gpio_pin, self._angle_to_pulsewidth(angle))
    
    def _stop_pwm(self):
        if not self.simulate and self.pi is not None:
            self.pi.set_servo_pulsewidth(self.gpio_pin, 0)

    def lock_to(self, angle, duration=None):
        """bloquea el servo en un angulo por duration segundos (usado en modo evento)"""
        if duration is None:
            duration = self.lock_duration
        target = (self.max_angle - angle) if self.invert else angle
        target = max(self.min_angle, min(self.max_angle, target))
        self.locked = True
        self.lock_until = time.time() + duration
        self.last_sent_angle = target
        self.angle_buffer.clear()
        self._move_to(target)
    
    def is_locked(self):
        if self.locked and time.time() >= self.lock_until:
            self.locked = False
            self._stop_pwm()
        return self.locked
        
    def update(self, doa_angle, confidence):
        """alimenta una estimacion. cuando el buffer se llena, mueve el servo (modo seguimiento)."""
        self.silence_count = 0
        
        if self.is_locked():
            return False
        
        if confidence < self.min_confidence:
            self.angle_buffer.append(None)
        else:
            angle = (self.max_angle - doa_angle) if self.invert else doa_angle
            angle = max(self.min_angle, min(self.max_angle, angle))
            self.angle_buffer.append(angle)
        
        if len(self.angle_buffer) < self.batch_size:
            return False
        
        valid_angles = [a for a in self.angle_buffer if a is not None]
        self.angle_buffer.clear()
        
        if len(valid_angles) < max(2, self.batch_size * 0.3):
            return False
        
        angles = np.array(valid_angles)
        median = np.median(angles)
        std = np.std(angles)
        if std > 1.0:
            angles = angles[np.abs(angles - median) <= 2.0 * std]
        if len(angles) == 0:
            return False
        
        target_angle = float(np.mean(angles))
        fast_threshold = self.max_step * 3
        diff = target_angle - self.last_sent_angle
        
        if abs(diff) < self.dead_zone:
            return False
        
        if abs(diff) > fast_threshold:
            new_angle = target_angle
        elif abs(diff) > self.max_step:
            new_angle = self.last_sent_angle + self.max_step * np.sign(diff)
        else:
            new_angle = target_angle
        
        new_angle = max(self.min_angle, min(self.max_angle, new_angle))
        self.last_sent_angle = new_angle
        self._move_to(new_angle)
        return True
    
    def notify_silence(self):
        if self.is_locked():
            return
        self.silence_count += 1
        self.angle_buffer.clear()
        if self.silence_count >= self.silence_threshold:
            if abs(self.last_sent_angle - 90.0) > self.dead_zone:
                diff = 90.0 - self.last_sent_angle
                if abs(diff) > self.max_step:
                    new_angle = self.last_sent_angle + self.max_step * np.sign(diff)
                else:
                    new_angle = 90.0
                self.last_sent_angle = new_angle
                self._move_to(new_angle)
            self.silence_count = self.silence_threshold - 5
    
    def get_servo_angle(self):
        return self.last_sent_angle
    
    def get_buffer_fill(self):
        return len(self.angle_buffer), self.batch_size
    
    def stop(self):
        if not self.simulate and self.pi is not None:
            self._move_to(90)
            time.sleep(0.3)
            self._stop_pwm()
            self.pi.stop()
            print("[SERVO] Desactivado")
            
# ------------------------------------------------------------------------------------------------
# REGISTRO DE EVENTOS
#
# detecta cuando la energia supera umbral_evento (señal significativa, no solo ruido)
# y registra el evento en un archivo de log con timestamp, angulo, confianza y energia
#
# el log se guarda como CSV para facilitar análisis posterior
# cada linea: timestamp, angulo_deg, confianza, energia, valid
#
# referencia: umbral_evento = umbral_ruido * k (configurable, default k=10)

class EventLogger:
    def __init__(self, umbral_evento, log_path="eventos_doa.csv"):
        self.umbral_evento = umbral_evento
        self.log_path = log_path
        self.event_count = 0
        self.in_event = False       # para agrupar frames consecutivos como un solo evento
        self.event_start_time = None
        self.event_angles = []      # angulos durante el evento actual
        self.event_confidences = [] # confianzas durante el evento actual
        self.event_peak_energy = 0  # energia maxima del evento actual
        
        # crear archivo con encabezado si no existe
        self._init_log()
        print(f"[EVENTOS] Registrando en {log_path} | umbral={umbral_evento:.2e}")
        
    def _init_log(self):
        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w') as f:
                f.write("Evento,Hora de inicio,Hora de fin,"
                        "Duracion [ms],Angulo promedio [grados],"
                        "Confianza promedio,Energia pico,Frames\n")
    
    def update(self, angle_deg, confidence, energy, valid):
        """
        alimentar con cada frame procesado.
        si la energia supera umbral_evento, se acumula como parte de un evento.
        cuando la energia baja, se cierra el evento y se escribe al log.
        
        retorna True en el frame exacto en que comienza un evento nuevo.
        """
        event_started = False
        
        if energy >= self.umbral_evento and valid:
            # estamos en un evento
            if not self.in_event:
                # inicio de evento nuevo
                self.in_event = True
                self.event_start_time = time.time()
                self.event_angles = []
                self.event_confidences = []
                self.event_peak_energy = 0
                event_started = True
            
            self.event_angles.append(angle_deg)
            self.event_confidences.append(confidence)
            self.event_peak_energy = max(self.event_peak_energy, energy)
            
        else:
            # no hay evento o terminó
            if self.in_event:
                self._close_event()
        
        return event_started
    
    def _close_event(self):
        """cierra el evento actual y lo escribe al archivo de log"""
        self.in_event = False
        
        if len(self.event_angles) < 2:
            # evento demasiado corto (1 frame), probablemente espurio
            return
        
        self.event_count += 1
        t_end = time.time()
        duration_ms = (t_end - self.event_start_time) * 1000
        
        angles = np.array(self.event_angles)
        confs = np.array(self.event_confidences)
        
        # escribir al log
        try:
            hora_inicio = datetime.fromtimestamp(self.event_start_time).strftime('%Y-%m-%d %H:%M:%S')
            hora_fin = datetime.fromtimestamp(t_end).strftime('%Y-%m-%d %H:%M:%S')
            
            with open(self.log_path, 'a') as f:
                f.write(f"{self.event_count},"
                        f"{hora_inicio},"
                        f"{hora_fin},"
                        f"{duration_ms:.1f},"
                        f"{np.mean(angles):.1f},"
                        f"{np.mean(confs):.3f},"
                        f"{self.event_peak_energy:.2e},"
                        f"{len(self.event_angles)}\n")
        except Exception as e:
            print(f"\n[EVENTOS] Error escribiendo log: {e}", file=sys.stderr)
            return
            
        # aviso en terminal
        print(f"\n[EVENTO #{self.event_count}] "
              f"θ={np.mean(angles):.1f}°±{np.std(angles):.1f}° | "
              f"dur={duration_ms:.0f}ms | "
              f"E_pico={self.event_peak_energy:.2e} | "
              f"frames={len(self.event_angles)}")
    
    def flush(self):
        """cierra un evento pendiente al terminar el programa"""
        if self.in_event:
            self._close_event()
    
    def get_event_count(self):
        return self.event_count

            
# ------------------------------------------------------------------------------------------------
# BUCLE PRINCIPAL

# parseo de argumentos de línea de comandos para ejecutar el programa con diferentes configuraciones
def parse_args():
    p = argparse.ArgumentParser(
        description="Sistema de DOA acústico con GCC-PHAT para Raspberry Pi y 2 micrófonos MEMS")
    p.add_argument("--simulate", action="store_true",
                   help="Simular el sistema sin hardware")
    p.add_argument("--sim-angle", type=float, default=60.0,
                   help="Ángulo de la fuente simulada en grados (default: 60)")
    p.add_argument("--sim-freq", type=float, default=1000.0,
                   help="Frecuencia de la fuente simulada en Hz (default: 1000)")
    p.add_argument("--sim-snr", type=float, default=20.0,
                   help="SNR de la simulación en dB (default: 20)")
    p.add_argument("--mic-distance", type=float, default=DEFAULT_CONFIG["mic_distance"],
                   help=f"Separación entre micrófonos en metros (default: {DEFAULT_CONFIG['mic_distance']})")
    p.add_argument("--sample-rate", type=int, default=DEFAULT_CONFIG["sample_rate"],
                   help=f"Tasa de muestreo en Hz (default: {DEFAULT_CONFIG['sample_rate']})")
    p.add_argument("--block-size", type=int, default=DEFAULT_CONFIG["block_size"],
                   help=f"Tamaño del bloque FFT (default: {DEFAULT_CONFIG['block_size']})")
    p.add_argument("--acc-blocks", type=int, default=3,
                   help="Bloques a promediar antes de estimar DOA (default: 3). "
                        "Más bloques = mejor SNR con micrófonos muy cercanos, más latencia.")
    p.add_argument("--device", type=str, default=None,
                   help="Dispositivo de audio (índice o nombre). Listar con: python3 -m sounddevice")
    p.add_argument("--umbral-ruido", type=float,
                   default=DEFAULT_CONFIG["umbral_ruido"],
                   help="Umbral de energía para detección de actividad")
    p.add_argument("--umbral-evento", type=float,
                   default=DEFAULT_CONFIG["umbral_evento"],
                   help=f"Umbral de energía para registrar eventos (default: umbral_ruido * {k})")
    p.add_argument("--log-path", type=str, default="eventos_doa.csv",
                   help="Ruta del archivo de log de eventos (default: eventos_doa.csv)")
    
    # modos de servo (mutuamente excluyentes)
    servo_group = p.add_mutually_exclusive_group()
    servo_group.add_argument("--seguimiento", action="store_true",
                   help="Servo sigue el DOA en tiempo real")
    servo_group.add_argument("--evento", action="store_true",
                   help="Servo solo se mueve al detectar un evento y se queda fijo")
    
    # parametros del servo (aplican a ambos modos)
    p.add_argument("--servo-pin", type=int, default=12,
                   help="GPIO pin (BCM) para el servo (default: 12)")
    p.add_argument("--servo-batch", type=int, default=8,
                   help="Estimaciones a acumular antes de mover (default: 8, solo modo seguimiento)")
    p.add_argument("--servo-min-conf", type=float, default=0.2,
                   help="Confianza mínima GCC (default: 0.2, solo modo seguimiento)")
    p.add_argument("--servo-dead-zone", type=float, default=5.0,
                   help="Zona muerta en grados (default: 5.0, solo modo seguimiento)")
    p.add_argument("--servo-max-step", type=float, default=20.0,
                   help="Grados máximos por movimiento (default: 20.0, solo modo seguimiento)")
    p.add_argument("--servo-lock-time", type=float, default=3.0,
                   help="Segundos que el servo se queda fijo en un evento (default: 3.0)")
    p.add_argument("--servo-invert", action="store_true",
                   help="Invertir dirección del servo")
    return p.parse_args()

# muestra un resumen de la configuracion
def print_system_info(args):
    d = args.mic_distance
    c = DEFAULT_CONFIG["speed_of_sound"]
    fs = args.sample_rate
    N = args.block_size

    tau_max_sec = d / c
    tau_max_samples = tau_max_sec * fs
    f_max_spatial = c / (2 * d)
    latency_ms = N / fs * 1000
    angular_res_approx = 180.0 / (2 * tau_max_samples)  # Estimación gruesa
    
    print("=" * 65)
    print("  SISTEMA DOA — GCC-PHAT — 2 Micrófonos INMP441")
    print("=" * 65)
    print(f"  Separación mics:       {d*100:.1f} cm")
    print(f"  Velocidad del sonido:  {c:.0f} m/s")
    print(f"  Tasa de muestreo:      {fs} Hz")
    print(f"  Tamaño de bloque:      {N} muestras ({latency_ms:.1f} ms)")
    print(f"  τ_max:                 {tau_max_sec*1e6:.1f} μs ({tau_max_samples:.1f} muestras)")
    print(f"  f_max espacial:        {f_max_spatial:.0f} Hz (evitar aliasing)")
    print(f"  Resolución angular:    ~{angular_res_approx:.1f}° (sin interpolación)")
    print(f"  Modo:                  {'SIMULACIÓN' if args.simulate else 'CAPTURA REAL'}")
    if args.simulate:
        print(f"  Ángulo simulado:       {args.sim_angle}°")
    if args.seguimiento or args.evento:
        modo_servo = "SEGUIMIENTO" if args.seguimiento else "EVENTO"
        batch_ms = args.servo_batch * (args.block_size / fs * 1000)
        print(f"  Servo:                 GPIO {args.servo_pin} | modo {modo_servo}")
        if args.seguimiento:
            print(f"                         batch={args.servo_batch} (~{batch_ms:.0f}ms) | "
                  f"dead_zone={args.servo_dead_zone}° | max_step={args.servo_max_step}°"
                  f"{' | INV' if args.servo_invert else ''}")
        if args.evento:
            print(f"                         lock={args.servo_lock_time}s"
                  f"{' | INV' if args.servo_invert else ''}")
    print(f"  Registro eventos:      {args.log_path} | umbral={args.umbral_evento:.2e}")
    print("=" * 65)
    print("  Ctrl+C para detener")
    print()
    
def main():
    args = parse_args()
    print_system_info(args)
    
    c = DEFAULT_CONFIG["speed_of_sound"]
    d = args.mic_distance
    fs = args.sample_rate
    #ceil -> redondeo al integro igual o mayor, suma 2 muestras para tener margen
    max_delay_samples = int(np.ceil((d / c) * fs)) + 2
    
    # iniciar simulacion o captura
    if args.simulate:
        capture = SimulatedCapture(
            sample_rate=fs,
            block_size=args.block_size,
            mic_distance=d,
            speed_of_sound=c,
            source_angle_deg=args.sim_angle,
            source_freq=args.sim_freq,
            snr_db=args.sim_snr,
        )
    else:
        capture = AudioCapture(
            sample_rate=fs,
            block_size=args.block_size,
            device=int(args.device) if args.device and args.device.isdigit() else args.device,
        )
        
    display = TerminalDisplay()

    # iniciar servo si se eligió un modo
    servo = None
    servo_mode = None
    if args.seguimiento or args.evento:
        servo_mode = "seguimiento" if args.seguimiento else "evento"
        servo = ServoController(
            gpio_pin=args.servo_pin,
            batch_size=args.servo_batch,
            min_confidence=args.servo_min_conf,
            dead_zone=args.servo_dead_zone,
            max_step=args.servo_max_step,
            lock_duration=args.servo_lock_time,
            invert=args.servo_invert,
            simulate=args.simulate,
        )
    
    # registro de eventos (siempre activo)
    event_logger = EventLogger(
        umbral_evento=args.umbral_evento,
        log_path=args.log_path,
    )
        
    # ctrl+C para salida limpia
    running = [True]

    def signal_handler(sig, frame):
        running[0] = False
        print("\n\n[INFO] Deteniendo...")
        
    signal.signal(signal.SIGINT, signal_handler)

    capture.start()
    print("[INFO] Iniciando procesamiento de audio...\n")

    frame_count = 0
    t_start = time.time()
    
    # acumulador de cross-spectrums PHAT para promediar antes de resolver
    acc_blocks = args.acc_blocks
    phat_acc = None
    acc_count = 0
    acc_energy = 0.0

    try:
        while running[0]:
            # 1. capturar bloque
            ch_left, ch_right = capture.read_block()
            
            # 2. deteccion de actividad
            energy = np.mean(ch_left**2 + ch_right**2) / 2
            is_silent = energy < args.umbral_ruido
            
            if is_silent:
                display.show(0, 0, True, energy, is_silent=True)
                display.clear_history()
                if servo is not None and servo_mode == "seguimiento":
                    servo.notify_silence()
                event_logger.update(0, 0, energy, False)
                # resetear acumulador en silencio
                phat_acc = None
                acc_count = 0
                acc_energy = 0.0
                continue
            
            # 3. calcular PHAT y acumular
            phat = gcc_phat(ch_left, ch_right, max_delay_samples)
            if phat_acc is None:
                phat_acc = phat.copy()
            else:
                phat_acc += phat
            acc_count += 1
            acc_energy += energy
            
            # solo resolver cuando se acumularon suficientes bloques
            if acc_count < acc_blocks:
                frame_count += 1
                if args.simulate:
                    time.sleep(args.block_size / fs)
                continue
            
            # 4. resolver TDOA con el promedio de los cross-spectrums
            tau, confidence = gcc_phat_resolve(phat_acc / acc_blocks,
                                               args.block_size, max_delay_samples)
            energy_avg = acc_energy / acc_blocks
            
            # resetear acumulador
            phat_acc = None
            acc_count = 0
            acc_energy = 0.0
            
            # 5. convertir TDOA a DOA
            angle_deg, valid = tdoa_to_angle(tau, fs, d, c)
            
            # 6. registrar evento
            event_started = event_logger.update(angle_deg, confidence, energy_avg, valid)
            
            # 7. controlar servo segun el modo
            if servo is not None and valid:
                if servo_mode == "seguimiento":
                    if event_started:
                        servo.lock_to(angle_deg)
                    servo.update(angle_deg, confidence)
                elif servo_mode == "evento":
                    if event_started:
                        servo.lock_to(angle_deg)
                
            # 8. mostrar en terminal
            buf_fill = servo.get_buffer_fill() if servo and servo_mode == "seguimiento" else None
            display.show(angle_deg, confidence, valid, energy_avg, is_silent=False,
                            servo_angle=servo.get_servo_angle() if servo else None,
                            buf_fill=buf_fill)

            frame_count += 1
            
            if args.simulate:
                time.sleep(args.block_size / fs)
            
    except Exception as e:
        print(f"\n[ERROR] {e}", file=sys.stderr)
        raise
    finally:
        capture.stop()
        event_logger.flush()
        if servo is not None:
            servo.stop()
        elapsed = time.time() - t_start
        if frame_count > 0:
            fps = frame_count / elapsed
            print(f"\n[INFO] {frame_count} bloques en {elapsed:.1f}s ({fps:.1f} bloques/s)")
        print(f"[INFO] {event_logger.get_event_count()} eventos en {args.log_path}")

if __name__ == "__main__":
    main()
