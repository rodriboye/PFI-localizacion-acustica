"""
servo_control.py — Control de los dos servos con anti-temblor: zona muerta, paso
máximo por actualización, batching de estimaciones, confianza mínima, detach del
PWM tras posicionar y retorno gradual al centro tras silencio.

Requiere pigpio con el daemon corriendo (sudo pigpiod). Sin él el controlador
queda inerte y el resto del pipeline sigue funcionando.
"""

import time
import threading
import numpy as np

try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    PIGPIO_AVAILABLE = False


def _deg_to_pulse(deg, deg_min, deg_max, pulse_min=500, pulse_max=2500):
    """Convierte grados mecánicos a microsegundos de pulso PWM."""
    t = (deg - deg_min) / (deg_max - deg_min)
    return int(pulse_min + t * (pulse_max - pulse_min))


class ServoController:

    def __init__(self, config):
        self.cfg      = config
        # Punto medio del rango usable: posición de reposo sin actividad.
        self._az_center = (config.SERVO_AZ_USABLE_MIN + config.SERVO_AZ_USABLE_MAX) / 2.0
        self._el_center = (config.SERVO_EL_USABLE_MIN + config.SERVO_EL_USABLE_MAX) / 2.0
        self._az_pos  = self._az_center
        self._el_pos  = self._el_center
        self._pi      = None
        self._buf_az  = []
        self._buf_el  = []
        self._buf_conf = []
        self._last_event_t = time.time()
        self._lock    = threading.Lock()

        # Objetivo compartido con el hilo escritor; el contador de secuencia
        # evita comparar floats por igualdad para detectar cambios.
        self._target_az  = self._az_center
        self._target_el  = self._el_center
        self._target_seq = 0
        # LOCK DEDICADO, distinto de self._lock: update(), point_to() y tick()
        # ya toman self._lock y llaman a _set() desde adentro, y threading.Lock
        # NO es reentrante — reusarlo acá deadlockea al primer comando.
        self._target_lock = threading.Lock()
        self._writer      = None
        self._writer_stop = threading.Event()

        if not PIGPIO_AVAILABLE:
            print("[servo] ADVERTENCIA: pigpio no disponible. Servos desactivados.")
            return

        self._pi = pigpio.pi()
        if not self._pi.connected:
            print("[servo] ADVERTENCIA: pigpiod no está corriendo. Servos desactivados.")
            self._pi = None
            return

        # El escritor arranca ANTES del primer _set para que el centrado inicial
        # salga por el mismo camino que todo lo demás.
        self._writer = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer.start()
        self._set(self._az_center, self._el_center)

    def update(self, doa_result):
        """Seguimiento continuo: acumula y mueve al completar el batch, con
        zona muerta y paso máximo."""
        if self._pi is None:
            return
        if not doa_result or not doa_result.valid:
            return
        if doa_result.confidence < self.cfg.SERVO_MIN_CONFIDENCE:
            return

        self._last_event_t = time.time()

        az_mec = self._map_doa_to_mech(
            doa_result.azimuth,
            self.cfg.AZIMUTH_MIN, self.cfg.AZIMUTH_MAX,
            self.cfg.SERVO_AZ_USABLE_MIN, self.cfg.SERVO_AZ_USABLE_MAX,
            getattr(self.cfg, 'SERVO_AZ_INVERT', False),
        )
        el_mec = self._map_doa_to_mech(
            doa_result.elevation,
            getattr(self.cfg, 'SERVO_EL_DOA_MIN', self.cfg.ELEVATION_MIN),
            getattr(self.cfg, 'SERVO_EL_DOA_MAX', self.cfg.ELEVATION_MAX),
            self.cfg.SERVO_EL_USABLE_MIN, self.cfg.SERVO_EL_USABLE_MAX,
            getattr(self.cfg, 'SERVO_EL_INVERT', False),
        )

        with self._lock:
            self._buf_az.append(az_mec)
            self._buf_el.append(el_mec)
            self._buf_conf.append(doa_result.confidence)

            if len(self._buf_az) >= self.cfg.SERVO_BATCH:
                target_az = float(np.mean(self._buf_az))
                target_el = float(np.mean(self._buf_el))
                self._buf_az.clear()
                self._buf_el.clear()
                self._buf_conf.clear()
                self._move_towards(target_az, target_el)

    def point_to(self, doa_result, force=False):
        """
        Movimiento único e inmediato al ángulo indicado (snap de evento).

        Bypassa el anti-temblor: sin batching, sin zona muerta y sin paso máximo.
        El detach del PWM se mantiene. Cuándo apuntar lo decide main.py; el
        controlador se mantiene tonto.

        force=True omite SERVO_MIN_CONFIDENCE porque main.py ya aplicó el suyo
        (EVENT_MIN_CONFIDENCE, más permisivo): con el detector confirmando
        energía real vale apuntar aunque el pico sea moderado.
        """
        if self._pi is None:
            return
        if not doa_result or not doa_result.valid:
            return
        if not force and doa_result.confidence < self.cfg.SERVO_MIN_CONFIDENCE:
            return

        self._last_event_t = time.time()

        az_mec = self._map_doa_to_mech(
            doa_result.azimuth,
            self.cfg.AZIMUTH_MIN, self.cfg.AZIMUTH_MAX,
            self.cfg.SERVO_AZ_USABLE_MIN, self.cfg.SERVO_AZ_USABLE_MAX,
            getattr(self.cfg, 'SERVO_AZ_INVERT', False),
        )
        el_mec = self._map_doa_to_mech(
            doa_result.elevation,
            getattr(self.cfg, 'SERVO_EL_DOA_MIN', self.cfg.ELEVATION_MIN),
            getattr(self.cfg, 'SERVO_EL_DOA_MAX', self.cfg.ELEVATION_MAX),
            self.cfg.SERVO_EL_USABLE_MIN, self.cfg.SERVO_EL_USABLE_MAX,
            getattr(self.cfg, 'SERVO_EL_INVERT', False),
        )

        with self._lock:
            # Descartar el batch pendiente: no debe aplicarse tras el snap.
            self._buf_az.clear()
            self._buf_el.clear()
            self._buf_conf.clear()
            self._set(az_mec, el_mec)

    def tick(self):
        """Llamar por frame: devuelve el servo al centro tras silencio."""
        if self._pi is None:
            return
        if self.cfg.SERVO_SILENCE_RETURN <= 0:
            return
        elapsed = time.time() - self._last_event_t
        if elapsed > self.cfg.SERVO_SILENCE_RETURN:
            with self._lock:
                self._move_towards(self._az_center,
                                   self._el_center,
                                   step_fraction=0.1)

    @staticmethod
    def _map_doa_to_mech(doa_value, doa_min, doa_max,
                         mech_min, mech_max, invert):
        """Mapeo lineal doa_min → mech_min, doa_max → mech_max (invertido si
        invert=True), saturado a [mech_min, mech_max]."""
        if doa_max == doa_min:
            return 0.5 * (mech_min + mech_max)
        t = (doa_value - doa_min) / (doa_max - doa_min)
        t = max(0.0, min(1.0, t))
        if invert:
            return mech_max - t * (mech_max - mech_min)
        return mech_min + t * (mech_max - mech_min)

    def _move_towards(self, target_az, target_el, step_fraction=1.0):
        """Avanza hacia el target aplicando zona muerta y paso máximo."""
        err_az = target_az - self._az_pos
        err_el = target_el - self._el_pos

        if abs(err_az) < self.cfg.SERVO_DEAD_ZONE:
            err_az = 0
        if abs(err_el) < self.cfg.SERVO_DEAD_ZONE:
            err_el = 0

        max_step = self.cfg.SERVO_MAX_STEP * step_fraction
        step_az = np.clip(err_az, -max_step, max_step)
        step_el = np.clip(err_el, -max_step, max_step)

        new_az = self._az_pos + step_az
        new_el = self._el_pos + step_el

        if abs(step_az) > 0 or abs(step_el) > 0:
            self._set(new_az, new_el)

    def _set(self, az_deg, el_deg):
        """
        Fija la posición OBJETIVO. No toca pigpio: solo escribe tres variables.

        El lazo de DOA no puede hacer I/O acá — pigpio habla con su daemon por
        socket, dos IPC por frame. La escritura física la hace _writer_loop a
        SERVO_WRITE_HZ, y diferirla no pierde nada: el SG90 tarda 100-200 ms en
        moverse 60°, mucho más que los 23 ms del frame.

        _az_pos/_el_pos son la posición COMANDADA, no medida; se actualizan acá
        porque update() y tick() calculan el error contra ellas.
        """
        with self._target_lock:
            self._target_az  = az_deg
            self._target_el  = el_deg
            self._target_seq += 1
        self._az_pos = az_deg
        self._el_pos = el_deg

    def _writer_loop(self):
        """
        Único punto que habla con pigpio, a SERVO_WRITE_HZ. Escribe SOLO cuando
        el objetivo cambió: reescribir sin cambios impediría que el detach llegue
        a actuar. El detach usa una marca de tiempo y no threading.Timer, que
        creaba un hilo por llamada.
        """
        period    = 1.0 / max(1.0, float(getattr(self.cfg, 'SERVO_WRITE_HZ', 30)))
        detach_s  = float(getattr(self.cfg, 'SERVO_DETACH_DELAY', 0.0))
        last_seq  = -1
        last_write = 0.0
        attached  = False

        while not self._writer_stop.is_set():
            with self._target_lock:
                seq, az, el = self._target_seq, self._target_az, self._target_el
            now = time.time()

            if seq != last_seq:
                pulse_az = _deg_to_pulse(az, self.cfg.SERVO_AZ_MIN,
                                         self.cfg.SERVO_AZ_MAX)
                pulse_el = _deg_to_pulse(el, self.cfg.SERVO_EL_MIN,
                                         self.cfg.SERVO_EL_MAX)
                try:
                    self._pi.set_servo_pulsewidth(self.cfg.SERVO_AZ_PIN, pulse_az)
                    self._pi.set_servo_pulsewidth(self.cfg.SERVO_EL_PIN, pulse_el)
                except Exception:
                    pass          # el lazo de DOA nunca debe caerse por el servo
                last_seq   = seq
                last_write = now
                attached   = True

            elif attached and detach_s > 0 and (now - last_write) >= detach_s:
                self._detach_pwm()
                attached = False

            self._writer_stop.wait(period)

    def _detach_pwm(self):
        """Apaga el PWM para eliminar el jitter del SG90 cuando está quieto."""
        if self._pi is None:
            return
        try:
            self._pi.set_servo_pulsewidth(self.cfg.SERVO_AZ_PIN, 0)
            self._pi.set_servo_pulsewidth(self.cfg.SERVO_EL_PIN, 0)
        except Exception:
            pass

    @property
    def position(self):
        return self._az_pos, self._el_pos

    def close(self):
        # Parar el escritor ANTES de apagar el PWM: si no, podría reescribir un
        # pulso después del apagado y dejar el servo energizado.
        self._writer_stop.set()
        if getattr(self, "_writer", None) is not None:
            self._writer.join(timeout=1.0)
        if self._pi:
            self._pi.set_servo_pulsewidth(self.cfg.SERVO_AZ_PIN, 0)
            self._pi.set_servo_pulsewidth(self.cfg.SERVO_EL_PIN, 0)
            self._pi.stop()
