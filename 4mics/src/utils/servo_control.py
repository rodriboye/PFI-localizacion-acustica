"""
servo_control.py — Control de servomotores con anti-temblor.

Anti-temblor implementado con 6 mecanismos:
    1. Zona muerta: si el error es pequeño, no mover.
    2. Paso máximo: limitar la velocidad de giro por frame.
    3. Batching: promediar N estimaciones DOA antes de mover.
    4. Confianza mínima: ignorar estimaciones poco confiables.
    5. Detach PWM: apagar la señal PWM después de posicionar
       para eliminar el jitter eléctrico del SG90.
    6. Retorno al centro: si hay silencio prolongado, volver
       gradualmente a la posición central.

Requiere: pigpio (daemon corriendo: sudo pigpiod)
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
        # Centro mecánico calculado como punto medio del rango usable.
        # Es la posición de reposo cuando no hay actividad detectada.
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
        self._detach_timer = None

        if not PIGPIO_AVAILABLE:
            print("[servo] ADVERTENCIA: pigpio no disponible. Servos desactivados.")
            return

        self._pi = pigpio.pi()
        if not self._pi.connected:
            print("[servo] ADVERTENCIA: pigpiod no está corriendo. Servos desactivados.")
            self._pi = None
            return

        self._set(self._az_center, self._el_center)

    def update(self, doa_result):
        """Recibe una estimación DOA y decide si mover los servos."""
        if self._pi is None:
            return
        if not doa_result or not doa_result.valid:
            return
        if doa_result.confidence < self.cfg.SERVO_MIN_CONFIDENCE:
            return

        self._last_event_t = time.time()

        # Mapeo lineal del rango DOA al rango mecánico USABLE de cada servo.
        # Comprime/expande el ángulo según la mecánica real del servo.
        az_mec = self._map_doa_to_mech(
            doa_result.azimuth,
            self.cfg.AZIMUTH_MIN, self.cfg.AZIMUTH_MAX,
            self.cfg.SERVO_AZ_USABLE_MIN, self.cfg.SERVO_AZ_USABLE_MAX,
            getattr(self.cfg, 'SERVO_AZ_INVERT', False),
        )
        el_mec = self._map_doa_to_mech(
            doa_result.elevation,
            self.cfg.ELEVATION_MIN, self.cfg.ELEVATION_MAX,
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
        Movimiento único e inmediato a la posición objetivo (modo evento).

        A diferencia de update(), bypassa los mecanismos anti-temblor:
          - Sin batching (SERVO_BATCH): cada llamada dispara el movimiento.
          - Sin zona muerta: el servo va al ángulo indicado aunque la
            diferencia sea pequeña.
          - Sin paso máximo: se escribe el pulso PWM final directamente; la
            velocidad real del giro la marca el hardware del SG90 (~600°/s
            sin carga).

        El detach asincrónico del PWM (SERVO_DETACH_DELAY) se mantiene, así
        el servo no zumba mientras espera el próximo evento.

        Pensado para el modo evento: la decisión de cuándo y sobre cuántas
        muestras promediar se toma en el loop principal (main.py) antes de
        invocar este método, así el controlador se mantiene tonto y
        predecible.

        force=True omite el gate de SERVO_MIN_CONFIDENCE. Se usa para el snap
        de evento, donde el filtro de confianza ya lo aplicó main.py con su
        propio umbral (EVENT_MIN_CONFIDENCE, más permisivo): el detector ya
        confirmó energía real, así que vale apuntar aunque el pico MUSIC sea
        moderado, en vez de perder el evento.
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
            self.cfg.ELEVATION_MIN, self.cfg.ELEVATION_MAX,
            self.cfg.SERVO_EL_USABLE_MIN, self.cfg.SERVO_EL_USABLE_MAX,
            getattr(self.cfg, 'SERVO_EL_INVERT', False),
        )

        with self._lock:
            # Limpiar cualquier buffer de continuous mode que haya quedado
            # (defensivo: no debería haber, pero asegura consistencia si se
            # alterna entre modos durante la ejecución).
            self._buf_az.clear()
            self._buf_el.clear()
            self._buf_conf.clear()
            self._set(az_mec, el_mec)

    def tick(self):
        """Llamar periódicamente para el retorno al centro en silencio."""
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
        """Mapeo lineal del rango DOA al rango mecánico usable.

        Interpola: doa_min -> mech_min, doa_max -> mech_max.
        Si invert=True, se invierte: doa_min -> mech_max, doa_max -> mech_min.
        El resultado queda saturado dentro de [mech_min, mech_max] aun si
        el DOA queda fuera de [doa_min, doa_max].
        """
        if doa_max == doa_min:
            return 0.5 * (mech_min + mech_max)
        t = (doa_value - doa_min) / (doa_max - doa_min)
        t = max(0.0, min(1.0, t))
        if invert:
            return mech_max - t * (mech_max - mech_min)
        return mech_min + t * (mech_max - mech_min)

    def _move_towards(self, target_az, target_el, step_fraction=1.0):
        """Mueve los servos hacia el target con zona muerta y paso máximo."""
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
        """Envía pulsos PWM y agenda detach en hilo aparte (no bloquea)."""
        pulse_az = _deg_to_pulse(az_deg, self.cfg.SERVO_AZ_MIN,
                                 self.cfg.SERVO_AZ_MAX)
        pulse_el = _deg_to_pulse(el_deg, self.cfg.SERVO_EL_MIN,
                                 self.cfg.SERVO_EL_MAX)

        self._pi.set_servo_pulsewidth(self.cfg.SERVO_AZ_PIN, pulse_az)
        self._pi.set_servo_pulsewidth(self.cfg.SERVO_EL_PIN, pulse_el)

        self._az_pos = az_deg
        self._el_pos = el_deg

        if self.cfg.SERVO_DETACH_DELAY > 0:
            # Detach asincrónico: no bloquear el loop de adquisición/DOA.
            # Cancelar timer previo si todavía está pendiente, para que
            # movimientos consecutivos no apaguen el PWM en medio del giro.
            if getattr(self, "_detach_timer", None) is not None:
                self._detach_timer.cancel()
            self._detach_timer = threading.Timer(
                self.cfg.SERVO_DETACH_DELAY, self._detach_pwm
            )
            self._detach_timer.daemon = True
            self._detach_timer.start()

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
        if getattr(self, "_detach_timer", None) is not None:
            self._detach_timer.cancel()
        if self._pi:
            self._pi.set_servo_pulsewidth(self.cfg.SERVO_AZ_PIN, 0)
            self._pi.set_servo_pulsewidth(self.cfg.SERVO_EL_PIN, 0)
            self._pi.stop()
