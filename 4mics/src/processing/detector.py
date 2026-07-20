"""
detector.py — Detector de eventos acústicos por energía.

Máquina de estados:
    IDLE      → espera que la energía supere umbral_ruido
    ONSET     → confirma el evento durante EVENT_MIN_FRAMES frames
    ACTIVE    → evento confirmado, acumula estimaciones DOA
    COOLDOWN  → espera antes de poder detectar el próximo evento

Umbral FIJO (no adaptativo):
    El ruido de piso se mide UNA vez y queda constante toda la corrida:

    - CALIBRACIÓN (arranque): los primeros `calib_frames` se promedian para fijar
      el piso. Se asume SILENCIO AMBIENTE y no se detecta nada en esa fase.
      (Alternativa: fijar el piso a mano con noise_floor y saltear la calibración.)

    - OPERACIÓN: el piso NO se adapta. Los umbrales son constantes. Si el
      escenario cambia (otro ambiente de ruido), hay que RECALIBRAR: reiniciar o
      llamar detector.recalibrate().

    Se eligió fijo sobre adaptativo a propósito: el adaptativo "dejaba de
    escuchar" en seguimiento (el piso trepaba hacia la fuente sostenida o su cola
    reverberante). El fijo es predecible y nunca pierde la señal por deriva del
    piso, a costa de recalibrar por escenario.

    umbral_evento   = ruido_piso * (1 + k)
    umbral_silencio = ruido_piso * (1 + k * silence_ratio)

    Parámetros (k, silence_ratio, calib_frames, noise_floor) desde config.py.
"""

import time
import numpy as np
from enum import Enum, auto


class State(Enum):
    IDLE     = auto()
    ONSET    = auto()
    ACTIVE   = auto()
    COOLDOWN = auto()


class EventDetector:

    # Defaults espejo de config.py (main.py siempre pasa los de config; estos
    # aplican solo si se instancia el detector a mano, p.ej. en tests).
    # k es O(1) desde el rediseño del piso fijo: ~1.0 sensible (fuente
    # débil/lejana, más falsos), 2-3 solo eventos fuertes.
    def __init__(self, k=1.5, min_frames=8, cooldown_frames=3, silence_ratio=0.75,
                 calib_frames=86, noise_floor=None):
        self.k               = k
        self.silence_ratio   = silence_ratio
        self.min_frames      = min_frames
        self.cooldown_frames = cooldown_frames

        self.state         = State.IDLE
        self.onset_count   = 0
        self.cooldown_left = 0

        # --- Piso de ruido FIJO ---
        # El piso se mide UNA vez (calibración de arranque, o se fija a mano con
        # noise_floor) y NO se adapta después. Los umbrales quedan constantes:
        #   threshold_event   = noise_floor * (1 + k)
        #   threshold_silence = noise_floor * (1 + k*silence_ratio)
        # Ventaja sobre la adaptación: NUNCA "deja de escuchar" por trepada del
        # piso (la fuente sostenida o su cola reverberante no lo mueven). Costo:
        # hay que RECALIBRAR al cambiar de escenario (reiniciar o recalibrate()).
        self._calib_count  = 0
        self._calib_accum  = 0.0
        if noise_floor is not None:                 # piso fijado a mano
            self.noise_floor   = max(float(noise_floor), 1e-12)
            self._calib_target = 0
            self.calibrating   = False
        else:                                       # piso por calibración
            self.noise_floor   = 1e-6               # placeholder hasta calibrar
            self._calib_target = max(int(calib_frames), 0)
            self.calibrating   = self._calib_target > 0

    def recalibrate(self):
        """Vuelve a medir el piso de ruido (p.ej. al cambiar de escenario).
        Asumí silencio ambiente durante los próximos calib_frames."""
        if self._calib_target > 0:
            self._calib_count = 0
            self._calib_accum = 0.0
            self.calibrating  = True
            self.state        = State.IDLE

    @property
    def threshold_event(self):
        """umbral_ruido: energía mínima para detectar inicio de evento."""
        return self.noise_floor * (1.0 + self.k)

    @property
    def threshold_silence(self):
        """umbral_silencio: energía por debajo de la cual el evento termina."""
        return self.noise_floor * (1.0 + self.k * self.silence_ratio)

    def update(self, frame):
        """
        Actualiza la máquina de estados con el frame actual.

        Args:
            frame: ndarray (hop_size, M) — audio del frame actual.

        Returns:
            'event'   si se acaba de confirmar un evento
            'active'  si el evento sigue activo
            'idle'    en cualquier otro caso
        """
        energy = float(np.mean(frame ** 2))

        # --- Fase de calibración: estimar el piso con los primeros frames ---
        # No se detecta nada hasta tener una referencia de silencio fiable.
        if self.calibrating:
            self._calib_accum += energy
            self._calib_count += 1
            if self._calib_count >= self._calib_target:
                self.noise_floor = max(self._calib_accum / self._calib_count, 1e-12)
                self.calibrating = False
            return 'idle'

        if self.state == State.IDLE:
            # Piso FIJO: no se adapta. Solo se chequea el cruce del umbral de
            # evento. Si el escenario cambió y el piso quedó mal, recalibrar.
            if energy > self.threshold_event:
                self.state = State.ONSET
                self.onset_count = 1
            return 'idle'

        elif self.state == State.ONSET:
            self.onset_count += 1
            if energy < self.threshold_silence:
                # Falso onset, volver a IDLE
                self.state = State.IDLE
                return 'idle'
            if self.onset_count >= self.min_frames:
                self.state = State.ACTIVE
                return 'event'
            # Frames del flanco de subida (camino directo del aplauso). Se
            # devuelve 'onset' —no 'idle'— para que main.py NO resetee el motor
            # aca: el modo onset de SRP necesita ver estos frames (los mas
            # limpios, antes de los ecos) antes de disparar el evento.
            return 'onset'

        elif self.state == State.ACTIVE:
            if energy < self.threshold_silence:
                self.state = State.COOLDOWN
                self.cooldown_left = self.cooldown_frames
                return 'idle'
            return 'active'

        elif self.state == State.COOLDOWN:
            self.cooldown_left -= 1
            if self.cooldown_left <= 0:
                self.state = State.IDLE
            return 'idle'

        return 'idle'
