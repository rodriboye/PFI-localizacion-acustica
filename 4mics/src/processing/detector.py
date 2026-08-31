"""
detector.py — Gate 1 de la cascada: detector de eventos por energía.

Máquina de estados: IDLE -> ONSET (confirmando min_frames) -> ACTIVE ->
COOLDOWN -> IDLE.

Umbral FIJO, no adaptativo. El piso se mide una vez (calibración de arranque o
valor a mano) y queda constante:
    umbral_evento   = piso * (1 + k)
    umbral_silencio = piso * (1 + k * silence_ratio)
El adaptativo "dejaba de escuchar" en seguimiento porque el piso trepaba hacia
la fuente sostenida. Costo del fijo: RECALIBRAR al cambiar de escenario.
"""

import time
import numpy as np
from enum import Enum, auto


class State(Enum):
    IDLE     = auto()
    ONSET    = auto()
    ACTIVE   = auto()
    COOLDOWN = auto()


class BandEnergy:
    """Energía media por muestra dentro de [f_lo, f_hi), en un paso de FFT.

    `np.mean(frame**2)` no alcanza porque es de BANDA COMPLETA: en este array la
    energía por debajo de 200 Hz supera ~16x a la de la banda de trabajo, así que
    con el piso FIJO el umbral quedaba determinado por infrasonido que el motor
    DOA ni siquiera mira. Se usa FFT y no un filtro porque es la opción más
    barata que da la banda exacta (~34 us/frame contra 9.5 de la banda completa
    y 70 de un FIR de 32 taps).

    La normalización es Parseval con corrección de la potencia de la ventana: la
    escala se cancela en el detector, pero mantenerla física hace comparables los
    números impresos y el override DETECTOR_NOISE_FLOOR.
    """

    __slots__ = ('win', 'w', 'num_ch', 'n_bins')

    def __init__(self, hop_size, num_ch, sample_rate, f_lo, f_hi):
        # Hann PERIÓDICA (coherente con doa_engine.py)
        self.win = np.hanning(hop_size + 1)[:-1].astype(np.float64)[:, np.newaxis]

        freqs = np.fft.rfftfreq(hop_size, d=1.0 / sample_rate)
        mask  = (freqs >= f_lo) & (freqs < f_hi)
        if not mask.any():
            raise ValueError(
                f"No hay bins en [{f_lo}, {f_hi}) Hz con hop={hop_size} y "
                f"fs={sample_rate} (df={sample_rate/hop_size:.1f} Hz)")

        w = np.zeros(freqs.shape, dtype=np.float64)
        w[mask] = 2.0                      # espectro de un solo lado
        if mask[0]:
            w[0] = 1.0                     # DC no se duplica
        if hop_size % 2 == 0 and mask[-1]:
            w[-1] = 1.0                    # Nyquist tampoco
        w /= (hop_size ** 2) * float((self.win[:, 0] ** 2).mean())

        self.w      = w
        self.num_ch = num_ch
        self.n_bins = int(mask.sum())

    def __call__(self, frame):
        """frame: (hop_size, num_ch) -> energía media por muestra."""
        X = np.fft.rfft(frame * self.win, axis=0)
        # Los canales se suman ANTES del producto con los pesos (w es común a
        # todos), así el producto escalar se hace una sola vez.
        return float(self.w @ (X.real ** 2 + X.imag ** 2).sum(axis=1)) / self.num_ch


class EventDetector:

    # Defaults espejo de config.py; main.py siempre pasa los suyos.
    def __init__(self, k=1.5, min_frames=8, cooldown_frames=3, silence_ratio=0.75,
                 calib_frames=86, noise_floor=None, calib_percentile=20.0,
                 band=None):
        """band: None = energía de banda completa (LEGADO, mide infrasonido);
        (hop_size, num_ch, sample_rate, f_lo, f_hi) = energía en [f_lo, f_hi),
        que debe ser la MISMA banda que enmascara el motor DOA.

        Cambiar `band` invalida cualquier DETECTOR_NOISE_FLOOR fijado a mano: el
        override está en las unidades de la medida elegida.
        """
        self.k               = k
        self.silence_ratio   = silence_ratio
        self.min_frames      = min_frames
        self.cooldown_frames = cooldown_frames
        self.band_energy     = BandEnergy(*band) if band is not None else None

        self.state         = State.IDLE
        self.onset_count   = 0
        self.cooldown_left = 0

        # Calibración por PERCENTIL, no media: la media no es robusta y un solo
        # transitorio la arrastra hacia arriba, dejando el umbral inalcanzable el
        # resto de la corrida sin ningún síntoma. calib_percentile=None restaura
        # la media.
        self.calib_percentile = calib_percentile
        self._calib_count  = 0
        self._calib_accum  = 0.0
        self._calib_energies = []
        if noise_floor is not None:                 # piso fijado a mano
            self.noise_floor   = max(float(noise_floor), 1e-12)
            self._calib_target = 0
            self.calibrating   = False
        else:                                       # piso por calibración
            self.noise_floor   = 1e-6               # placeholder
            self._calib_target = max(int(calib_frames), 0)
            self.calibrating   = self._calib_target > 0

    def recalibrate(self):
        """Vuelve a medir el piso. Asume silencio ambiente durante los próximos
        calib_frames."""
        if self._calib_target > 0:
            self._calib_count = 0
            self._calib_accum = 0.0
            self._calib_energies = []
            self.calibrating  = True
            self.state        = State.IDLE

    @property
    def threshold_event(self):
        """Energía mínima para iniciar un evento."""
        return self.noise_floor * (1.0 + self.k)

    @property
    def threshold_silence(self):
        """Energía por debajo de la cual el evento termina."""
        return self.noise_floor * (1.0 + self.k * self.silence_ratio)

    def energy(self, frame):
        """Medida de energía del detector. Publica a propósito: main.py y
        measure_noise_floor.py deben usar esta misma función, o el piso medido y
        el umbral aplicado quedan en unidades distintas sin aviso."""
        if self.band_energy is not None:
            return self.band_energy(frame)
        return float(np.mean(frame ** 2))

    def update(self, frame, energy=None):
        """Avanza la máquina de estados. Devuelve 'event' (recién confirmado),
        'active', 'onset' (flanco sin confirmar) o 'idle' (incluye cooldown y
        calibración).

        `energy` evita repetir la FFT cuando el llamador ya midió con energy();
        tiene que venir de ESE método o el umbral queda en otras unidades."""
        if energy is None:
            energy = self.energy(frame)

        # Calibración: no se detecta nada hasta tener referencia.
        if self.calibrating:
            self._calib_count += 1
            if self.calib_percentile is None:
                self._calib_accum += energy
            else:
                self._calib_energies.append(energy)
            if self._calib_count >= self._calib_target:
                if self.calib_percentile is None:
                    est = self._calib_accum / self._calib_count
                else:
                    est = float(np.percentile(self._calib_energies,
                                              self.calib_percentile))
                self.noise_floor = max(est, 1e-12)
                self.calibrating = False
                self._calib_energies = []
            return 'idle'

        if self.state == State.IDLE:
            if energy > self.threshold_event:
                self.state = State.ONSET
                self.onset_count = 1
            return 'idle'

        elif self.state == State.ONSET:
            self.onset_count += 1
            if energy < self.threshold_silence:
                self.state = State.IDLE      # falso onset
                return 'idle'
            if self.onset_count >= self.min_frames:
                self.state = State.ACTIVE
                return 'event'
            # 'onset' y no 'idle' para que main.py NO resetee el motor acá: el
            # modo onset de SRP necesita ver estos frames, antes de los ecos.
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
