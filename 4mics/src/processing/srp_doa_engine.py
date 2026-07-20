"""
srp_doa_engine.py — SRP-PHAT broadband para array 3D arbitrario.

SRP-PHAT (Steered Response Power con Phase Transform) es el metodo de DOA mas
robusto a reverberacion y SNR moderado, y el motivo por el que se eligio sobre
MUSIC para este array de 4 mics / 5 cm:

  - MUSIC es un metodo de SUBESPACIO: asume un subespacio de senal bien definido.
    Con senal angosta de banda + reverberacion + coherencia moderada (~0.6) el
    subespacio se corrompe y MUSIC produce picos espurios de alta confianza en
    direcciones equivocadas. Ademas su pico es ancho en arrays chicos.

  - SRP-PHAT NO se compromete con ningun pico. Para cada direccion candidata
    suma el valor de la GCC-PHAT de los 6 pares en el retardo teorico de esa
    direccion, e integra. Un pico espurio en un par queda en MINORIA frente al
    consenso de los otros cinco, dando una estimacion estable bajo reverberacion.

Algoritmo (por frame):
  1. FFT de los M canales (ventana Hann, zero-pad a potencia de 2 >= 2N para
     correlacion lineal, no circular).
  2. Para cada par (i,j): R = X_i * conj(X_j); se anulan los bins fuera de
     [freq_min, freq_max] y se normaliza |R|=1 (PHAT) en la banda. La IFFT da
     la GCC-PHAT (cross-correlacion blanqueada) en funcion del retardo.
  3. Para cada punto de la grilla se interpola la GCC-PHAT de cada par en su
     retardo teorico tau_ij(az,el) y se suman los 6 -> mapa SRP.
  4. El maximo del mapa es la DOA; se refina con interpolacion parabolica
     sub-grilla en espacio-u.

Notas de implementacion:
  - La grilla escanea en espacio-u (u=sin(az), v=sin(el)), igual que el motor
    MUSIC, para distribuir los puntos donde el array tiene resolucion real y
    acotar el costo en la RPi.
  - tau_ij(az,el) se pre-calcula en el constructor para toda la grilla y todos
    los pares; el loop por frame queda en ~6 FFTs + 6 interpolaciones de grilla.
  - Confianza: relacion pico/piso del mapa SRP en dB, con el mapa desplazado a
    valores no-negativos (la GCC puede ser negativa). Corre en una escala MAS
    BAJA que MUSIC (pico ancho de array chico); por eso main.py usa umbrales de
    confianza propios para SRP (ver config: SRP_*).
"""

import numpy as np
from src.processing.doa_engine import DOAResult


class SRPDoaEngine:
    """SRP-PHAT con escaneo 2D (azimut x elevacion) en espacio-u."""

    def __init__(self, mic_positions, sample_rate, frame_size,
                 freq_min, freq_max, speed_of_sound=343.0,
                 angle_resolution=5, az_range=(-65, 65), el_range=(0, 80),
                 mode='onset', accum_alpha=0.6):
        self.mic_pos = np.array(mic_positions, dtype=np.float64)
        self.M = self.mic_pos.shape[0]
        self.fs = sample_rate
        self.N = frame_size
        self.c = speed_of_sound

        # MODO de combinacion entre frames de un evento:
        #   'onset'  → se queda con el frame de pico SRP MAS NITIDO (mayor
        #              confianza pico/mediana) desde el ultimo reset() —
        #              tipicamente el camino directo del impulso, cuyo pico es
        #              agudo, antes de que la cola reverberante (difusa, pico
        #              chato) llegue — y la mantiene. Robusto a reverberacion.
        #              ES EL MODO PARA FUENTES IMPULSIVAS (aplausos, golpes).
        #   'accum'  → EMA del mapa SRP sobre los frames activos (como COV_ALPHA
        #              de MUSIC). Para fuentes SOSTENIDAS (drone): promedia y da
        #              direccion estable. En impulsivos METE la reverb -> no usar.
        self.mode = mode
        self.accum_alpha = accum_alpha
        self._accum = None           # mapa acumulado (modo 'accum')
        self._best = None            # (az, el, conf, spectrum) del mejor frame ('onset')

        # --- Grilla en espacio-u (igual que MUSICEngine) ---
        az_lo, az_hi = np.radians(az_range[0]), np.radians(az_range[1])
        el_lo, el_hi = np.radians(el_range[0]), np.radians(el_range[1])
        num_az = max(3, round((az_range[1] - az_range[0]) / angle_resolution) + 1)
        num_el = max(3, round((el_range[1] - el_range[0]) / angle_resolution) + 1)
        u_az = np.linspace(np.sin(az_lo), np.sin(az_hi), num_az)
        u_el = np.linspace(np.sin(el_lo), np.sin(el_hi), num_el)

        self.az_vals = np.degrees(np.arcsin(np.clip(u_az, -1, 1)))
        self.el_vals = np.degrees(np.arcsin(np.clip(u_el, -1, 1)))
        self.u_az, self.u_el = u_az, u_el
        self.num_az, self.num_el = num_az, num_el

        # Vector de direccion por punto de grilla (misma convencion que MUSIC):
        #   d = [sin(az)cos(el), cos(az)cos(el), sin(el)]
        UA, UE = np.meshgrid(u_az, u_el, indexing='ij')          # (na, ne)
        cos_az = np.sqrt(np.maximum(0.0, 1.0 - UA ** 2))
        cos_el = np.sqrt(np.maximum(0.0, 1.0 - UE ** 2))
        dhat = np.stack([UA * cos_el, cos_az * cos_el, UE], axis=-1)  # (na, ne, 3)

        # Pares de microfonos y retardo teorico (muestras) por par y direccion.
        self.pairs = [(i, j) for i in range(self.M) for j in range(i + 1, self.M)]
        self.tau = np.zeros((len(self.pairs), num_az, num_el))
        for p, (i, j) in enumerate(self.pairs):
            dv = self.mic_pos[i] - self.mic_pos[j]
            self.tau[p] = (dhat @ dv) / self.c * self.fs
        self._tau_flat = self.tau.reshape(len(self.pairs), -1)      # (P, G)

        # FFT lineal (zero-pad a potencia de 2 >= 2N) y mascara de banda
        Nfft = 1
        while Nfft < 2 * frame_size:
            Nfft <<= 1
        self.Nfft = Nfft
        freqs = np.fft.rfftfreq(Nfft, d=1.0 / sample_rate)
        self.band = (freqs >= freq_min) & (freqs <= freq_max)

        # Eje de lags (enteros) alrededor de 0 para recortar la GCC tras fftshift
        self.maxlag = int(np.ceil(np.max(np.abs(self.tau)))) + 2
        self.lag_axis = np.arange(-self.maxlag, self.maxlag + 1)
        self._mid = Nfft // 2
        self.window = np.hanning(frame_size)

    def reset(self):
        """Reinicia el estado entre eventos. Llamar al volver al silencio para
        no arrastrar la direccion del evento anterior."""
        self._accum = None
        self._best = None

    def _frame_srp(self, frame):
        """Mapa SRP-PHAT de UN frame (na*ne,) — GCC-PHAT en banda + steering."""
        X = np.fft.rfft(frame * self.window[:, None], self.Nfft, axis=0)
        srp = np.zeros(self.num_az * self.num_el)
        for p, (i, j) in enumerate(self.pairs):
            R = X[:, i] * np.conj(X[:, j])
            mag = np.abs(R)
            safe = (mag > 1e-9) & self.band          # PHAT solo en banda, sin nan
            Rn = np.zeros_like(R)
            Rn[safe] = R[safe] / mag[safe]
            cc = np.fft.fftshift(np.fft.irfft(Rn, self.Nfft))
            seg = cc[self._mid - self.maxlag: self._mid + self.maxlag + 1]
            srp += np.interp(self._tau_flat[p], self.lag_axis, seg,
                             left=0.0, right=0.0)
        return srp

    def _extract(self, srp_flat):
        """De un mapa SRP saca (az, el, conf, spectrum) con refinamiento."""
        spectrum = srp_flat.reshape(self.num_az, self.num_el)
        i_az, i_el = np.unravel_index(np.argmax(spectrum), spectrum.shape)
        az_u = self._parabolic(self.u_az, spectrum[:, i_el], i_az)
        el_u = self._parabolic(self.u_el, spectrum[i_az, :], i_el)
        az = float(np.degrees(np.arcsin(np.clip(az_u, -1.0, 1.0))))
        el = float(np.degrees(np.arcsin(np.clip(el_u, -1.0, 1.0))))
        s = spectrum - spectrum.min()
        conf = 10.0 * np.log10((s[i_az, i_el] + 1e-12) / (np.median(s) + 1e-12))
        return az, el, conf, spectrum

    def process(self, frame):
        """Procesa un frame (N, M) normalizado y retorna un DOAResult.

        Modo 'onset' (impulsivos): reporta la DOA del frame con el pico SRP MAS
        NITIDO (mayor confianza) desde el ultimo reset — tipicamente el camino
        directo del aplauso, cuyo pico es agudo, antes de que la cola
        reverberante (difusa, pico chato) llegue. No detecta flancos: compara
        la confianza de cada frame y se queda con el maximo. La mantiene hasta
        el proximo evento (reset en silencio).

        Modo 'accum' (sostenidos): EMA del mapa sobre los frames activos.
        """
        if frame.shape[0] != self.N or frame.shape[1] != self.M:
            return DOAResult()

        srp = self._frame_srp(frame)

        if self.mode == 'accum':
            if self._accum is None:
                self._accum = srp
            else:
                a = self.accum_alpha
                self._accum = a * self._accum + (1 - a) * srp
            az, el, conf, spectrum = self._extract(self._accum)
            return DOAResult(az=az, el=el, conf=conf, spectrum=spectrum, valid=True)

        # --- modo 'onset': quedarse con el frame de pico mas nitido (directo) ---
        az, el, conf, spectrum = self._extract(srp)
        if self._best is None or conf > self._best[2]:
            self._best = (az, el, conf, spectrum)
        baz, bel, bconf, bspec = self._best
        return DOAResult(az=baz, el=bel, conf=bconf, spectrum=bspec, valid=True)

    @staticmethod
    def _parabolic(grid, values, idx):
        if idx <= 0 or idx >= len(values) - 1:
            return grid[idx]
        y0, y1, y2 = values[idx - 1], values[idx], values[idx + 1]
        denom = 2 * (2 * y1 - y0 - y2)
        if abs(denom) < 1e-30:
            return grid[idx]
        offset = (y0 - y2) / denom
        step = grid[idx + 1] - grid[idx]
        return grid[idx] + offset * step
