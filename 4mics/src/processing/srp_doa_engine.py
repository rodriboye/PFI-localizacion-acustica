"""
srp_doa_engine.py — SRP-PHAT broadband, motor alternativo a MUSIC (--engine srp)
con la misma interfaz process(frame) -> DOAResult.

Más robusto a reverberación y SNR moderado: MUSIC asume un subespacio de señal
bien definido y con coherencia baja produce picos espurios de alta confianza,
mientras que SRP suma la GCC-PHAT de los 6 pares en el retardo teórico de cada
dirección, así que un pico espurio en un par queda en minoría.

Por frame: FFT de los M canales (Hann, zero-pad a potencia de 2 >= 2N para
correlación lineal) -> por par, R = X_i conj(X_j) normalizada a |R|=1 en la
banda -> IFFT = GCC-PHAT -> se interpola en tau_ij(az,el) y se suman los 6 ->
máximo del mapa, refinado en espacio-u.

La confianza corre en una escala MÁS BAJA que la de MUSIC, por eso config tiene
umbrales SRP_* propios.
"""

import numpy as np
from src.processing.doa_engine import DOAResult, MUSICEngine


class SRPDoaEngine:
    """SRP-PHAT con escaneo 2D (azimut x elevación) en espacio-u."""

    def __init__(self, mic_positions, sample_rate, frame_size,
                 freq_min, freq_max, speed_of_sound=343.0,
                 az_resolution=5, az_range=(-65, 65), el_range=(0, 80),
                 mode='onset', accum_alpha=0.6, el_resolution=None):
        self.mic_pos = np.array(mic_positions, dtype=np.float64)
        self.M = self.mic_pos.shape[0]
        self.fs = sample_rate
        self.N = frame_size
        self.c = speed_of_sound

        # 'onset' → se queda con el frame de pico más nítido desde el último
        #           reset (el camino directo, antes de la cola reverberante).
        #           Para fuentes IMPULSIVAS.
        # 'accum' → EMA del mapa sobre frames activos. Para SOSTENIDAS; en
        #           impulsivas mete la reverb.
        self.mode = mode
        self.accum_alpha = accum_alpha
        self._accum = None
        self._best = None            # (az, el, conf, spectrum) del mejor frame

        # --- Grilla en espacio-u (igual que MUSICEngine) ---
        az_lo, az_hi = np.radians(az_range[0]), np.radians(az_range[1])
        el_lo, el_hi = np.radians(el_range[0]), np.radians(el_range[1])
        num_az = max(3, round((az_range[1] - az_range[0]) / az_resolution) + 1)
        el_res = az_resolution if el_resolution is None else el_resolution
        num_el = max(3, round((el_range[1] - el_range[0]) / el_res) + 1)
        u_az = np.linspace(np.sin(az_lo), np.sin(az_hi), num_az)
        u_el = np.linspace(np.sin(el_lo), np.sin(el_hi), num_el)

        self.az_vals = np.degrees(np.arcsin(np.clip(u_az, -1, 1)))
        self.el_vals = np.degrees(np.arcsin(np.clip(u_el, -1, 1)))
        self.u_az, self.u_el = u_az, u_el
        self.num_az, self.num_el = num_az, num_el

        # d = [sin(az)cos(el), cos(az)cos(el), sin(el)], misma convención que MUSIC
        UA, UE = np.meshgrid(u_az, u_el, indexing='ij')          # (na, ne)
        cos_az = np.sqrt(np.maximum(0.0, 1.0 - UA ** 2))
        cos_el = np.sqrt(np.maximum(0.0, 1.0 - UE ** 2))
        dhat = np.stack([UA * cos_el, cos_az * cos_el, UE], axis=-1)  # (na, ne, 3)

        # SIGNO: ifft(X_i conj(X_j)) tiene su pico donde x_i(t) = x_j(t - tau),
        # o sea tau = t_i - t_j. Como d_hat apunta HACIA la fuente,
        # t_m = -(r_m . d_hat)/c  =>  tau_ij = -((r_i - r_j) . d_hat)/c.
        # Sin el negativo el mapa queda espejado en los dos ejes.
        self.pairs = [(i, j) for i in range(self.M) for j in range(i + 1, self.M)]
        self.tau = np.zeros((len(self.pairs), num_az, num_el))
        for p, (i, j) in enumerate(self.pairs):
            dv = self.mic_pos[i] - self.mic_pos[j]
            self.tau[p] = -(dhat @ dv) / self.c * self.fs
        self._tau_flat = self.tau.reshape(len(self.pairs), -1)      # (P, G)

        Nfft = 1
        while Nfft < 2 * frame_size:
            Nfft <<= 1
        self.Nfft = Nfft
        freqs = np.fft.rfftfreq(Nfft, d=1.0 / sample_rate)
        self.band = (freqs >= freq_min) & (freqs <= freq_max)

        # Lags enteros alrededor de 0, para recortar la GCC tras fftshift
        self.maxlag = int(np.ceil(np.max(np.abs(self.tau)))) + 2
        self.lag_axis = np.arange(-self.maxlag, self.maxlag + 1)
        self._mid = Nfft // 2
        self.window = np.hanning(frame_size)

    def reset(self):
        """Limpia el estado entre eventos."""
        self._accum = None
        self._best = None

    def _frame_srp(self, frame):
        """Mapa SRP-PHAT de un frame (na*ne,)."""
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
        # La GCC puede ser negativa: se desplaza el mapa antes del cociente.
        s = spectrum - spectrum.min()
        conf = 10.0 * np.log10((s[i_az, i_el] + 1e-12) / (np.median(s) + 1e-12))
        return az, el, conf, spectrum

    def process(self, frame):
        """frame: (N, M) normalizado -> DOAResult.

        'onset' no detecta flancos: compara la confianza de cada frame, se queda
        con el máximo y lo mantiene hasta el próximo reset.
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

        az, el, conf, spectrum = self._extract(srp)
        if self._best is None or conf > self._best[2]:
            self._best = (az, el, conf, spectrum)
        baz, bel, bconf, bspec = self._best
        return DOAResult(az=baz, el=bel, conf=bconf, spectrum=bspec, valid=True)

    # Refinamiento sub-grilla del pico, en espacio-u. Se REUSA el de MUSIC en vez
    # de duplicarlo: la copia local tenía el denominador opuesto y refinaba en
    # dirección contraria al pico, el mismo bug que test_parabolic_interp.py
    # documenta para MUSIC y que nunca se había propagado acá.
    _parabolic = staticmethod(MUSICEngine._parabolic_interp)
