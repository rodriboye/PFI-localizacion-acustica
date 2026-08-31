"""
doa_engine.py — MUSIC broadband incoherente para array 3D arbitrario.

Ref.: Schmidt (1986), Múltiple emitter location and signal parameter estimation,
IEEE Trans. Antennas Propag. 34(3), 276-280.

Por frame: buffer deslizante de fft_size muestras -> FFT con ventana de Hann ->
por cada bin de [f_min, f_max], covarianza espacial con promediado exponencial,
eigendescomposición y pseudoespectro 1/||E_n^H a||^2 -> suma incoherente sobre
los bins -> máximo 2D -> refinamiento parabólico.

Tres decisiones no obvias:
  - La FFT está DESACOPLADA del hop: el hop fija latencia y tasa de snapshots,
    la FFT fija df = fs/N. Con fft_size = 2*hop hay 50% de solape sin tocar
    latencia ni firmware.
  - La suma incoherente cubre TODA la ROI sin pesos. Restringirla a los bins del
    peine del dron empeora el error y triplica los fallos a bajo SNR: los bins
    entre líneas no son ruido, la fuga arrastra las mismas diferencias de fase.
    MUSIC localiza la fuente dominante de la banda, sea o no el dron; el gate
    espectral filtra después, desde main.py.
  - La grilla es uniforme en espacio-u (u = sin(az)) y no en grados: distribuye
    los puntos donde el array tiene resolución real y evita puntos degenerados
    cerca de end-fire.
"""

import numpy as np
from numpy.linalg import eigh


class DOAResult:
    """Resultado de una estimación DOA."""
    __slots__ = ['azimuth', 'elevation', 'confidence', 'spectrum', 'valid', 'n_bins']

    def __init__(self, az=0.0, el=0.0, conf=0.0, spectrum=None, valid=False,
                 n_bins=0):
        self.azimuth    = az
        self.elevation  = el
        self.confidence = conf      # dB del pico sobre el piso del espectro
        self.spectrum   = spectrum  # ndarray (num_az, num_el)
        self.valid      = valid
        self.n_bins     = n_bins    # bins en la suma incoherente


class MUSICEngine:
    """MUSIC broadband incoherente, escaneo 2D en espacio-u."""

    def __init__(self, mic_positions, sample_rate, frame_size,
                 freq_min, freq_max, speed_of_sound=343.0,
                 az_resolution=5, num_sources=1,
                 cov_alpha=0.5, diag_loading=0.01,
                 az_range=(-90, 90), el_range=(-60, 60),
                 fft_size=None, bin_stride=1, el_resolution=None):
        """frame_size = hop del firmware; fft_size = longitud de análisis,
        múltiplo entero del hop (None => sin solape); bin_stride submuestrea la
        ROI de frecuencia."""
        self.mic_pos = np.array(mic_positions, dtype=np.float64)
        self.M       = self.mic_pos.shape[0]
        self.fs      = sample_rate
        self.c       = speed_of_sound
        self.nsrc    = num_sources
        self.alpha   = cov_alpha
        self.dload   = diag_loading

        # Nfft múltiplo del hop para que el buffer avance en bloques enteros: el
        # desplazamiento es un slice, sin resampleo ni resto pendiente.
        self.hop  = int(frame_size)
        self.Nfft = int(fft_size) if fft_size else self.hop
        if self.Nfft < self.hop or self.Nfft % self.hop != 0:
            raise ValueError(
                f"fft_size ({self.Nfft}) debe ser un multiplo entero de "
                f"frame_size/hop ({self.hop}) y no menor que el.")
        self.n_hops = self.Nfft // self.hop

        self.N = self.Nfft   # alias histórico

        # Hann PERIÓDICA: np.hanning(N) devuelve la simétrica, que sesga el
        # análisis por FFT. Equivale a scipy get_window('hann', N).
        self.window = np.hanning(self.Nfft + 1)[:-1].astype(np.float64)

        az_lo, az_hi = np.radians(az_range[0]), np.radians(az_range[1])
        el_lo, el_hi = np.radians(el_range[0]), np.radians(el_range[1])
        # Resolución propia por eje: el costo es num_az * num_el.
        el_res = az_resolution if el_resolution is None else el_resolution
        num_az = max(3, round((az_range[1] - az_range[0]) / az_resolution) + 1)
        num_el = max(3, round((el_range[1] - el_range[0]) / el_res) + 1)
        self.az_resolution = az_resolution
        self.el_resolution = el_res

        u_az = np.linspace(np.sin(az_lo), np.sin(az_hi), num_az)
        u_el = np.linspace(np.sin(el_lo), np.sin(el_hi), num_el)

        self.az_vals = np.degrees(np.arcsin(np.clip(u_az, -1, 1)))
        self.el_vals = np.degrees(np.arcsin(np.clip(u_el, -1, 1)))
        self.u_az    = u_az
        self.u_el    = u_el
        self.num_az  = num_az
        self.num_el  = num_el

        # Bins de la ROI sobre la grilla de la FFT de análisis (Nfft, no el hop).
        freqs = np.fft.rfftfreq(self.Nfft, d=1.0 / sample_rate)
        mask  = (freqs >= freq_min) & (freqs <= freq_max)
        idx   = np.where(mask)[0]
        self.bin_stride = max(1, int(bin_stride))
        if self.bin_stride > 1:
            idx = idx[::self.bin_stride]
        self.freq_idx = idx
        self.freqs    = freqs[self.freq_idx]
        if self.freqs.size == 0:
            raise ValueError(
                f"No hay bins en [{freq_min}, {freq_max}] Hz con Nfft={self.Nfft} "
                f"y fs={sample_rate}.")

        # UNA covarianza M x M POR BIN. MUSIC incoherente necesita un subespacio
        # de ruido independiente por bin porque el steering rota con f;
        # compartir una sola R esparce el pico.
        num_f = len(self.freqs)
        self._R0 = np.repeat(
            (np.eye(self.M, dtype=np.complex128) * 1e-10)[np.newaxis, :, :],
            num_f, axis=0)
        self.R = self._R0.copy()

        self.df = self.fs / self.Nfft   # Hz por bin, informativo

        # Buffer deslizante de Nfft muestras, alimentado de a un hop.
        # _hops_filled cuenta hops REALES desde el último reset(): hasta llegar a
        # n_hops el buffer arrastra ceros y process() devuelve inválido.
        self._buf = np.zeros((self.Nfft, self.M), dtype=np.float64)
        self._hops_filled = 0

        self._steering = self._precompute_steering()

    def reset(self):
        """Vacía covarianza y buffer. main.py lo llama tras silencio SOSTENIDO
        para que el próximo evento no herede la cola reverberante ni audio de
        otra dirección. Cuesta n_hops-1 frames inválidos de calentamiento."""
        self.R = self._R0.copy()
        self._buf[:] = 0.0
        self._hops_filled = 0

    def _precompute_steering(self):
        """
        a(az, el, f)[m] = exp(-j*2*pi*f*tau_m),  tau_m = -(r_m . d_hat)/c
        con d_hat unitario HACIA la fuente. (num_az, num_el, num_f, M).

        EL SIGNO NEGATIVO NO ES OPCIONAL: el micrófono con mayor (r_m . d_hat) es
        el más cercano y por lo tanto el de retardo MENOR. Con signo positivo la
        dirección sale ESPEJADA en los dos ejes.

        Queda atado al signo de la columna Z de MIC_POSITIONS: negarla invierte
        exactamente la elevación estimada (el azimut no cambia). Las dos cosas
        juntas se cancelan y la elevación vuelve a salir bien con el azimut
        espejado, que es el estado que tuvo el sistema hasta 2026-08. Si la
        elevación sale invertida, el error está en el cableado o en el orden de
        canales, no en este signo.
        """
        num_f = len(self.freqs)
        A = np.zeros((self.num_az, self.num_el, num_f, self.M), dtype=np.complex128)

        for i, u_a in enumerate(self.u_az):        # u_a = sin(az)
            cos_az = np.sqrt(max(0.0, 1.0 - u_a**2))
            for j, u_e in enumerate(self.u_el):    # u_e = sin(el)
                cos_el = np.sqrt(max(0.0, 1.0 - u_e**2))
                # u_a es sin(az), NO el coseno director x: por eso x lleva cos(el).
                d_hat = np.array([
                    u_a * cos_el,       # x = sin(az)*cos(el)
                    cos_az * cos_el,    # y = cos(az)*cos(el)  (frente)
                    u_e,                # z = sin(el)
                ])
                tau = -(self.mic_pos @ d_hat) / self.c  # (M,) segundos
                phases = np.exp(-1j * 2 * np.pi *
                                np.outer(self.freqs, tau))  # (num_f, M)
                A[i, j, :, :] = phases

        return A  # (num_az, num_el, num_f, M)

    def process(self, frame):
        """frame: (hop, M) float64 normalizado. Devuelve DOAResult, inválido
        mientras el buffer no tenga n_hops hops reales."""
        if frame.shape[0] != self.hop or frame.shape[1] != self.M:
            return DOAResult()

        # Entra un hop, sale el más viejo. Con n_hops = 1 no hay solape.
        if self.n_hops > 1:
            self._buf[:-self.hop] = self._buf[self.hop:]
        self._buf[-self.hop:] = frame

        if self._hops_filled < self.n_hops:
            # Quedan ceros del reset() en la ventana: no se actualiza R (un
            # snapshot medio-nulo sesgaría la covarianza) y se devuelve inválido.
            self._hops_filled += 1
            if self._hops_filled < self.n_hops:
                return DOAResult()

        # La ventana multiplica igual a los M canales: las diferencias de fase
        # entre micrófonos no se alteran. No se compensa su ganancia coherente
        # porque MUSIC es invariante a escala.
        X = np.fft.rfft(self._buf * self.window[:, np.newaxis], axis=0)
        X_roi = X[self.freq_idx, :]      # (num_f, M)

        # R[f] = alpha*R[f] + (1-alpha) * x_f x_f^H, una vez por frame
        R_inst = np.einsum('fm,fn->fmn', X_roi, X_roi.conj())  # (num_f, M, M)
        self.R = self.alpha * self.R + (1 - self.alpha) * R_inst

        traces = np.real(np.einsum('fmm->f', self.R))          # (num_f,)
        R_reg = self.R + (np.eye(self.M)[np.newaxis, :, :] *
                          (self.dload * traces)[:, np.newaxis, np.newaxis])

        R_use = R_reg
        A     = self._steering  # (num_az, num_el, num_f, M)

        # eigh es batched sobre los dos últimos ejes; autovalores ascendentes.
        eigenvalues, eigenvectors = eigh(R_use)  # (nf, M), (nf, M, M)

        # IDENTIDAD EXACTA: los autovectores son base ortonormal completa, así
        # que E_s E_s^H + E_n E_n^H = I, y como el steering es de módulo unitario
        # (a^H a = M):   a^H P_n a = M - ||E_s^H a||^2.
        # Con nsrc = 1 eso es un único producto interno (4 MAC en vez de 16) y el
        # temporal pasa de (num_az, num_el, nf, M) a (num_az, num_el, nf) — un
        # cuarto del tráfico de memoria, que es el cuello de botella real.
        # La conjugación importa: es conj(a) . E_s; al revés la dirección sale
        # espejada. La carga diagonal no interviene (corre todos los autovalores
        # por igual y deja P_n idéntico).
        if self.nsrc == 1:
            E_s = eigenvectors[:, :, -1]                       # (nf, M)
            proj = np.einsum('aefm,fm->aef', A.conj(), E_s)    # (num_az,num_el,nf)
            denom = self.M - np.real(proj * np.conj(proj))
        else:
            # Con nsrc > 1 el ahorro se achica: vía clásica por claridad.
            E_n = eigenvectors[:, :, :self.M - self.nsrc]      # (nf, M, M-nsrc)
            P_n = E_n @ E_n.conj().transpose(0, 2, 1)          # (nf, M, M)
            Ap = (A.conj()[:, :, :, None, :] @ P_n[None, None, :, :, :])[:, :, :, 0, :]
            denom = np.real(np.sum(Ap * A, axis=-1))           # (num_az, num_el, nf)

        denom = np.maximum(denom, 1e-30)

        # Suma sin pesos: 1/(a^H P_n a) ya se auto-pondera. En un bin de ruido los
        # autovalores son casi iguales, P_n queda isotrópico y el término vale
        # ~1/(M-nsrc), acotado. El rango dinámico lo fija el PASO DE GRILLA, no la
        # carga diagonal: refinar la grilla estrecha los picos, subir la carga no
        # cambia nada. (Deja de valer si se pasa a una formulación que invierta R.)
        spectrum = np.sum(1.0 / denom, axis=-1)                   # (num_az, num_el)

        idx = np.unravel_index(np.argmax(spectrum), spectrum.shape)
        i_az, i_el = idx

        # Refinamiento en espacio-u: la fórmula asume paso constante y la grilla
        # en grados es arcsin de una grilla u uniforme, o sea NO uniforme.
        az_u = self._parabolic_interp(self.u_az, spectrum[:, i_el], i_az)
        el_u = self._parabolic_interp(self.u_el, spectrum[i_az, :], i_el)
        az_refined = float(np.degrees(np.arcsin(np.clip(az_u, -1.0, 1.0))))
        el_refined = float(np.degrees(np.arcsin(np.clip(el_u, -1.0, 1.0))))

        peak  = spectrum[i_az, i_el]
        floor = np.median(spectrum)
        conf  = 10 * np.log10(peak / (floor + 1e-30))

        n_bins = int(A.shape[2])
        return DOAResult(az=az_refined, el=el_refined, conf=conf,
                         spectrum=spectrum, valid=True, n_bins=n_bins)

    @staticmethod
    def _parabolic_interp(grid, values, idx):
        """
        Refinamiento sub-grilla del pico. Llamar SIEMPRE en espacio-u (paso
        constante); el llamador convierte a grados con arcsin.

        Vértice de la parábola por (-1,y0), (0,y1), (+1,y2):
            p = 0.5*(y0 - y2) / (y0 - 2*y1 + y2)
        Para un máximo el denominador es NEGATIVO. Escribirlo como
        2*(2*y1 - y0 - y2) —su opuesto— corre la estimación en dirección
        CONTRARIA al pico. Cubierto por tests/test_parabolic_interp.py.

        El offset se satura a +-1 paso: con un pico más angosto que la grilla las
        muestras vecinas quedan sobre el piso y sin saturar el ajuste escupe
        offsets enormes.
        """
        if idx <= 0 or idx >= len(values) - 1:
            return grid[idx]
        y0, y1, y2 = values[idx - 1], values[idx], values[idx + 1]
        denom = y0 - 2.0 * y1 + y2
        if abs(denom) < 1e-30:
            return grid[idx]
        offset = 0.5 * (y0 - y2) / denom    # en pasos de grilla
        offset = min(max(offset, -1.0), 1.0)
        step = grid[idx + 1] - grid[idx]
        return grid[idx] + offset * step
