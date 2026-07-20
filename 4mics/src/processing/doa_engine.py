"""
doa_engine.py — Motor MUSIC broadband incoherente para array 3D arbitrario.

Referencia:
    Schmidt, R. O. (1986). Multiple emitter location and signal parameter
    estimation. IEEE Trans. Antennas Propag., 34(3), 276-280.

Algoritmo (resumen):
    1. Para cada frame, calcular la FFT de las M senales del array.
    2. Para cada bin de frecuencia en [f_min, f_max], construir el vector
       de correlacion cruzada R_k = x_k * x_k^H y acumularlo en la
       covarianza espacial R con promediado exponencial.
    3. Eigendescomposicion de R: los (M - num_sources) eigenvectores con
       eigenvalores mas pequenos forman el subespacio de ruido E_n.
    4. Para cada punto (az, el) de la grilla, calcular el steering vector
       a(az, el, f_k) y el pseudoespectro MUSIC:
           P(az, el, f_k) = 1 / ||E_n^H * a||^2
    5. Sumar P sobre todos los bins de frecuencia (promediado incoherente).
    6. Estimar (az, el) como el maximo de P_total.
    7. Refinar con interpolacion parabolica sub-grilla para resolucion
       mejor que ANGLE_RESOLUTION.

Notas de implementacion:
    - La grilla de escaneo es uniforme en espacio-u (u = sin(az), v = sin(el))
      en lugar de grados uniformes. Esto distribuye los puntos donde el array
      tiene resolucion real (cerca de broadside) y evita acumular puntos
      degenerados cerca de end-fire (+-90 az) donde los steering vectors son
      casi identicos entre si.
    - Los steering vectors se pre-calculan en el constructor para todos los
      puntos de la grilla y todos los bins de frecuencia. Esto tarda ~200 ms
      al arrancar pero el loop de procesamiento queda en ~10 ms/frame.
    - La covarianza se mantiene en forma Hermitiana usando
      R = alpha*R + (1-alpha) * x_k * x_k^H con carga diagonal.
    - numpy.linalg.eigh (en vez de eig) explota la simetria Hermitiana y
      es ~3x mas rapido. Los eigenvalores salen ordenados de menor a mayor.
"""

import numpy as np
from numpy.linalg import eigh


class DOAResult:
    """Resultado de una estimacion DOA."""
    __slots__ = ['azimuth', 'elevation', 'confidence', 'spectrum', 'valid']

    def __init__(self, az=0.0, el=0.0, conf=0.0, spectrum=None, valid=False):
        self.azimuth    = az
        self.elevation  = el
        self.confidence = conf   # dB del pico sobre el piso del espectro
        self.spectrum   = spectrum  # ndarray (num_az, num_el) del pseudoespectro
        self.valid      = valid


class MUSICEngine:
    """
    MUSIC broadband incoherente con escaneo 2D (azimut x elevacion).
    La grilla escanea en espacio-u (u = sin(az)) para distribuir los puntos
    de forma proporcional a la resolucion fisica del array.
    """

    def __init__(self, mic_positions, sample_rate, frame_size,
                 freq_min, freq_max, speed_of_sound=343.0,
                 angle_resolution=5, num_sources=1,
                 cov_alpha=0.5, diag_loading=0.01,
                 az_range=(-90, 90), el_range=(-60, 60)):

        self.mic_pos = np.array(mic_positions, dtype=np.float64)
        self.M       = self.mic_pos.shape[0]
        self.fs      = sample_rate
        self.N       = frame_size
        self.c       = speed_of_sound
        self.nsrc    = num_sources
        self.alpha   = cov_alpha
        self.dload   = diag_loading

        # Grilla de escaneo en espacio-u (u = sin(az), v = sin(el)).
        #
        # Por que espacio-u y no grados uniformes:
        #   El array responde a TDOA = d*sin(theta)/c. En grados uniformes,
        #   los puntos cerca de +-90 tienen steering vectors casi identicos
        #   (d(sin theta)/d(theta) -> 0), produciendo picos falsos en los
        #   bordes. En espacio-u los puntos estan espaciados uniformemente
        #   en la magnitud fisica que el array puede medir.
        #
        #   Numero de puntos: mismo que en grados uniformes para mantener
        #   el costo computacional.
        az_lo, az_hi = np.radians(az_range[0]), np.radians(az_range[1])
        el_lo, el_hi = np.radians(el_range[0]), np.radians(el_range[1])
        num_az = max(3, round((az_range[1] - az_range[0]) / angle_resolution) + 1)
        num_el = max(3, round((el_range[1] - el_range[0]) / angle_resolution) + 1)

        u_az = np.linspace(np.sin(az_lo), np.sin(az_hi), num_az)
        u_el = np.linspace(np.sin(el_lo), np.sin(el_hi), num_el)

        # Valores en grados para reportar el resultado del pico
        self.az_vals = np.degrees(np.arcsin(np.clip(u_az, -1, 1)))
        self.el_vals = np.degrees(np.arcsin(np.clip(u_el, -1, 1)))
        self.u_az    = u_az
        self.u_el    = u_el
        self.num_az  = num_az
        self.num_el  = num_el

        # Bins de frecuencia dentro del rango de interes
        freqs = np.fft.rfftfreq(frame_size, d=1.0 / sample_rate)
        mask  = (freqs >= freq_min) & (freqs <= freq_max)
        self.freq_idx = np.where(mask)[0]
        self.freqs    = freqs[self.freq_idx]

        # Covarianza espacial acumulada: UNA matriz M x M por bin de
        # frecuencia. Shape (num_f, M, M). MUSIC incoherente necesita un
        # subespacio de ruido independiente por bin, porque el steering
        # vector rota con la frecuencia; compartir una unica covarianza
        # entre todos los bins corrompe el pseudoespectro.
        num_f = len(self.freqs)
        self._R0 = np.repeat(
            (np.eye(self.M, dtype=np.complex128) * 1e-10)[np.newaxis, :, :],
            num_f, axis=0)
        self.R = self._R0.copy()

        # Pre-calcular steering vectors: shape (num_az, num_el, num_freq, M)
        self._steering = self._precompute_steering()

    def reset(self):
        """Reinicia la covarianza acumulada a su estado inicial. Llamar al volver
        al silencio (main.py lo hace) para que el subespacio de ruido del proximo
        evento no quede contaminado por la cola reverberante o el silencio previo.
        Nota: MUSIC necesita promediar varios frames para condicionar R, asi que
        no puede usar un solo frame de onset como SRP; para fuentes impulsivas en
        reverberacion, SRP en modo 'onset' es mas robusto."""
        self.R = self._R0.copy()

    def _precompute_steering(self):
        """
        Pre-calcula los steering vectors para toda la grilla y todos los bins.

        a(az, el, f)[m] = exp(-j * 2*pi * f/c * (r_m . d_hat))

        donde d_hat es el vector unitario hacia la fuente y r_m es la posicion
        del microfono m. El retardo de propagacion hasta el microfono m es
        tau_m = (r_m . d_hat) / c.

        La grilla es uniforme en espacio-u con u_az = sin(az) y u_el = sin(el)
        (ver __init__). El vector unitario de direccion se reconstruye aqui:
            d = [sin(az)cos(el), cos(az)cos(el), sin(el)]
        con sin(az)=u_a, sin(el)=u_e, cos(az)=sqrt(1-u_a^2), cos(el)=sqrt(1-u_e^2).
        """
        num_f = len(self.freqs)
        A = np.zeros((self.num_az, self.num_el, num_f, self.M), dtype=np.complex128)

        for i, u_a in enumerate(self.u_az):        # u_a = sin(az)
            cos_az = np.sqrt(max(0.0, 1.0 - u_a**2))
            for j, u_e in enumerate(self.u_el):    # u_e = sin(el)
                cos_el = np.sqrt(max(0.0, 1.0 - u_e**2))
                # Coseno director de la fuente. NOTA: u_a es sin(az), NO el
                # coseno director x; por eso x lleva el factor cos(el).
                d_hat = np.array([
                    u_a * cos_el,       # x = sin(az)*cos(el)
                    cos_az * cos_el,    # y = cos(az)*cos(el)  (frente)
                    u_e,                # z = sin(el)
                ])
                # Retardo de cada microfono (segundos)
                tau = self.mic_pos @ d_hat / self.c  # (M,)
                # Fase para cada frecuencia: e^{-j 2*pi*f*tau}
                phases = np.exp(-1j * 2 * np.pi *
                                np.outer(self.freqs, tau))  # (num_f, M)
                A[i, j, :, :] = phases

        return A  # (num_az, num_el, num_f, M)

    def process(self, frame):
        """
        Procesa un frame y retorna un DOAResult.

        Args:
            frame: ndarray float64, shape (hop_size, M) — audio normalizado [-1,1]

        Returns:
            DOAResult con azimuth, elevation, confidence y spectrum.
        """
        if frame.shape[0] != self.N or frame.shape[1] != self.M:
            return DOAResult()

        # FFT de todos los canales: (N//2+1, M)
        X = np.fft.rfft(frame, axis=0)  # (N//2+1, M)
        X_roi = X[self.freq_idx, :]      # (num_f, M)

        # --- Covarianza espacial POR BIN, con promediado temporal exponencial ---
        # R[f] = alpha * R[f] + (1 - alpha) * x_f x_f^H
        # El alpha temporal se aplica una vez por frame. Cada bin mantiene su
        # propia covarianza: es lo que exige MUSIC incoherente. El steering
        # depende de f, asi que compartir una unica R entre todos los bins
        # esparce el pico del pseudoespectro y degrada la estimacion.
        R_inst = np.einsum('fm,fn->fmn', X_roi, X_roi.conj())  # (num_f, M, M)
        self.R = self.alpha * self.R + (1 - self.alpha) * R_inst

        # Regularizacion diagonal por bin (carga = fraccion de la traza de R[f])
        traces = np.real(np.einsum('fmm->f', self.R))          # (num_f,)
        R_reg = self.R + (np.eye(self.M)[np.newaxis, :, :] *
                          (self.dload * traces)[:, np.newaxis, np.newaxis])

        # Eigendescomposicion por bin. numpy.linalg.eigh es batched: opera
        # sobre los dos ultimos ejes y deja los autovalores en orden ascendente.
        eigenvalues, eigenvectors = eigh(R_reg)  # (num_f, M), (num_f, M, M)

        # Subespacio de ruido por bin: los (M - num_sources) autovectores menores
        E_n = eigenvectors[:, :, :self.M - self.nsrc]          # (num_f, M, M-nsrc)
        P_n = E_n @ E_n.conj().transpose(0, 2, 1)              # (num_f, M, M)

        # Pseudoespectro MUSIC — promediado incoherente sobre frecuencias.
        # Para cada bin f: denom = a(f)^H P_n[f] a(f)  (forma Hermitiana).
        # IMPORTANTE: es conj(a) . P_n . a. Calcular a . P_n . conj(a)
        # proyecta sobre el subespacio de ruido CONJUGADO y devuelve la
        # direccion ESPEJADA (az->-az, el->-el).
        A = self._steering  # (num_az, num_el, num_f, M)
        # matmul batcheado: para cada bin f, conj(a) multiplica P_n[f].
        # (np.einsum hace lo mismo pero es ~3x mas lento aqui.)
        Ap = (A.conj()[:, :, :, None, :] @ P_n[None, None, :, :, :])[:, :, :, 0, :]
        denom = np.real(np.sum(Ap * A, axis=-1))               # (num_az, num_el, num_f)
        denom = np.maximum(denom, 1e-30)

        spectrum = np.sum(1.0 / denom, axis=-1)  # (num_az, num_el)

        # Encontrar pico
        idx = np.unravel_index(np.argmax(spectrum), spectrum.shape)
        i_az, i_el = idx

        # Refinar con interpolación parabólica en espacio-u (grilla uniforme).
        # La grilla en grados NO es uniforme (es arcsin de una grilla u uniforme),
        # así que la fórmula parabólica —que asume paso constante— debe aplicarse
        # sobre u_az/u_el, y el resultado se convierte a grados con arcsin.
        az_u = self._parabolic_interp(self.u_az, spectrum[:, i_el], i_az)
        el_u = self._parabolic_interp(self.u_el, spectrum[i_az, :], i_el)
        az_refined = float(np.degrees(np.arcsin(np.clip(az_u, -1.0, 1.0))))
        el_refined = float(np.degrees(np.arcsin(np.clip(el_u, -1.0, 1.0))))

        # Confianza: relacion pico/piso en dB
        peak  = spectrum[i_az, i_el]
        floor = np.median(spectrum)
        conf  = 10 * np.log10(peak / (floor + 1e-30))

        return DOAResult(az=az_refined, el=el_refined, conf=conf,
                         spectrum=spectrum, valid=True)

    @staticmethod
    def _parabolic_interp(grid, values, idx):
        """
        Interpolación parabólica para sub-resolución de grilla.

        La fórmula asume paso constante entre puntos (grilla uniforme).
        Llamar siempre con u_az/u_el (espacio-u, uniforme), nunca con az_vals/el_vals
        (grados, no uniformes por ser arcsin de una grilla u uniforme).
        El llamador convierte el resultado de regreso a grados con arcsin.
        """
        if idx <= 0 or idx >= len(values) - 1:
            return grid[idx]
        y0, y1, y2 = values[idx - 1], values[idx], values[idx + 1]
        denom = 2 * (2 * y1 - y0 - y2)
        if abs(denom) < 1e-30:
            return grid[idx]
        offset = (y0 - y2) / denom          # en unidades de paso de grilla
        step = grid[idx + 1] - grid[idx]    # paso uniforme en espacio-u
        return grid[idx] + offset * step
