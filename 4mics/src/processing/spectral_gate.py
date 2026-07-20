"""
spectral_gate.py — Gate espectral armónico: confirma "es un dron" por su huella
acústica antes de gastar cómputo en localizar (MUSIC).

Motivación (ver docs/notes/deteccion_espectral.md)
--------------------------------------------------------------------------------
El detector de energía (detector.py) colapsa el frame a un escalar y dispara con
CUALQUIER sonido fuerte: viento, voces, tráfico, una puerta. Para el objetivo de
rastrear un dron eso es demasiado permisivo. El ruido de un multirrotor, en
cambio, tiene una firma distintiva: cada rotor genera un tono a la *frecuencia de
paso de pala* (BPF = N_palas × RPM / 60, típicamente ~80–300 Hz) con una serie de
ARMÓNICOS regularmente espaciados (2·BPF, 3·BPF, …). Esa estructura de "peine"
es lo que este gate busca y valida.

Arquitectura: dos gates en cascada (este NO reemplaza al de energía, lo encadena)
--------------------------------------------------------------------------------
    audio → [gate energía]  → [gate armónico (este)] → [MUSIC]
            barato, ¿vale?      ¿es un dron?            localizar

El gate armónico solo se evalúa cuando el de energía ya marcó actividad, así que
el costo extra (una FFT por frame sobre una mezcla mono) se paga únicamente
cuando hay algo que analizar.

Algoritmo
--------------------------------------------------------------------------------
1. Ventana de análisis: se acumulan los últimos `window_size` samples de una
   MEZCLA MONO (promedio de los 4 micrófonos). La dirección no importa para la
   firma; mezclar mejora el SNR del peine. Una ventana larga (p.ej. 2048) da
   buena resolución en frecuencia (df = fs/window) para resolver el peine.

2. Estimación de la BPF (fundamental) por HARMONIC PRODUCT SPECTRUM (HPS):
   se multiplica el espectro de magnitud por versiones decimadas (÷2, ÷3, …).
   Los armónicos de una misma fundamental se ALINEAN y se refuerzan; el ruido de
   banda ancha no. El pico del HPS dentro de [bpf_min, bpf_max] es el candidato a
   BPF. HPS es barato, robusto y no necesita entrenamiento (alineado con la
   prioridad de robustez y simplicidad del proyecto).

3. Validación del peine: conocida la BPF, se sabe EXACTAMENTE dónde caen los
   armónicos (h·BPF). Para cada uno se mide el SNR local (pico sobre la mediana
   del ruido vecino). Se cuentan los armónicos con SNR suficiente.

   El SNR de cada armónico se mide contra un piso de ruido robusto: el MÁXIMO
   entre la mediana local y un PISO GLOBAL (mediana del espectro completo). El
   piso global es clave para no confundir un TONO PURO con un dron: las colas de
   fuga (leakage) de la ventana caen sobre 2f, 3f… con un nivel pequeño pero por
   encima de la mediana local (~0 ahí), fingiendo un peine; exigir que superen el
   piso global descarta esa fuga (un armónico real lo supera holgadamente).

4. Decisión (todas deben cumplirse):
     - hay una corrida de al menos `min_harmonics` armónicos CONSECUTIVOS
       (h, h+1, h+2, …) que superan `harmonic_snr_db` — un peine real es
       contiguo; un tono puro da, a lo sumo, su fundamental más armónicos
       sueltos y espurios (ruido en k·BPF) que no forman corrida;
     - al menos `min_harmonics_in_band` de esos caen DENTRO de la banda útil de
       localización (`music_band`, ~300–2425 Hz) con SNR suficiente — esto
       garantiza que MUSIC tendrá energía tonal real en su banda;
     - el Harmonic-to-Noise Ratio global supera `score_min` dB;
     - la FRACCIÓN de energía en el peine sobre la energía total de la banda
       supera `harmonic_fraction_min` — separa el dron del RUIDO de banda ancha
       (que reparte su energía y deja poca en las pocas líneas del peine).

5. Confirmación + histéresis: se requieren `confirm_windows` ventanas positivas
   CONSECUTIVAS para latchear el veredicto (suprime ventanas espurias aisladas).
   Una vez confirmado, se MANTIENE `hold_frames` frames aunque algún frame no
   valide (las fluctuaciones de amplitud del dron —efecto de pala, multipath—
   hacen parpadear el peine). Evita cortar el tracking por un bajón transitorio.

Solo numpy: corre en la RPi sin dependencias extra (scipy es opcional en el
resto del proyecto; acá no hace falta).
"""

import numpy as np


class SpectralResult:
    """Resultado de una evaluación del gate espectral."""
    __slots__ = ['is_drone', 'bpf', 'n_harmonics', 'n_in_band',
                 'hnr_db', 'harmonics', 'ready', 'harm_fraction', 'n_consecutive']

    def __init__(self, is_drone=False, bpf=0.0, n_harmonics=0, n_in_band=0,
                 hnr_db=0.0, harmonics=None, ready=False, harm_fraction=0.0,
                 n_consecutive=0):
        self.is_drone      = is_drone     # veredicto (ya con histéresis)
        self.bpf           = bpf          # BPF estimada (Hz)
        self.n_harmonics   = n_harmonics  # armónicos del peine con SNR suficiente
        self.n_in_band     = n_in_band    # de esos, cuántos caen en la banda MUSIC
        self.hnr_db        = hnr_db       # Harmonic-to-Noise Ratio global (dB)
        self.harmonics     = harmonics or []  # lista de (h, freq_Hz, snr_db)
        self.ready         = ready        # True si la ventana ya está llena de audio real
        self.harm_fraction = harm_fraction  # energía del peine / energía total de la banda
        self.n_consecutive = n_consecutive  # corrida más larga de armónicos consecutivos


class HarmonicDroneGate:
    """
    Gate de confirmación de dron por estructura armónica.

    Uso típico (en el loop de main.py, solo cuando el detector de energía marca
    actividad):

        gate = HarmonicDroneGate(sample_rate=cfg.SAMPLE_RATE, ...)
        ...
        res = gate.update(frame)          # frame: (hop, M) o (hop,) mono
        if res.is_drone:                  # confirmado → localizar con MUSIC
            ...

    En silencio conviene llamar gate.reset() para limpiar la ventana y el latch
    de histéresis (que el próximo evento no herede el veredicto del anterior).
    """

    def __init__(self, sample_rate, window_size=2048, hop_size=256,
                 bpf_min=80.0, bpf_max=400.0, n_harmonics=8, hps_downsample=5,
                 music_band=(300.0, 2425.0), harmonic_snr_db=8.0,
                 min_harmonics=3, min_harmonics_in_band=1,
                 score_min=6.0, harmonic_tol_hz=18.0, hold_frames=10,
                 harmonic_fraction_min=0.10, confirm_windows=2):
        self.fs            = float(sample_rate)
        self.window_size   = int(window_size)
        self.hop_size      = int(hop_size)
        self.bpf_min       = float(bpf_min)
        self.bpf_max       = float(bpf_max)
        self.n_harmonics   = int(n_harmonics)
        self.hps_down      = max(2, int(hps_downsample))
        self.music_lo      = float(music_band[0])
        self.music_hi      = float(music_band[1])
        self.snr_thr_db    = float(harmonic_snr_db)
        self.min_harm      = int(min_harmonics)
        self.min_harm_band = int(min_harmonics_in_band)
        self.score_min     = float(score_min)
        # Fracción mínima de energía en el peine sobre la energía total de la
        # banda: separa el dron (peine con mucha energía) del RUIDO de banda
        # ancha (energía repartida; fracción baja en las pocas líneas del peine).
        self.frac_min      = float(harmonic_fraction_min)
        # Ventanas positivas consecutivas requeridas antes de latchear el
        # veredicto: suprime ventanas espurias aisladas (un golpe de ruido que
        # casualmente alinea 3 bins). Las ventanas se solapan mucho (hop<<window),
        # así que 2 confirmaciones cuestan ~1 hop de latencia.
        self.confirm_need  = max(1, int(confirm_windows))
        self.hold_frames   = int(hold_frames)

        # Resolución espectral y geometría de bins
        self.df       = self.fs / self.window_size          # Hz por bin
        self.window   = np.hanning(self.window_size)
        self.freqs    = np.fft.rfftfreq(self.window_size, d=1.0 / self.fs)
        # Tolerancia (en bins) alrededor de cada armónico: la BPF no es
        # perfectamente estacionaria (RPM varía con el control de vuelo).
        self.tol_bins = max(1, int(round(harmonic_tol_hz / self.df)))
        # Semiancho (en bins) de la ventana local de ruido alrededor del armónico.
        self.noise_halfwidth = max(self.tol_bins * 4, int(round(60.0 / self.df)))

        # Búsqueda de BPF acotada a [bpf_min, bpf_max]
        self._bpf_lo_bin = max(1, int(np.floor(self.bpf_min / self.df)))
        self._bpf_hi_bin = int(np.ceil(self.bpf_max / self.df))

        # Buffer circular de la mezcla mono (se inicializa en silencio)
        self._buf      = np.zeros(self.window_size, dtype=np.float64)
        self._filled   = 0          # cuántas muestras reales lleva acumuladas
        self._hold     = 0          # frames restantes del latch de histéresis
        self._confirmed = False
        self._confirm_count = 0     # ventanas positivas consecutivas acumuladas

    # ------------------------------------------------------------------ utils
    def reset(self):
        """Limpia la ventana de audio y el latch de histéresis. Llamar al volver
        al silencio para que el próximo evento se evalúe desde cero."""
        self._buf.fill(0.0)
        self._filled    = 0
        self._hold      = 0
        self._confirmed = False
        self._confirm_count = 0

    @property
    def confirmed(self):
        return self._confirmed

    def idle_tick(self):
        """Decaimiento del latch en frames SIN actividad. main.py no llama a
        update() en silencio (no hay señal que evaluar), pero la semántica de
        hold_frames debe mantenerse igual: el veredicto confirmado no
        sobrevive más de hold_frames frames sin evidencia, haya o no señal.
        Llamar una vez por frame idle/cooldown."""
        self._confirm_count = 0
        if self._hold > 0:
            self._hold -= 1
            if self._hold <= 0:
                self._confirmed = False

    def _push(self, frame):
        """Agrega un hop (mezcla mono) al buffer circular."""
        if frame.ndim == 2:
            mono = frame.mean(axis=1)
        else:
            mono = frame
        n = len(mono)
        if n >= self.window_size:
            self._buf[:] = mono[-self.window_size:]
        else:
            self._buf[:-n] = self._buf[n:]
            self._buf[-n:] = mono
        self._filled = min(self.window_size, self._filled + n)

    # ----------------------------------------------------------------- núcleo
    def _spectrum(self):
        """PSD de la ventana actual (sin DC). Devuelve potencia por bin."""
        x = self._buf - self._buf.mean()
        X = np.fft.rfft(x * self.window)
        return (X.real ** 2 + X.imag ** 2)

    def _estimate_bpf(self, power):
        """Estima la BPF por Harmonic Product Spectrum sobre el espectro de
        magnitud. Devuelve (bpf_hz, bin) o (0.0, -1) si no hay candidato."""
        mag = np.sqrt(power)
        # log para que el producto sea suma (más estable numéricamente) y para
        # no dejar que un único bin enorme domine.
        hps = np.log(mag + 1e-12).copy()
        L = len(mag)
        for d in range(2, self.hps_down + 1):
            dec = mag[::d]                      # decimación por d
            hps[:len(dec)] += np.log(dec + 1e-12)
        lo, hi = self._bpf_lo_bin, min(self._bpf_hi_bin, L - 1)
        if hi <= lo:
            return 0.0, -1
        seg = hps[lo:hi + 1]
        k = int(np.argmax(seg)) + lo
        return self.freqs[k], k

    def _harmonic_snr(self, power, freq, global_floor):
        """SNR (dB) de un armónico esperado en `freq`: pico local sobre el piso
        de ruido. El piso es el MÁXIMO entre la mediana del ruido vecino y un
        PISO GLOBAL robusto (mediana del espectro completo).

        Por qué el piso global: con una sola línea fuerte (tono puro), las colas
        de fuga (leakage) de la ventana caen sobre las posiciones 2f, 3f… con un
        nivel pequeño PERO mayor que la mediana local (que ahí es ~0), fingiendo
        un peine. Exigir además que el pico supere el piso global del espectro
        descarta esa fuga: una línea de fuga está MUY por debajo del piso global,
        mientras que un armónico real del dron lo supera holgadamente."""
        b = int(round(freq / self.df))
        if b <= 0 or b >= len(power):
            return -np.inf, 0.0
        t = self.tol_bins
        a0, a1 = max(0, b - t), min(len(power), b + t + 1)
        peak = power[a0:a1].max()
        # Ruido local: ventana alrededor, sacando la zona del pico
        w = self.noise_halfwidth
        n0, n1 = max(0, b - w), min(len(power), b + w + 1)
        local = power[n0:n1].copy()
        pa, pb = max(0, (b - t) - n0), min(len(local), (b + t + 1) - n0)
        local[pa:pb] = np.nan
        local_noise = np.nanmedian(local)
        if not np.isfinite(local_noise):
            local_noise = 0.0
        noise = max(local_noise, global_floor)
        if noise <= 0:
            return -np.inf, peak
        return 10.0 * np.log10(peak / noise), peak

    def update(self, frame):
        """
        Acumula `frame` (un hop) y reevalúa la firma armónica.

        Args:
            frame: ndarray (hop, M) multicanal o (hop,) mono, normalizado [-1,1].

        Returns:
            SpectralResult con el veredicto (is_drone, ya con histéresis) y los
            diagnósticos del peine.
        """
        self._push(frame)
        ready = self._filled >= self.window_size

        power = self._spectrum()
        bpf, k = self._estimate_bpf(power)

        harmonics = []
        n_detected = 0
        n_in_band  = 0
        harm_power = 0.0
        # Piso global robusto: mediana del espectro completo (sin DC). Domina los
        # bins de ruido (la mayoría), así que es ~el nivel de ruido de banda ancha.
        noise_ref  = float(np.median(power)) + 1e-30
        # Energía total en la banda de interés (para la fracción armónica).
        roi = (self.freqs >= self.bpf_min) & (self.freqs <= self.music_hi)
        total_roi = float(power[roi].sum()) + 1e-30

        if k > 0:
            for h in range(1, self.n_harmonics + 1):
                f = bpf * h
                if f >= self.fs / 2:
                    break
                snr_db, peak = self._harmonic_snr(power, f, noise_ref)
                if snr_db >= self.snr_thr_db:
                    n_detected += 1
                    harm_power += peak
                    harmonics.append((h, float(f), float(snr_db)))
                    if self.music_lo <= f <= self.music_hi:
                        n_in_band += 1

        # Harmonic-to-Noise Ratio global del peine detectado y fracción de energía
        hnr_db = 10.0 * np.log10(harm_power / noise_ref) if harm_power > 0 else -np.inf
        # NOTA (sesgo del estimador): harm_power suma solo el BIN PICO de cada
        # armónico, mientras total_roi integra TODOS los bins de la banda; con
        # ventana de Hann la energía de un tono se reparte en ~3 bins, así que
        # la fracción SUBESTIMA ~2x la energía real del peine. Es
        # autoconsistente porque harmonic_fraction_min se calibró con este
        # mismo estimador — si se cambia la ventana o window_size, RECALIBRAR
        # la fracción mínima (config.SPECTRAL_HARMONIC_FRACTION_MIN).
        harm_fraction = harm_power / total_roi

        # Corrida más larga de armónicos CONSECUTIVOS (h, h+1, h+2, …) con SNR.
        # Un peine de dron es CONTIGUO; un tono puro produce, a lo sumo, su
        # fundamental más armónicos sueltos y espurios (ruido que cae justo en
        # k·BPF) que NO son consecutivos. Exigir una corrida consecutiva ≥
        # min_harmonics rechaza el tono puro sin castigar al dron (que mantiene
        # la corrida incluso si le falta la fundamental: arranca en h=2).
        present = set(h for h, _, _ in harmonics)
        run = best_run = 0
        for h in range(1, self.n_harmonics + 1):
            run = run + 1 if h in present else 0
            if run > best_run:
                best_run = run

        raw_is_drone = (
            ready
            and best_run    >= self.min_harm
            and n_in_band   >= self.min_harm_band
            and np.isfinite(hnr_db) and hnr_db >= self.score_min
            and harm_fraction >= self.frac_min
        )

        # Confirmación + histéresis: requiere `confirm_need` ventanas positivas
        # consecutivas para latchear; una vez confirmado, el veredicto se sostiene
        # `hold_frames` frames aunque alguno no valide (fluctuaciones del dron).
        if raw_is_drone:
            self._confirm_count += 1
            if self._confirm_count >= self.confirm_need:
                self._confirmed = True
                self._hold = self.hold_frames
        else:
            self._confirm_count = 0
            if self._hold > 0:
                self._hold -= 1
                if self._hold <= 0:
                    self._confirmed = False

        return SpectralResult(
            is_drone=self._confirmed, bpf=float(bpf), n_harmonics=n_detected,
            n_in_band=n_in_band, hnr_db=float(hnr_db) if np.isfinite(hnr_db) else 0.0,
            harmonics=harmonics, ready=ready, harm_fraction=float(harm_fraction),
            n_consecutive=int(best_run),
        )
