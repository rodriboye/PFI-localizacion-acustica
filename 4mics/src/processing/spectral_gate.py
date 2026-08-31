"""
spectral_gate.py — Gate 2 de la cascada: confirma "es un dron" por su firma
armónica antes de habilitar localización, servo y registro.

El gate de energía dispara con cualquier sonido fuerte; un multirrotor además
tiene un peine de armónicos sobre la frecuencia de paso de pala
(BPF = N_palas x RPM/60, ~80-300 Hz). Solo se evalúa cuando ya hay actividad,
así que la FFT extra se paga únicamente cuando hay algo que analizar.

Por ventana:
  1. Últimas window_size muestras de la MEZCLA MONO (la dirección no importa
     para la firma y mezclar mejora el SNR).
  2. BPF por Harmonic Product Spectrum: se multiplica el espectro por versiones
     decimadas, los armónicos de una misma fundamental se refuerzan.
  3. SNR de cada armónico h*BPF contra el MÁXIMO entre la mediana local y un
     piso global. El piso global evita que las colas de fuga de un TONO PURO,
     que caen sobre 2f y 3f por encima de la mediana local, finjan un peine.
  4. Confirma si hay una corrida de min_harmonics armónicos CONSECUTIVOS sobre
     el umbral (un peine real es contiguo), min_harmonics_in_band dentro de la
     banda de MUSIC, HNR sobre score_min y fracción de energía sobre frac_min.
  5. confirm_windows ventanas positivas seguidas para latchear; hold_frames de
     histéresis, porque el peine parpadea por efecto de pala y multipath.

Solo numpy: corre en la RPi sin dependencias extra.
"""

import numpy as np


class SpectralResult:
    """Resultado de una evaluación del gate."""
    __slots__ = ['is_drone', 'bpf', 'n_harmonics', 'n_in_band',
                 'hnr_db', 'harmonics', 'ready', 'harm_fraction', 'n_consecutive']

    def __init__(self, is_drone=False, bpf=0.0, n_harmonics=0, n_in_band=0,
                 hnr_db=0.0, harmonics=None, ready=False, harm_fraction=0.0,
                 n_consecutive=0):
        self.is_drone      = is_drone     # veredicto, ya con histéresis
        self.bpf           = bpf          # Hz
        self.n_harmonics   = n_harmonics  # armónicos con SNR suficiente
        self.n_in_band     = n_in_band    # de esos, cuántos en la banda MUSIC
        self.hnr_db        = hnr_db       # Harmonic-to-Noise Ratio global
        self.harmonics     = harmonics or []  # (h, freq_Hz, snr_db)
        self.ready         = ready        # ventana llena de audio real
        self.harm_fraction = harm_fraction  # energía peine / energía de banda
        self.n_consecutive = n_consecutive  # corrida más larga


class HarmonicDroneGate:
    """
    Gate de confirmación de dron por estructura armónica.

        res = gate.update(frame)   # frame: (hop, M) o (hop,) mono
        if res.is_drone: ...

    Llamar reset() al volver al silencio para que el próximo evento no herede el
    veredicto del anterior.
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
        self.frac_min      = float(harmonic_fraction_min)
        self.confirm_need  = max(1, int(confirm_windows))
        self.hold_frames   = int(hold_frames)

        self.df       = self.fs / self.window_size          # Hz por bin
        # Hann PERIÓDICA, la convención de doa_engine.py y detector.py
        # (np.hanning devuelve la simétrica, que sesga el análisis por FFT).
        self.window   = np.hanning(self.window_size + 1)[:-1]
        self.freqs    = np.fft.rfftfreq(self.window_size, d=1.0 / self.fs)
        # Tolerancia por armónico: la BPF no es estacionaria (las RPM varían).
        self.tol_bins = max(1, int(round(harmonic_tol_hz / self.df)))
        self.noise_halfwidth = max(self.tol_bins * 4, int(round(60.0 / self.df)))

        # _off_peak: bins donde se busca el pico. _off_noise: ventana de ruido
        # EXCLUYENDO la zona del pico. Separarlos permite np.median en vez de
        # np.nanmedian, cuyo overhead por llamada domina el costo del gate.
        self._off_peak  = np.arange(-self.tol_bins, self.tol_bins + 1)
        self._off_noise = np.concatenate([
            np.arange(-self.noise_halfwidth, -self.tol_bins),
            np.arange(self.tol_bins + 1, self.noise_halfwidth + 1),
        ])

        self._bpf_lo_bin = max(1, int(np.floor(self.bpf_min / self.df)))
        self._bpf_hi_bin = int(np.ceil(self.bpf_max / self.df))

        self._buf      = np.zeros(self.window_size, dtype=np.float64)
        self._filled   = 0          # muestras reales acumuladas
        self._hold     = 0          # frames restantes del latch
        self._confirmed = False
        self._confirm_count = 0     # ventanas positivas consecutivas

    # ------------------------------------------------------------------ utils
    def reset(self):
        """Limpia la ventana de audio y el latch de histéresis."""
        self._buf.fill(0.0)
        self._filled    = 0
        self._hold      = 0
        self._confirmed = False
        self._confirm_count = 0

    @property
    def confirmed(self):
        return self._confirmed

    def idle_tick(self):
        """Decae el latch en frames sin actividad: el veredicto no debe
        sobrevivir más de hold_frames sin evidencia. Una vez por frame idle."""
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
        """Potencia por bin de la ventana actual, sin DC."""
        x = self._buf - self._buf.mean()
        X = np.fft.rfft(x * self.window)
        return (X.real ** 2 + X.imag ** 2)

    def _estimate_bpf(self, power):
        """BPF por Harmonic Product Spectrum -> (bpf_hz, bin), (0.0, -1) si no
        hay candidato. En log, para que el producto sea suma y ningún bin domine."""
        mag = np.sqrt(power)
        hps = np.log(mag + 1e-12).copy()
        L = len(mag)
        for d in range(2, self.hps_down + 1):
            dec = mag[::d]
            hps[:len(dec)] += np.log(dec + 1e-12)
        lo, hi = self._bpf_lo_bin, min(self._bpf_hi_bin, L - 1)
        if hi <= lo:
            return 0.0, -1
        seg = hps[lo:hi + 1]
        k = int(np.argmax(seg)) + lo
        return self.freqs[k], k

    def update(self, frame):
        """Acumula un hop y reevalúa la firma armónica -> SpectralResult."""
        self._push(frame)
        ready = self._filled >= self.window_size

        power = self._spectrum()
        bpf, k = self._estimate_bpf(power)

        harmonics = []
        n_detected = 0
        n_in_band  = 0
        harm_power = 0.0
        # Piso global: la mediana del espectro está dominada por los bins de
        # ruido, que son la mayoría.
        noise_ref  = float(np.median(power)) + 1e-30
        roi = (self.freqs >= self.bpf_min) & (self.freqs <= self.music_hi)
        total_roi = float(power[roi].sum()) + 1e-30

        if k > 0:
            # Los n_harmonics armónicos se evaluan de una sola vez indexando los
            # offsets pre-calculados: sin copia, sin NaN, una sola np.median.
            L  = power.shape[0]
            hs = np.arange(1, self.n_harmonics + 1)
            fh = bpf * hs
            keep = fh < (self.fs / 2.0)
            hs, fh = hs[keep], fh[keep]
            if hs.size:
                b = np.round(fh / self.df).astype(np.intp)
                i_pk = np.clip(b[:, None] + self._off_peak[None, :],  0, L - 1)
                i_no = np.clip(b[:, None] + self._off_noise[None, :], 0, L - 1)
                peaks = power[i_pk].max(axis=1)
                noise = np.maximum(np.median(power[i_no], axis=1), noise_ref)
                snr = 10.0 * np.log10(np.maximum(peaks, 1e-300) /
                                      np.maximum(noise, 1e-300))
                ok = snr >= self.snr_thr_db
                n_detected = int(ok.sum())
                harm_power = float(peaks[ok].sum())
                n_in_band  = int(((fh >= self.music_lo) &
                                  (fh <= self.music_hi) & ok).sum())
                harmonics = [(int(h), float(f), float(s))
                             for h, f, s in zip(hs[ok], fh[ok], snr[ok])]

        hnr_db = 10.0 * np.log10(harm_power / noise_ref) if harm_power > 0 else -np.inf
        # SESGO: harm_power suma solo el BIN PICO de cada armónico y total_roi
        # integra todos los bins; con Hann la energía de un tono se reparte en ~3
        # bins, así que la fracción subestima ~2x. Es autoconsistente porque
        # frac_min se calibró con este estimador: si cambia la ventana o
        # window_size, RECALIBRARLO.
        harm_fraction = harm_power / total_roi

        # Corrida más larga de armónicos CONSECUTIVOS. Un peine de dron es
        # contiguo; un tono puro da armónicos sueltos y espurios que no forman
        # corrida. El dron la mantiene aunque le falte la fundamental (arranca
        # en h=2).
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
