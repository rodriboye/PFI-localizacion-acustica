#!/usr/bin/env python3
"""
check_capture.py — Diagnóstico rápido de una captura WAV de 4 canales.

Valida la SALUD DE LA CAPTURA antes de correr main.py / MUSIC. Pensado para
iterar cambios de cableado/firmware: corré esto sobre cada wav nuevo y mirá
si los 4 canales realmente comparten un frente de onda coherente.

Qué reporta:
  1. Por canal: DC, nivel (std), % de clipping.
  2. Reparto de energía por banda (<300 / 300-2000 / >2000 Hz). Si la mayoría
     está <300 Hz, hay basura sub-acústica (DC/wander) comiendo rango dinámico.
  3. Matriz de coherencia magnitud-cuadrado en la banda de análisis (300-2000 Hz).
     OJO: la coherencia es INVARIANTE al retardo fijo entre mics. Por eso un par
     de mics sincronizados —aunque tengan un offset de muestras— debe dar
     coherencia ALTA (~1). Una coherencia ~0 NO es por el retardo geométrico:
     indica que esos dos canales no comparten fase (clocks independientes o
     canal muerto). Esto es lo que más comúnmente rompe MUSIC.
  4. Confianza MUSIC esperada con los 4 mics y con cada subconjunto de 3,
     para aislar un canal culpable.

Criterio rápido de "captura sana":
  - Todos los pares de mics con coherencia > ~0.7 en banda.
  - Energía en 300-2000 Hz comparable o mayor a la de <300 Hz.
  - Confianza MUSIC mediana claramente > DOA_MIN_CONF_UPDATE (config.py).

Uso:
    python3 check_capture.py captura.wav
    python3 check_capture.py captura.wav --band 300 2000
"""

import sys
import os
import argparse
import wave
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as cfg
from src.processing.doa_engine import MUSICEngine

# Tilt del array: el motor escanea la elevacion en el marco del array
# [EL_LO, EL_HI] = [ELEVATION_MIN-tilt, ELEVATION_MAX-tilt]; la elevacion real
# reportada es la del array + tilt.
_TILT = float(getattr(cfg, 'ARRAY_TILT_DEG', 0.0))
EL_LO = cfg.ELEVATION_MIN - _TILT
EL_HI = cfg.ELEVATION_MAX - _TILT

try:
    from scipy.signal import coherence, welch
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False


def load_wav(path):
    w = wave.open(path, 'rb')
    ch = w.getnchannels()
    fs = w.getframerate()
    sw = w.getsampwidth()
    raw = w.readframes(w.getnframes())
    w.close()
    if sw != 2:
        raise ValueError(f"Se esperaban muestras int16 (2 bytes); el wav tiene {sw} bytes/muestra.")
    x = np.frombuffer(raw, dtype='<i2').reshape(-1, ch).astype(np.float64)
    return x, fs, ch


def energetic_segment(x, fs, win_s=1.0):
    """Devuelve ~win_s segundos centrados en el frame de mayor energía."""
    H = cfg.HOP_SIZE
    n = len(x) // H
    if n == 0:
        return x
    en = np.array([x[k * H:(k + 1) * H].std() for k in range(n)])
    kb = int(np.argmax(en))
    half = int(win_s * fs / 2)
    a = max(0, kb * H - half)
    b = min(len(x), kb * H + half)
    return x[a:b]


def per_channel(x):
    print("Por canal:")
    print("  canal    DC     std    clip%")
    for i in range(x.shape[1]):
        dc = x[:, i].mean()
        sd = x[:, i].std()
        clip = 100 * np.mean(np.abs(x[:, i]) >= 32760)
        flag = "  <-- revisar" if (clip > 0.5 or sd < 200 or abs(dc) > 8000) else ""
        print(f"  ch{i}   {dc:7.0f} {sd:7.0f}  {clip:5.2f}{flag}")


def band_energy(x, fs):
    if not HAVE_SCIPY:
        return
    xd = x - x.mean(0)
    f, P = welch(xd, fs=fs, nperseg=1024, axis=0)
    P = P.mean(1)
    tot = P.sum() + 1e-30
    lo = 100 * P[f < 300].sum() / tot
    mid = 100 * P[(f >= 300) & (f < 2000)].sum() / tot
    hi = 100 * P[f >= 2000].sum() / tot
    print(f"\nReparto de energia:  <300Hz={lo:.0f}%   300-2000Hz={mid:.0f}%   >2000Hz={hi:.0f}%")
    if lo > 50:
        print("  AVISO: mayoria de energia <300 Hz (basura sub-acustica / DC). "
              "Revisar high-pass o nivel de entrada.")


def coherence_matrix(x, fs, band):
    if not HAVE_SCIPY:
        print("\n(scipy no instalado; se omite la matriz de coherencia. "
              "pip install scipy --break-system-packages)")
        return
    seg = energetic_segment(x - x.mean(0), fs)
    M = x.shape[1]
    lo, hi = band
    mat = np.zeros((M, M))
    for i in range(M):
        for j in range(M):
            f, C = coherence(seg[:, i], seg[:, j], fs=fs, nperseg=256)
            m = (f >= lo) & (f < hi)
            mat[i, j] = C[m].mean()
    print(f"\nCoherencia magnitud-cuadrado en {lo}-{hi} Hz (1.0 = canales en fase):")
    print("       " + "".join(f"  ch{j}  " for j in range(M)))
    for i in range(M):
        print(f"  ch{i} " + "".join(f" {mat[i, j]:.3f}" for j in range(M)))
    # diagnostico de pares
    off = [(i, j, mat[i, j]) for i in range(M) for j in range(i + 1, M)]
    bad = [p for p in off if p[2] < 0.3]
    good = [p for p in off if p[2] > 0.7]
    if good:
        print("  Pares coherentes (>0.7): " +
              ", ".join(f"ch{i}-ch{j}({v:.2f})" for i, j, v in good))
    if bad:
        print("  Pares DESACOPLADOS (<0.3): " +
              ", ".join(f"ch{i}-ch{j}({v:.2f})" for i, j, v in bad))
        print("  -> coherencia baja NO es por geometria/retardo. Sospechar "
              "clocks I2S no compartidos o canal muerto.")


def _bandpass(x, fs, band):
    """Filtra a la banda de analisis. Si no hay scipy, devuelve x sin filtrar."""
    if not HAVE_SCIPY:
        return x - x.mean(0)
    from scipy.signal import butter, sosfiltfilt
    lo, hi = band
    sos = butter(4, [lo, hi], btype='band', fs=fs, output='sos')
    return sosfiltfilt(sos, x - x.mean(0), axis=0)


def _best_lag_corr(a, b, maxlag):
    """Maxima correlacion de Pearson entre a y b permitiendo un desplazamiento
    entero de hasta +-maxlag muestras. Devuelve (corr, lag).

    Por que con lag y no a lag-cero: el I2S estandar multiplexa L/R (los dos mics
    de un bus salen en mitades distintas del frame -> hasta media muestra de
    offset fijo), y ademas existe el TDOA acustico (hasta ~1.2 muestras a 5 cm).
    Esos retardos son FIJOS y reales; medir a lag-cero los castiga como si fuera
    desincronizacion. La correlacion al mejor lag es la metrica justa de 'fase'.
    """
    a = (a - a.mean()) / (a.std() + 1e-12)
    b = (b - b.mean()) / (b.std() + 1e-12)
    N = len(a)
    best = (0.0, 0)
    for L in range(-maxlag, maxlag + 1):
        if L >= 0:
            c = np.dot(a[L:], b[:N - L]) / (N - L) if N - L > 10 else 0.0
        else:
            c = np.dot(a[:N + L], b[-L:]) / (N + L) if N + L > 10 else 0.0
        if abs(c) > abs(best[0]):
            best = (c, L)
    return best


def temporal_correlation(x, fs, band):
    """
    Correlacion temporal entre canales AL MEJOR LAG (no a lag-cero), en banda,
    sobre el segmento energetico. Es una metrica de 'fase compartida' robusta a
    los retardos fijos del I2S (multiplexado L/R) y al TDOA acustico.

    ~+0.9 al mejor lag = misma fuente, bien sincronizados.
    ~0   incluso al mejor lag = senales independientes (sin frente comun).
    OJO: esta es complementaria a la coherencia (delay-invariante) y a la
    consistencia de retardos; el veredicto se basa en esas, no en lag-cero.

    Retorna la correlacion media (al mejor lag) entre todos los pares.
    """
    M = x.shape[1]
    seg = energetic_segment(x, fs)
    segb = _bandpass(seg, fs, band)
    # ventana de lags: cubre TDOA del array + offset L/R del I2S, con margen
    maxlag = int(np.ceil(cfg.MIC_DISTANCE * np.sqrt(2) / cfg.SPEED_OF_SOUND * fs)) + 3

    C0 = np.corrcoef(segb.T)                       # lag-cero (referencia)
    Cbest = np.eye(M); Lbest = np.zeros((M, M), int)
    for i in range(M):
        for j in range(i + 1, M):
            c, L = _best_lag_corr(segb[:, i], segb[:, j], maxlag)
            Cbest[i, j] = Cbest[j, i] = c
            Lbest[i, j] = L; Lbest[j, i] = -L

    print(f"\nCorrelacion TEMPORAL al MEJOR LAG en {int(band[0])}-{int(band[1])} Hz"
          f"  (ventana +-{maxlag} muestras)")
    print(" (~+0.9 = misma fuente sincronizada; robusta a offsets fijos del I2S/TDOA):")
    print("       " + "".join(f"  ch{j}  " for j in range(M)))
    for i in range(M):
        print(f"  ch{i} " + "".join(f" {Cbest[i, j]:+.3f}" for j in range(M)))
    print("  lag del pico (muestras):")
    for i in range(M):
        print(f"  ch{i} " + "".join(f" {Lbest[i, j]:+4d} " for j in range(M)))

    off = [abs(Cbest[i, j]) for i in range(M) for j in range(i + 1, M)]
    off0 = [abs(C0[i, j]) for i in range(M) for j in range(i + 1, M)]
    mean_off = float(np.mean(off))
    print(f"  media |corr| al mejor lag = {mean_off:.2f}  (a lag-cero seria "
          f"{np.mean(off0):.2f} — la diferencia es el efecto de los retardos fijos)")
    return mean_off


def music_conf(x, fs, mics):
    H = cfg.HOP_SIZE
    pos = cfg.MIC_POSITIONS[mics]
    eng = MUSICEngine(
        pos, cfg.SAMPLE_RATE, H, cfg.FREQ_MIN, cfg.FREQ_MAX,
        cfg.SPEED_OF_SOUND, cfg.ANGLE_RESOLUTION, cfg.NUM_SOURCES,
        cfg.COV_ALPHA, cfg.DIAGONAL_LOADING,
        (cfg.AZIMUTH_MIN, cfg.AZIMUTH_MAX), (EL_LO, EL_HI))
    xn = x / 32768.0
    confs, az, el = [], [], []
    thr = cfg.DOA_MIN_CONF_UPDATE
    for k in range(len(xn) // H):
        r = eng.process(xn[k * H:(k + 1) * H, mics])
        confs.append(r.confidence)
        if r.confidence > thr:
            az.append(r.azimuth)
            el.append(r.elevation + _TILT)        # marco array -> real
    confs = np.array(confs)
    out = (f"  mics {mics}: conf med {np.median(confs):.2f}  max {confs.max():.2f} dB"
           f"  | frames>{thr}: {(confs > thr).sum()}/{len(confs)}")
    if az:
        out += f"  | az {np.mean(az):+.0f}+-{np.std(az):.0f}  el {np.mean(el):+.0f}+-{np.std(el):.0f}"
    print(out)
    return float(confs.max()) if len(confs) else 0.0


def _gcc_lag(a, b, maxlag=8):
    """Retardo GCC-PHAT entre a y b con refinamiento parabolico sub-muestra.
    Devuelve (lag_muestras, nitidez_pico). nitidez alta = retardo confiable."""
    N = 1
    while N < 2 * len(a):
        N <<= 1
    A = np.fft.rfft(a * np.hanning(len(a)), N)
    B = np.fft.rfft(b * np.hanning(len(b)), N)
    R = A * np.conj(B)
    R /= np.abs(R) + 1e-9
    cc = np.fft.fftshift(np.fft.irfft(R, N))
    mid = N // 2
    win = cc[mid - maxlag: mid + maxlag + 1]
    k = int(np.argmax(win))
    sharp = win.max() / (np.sqrt(np.mean(win ** 2)) + 1e-12)
    if 0 < k < len(win) - 1:
        y0, y1, y2 = win[k - 1], win[k], win[k + 1]
        denom = (y0 - 2 * y1 + y2)
        off = 0.5 * (y0 - y2) / denom if abs(denom) > 1e-12 else 0.0
    else:
        off = 0.0
    return (k - maxlag + off), sharp


def delay_consistency(x, fs, band, buses):
    """
    Mide si los retardos entre pares de mics concuerdan con UNA sola direccion
    de fuente. Es el mejor predictor de "esta captura se puede localizar":
    mejor que la correlacion, porque la correlacion puede ser moderada y aun asi
    los retardos apuntar todos al mismo lado (localizable), o al reves.

    Para cada par calcula el retardo por GCC-PHAT (sub-muestra) sobre los frames
    mas energeticos, y ajusta (az, el) + un offset fijo por CRUCE DE BUS que
    minimiza el residuo. El offset de bus modela el desfase constante master-slave.

      RMS chico  (< ~0.15 muestras) -> retardos consistentes: localizable.
      RMS grande (> ~0.3 muestras)  -> reverberacion/multipath o SNR marginal;
                                       ningun metodo (MUSIC, SRP-PHAT) localiza.

    El offset_bus se ajusta solo como parametro de estorbo (nuisance) del fit:
    absorbe un eventual desfase constante master-slave para no contaminar el
    residuo. Es DIAGNOSTICO, no calibracion: el pipeline no aplica compensacion
    de offsets (el firmware sync comparte clock entre buses y el offset
    esperado es ~0; si aca da grande, revisar firmware/cableado de clocks).
    """
    if not HAVE_SCIPY:
        print("\n(scipy no instalado; se omite la consistencia de retardos.)")
        return None
    from scipy.signal import butter, sosfiltfilt
    lo, hi = band
    sos = butter(4, [lo, hi], btype='band', fs=fs, output='sos')
    xb = sosfiltfilt(sos, x - x.mean(0), axis=0)
    M = x.shape[1]
    H = cfg.HOP_SIZE
    n = len(xb) // H
    if n < 2:
        return None
    en = np.array([xb[k * H:(k + 1) * H].std() for k in range(n)])
    thr = np.median(en) * 3
    frames = [k for k in range(n) if en[k] > thr]
    if len(frames) < 2:                       # sin evento claro: top-8 energeticos
        frames = list(np.argsort(en)[::-1][:8])

    pairs = [(i, j) for i in range(M) for j in range(i + 1, M)]
    bus_of = {}
    for b, grp in enumerate(buses):
        for cmic in grp:
            bus_of[cmic] = b
    iscross = np.array([1.0 if bus_of.get(i) != bus_of.get(j) else 0.0
                        for (i, j) in pairs])

    meas = np.full(len(pairs), np.nan)
    for p, (i, j) in enumerate(pairs):
        lags = [l for l, s in
                (_gcc_lag(xb[k * H:(k + 1) * H, i], xb[k * H:(k + 1) * H, j])
                 for k in frames) if s > 2.5]
        if lags:
            meas[p] = np.median(lags)
    valid = ~np.isnan(meas)
    if valid.sum() < 3:
        print("\nConsistencia de retardos: pocos picos nitidos (senal debil "
              "frente al ruido). Necesita una fuente mas fuerte/cercana.")
        return None

    mic = cfg.MIC_POSITIONS
    dpos = np.array([mic[i] - mic[j] for (i, j) in pairs])
    az = np.radians(np.arange(cfg.AZIMUTH_MIN, cfg.AZIMUTH_MAX + 1, 2.0))
    el = np.radians(np.arange(EL_LO, EL_HI + 1, 2.0))   # marco del array
    AZ, EL = np.meshgrid(az, el, indexing='ij')
    dhat = np.stack([np.sin(AZ) * np.cos(EL), np.cos(AZ) * np.cos(EL),
                     np.sin(EL)], -1).reshape(-1, 3)
    base = (dpos @ dhat.T) / cfg.SPEED_OF_SOUND * fs            # (P, G)
    best = (1e18, 0, 0.0)
    for off in np.arange(-2.0, 2.01, 0.05):
        model = base + iscross[:, None] * off
        r = np.sum((model[valid] - meas[valid][:, None]) ** 2, axis=0)
        g = int(np.argmin(r))
        if r[g] < best[0]:
            best = (r[g], g, off)
    ssr, g, off = best
    rms = np.sqrt(ssr / valid.sum())
    a = np.degrees(AZ.reshape(-1)[g])
    e = np.degrees(EL.reshape(-1)[g]) + _TILT        # marco array -> real
    print(f"\nConsistencia de retardos ({len(frames)} frames del evento, "
          f"{int(valid.sum())}/{len(pairs)} pares con pico nitido):")
    print(f"  Mejor direccion: az={a:+.0f} el={e:+.0f}  "
          f"offset_bus={off:+.2f} muestras")
    print(f"  RMS residuo = {rms:.3f} muestras ({rms / fs * 1e6:.0f} us)  "
          f"[<0.15 localizable | >0.3 reverb/SNR]")
    return rms


def main():
    ap = argparse.ArgumentParser(description="Diagnostico de captura WAV 4 canales para DOA/MUSIC")
    ap.add_argument('wav')
    ap.add_argument('--band', type=float, nargs=2, default=[cfg.FREQ_MIN, cfg.FREQ_MAX],
                    metavar=('LO', 'HI'), help="Banda para coherencia/correlacion (Hz)")
    ap.add_argument('--skip', type=float, default=0.0, metavar='SEC',
                    help="Descartar los primeros SEC segundos (transitorio de encendido)")
    ap.add_argument('--buses', default='0,1/2,3', metavar='G0/G1',
                    help="Agrupamiento de canales por bus I2S, p.ej. '0,1/2,3'. "
                         "Usado para estimar el offset constante entre buses.")
    args = ap.parse_args()
    buses = [[int(c) for c in grp.split(',')] for grp in args.buses.split('/')]

    x, fs, ch = load_wav(args.wav)
    dur0 = len(x) / fs
    if args.skip > 0:
        x = x[int(args.skip * fs):]
    print("=" * 60)
    print(f"  {os.path.basename(args.wav)}  |  {ch} canales  fs={fs} Hz  dur={dur0:.2f}s"
          + (f"  (descartando {args.skip:.2f}s iniciales)" if args.skip > 0 else ""))
    print("=" * 60)

    per_channel(x)
    band_energy(x, fs)
    corr_off = temporal_correlation(x, fs, args.band)
    coherence_matrix(x, fs, args.band)
    rms = delay_consistency(x, fs, args.band, buses) if ch == 4 else None

    music_max = None
    if ch == 4:
        print(f"\nConfianza MUSIC (umbral tracker = {cfg.DOA_MIN_CONF_UPDATE} dB):")
        music_max = music_conf(x, fs, [0, 1, 2, 3])
        print("  -- aislando cada canal (3 mics) --")
        for drop in range(4):
            mics = [i for i in range(4) if i != drop]
            music_conf(x, fs, mics)

    # --- VEREDICTO automatico ---
    # La consistencia de retardos (rms) es el predictor mas directo de si la
    # captura se puede localizar; la correlacion es el respaldo cuando no hay
    # un evento claro para medir retardos.
    print("\n" + "=" * 60)
    print("  VEREDICTO")
    print("=" * 60)
    if rms is not None:
        if rms < 0.15:
            print(f"  [OK] Consistencia de retardos: RMS={rms:.3f} muestras (<0.15).")
            print("       Los retardos apuntan a una sola direccion: LOCALIZABLE.")
            if music_max is not None and music_max < cfg.DOA_MIN_CONF_UPDATE:
                print("       Si MUSIC aun no supera el umbral y offset_bus dio")
                print("       grande (>0.5 muestras), revisar firmware sync /")
                print("       cableado de clocks; si no, probar --engine srp.")
        elif rms < 0.30:
            print(f"  [DUDOSO] Consistencia de retardos: RMS={rms:.3f} (0.15-0.30).")
            print("       Direccion parcialmente consistente. Mejorar SNR/reverb:")
            print("       fuente mas fuerte y cercana, array lejos de superficies.")
        else:
            print(f"  [FALLA] Consistencia de retardos: RMS={rms:.3f} muestras (>0.30).")
            print("       Los retardos NO concuerdan con una sola direccion:")
            print("       reverberacion/multipath o SNR marginal. Ningun metodo")
            print("       (MUSIC ni SRP-PHAT) localiza esto. Grabar en ambiente seco,")
            print("       fuente impulsiva fuerte a 15-30 cm, array fuera de la mesa.")
        print(f"       (correlacion temporal media de respaldo = {corr_off:.2f})")
    elif corr_off >= 0.7:
        print(f"  [OK] Correlacion temporal media entre pares = {corr_off:.2f} (>=0.70).")
        print("       Los mics captan una fuente comun: el array esta sano.")
    elif corr_off >= 0.3:
        print(f"  [DUDOSO] Correlacion temporal media = {corr_off:.2f} (0.30-0.70).")
        print("       Fuente comun debil o campo reverberante. Probar fuente "
              "mas fuerte y cercana, ambiente menos reverberante.")
    else:
        print(f"  [FALLA] Correlacion temporal media = {corr_off:.2f} (<0.30).")
        print("       Sin evento claro y sin frente de onda comun. Repetir con")
        print("       una palmada fuerte a ~30 cm en ambiente seco.")
    print("=" * 60)


if __name__ == '__main__':
    main()
