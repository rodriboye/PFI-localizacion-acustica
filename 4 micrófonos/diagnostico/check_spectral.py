#!/usr/bin/env python3
"""
check_spectral.py — ¿Este WAV pasa el detector espectral armónico?

Corre el MISMO gate que usa el sistema en vivo (HarmonicDroneGate, el "gate 2"
de la cascada: confirma la firma de dron por su peine armónico) sobre un archivo
WAV de 1 o más canales, ventana por ventana, y da un veredicto:

    ✓ PASA  → el audio tiene la estructura armónica de un dron (lo confirmaría).
    ✗ NO PASA → no la tiene (viento, voz, ruido, tono puro, silencio…).

Cuando NO pasa, muestra el desglose de cada criterio en la mejor ventana, para
saber QUÉ falló (faltan armónicos, poca fracción de energía, HNR bajo, etc.).

Usa los parámetros de config.py (sección GATE ESPECTRAL), así que el veredicto
coincide con el del sistema real. Se pueden sobreescribir por CLI para tantear.

Uso:
    python3 check_spectral.py grabacion.wav
    python3 check_spectral.py dron.wav --bpf-min 100 --bpf-max 350
    python3 check_spectral.py dron.wav --plot                 # guarda PNG diagnóstico
    python3 check_spectral.py dron.wav --verbose               # traza por ventana

Nota: este script evalúa SOLO el gate espectral, sobre todo el archivo (no lo
condiciona el gate de energía). Para validar la salud de la captura (coherencia
entre canales, localizabilidad) usá check_capture.py.
"""

import sys
import os
import argparse
import wave
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as cfg
from src.processing.spectral_gate import HarmonicDroneGate


# --- colores (se desactivan si la salida no es una terminal) --------------
_TTY = sys.stdout.isatty()
def _c(code, s): return f"\033[{code}m{s}\033[0m" if _TTY else s
def ok(s):   return _c("32", s)   # verde
def bad(s):  return _c("31", s)   # rojo
def warn(s): return _c("33", s)   # amarillo
def bold(s): return _c("1", s)


def load_wav(path):
    """Lee un WAV PCM (8/16/32-bit, 1+ canales) → (x float64 [-1,1), fs, ch)."""
    w = wave.open(path, 'rb')
    ch = w.getnchannels()
    fs = w.getframerate()
    sw = w.getsampwidth()
    raw = w.readframes(w.getnframes())
    w.close()
    if sw == 1:      # PCM unsigned 8-bit
        x = (np.frombuffer(raw, dtype=np.uint8).astype(np.float64) - 128.0) / 128.0
    elif sw == 2:    # PCM signed 16-bit (lo normal en este proyecto)
        x = np.frombuffer(raw, dtype='<i2').astype(np.float64) / 32768.0
    elif sw == 4:    # PCM signed 32-bit
        x = np.frombuffer(raw, dtype='<i4').astype(np.float64) / 2147483648.0
    else:
        raise ValueError(f"Ancho de muestra no soportado: {sw} bytes.")
    x = x.reshape(-1, ch)
    return x, fs, ch


def build_gate(fs, args):
    """Instancia el gate con los parámetros de config.py, permitiendo overrides
    por CLI. fs se toma del WAV (las bandas están en Hz absolutos)."""
    return HarmonicDroneGate(
        sample_rate           = fs,
        window_size           = cfg.SPECTRAL_WINDOW,
        hop_size              = cfg.HOP_SIZE,
        bpf_min               = args.bpf_min if args.bpf_min is not None else cfg.SPECTRAL_BPF_MIN,
        bpf_max               = args.bpf_max if args.bpf_max is not None else cfg.SPECTRAL_BPF_MAX,
        n_harmonics           = cfg.SPECTRAL_N_HARMONICS,
        hps_downsample        = cfg.SPECTRAL_HPS_DOWNSAMPLE,
        music_band            = (cfg.SPECTRAL_MUSIC_BAND_LO, cfg.SPECTRAL_MUSIC_BAND_HI),
        harmonic_snr_db       = args.snr if args.snr is not None else cfg.SPECTRAL_HARMONIC_SNR_DB,
        min_harmonics         = args.min_harmonics if args.min_harmonics is not None else cfg.SPECTRAL_MIN_HARMONICS,
        min_harmonics_in_band = cfg.SPECTRAL_MIN_HARMONICS_IN_BAND,
        score_min             = args.score_min if args.score_min is not None else cfg.SPECTRAL_SCORE_MIN,
        harmonic_tol_hz       = cfg.SPECTRAL_HARMONIC_TOL_HZ,
        hold_frames           = cfg.SPECTRAL_HOLD_FRAMES,
        harmonic_fraction_min = args.frac_min if args.frac_min is not None else cfg.SPECTRAL_HARMONIC_FRACTION_MIN,
        confirm_windows       = cfg.SPECTRAL_CONFIRM_WINDOWS,
    )


def crit(label, value, fmt, thr, comp):
    """Imprime una línea de criterio con ✓/✗. comp: '>=' (value>=thr)."""
    passed = value >= thr if comp == '>=' else value <= thr
    mark = ok("✓") if passed else bad("✗")
    vtxt = fmt.format(value)
    print(f"    {label:<22}: {vtxt:<14} (mín {thr})   {mark}")
    return passed


def main():
    p = argparse.ArgumentParser(
        description="¿Un WAV pasa el detector espectral armónico (gate de dron)?")
    p.add_argument('wav', help="Archivo WAV a evaluar")
    p.add_argument('--bpf-min', type=float, default=None, dest='bpf_min',
                   help=f"Override banda BPF mín (config: {cfg.SPECTRAL_BPF_MIN} Hz)")
    p.add_argument('--bpf-max', type=float, default=None, dest='bpf_max',
                   help=f"Override banda BPF máx (config: {cfg.SPECTRAL_BPF_MAX} Hz)")
    p.add_argument('--snr', type=float, default=None,
                   help=f"Override SNR mín por armónico dB (config: {cfg.SPECTRAL_HARMONIC_SNR_DB})")
    p.add_argument('--min-harmonics', type=int, default=None, dest='min_harmonics',
                   help=f"Override armónicos requeridos (config: {cfg.SPECTRAL_MIN_HARMONICS})")
    p.add_argument('--score-min', type=float, default=None, dest='score_min',
                   help=f"Override HNR mín dB (config: {cfg.SPECTRAL_SCORE_MIN})")
    p.add_argument('--frac-min', type=float, default=None, dest='frac_min',
                   help=f"Override fracción armónica mín (config: {cfg.SPECTRAL_HARMONIC_FRACTION_MIN})")
    p.add_argument('--verbose', action='store_true', help="Traza por ventana")
    p.add_argument('--plot', action='store_true',
                   help="Guarda un PNG (PSD + HPS) de la mejor ventana")
    args = p.parse_args()

    if not os.path.exists(args.wav):
        print(bad(f"✗ No existe: {args.wav}"), file=sys.stderr); sys.exit(1)

    try:
        x, fs, ch = load_wav(args.wav)
    except Exception as e:
        print(bad(f"✗ No se pudo leer el WAV: {e}"), file=sys.stderr); sys.exit(1)

    dur = len(x) / fs
    H = cfg.HOP_SIZE
    W = cfg.SPECTRAL_WINDOW

    print("=" * 64)
    print(f"  {bold(os.path.basename(args.wav))}  |  {ch} canal(es)  fs={fs} Hz  dur={dur:.2f}s")
    print("=" * 64)
    if fs != cfg.SAMPLE_RATE:
        print(warn(f"  Aviso: fs del WAV ({fs}) != config.SAMPLE_RATE ({cfg.SAMPLE_RATE}). "
                   f"Se usa el fs del WAV para el análisis."))
    if len(x) < W:
        print(bad(f"✗ Archivo muy corto: {len(x)} muestras < ventana {W}. "
                  f"Necesita al menos {W/fs:.2f}s."), file=sys.stderr)
        sys.exit(1)

    gate = build_gate(fs, args)
    print(f"  Gate (config.py): BPF {gate.bpf_min:.0f}-{gate.bpf_max:.0f} Hz, "
          f">={gate.min_harm} armónicos, >={gate.min_harm_band} en banda MUSIC "
          f"[{gate.music_lo:.0f}-{gate.music_hi:.0f}], SNR/arm>={gate.snr_thr_db:.0f} dB, "
          f"HNR>={gate.score_min:.0f} dB, fracción>={gate.frac_min:.2f}, "
          f"ventana {W} ({W/fs*1000:.0f} ms), confirma {gate.confirm_need} ventanas")
    print()

    n_frames = len(x) // H
    any_confirmed = False
    first_confirm_t = None
    n_conf = 0
    best_any = None      # ventana ready con mayor fracción (si nunca confirma)
    best_conf = None     # ventana que confirmó con mayor corrida consecutiva
    best_any_t = best_conf_t = 0.0

    for k in range(n_frames):
        frame = x[k * H:(k + 1) * H, :]
        if frame.shape[0] < H:
            break
        res = gate.update(frame)
        t = (k * H) / fs
        if res.ready and (best_any is None or res.harm_fraction > best_any.harm_fraction):
            best_any, best_any_t = res, t
        if res.is_drone:
            n_conf += 1
            if best_conf is None or res.n_consecutive > best_conf.n_consecutive:
                best_conf, best_conf_t = res, t
            if not any_confirmed:
                any_confirmed = True
                first_confirm_t = t
        if args.verbose and res.ready:
            flag = ok("DRON") if res.is_drone else "    "
            print(f"    t={t:5.2f}s  {flag}  BPF={res.bpf:6.1f}  cons={res.n_consecutive} "
                  f"n={res.n_harmonics} (banda {res.n_in_band})  HNR={res.hnr_db:5.1f}  "
                  f"frac={res.harm_fraction:.2f}")

    # --- desglose de criterios en la ventana representativa ---
    # Si confirmó, se muestra la ventana confirmante con mayor corrida (coherente
    # con el veredicto); si no, la de mayor fracción (la "más cerca" de pasar).
    if best_conf is not None:
        best, best_t, etiqueta = best_conf, best_conf_t, "ventana confirmante"
    else:
        best, best_t, etiqueta = best_any, best_any_t, "mejor ventana (máx fracción)"
    print(f"\n  {bold('Desglose')} — {etiqueta}, t={best_t:.2f}s:")
    print(f"    BPF estimada          : {best.bpf:.0f} Hz   "
          f"(armónicos con SNR: {best.n_harmonics})")
    c1 = crit("armónicos consecutivos", best.n_consecutive, "{:d}", gate.min_harm, '>=')
    c2 = crit("en banda MUSIC", best.n_in_band, "{:d}", gate.min_harm_band, '>=')
    c3 = crit("HNR global (dB)", best.hnr_db, "{:.1f}", gate.score_min, '>=')
    c4 = crit("fracción armónica", best.harm_fraction, "{:.2f}", gate.frac_min, '>=')
    all_crit = c1 and c2 and c3 and c4

    print(f"\n  Frames que confirmaron: {n_conf}/{n_frames} "
          f"({100*n_conf/max(n_frames,1):.0f}%)")
    if first_confirm_t is not None:
        print(f"  Primera confirmación  : t={first_confirm_t:.2f}s")

    if args.plot:
        _save_plot(gate, x, H, best_t, args.wav)

    # --- veredicto ---
    print("\n" + "=" * 64)
    if any_confirmed:
        print("  " + ok(bold("✓ PASA el detector espectral")) +
              "  — el audio tiene firma armónica de dron.")
    else:
        print("  " + bad(bold("✗ NO PASA el detector espectral")) +
              "  — sin firma armónica de dron.")
        if all_crit:
            print(warn("    (la mejor ventana cumple los criterios pero no hubo "
                       f"{gate.confirm_need} ventanas consecutivas: señal intermitente.)"))
        else:
            faltan = []
            if not c1: faltan.append("pocos armónicos consecutivos")
            if not c2: faltan.append("ninguno en banda MUSIC")
            if not c3: faltan.append("HNR bajo")
            if not c4: faltan.append("poca fracción de energía en el peine")
            print(f"    Motivo: {', '.join(faltan)}.")
    print("=" * 64)

    sys.exit(0 if any_confirmed else 2)


def _save_plot(gate, x, H, best_t, wav_path):
    """Guarda PSD + HPS de la ventana en best_t (requiere matplotlib)."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print(warn("    (--plot: matplotlib no instalado; se omite el gráfico)"))
        return
    # reconstruir la ventana que terminaba en best_t
    fs = gate.fs
    end = int(round((best_t) * fs)) + H
    seg = x[max(0, end - gate.window_size):end, :].mean(axis=1)
    if len(seg) < gate.window_size:
        seg = np.pad(seg, (gate.window_size - len(seg), 0))
    win = np.hanning(gate.window_size)
    X = np.fft.rfft((seg - seg.mean()) * win)
    P = X.real**2 + X.imag**2
    freqs = np.fft.rfftfreq(gate.window_size, 1/fs)
    bpf, _ = gate._estimate_bpf(P)
    fig, ax = plt.subplots(figsize=(8, 3.2))
    ax.semilogy(freqs, P + 1e-12, lw=0.8)
    ax.axhline(np.median(P), color='r', ls='--', lw=1, label='piso global')
    for h in range(1, gate.n_harmonics + 1):
        f = bpf * h
        if f < fs / 2:
            ax.axvline(f, color='g', alpha=0.35, lw=0.8)
    ax.set_xlim(0, min(fs/2, gate.music_hi * 1.1))
    ax.set_xlabel("Frecuencia (Hz)"); ax.set_ylabel("PSD")
    ax.set_title(f"{os.path.basename(wav_path)} — ventana t={best_t:.2f}s, BPF≈{bpf:.0f} Hz")
    ax.legend(fontsize=8)
    out = os.path.splitext(wav_path)[0] + "_espectral.png"
    fig.savefig(out, bbox_inches='tight', dpi=130)
    print(f"    Gráfico guardado: {out}")


if __name__ == '__main__':
    main()
