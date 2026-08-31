"""
display.py — Visualización en terminal del estado del sistema.

Redibuja en el lugar con códigos ANSI: barra vertical de elevación, barra
horizontal de azimut, barra de energía con los umbrales marcados, estado del
detector, estadísticas de la entrada y posición de los servos.

Los rangos angulares llegan por parámetro y NO se importan de config: ningún
módulo de src/ importa otro ni la config, y acá solo funcionaba porque main.py
inserta la raíz en sys.path.
"""

import sys
import math


def _clear():
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def _az_bar(az_deg, az_range, valid=True):
    """
    Barra horizontal de azimut, ancho 41 (impar, para que el centro caiga
    exactamente en 0°). Las etiquetas de los 5 ticks se calculan desde
    az_range. El marcador ▲ va bajo la posición estimada y el valor numérico
    en la fila siguiente, centrado sobre él.
    """
    AZ_MIN = float(az_range[0])
    AZ_MAX = float(az_range[1])
    W = 41
    TICK_COLS = [0, 10, 20, 30, 40]

    if valid:
        pos = int(round((az_deg - AZ_MIN) / (AZ_MAX - AZ_MIN) * (W - 1)))
        pos = max(0, min(W - 1, pos))
    else:
        pos = None

    tick_defs = [
        (col, f"{round(AZ_MIN + (AZ_MAX - AZ_MIN) * col / (W - 1)):+d}°")
        for col in TICK_COLS
    ]
    lbl = list(' ' * (W + 6))
    for tp, tl in tick_defs:
        start = max(0, tp - len(tl) // 2)
        for i, c in enumerate(tl):
            if start + i < len(lbl):
                lbl[start + i] = c

    bar = list('─' * W)
    for tc in TICK_COLS:
        bar[tc] = '┼'

    arrow = list(' ' * W)
    val_row = list(' ' * (W + 6))
    if valid and pos is not None:
        arrow[pos] = '▲'
        val_str = f"{az_deg:+.1f}°"
        ls = max(0, min(W + 6 - len(val_str), pos - len(val_str) // 2))
        for i, c in enumerate(val_str):
            val_row[ls + i] = c

    p = "  "
    return [
        f"{p}Azimut:",
        f"{p}{''.join(lbl[:W + 4])}",
        f"{p}{''.join(bar)}",
        f"{p}{''.join(arrow)}",
        f"{p}{''.join(val_row[:W + 4])}",
    ]


def _el_bar(el_deg, el_range, valid=True):
    """
    Barra vertical de elevación (arriba = positivo), con ticks cada 20° desde
    el máximo e incluyendo SIEMPRE el mínimo como último, aunque el paso no
    divida exacto. El marcador ◄ va a la derecha del tick más cercano.
    """
    EL_MIN = float(el_range[0])
    EL_MAX = float(el_range[1])
    TICK_STEP = 20
    ticks = list(range(int(EL_MAX), int(EL_MIN), -TICK_STEP))
    if not ticks or ticks[-1] != int(EL_MIN):
        ticks.append(int(EL_MIN))
    H = len(ticks)

    if valid:
        # Tick más cercano al valor: los ticks pueden no ser equiespaciados en el
        # extremo inferior, y un mapeo lineal marcaría el equivocado.
        pos = min(range(H), key=lambda i: abs(ticks[i] - el_deg))
    else:
        pos = None

    lines = ["  Elevación:"]
    for i, t in enumerate(ticks):
        cap = '┐' if i == 0 else ('┘' if i == H - 1 else '┤')
        if valid and i == pos:
            lines.append(f"  {t:+4d}°  ─{cap}◄ {el_deg:+.1f}°")
        else:
            lines.append(f"  {t:+4d}°  ─{cap}")
    return lines


def render(doa_result, det_state, energy, threshold_event, threshold_silence,
           noise_floor, az_range, el_range, serial_stats=None,
           servo_az=None, servo_el=None, perf_stats=None):
    """Renderiza el estado completo del sistema en la terminal.

    az_range/el_range: (min, max) en grados, los mismos que escanea el motor."""
    _clear()
    out = []

    out.append("═" * 52)
    out.append("  SISTEMA DOA — 4 Micrófonos INMP441 + MUSIC")
    out.append("═" * 52)

    valid = doa_result is not None and doa_result.valid

    if valid:
        out.append(
            f"  El: {doa_result.elevation:+7.1f}°   "
            f"Az: {doa_result.azimuth:+7.1f}°   "
            f"Conf: {doa_result.confidence:.1f} dB"
        )
    else:
        out.append("  El:     ---     Az:     ---     Conf: ---")

    out.append("")

    az = doa_result.azimuth if valid else 0.0
    el = doa_result.elevation if valid else 0.0

    out.extend(_el_bar(el, el_range, valid=valid))

    out.append("")

    out.extend(_az_bar(az, az_range, valid=valid))

    out.append("")

    # --- barra de energía: escala fijada por el piso y el umbral de evento ---
    BAR_W = 30
    e_db     = 10 * math.log10(max(energy, 1e-12))
    ev_db    = 10 * math.log10(max(threshold_event, 1e-12))
    sil_db   = 10 * math.log10(max(threshold_silence, 1e-12))
    noise_db = 10 * math.log10(max(noise_floor, 1e-12))

    E_MIN, E_MAX = noise_db - 5, ev_db + 15
    rng = E_MAX - E_MIN
    fill     = max(0, min(BAR_W, int(BAR_W * (e_db  - E_MIN) / rng)))
    ev_mark  = max(0, min(BAR_W - 1, int(BAR_W * (ev_db  - E_MIN) / rng)))
    sil_mark = max(0, min(BAR_W - 1, int(BAR_W * (sil_db - E_MIN) / rng)))

    bar = ['░'] * BAR_W
    for i in range(fill):
        bar[i] = '█'
    bar[ev_mark]  = '▲'
    bar[sil_mark] = '△'

    out.append(f"  Energía  : {''.join(bar)}")
    out.append(f"             {e_db:+.1f} dB  [▲ ev: {ev_db:+.1f}  △ sil: {sil_db:+.1f}]")

    out.append("")

    # --- estado del detector ---
    state_str = det_state.name if hasattr(det_state, 'name') else str(det_state)
    state_colors = {
        'IDLE':     '\033[90m',   # gris
        'ONSET':    '\033[33m',   # amarillo
        'ACTIVE':   '\033[32m',   # verde
        'COOLDOWN': '\033[36m',   # cyan
    }
    color = state_colors.get(state_str, '')
    out.append(f"  Estado   : {color}{state_str}\033[0m")

    # --- estadísticas de la entrada ---
    if serial_stats:
        total = serial_stats.get('received', 0) + serial_stats.get('lost', 0)
        loss_pct = 100 * serial_stats.get('lost', 0) / total if total > 0 else 0
        out.append(
            f"  Entrada  : rx={serial_stats.get('received', 0)}  "
            f"lost={serial_stats.get('lost', 0)} ({loss_pct:.1f}%)  "
            f"corrupt={serial_stats.get('corrupt', 0)}"
        )

        # Contrapresión: frames leídos bien y tirados por cola llena. Es el modo
        # de falla que `lost` no puede ver, así que se marca en rojo apenas es
        # distinto de cero.
        if 'dropped' in serial_stats:
            drop  = serial_stats['dropped']
            dr    = 100.0 * serial_stats.get('drop_ratio', 0.0)
            color = '\033[31m' if drop > 0 else '\033[90m'
            out.append(
                f"  Descarte : {color}dropped={drop} ({dr:.1f}%)\033[0m  "
                f"cola_max={serial_stats.get('q_high', 0)}/"
                f"{serial_stats.get('q_max', 0)}"
            )

    # --- tiempos del lazo ---
    if perf_stats:
        budget = perf_stats.get('budget_ms')
        p50    = perf_stats.get('p50', 0.0)
        p95    = perf_stats.get('p95', 0.0)
        over   = perf_stats.get('over_pct')
        color  = ''
        if budget:
            color = '\033[31m' if p95 > budget else (
                    '\033[33m' if p95 > 0.7 * budget else '\033[32m')
        txt = f"  Lazo     : {color}p50={p50:.1f} ms  p95={p95:.1f} ms\033[0m"
        if budget:
            txt += f"  / {budget:.1f} ms"
        if over is not None:
            txt += f"  ({over:.0f}% excede)"
        out.append(txt)

    # --- servos ---
    if servo_az is not None:
        out.append(f"  Servo El : {servo_el:5.1f}°   Az: {servo_az:5.1f}°")

    out.append("─" * 52)
    out.append("  Ctrl+C para salir")

    print('\n'.join(out))
