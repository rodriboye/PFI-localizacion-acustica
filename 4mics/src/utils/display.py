"""
display.py — Visualización en terminal del estado del sistema DOA.

Muestra:
    - Barra vertical de elevación  (rango: ELEVATION_MIN … ELEVATION_MAX de config.py)
    - Barra horizontal de azimut   (rango: AZIMUTH_MIN … AZIMUTH_MAX de config.py)
    - Barra de energía con los umbrales marcados
    - Estado del detector
    - Estadísticas de paquetes seriales
    - Info de servos (si están habilitados)
"""

import sys
import math
from config import AZIMUTH_MIN, AZIMUTH_MAX, ELEVATION_MIN, ELEVATION_MAX


def _clear():
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def _az_bar(az_deg, valid=True):
    """
    Barra horizontal de azimut.

    Rango : AZIMUTH_MIN … AZIMUTH_MAX  (de config.py)
    Ancho : 41 caracteres — impar para que el centro caiga exactamente en 0°.
    Ticks : 5 posiciones equidistantes (cols 0, 10, 20, 30, 40); sus etiquetas
            se calculan dinámicamente desde el rango de config.
    El marcador ▲ aparece bajo la posición estimada; el valor numérico se
    muestra en la fila siguiente centrado sobre el marcador.
    """
    AZ_MIN = float(AZIMUTH_MIN)
    AZ_MAX = float(AZIMUTH_MAX)
    W = 41
    TICK_COLS = [0, 10, 20, 30, 40]

    # --- posición del marcador ---
    if valid:
        pos = int(round((az_deg - AZ_MIN) / (AZ_MAX - AZ_MIN) * (W - 1)))
        pos = max(0, min(W - 1, pos))
    else:
        pos = None

    # --- etiquetas de ticks calculadas desde el rango real ---
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

    # --- bar con ticks ---
    bar = list('─' * W)
    for tc in TICK_COLS:
        bar[tc] = '┼'

    # --- fila del marcador y del valor ---
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


def _el_bar(el_deg, valid=True):
    """
    Barra vertical de elevación.

    Rango : ELEVATION_MIN … ELEVATION_MAX  (de config.py)
    Ticks : cada 20° desde ELEVATION_MAX, incluyendo SIEMPRE ELEVATION_MIN
            como último tick (la escala termina en el mínimo del rango,
            aunque el paso no divida exactamente al rango).
    El marcador ◄ aparece a la derecha del tick más cercano al valor estimado,
    seguido del valor numérico exacto.
    """
    EL_MIN = float(ELEVATION_MIN)
    EL_MAX = float(ELEVATION_MAX)
    TICK_STEP = 20
    # Ticks de mayor a menor (la barra muestra arriba = positivo). La escala
    # SIEMPRE termina en EL_MIN: con rango 0-70 da [70, 50, 30, 10, 0].
    ticks = list(range(int(EL_MAX), int(EL_MIN), -TICK_STEP))
    if not ticks or ticks[-1] != int(EL_MIN):
        ticks.append(int(EL_MIN))
    H = len(ticks)

    if valid:
        # Tick más cercano al valor (los ticks pueden no ser equiespaciados
        # en el extremo inferior; un mapeo lineal marcaría el tick equivocado).
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
           noise_floor, serial_stats=None, servo_az=None, servo_el=None):
    """Renderiza el estado completo del sistema en la terminal."""
    _clear()
    out = []

    out.append("═" * 52)
    out.append("  SISTEMA DOA — 4 Micrófonos INMP441 + MUSIC")
    out.append("═" * 52)

    valid = doa_result is not None and doa_result.valid

    # --- valores numéricos ---
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

    # --- barra de elevación ---
    out.extend(_el_bar(el, valid=valid))

    out.append("")

    # --- barra de azimut ---
    out.extend(_az_bar(az, valid=valid))

    out.append("")

    # --- barra de energía ---
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

    # --- estadísticas serial ---
    if serial_stats:
        total = serial_stats.get('received', 0) + serial_stats.get('lost', 0)
        loss_pct = 100 * serial_stats.get('lost', 0) / total if total > 0 else 0
        out.append(
            f"  Serial   : rx={serial_stats.get('received', 0)}  "
            f"lost={serial_stats.get('lost', 0)} ({loss_pct:.1f}%)  "
            f"corrupt={serial_stats.get('corrupt', 0)}"
        )

    # --- servos ---
    if servo_az is not None:
        out.append(f"  Servo El : {servo_el:5.1f}°   Az: {servo_az:5.1f}°")

    out.append("─" * 52)
    out.append("  Ctrl+C para salir")

    print('\n'.join(out))