"""
diagnose_serial.py — Diagnóstico de la cadena ESP32 → RPi por USB-serial.

Herramienta independiente: NO importa nada de src/. La idea es que sirva
incluso cuando algo de audio_input.py está roto y queremos aislar si el
problema es de hardware, de driver, de firmware o de software.

El script ejecuta los chequeos en orden de "menos invasivo" a "más invasivo":

    1. Sanity de entorno   — ¿existe el puerto? ¿permisos? ¿dmesg?
    2. Apertura serial     — ¿podemos abrir? ¿termios raw aplica?
    3. Tasa de bytes       — ¿está mandando algo el ESP32?
    4. Sincronización      — ¿aparece el SYNC_BYTE 0xAA con la cadencia esperada?
    5. Decodificación      — ¿son paquetes válidos? ¿counter incrementa?
                             Acumula TODOS los samples para análisis robusto.
    6. Calidad de canales  — sobre toda la ventana (no un solo frame):
                             RMS, peak global, DC offset, %samples saturados,
                             frames con clipping. Distingue saturación continua
                             (chip / wiring) de saturación esporádica (eventos).

Cada chequeo se reporta con ✓ / ✗ / ⚠ y una sugerencia de acción.

Uso:
    python3 diagnose_serial.py /dev/ttyUSB0
    python3 diagnose_serial.py /dev/ttyUSB0 --duration 5
    python3 diagnose_serial.py /dev/ttyUSB0 --baud 921600 --hop 512
"""

from __future__ import annotations

import argparse
import array
import os
import statistics
import struct
import sys
import termios
import time

try:
    import serial
except ImportError:
    print("✗ pyserial no instalado. Ejecutá: pip3 install pyserial", file=sys.stderr)
    sys.exit(2)

# Defaults — deben coincidir con config.py / firmware
# (11.025 kHz / 16-bit little-endian — ver config.py y los .ino)
DEFAULT_BAUD       = 921600
DEFAULT_HOP        = 256
DEFAULT_CHANNELS   = 4
DEFAULT_DURATION_S = 3.0
DEFAULT_FS         = 11025   # 16 bits/muestra a 11.025 kHz (ver firmware)
BYTES_PER_SAMPLE   = 2       # int16 little-endian
SYNC_BYTE          = 0xAA
END_BYTE           = 0x55


# -----------------------------------------------------------------------------
# Helpers de impresión
# -----------------------------------------------------------------------------

def _ok(msg):    print(f"  \033[32m✓\033[0m {msg}")
def _warn(msg):  print(f"  \033[33m⚠\033[0m {msg}")
def _bad(msg):   print(f"  \033[31m✗\033[0m {msg}")
def _info(msg):  print(f"    {msg}")
def _section(title):
    print()
    print(f"\033[1m=== {title} ===\033[0m")


# -----------------------------------------------------------------------------
# Chequeo 1: entorno
# -----------------------------------------------------------------------------

def check_environment(port):
    _section("1. Entorno")

    # Existencia del dispositivo
    if not os.path.exists(port):
        _bad(f"{port} no existe")
        _info("Probá: ls /dev/ttyUSB* /dev/ttyACM*")
        _info("Si no aparece nada: revisá cable USB / dmesg | tail -20")
        return False
    _ok(f"{port} existe")

    # Permisos de lectura
    if not os.access(port, os.R_OK | os.W_OK):
        _bad(f"Sin permisos R/W sobre {port}")
        _info(f"Agregate al grupo dialout: sudo usermod -aG dialout {os.environ.get('USER','$USER')}")
        _info("Después logout/login (o newgrp dialout)")
        return False
    _ok("Permisos R/W OK")

    # Otro proceso usándolo (best-effort, requiere fuser/lsof)
    try:
        import subprocess
        out = subprocess.run(['fuser', port], capture_output=True, text=True, timeout=2)
        if out.stdout.strip():
            _warn(f"Otros procesos están usando {port}: {out.stdout.strip()}")
            _info("Cerrá cualquier monitor serial / Arduino IDE / minicom abierto")
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass  # fuser no disponible, no es crítico

    return True


# -----------------------------------------------------------------------------
# Chequeo 2: apertura serial + termios raw
# -----------------------------------------------------------------------------

def open_serial(port, baud):
    _section("2. Apertura del puerto")

    try:
        ser = serial.Serial()
        ser.port     = port
        ser.baudrate = baud
        ser.timeout  = 2.0
        ser.open()
    except serial.SerialException as e:
        _bad(f"No se pudo abrir {port}: {e}")
        return None
    _ok(f"Abierto a {baud} baud")

    # Modo RAW por termios — crítico para datos binarios
    try:
        fd = ser.fileno()
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
        attrs[3] &= ~(termios.ECHO | termios.ECHOE | termios.ICANON)
        # Modo binario puro: sin traducciones de NL/CR
        attrs[0] &= ~(termios.INLCR | termios.IGNCR | termios.ICRNL)
        attrs[1] &= ~termios.OPOST  # sin post-procesado de salida
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        _ok("Modo RAW termios aplicado (sin XON/XOFF, sin echo, sin canon)")
    except Exception as e:
        _bad(f"No se pudo aplicar termios raw: {e}")
        ser.close()
        return None

    # Reset por RTS/DTR — reinicia el ESP32 a modo RUN (secuencia esptool:
    # EN←!RTS, GPIO0←!DTR; DTR bajo mantiene GPIO0 alto durante el pulso RTS.
    # Un toggle de DTR solo puede dejar el chip en bootloader según el adaptador)
    try:
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        time.sleep(0.7)  # esperar boot completo
        ser.reset_input_buffer()
        _ok("ESP32 reiniciado vía RTS/DTR")
    except Exception as e:
        _warn(f"RTS/DTR no soportado o falló: {e}")
        _info("No es crítico: probá apretar el botón EN del ESP32 manualmente")

    return ser


# -----------------------------------------------------------------------------
# Chequeo 3: tasa de bytes (sin parsear nada)
# -----------------------------------------------------------------------------

def measure_byte_rate(ser, duration_s, expected_byterate):
    _section(f"3. Tasa de bytes recibidos (ventana de {duration_s:.1f}s)")

    ser.reset_input_buffer()
    t0 = time.time()
    total_bytes = 0
    samples = []
    last_t = t0

    while time.time() - t0 < duration_s:
        n = ser.in_waiting
        if n > 0:
            ser.read(n)
            total_bytes += n
        time.sleep(0.05)
        # snapshot cada 0.5s
        now = time.time()
        if now - last_t >= 0.5:
            samples.append(total_bytes / (now - t0))
            last_t = now

    elapsed = time.time() - t0
    rate_kBs = (total_bytes / elapsed) / 1024.0
    expected_kBs = expected_byterate / 1024.0

    if total_bytes == 0:
        _bad("Cero bytes recibidos en toda la ventana")
        _info("→ El ESP32 no está mandando nada. Causas más comunes:")
        _info("    a) Firmware no flasheado o corrupto")
        _info("    b) ESP32 colgado en boot (revisá GPIO 0/2/15)")
        _info("    c) Cable USB sin línea de datos (cable solo de carga)")
        _info("    d) Mismatch de baud entre firmware y este script")
        return False, 0.0

    _ok(f"Recibidos {total_bytes} bytes ({rate_kBs:.1f} KB/s)")
    _info(f"Esperado a {expected_byterate} B/s teóricos: {expected_kBs:.1f} KB/s")

    ratio = rate_kBs / expected_kBs if expected_kBs > 0 else 0
    if ratio < 0.4:
        _bad(f"Tasa MUY baja ({ratio*100:.0f}% del esperado)")
        _info("→ El ESP32 no está enviando a ritmo normal. Causas:")
        _info("    a) Baud mismatch (firmware vs script)")
        _info("    b) ESP32 atascado entre frames (i2s_read bloqueado)")
    elif ratio < 0.75:
        _warn(f"Tasa baja ({ratio*100:.0f}% del esperado)")
        _info("→ Probable saturación del Serial.write (buffer USB-CDC se llena)")
        _info("   El audio capturado tendrá GAPS TEMPORALES entre frames consecutivos")
        _info("   aunque el counter del firmware no muestre drops.")
        _info("   A 11.025 kHz / 16 bits el enlace va al ~96% del CP2102: si satura,")
        _info("   bajá SAMPLE_RATE a 10000 en firmware (.ino) Y config.py a la vez.")
    elif ratio > 1.2:
        _warn(f"Tasa más alta de lo esperado ({ratio*100:.0f}%)")
        _info("→ Sospechoso: revisá HOP_SIZE / SAMPLE_RATE en firmware vs config.py")
    else:
        _ok(f"Tasa coherente con el firmware ({ratio*100:.0f}% del teórico)")

    return True, rate_kBs


# -----------------------------------------------------------------------------
# Chequeo 4 + 5: sync, decodificación, counter
# -----------------------------------------------------------------------------

def decode_packets(ser, hop_size, num_ch, duration_s, fs=11025):
    _section(f"4. Decodificación de paquetes (ventana de {duration_s:.1f}s)")

    pkt_data_bytes = hop_size * num_ch * BYTES_PER_SAMPLE   # int16 = 2 bytes

    ser.reset_input_buffer()

    # Resincronizar buscando SYNC_BYTE
    t_sync = time.time()
    while time.time() - t_sync < 1.0:
        b = ser.read(1)
        if b and b[0] == SYNC_BYTE:
            break
    else:
        _bad("No se encontró SYNC_BYTE (0xAA) en 1s")
        _info("→ El stream no tiene el framing esperado. Causas:")
        _info("    a) Firmware viejo sin protocolo de framing")
        _info("    b) Termios raw no aplicó y los 0xAA se reinterpretan")
        return None
    _ok("SYNC_BYTE 0xAA encontrado — empezando decodificación")

    pkts_ok      = 0
    pkts_corrupt = 0
    pkts_lost    = 0
    counters     = []
    last_counter = None

    # Acumulamos TODOS los samples por canal sobre toda la ventana, no solo un frame.
    # array.array('h') = int16 con signo (el firmware manda 16 bits/muestra LE).
    channel_samples = [array.array('h') for _ in range(num_ch)]

    # Peak por frame por canal — útil para distinguir saturación esporádica
    # (algunos frames con peak alto, otros normales) de saturación continua.
    peak_per_frame = [[] for _ in range(num_ch)]

    t0 = time.time()
    while time.time() - t0 < duration_s:
        # Resincronizar antes de cada paquete
        b = ser.read(1)
        if not b:
            continue
        if b[0] != SYNC_BYTE:
            continue

        cnt_bytes = ser.read(2)
        if len(cnt_bytes) < 2:
            pkts_corrupt += 1
            continue
        counter = (cnt_bytes[0] << 8) | cnt_bytes[1]

        raw = ser.read(pkt_data_bytes)
        if len(raw) < pkt_data_bytes:
            pkts_corrupt += 1
            continue

        end = ser.read(1)
        if not end or end[0] != END_BYTE:
            pkts_corrupt += 1
            continue

        # Paquete válido
        pkts_ok += 1
        counters.append(counter)
        if last_counter is not None:
            expected = (last_counter + 1) & 0xFFFF
            if counter != expected:
                pkts_lost += (counter - expected) & 0xFFFF
        last_counter = counter

        # De-interleave int16 little-endian (2 bytes/muestra, con signo)
        interleaved = struct.unpack(f"<{hop_size * num_ch}h", raw)
        # Picos por frame (uno por canal) para tracking de saturación esporádica
        frame_peaks = [0] * num_ch
        for i, s in enumerate(interleaved):
            ch = i % num_ch
            channel_samples[ch].append(s)
            a = -s if s < 0 else s
            if a > frame_peaks[ch]:
                frame_peaks[ch] = a
        for ch in range(num_ch):
            peak_per_frame[ch].append(frame_peaks[ch])

    elapsed = time.time() - t0

    if pkts_ok == 0:
        _bad("Ningún paquete completo decodificado")
        _info(f"→ Encontró el SYNC pero los datos siguientes están mal formados.")
        _info(f"   pkts_corrupt = {pkts_corrupt}")
        _info("   Causas: HOP_SIZE/SAMPLE_RATE distinto entre firmware y script,")
        _info("   o termios raw no se aplicó correctamente.")
        return None

    rate = pkts_ok / elapsed
    expected_rate = fs / hop_size
    rate_ratio = rate / expected_rate

    if rate_ratio < 0.85:
        _warn(f"Paquetes válidos: {pkts_ok} ({rate:.1f} pkts/s) — debajo del ritmo")
        _info(f"Tasa esperada a {fs}Hz/{hop_size}: {expected_rate:.1f} pkts/s ({rate_ratio*100:.0f}%)")
        _info("→ El firmware NO está manteniendo el ritmo del DMA I2S.")
        _info("   Aunque el counter no muestra gaps, el audio recibido tiene")
        _info("   GAPS TEMPORALES: cada frame contiene muestras correctas,")
        _info("   pero entre frames consecutivos pasan más tiempo del esperado")
        _info("   (el DMA sobreescribe muestras mientras Serial.write bloquea).")
        _info("   Esto invalida cualquier análisis de continuidad temporal entre frames.")
    else:
        _ok(f"Paquetes válidos: {pkts_ok} ({rate:.1f} pkts/s)")
        _info(f"Tasa esperada a {fs}Hz/{hop_size}: {expected_rate:.1f} pkts/s ({rate_ratio*100:.0f}%)")

    if pkts_corrupt > 0:
        pct_corrupt = 100 * pkts_corrupt / (pkts_ok + pkts_corrupt)
        if pct_corrupt > 5:
            _bad(f"Paquetes corruptos: {pkts_corrupt} ({pct_corrupt:.1f}%)")
            _info("→ >5% es preocupante. Revisá: cable, baud, longitud HOP_SIZE")
        else:
            _warn(f"Paquetes corruptos: {pkts_corrupt} ({pct_corrupt:.1f}%) — aceptable")

    if pkts_lost > 0:
        pct_lost = 100 * pkts_lost / (pkts_ok + pkts_lost)
        if pct_lost > 50:
            _bad(f"Paquetes perdidos: {pkts_lost} ({pct_lost:.1f}%)")
            _info("→ Demasiados drops. A 11.025 kHz/16 bits el link va al ~96%:")
            _info("  bajá SAMPLE_RATE a 10000 (firmware Y config.py).")
        elif pct_lost > 25:
            _warn(f"Paquetes perdidos: {pkts_lost} ({pct_lost:.1f}%)")
            _info("→ A 11.025 kHz/16 bits el enlace está al ~96% del CP2102; algo de")
            _info("  pérdida es esperable. Si molesta, bajá SAMPLE_RATE a 10000.")
        else:
            _ok(f"Paquetes perdidos: {pkts_lost} ({pct_lost:.1f}%)")
    else:
        _ok("Sin pérdidas de paquetes (counter sin gaps)")

    # Contador monotónico
    if len(counters) >= 2:
        gaps = []
        for a, b in zip(counters[:-1], counters[1:]):
            diff = (b - a) & 0xFFFF
            gaps.append(diff)
        if all(g == 1 for g in gaps):
            _ok("Counter incrementa monotónicamente sin gaps")
        else:
            non_one = [g for g in gaps if g != 1]
            _warn(f"Counter con gaps: {len(non_one)} saltos > 1 (max gap = {max(gaps)})")

    return channel_samples, peak_per_frame


# -----------------------------------------------------------------------------
# Chequeo 6: calidad por canal
# -----------------------------------------------------------------------------

SAT_THRESHOLD = 32000  # |sample| >= esto se cuenta como saturado en int16 (max 32767)
DEAD_THRESHOLD = 768   # peak < esto = canal practicamente mudo (int16; ~3 en int8)
DC_THRESHOLD   = 5000  # |DC offset| > esto = sospechoso (int16; ~20 en int8)


def analyze_channels(channel_samples, peak_per_frame, num_ch):
    _section("5. Calidad de los canales")

    if not channel_samples or len(channel_samples[0]) == 0:
        _bad("No hay samples acumulados para analizar")
        return

    n_per_ch  = len(channel_samples[0])
    n_frames  = len(peak_per_frame[0]) if peak_per_frame else 0
    _info(f"Analizando {n_per_ch} samples/canal sobre {n_frames} frames "
          f"({n_per_ch * num_ch} samples totales)")
    print()

    print(f"    {'ch':>3} {'rms':>8} {'peak':>8} {'dc_off':>8} "
          f"{'%sat':>7} {'frames_clip':>12}  estado")
    print(f"    {'─'*3} {'─'*8} {'─'*8} {'─'*8} {'─'*7} {'─'*12}  {'─'*30}")

    any_dead = False
    any_clip_continuous = False
    any_clip_burst      = False

    for ch_idx in range(num_ch):
        samples = channel_samples[ch_idx]
        n = len(samples)

        # RMS sobre toda la ventana
        sum_sq = sum(s * s for s in samples)
        rms_v  = (sum_sq / n) ** 0.5

        # Peak global (peor caso) — tomado de los peaks por frame
        peak_v = max(peak_per_frame[ch_idx]) if peak_per_frame[ch_idx] else 0

        # DC offset
        dc_off = sum(samples) / n

        # % de samples saturados sobre toda la ventana
        n_sat   = sum(1 for s in samples if s >= SAT_THRESHOLD or s <= -SAT_THRESHOLD)
        pct_sat = 100.0 * n_sat / n

        # Frames donde apareció al menos un sample saturado
        frames_with_clip = sum(1 for p in peak_per_frame[ch_idx] if p >= SAT_THRESHOLD)
        frames_clip_str  = f"{frames_with_clip}/{n_frames}" if n_frames else "-"

        # Diagnóstico
        status = []
        if peak_v < DEAD_THRESHOLD:
            status.append("\033[31mMUDO\033[0m (cable/L-R/firmware)")
            any_dead = True
        elif pct_sat > 1.0:
            status.append(f"\033[31mSATURADO CONTINUO\033[0m ({pct_sat:.1f}% samples)")
            any_clip_continuous = True
        elif pct_sat > 0.0 or frames_with_clip > 0:
            status.append(f"\033[33mSATURADO ESPORÁDICO\033[0m "
                          f"({frames_with_clip}/{n_frames} frames)")
            any_clip_burst = True
        elif abs(dc_off) > DC_THRESHOLD:
            status.append("\033[33mDC offset alto\033[0m")
        else:
            status.append("\033[32mOK\033[0m")

        print(f"    {ch_idx:>3} {rms_v:>8.0f} {peak_v:>8d} {dc_off:>+8.0f} "
              f"{pct_sat:>6.2f}% {frames_clip_str:>12}  {' '.join(status)}")

    print()
    if any_dead:
        _info("Canal mudo → revisar L/R wiring (GND vs VDD), cable SD del bus, o el INMP441")
    if any_clip_continuous:
        _info("Saturación CONTINUA (>1% samples al rail):")
        _info("  - Si la forma de onda en Audacity NO se ve como audio recortado")
        _info("    sino como ruido aleatorio → el INMP441 está dando datos basura")
        _info("    (chip dañado, L/R flotante, soldadura fría en SD).")
        _info("  - Si SÍ se ve como audio recortado → es clipping real,")
        _info("    subí GAIN_SHIFT en firmware (16 bits: ~16 → 17/18) o alejá la fuente.")
    if any_clip_burst:
        _info("Saturación ESPORÁDICA (frames aislados con peak ≥32000 en int16):")
        _info("  - Eventos acústicos puntuales (golpes, palmadas) cerca del mic.")
        _info("  - O bit-corruption del bus (picos individuales aislados, no continuos).")
        _info("  - Aumentá --duration y volvé a correr para confirmar el patrón.")
    if not (any_dead or any_clip_continuous or any_clip_burst):
        _ok("Los 4 canales presentan señal sin clipping en toda la ventana")


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(
        description="Diagnóstico ESP32 → RPi por USB-serial",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument('port', help="Puerto serial (ej. /dev/ttyUSB0)")
    p.add_argument('--baud',     type=int,   default=DEFAULT_BAUD)
    p.add_argument('--hop',      type=int,   default=DEFAULT_HOP)
    p.add_argument('--ch',       type=int,   default=DEFAULT_CHANNELS)
    p.add_argument('--fs',       type=int,   default=DEFAULT_FS,
                   help="Sample rate del firmware (Hz)")
    p.add_argument('--duration', type=float, default=DEFAULT_DURATION_S,
                   help="Duración de cada ventana de muestreo (s)")
    args = p.parse_args()

    print(f"\n\033[1mDiagnóstico ESP32 → RPi\033[0m")
    print(f"  Puerto   : {args.port}")
    print(f"  Baud     : {args.baud}")
    print(f"  Hop size : {args.hop} muestras")
    print(f"  Canales  : {args.ch}")

    # Tasa de bytes esperada: data + framing overhead
    # data = sample_rate * channels * BYTES_PER_SAMPLE (2 bytes, int16)
    # framing = 4 bytes (SYNC + counter + END) por paquete; pkts/s = sample_rate / hop
    fs_assumed = args.fs
    data_byterate     = fs_assumed * args.ch * BYTES_PER_SAMPLE
    framing_byterate  = 4 * (fs_assumed / args.hop)
    expected_byterate = data_byterate + framing_byterate

    if not check_environment(args.port):
        sys.exit(1)

    ser = open_serial(args.port, args.baud)
    if ser is None:
        sys.exit(1)

    try:
        ok, _ = measure_byte_rate(ser, args.duration, expected_byterate)
        if not ok:
            sys.exit(1)

        result = decode_packets(ser, args.hop, args.ch, args.duration, fs=args.fs)
        if result is None:
            sys.exit(1)
        channel_samples, peak_per_frame = result

        analyze_channels(channel_samples, peak_per_frame, args.ch)

    finally:
        ser.close()
        print()
        _info("Puerto cerrado.")
    print()


if __name__ == '__main__':
    main()
