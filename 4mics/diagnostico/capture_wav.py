"""
capture_wav.py — Captura audio crudo del ESP32 a un archivo WAV multicanal.

Útil para inspeccionar la señal de cada micrófono en Audacity u otra
herramienta de análisis, sin que el pipeline DOA modifique los datos.

El WAV resultante tiene:
    - 4 canales (mic 0, 1, 2, 3)
    - 11.025 kHz, 16-bit signed PCM (formato WAV estándar)
    - sin procesamiento: lo mismo que el ESP32 envió por serial
      (muestras int16 little-endian, idéntico a lo que lee audio_input.py)

Uso:
    python3 capture_wav.py /dev/ttyUSB0
    python3 capture_wav.py /dev/ttyUSB0 --seconds 10 --out test1.wav

Tip de análisis en Audacity:
    Open con Tools → Import → Raw Data si querés inspeccionar canales separados,
    o directamente abrí el WAV y mirá las 4 pistas. Si una pista está plana,
    el INMP441 correspondiente está mudo (revisá L/R wiring o cable SD).
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import termios
import time
import wave

try:
    import serial
except ImportError:
    print("✗ pyserial no instalado. Ejecutá: pip3 install pyserial", file=sys.stderr)
    sys.exit(2)

# Defaults — coinciden con firmware/config.py (11.025 kHz / 16-bit little-endian)
DEFAULT_BAUD     = 921600
DEFAULT_HOP      = 256
DEFAULT_CHANNELS = 4
DEFAULT_FS       = 11025
BYTES_PER_SAMPLE = 2          # int16 (firmware actual)
DEFAULT_SECONDS  = 5.0
SYNC_BYTE        = 0xAA
END_BYTE         = 0x55


def open_serial(port, baud):
    ser = serial.Serial()
    ser.port     = port
    ser.baudrate = baud
    ser.timeout  = 2.0
    ser.open()

    fd = ser.fileno()
    attrs = termios.tcgetattr(fd)
    attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    attrs[3] &= ~(termios.ECHO | termios.ECHOE | termios.ICANON)
    attrs[0] &= ~(termios.INLCR | termios.IGNCR | termios.ICRNL)
    attrs[1] &= ~termios.OPOST
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    # Reset del ESP32 a modo RUN (secuencia esptool: EN←!RTS, GPIO0←!DTR;
    # DTR bajo mantiene GPIO0 alto y el pulso de RTS reinicia — un toggle de
    # DTR solo puede dejar el chip en bootloader según el adaptador).
    ser.dtr = False
    ser.rts = True
    time.sleep(0.1)
    ser.rts = False
    time.sleep(0.7)
    ser.reset_input_buffer()
    return ser


def find_sync(ser, timeout_s=2.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        b = ser.read(1)
        if b and b[0] == SYNC_BYTE:
            return True
    return False


def read_packet(ser, hop_size, num_ch):
    """
    Lee un paquete completo asumiendo que ya consumimos el SYNC_BYTE.
    Retorna bytes raw del bloque de datos, o None si paquete corrupto.
    """
    cnt_bytes = ser.read(2)
    if len(cnt_bytes) < 2:
        return None

    pkt_data_bytes = hop_size * num_ch * BYTES_PER_SAMPLE   # int16 = 2 bytes
    raw = ser.read(pkt_data_bytes)
    if len(raw) < pkt_data_bytes:
        return None

    end = ser.read(1)
    if not end or end[0] != END_BYTE:
        return None

    return raw


def main():
    p = argparse.ArgumentParser(description="Captura WAV multicanal desde ESP32")
    p.add_argument('port',          help="Puerto serial (ej. /dev/ttyUSB0)")
    p.add_argument('--out',         default="capture.wav", help="Archivo WAV de salida")
    p.add_argument('--seconds',     type=float, default=DEFAULT_SECONDS)
    p.add_argument('--baud',        type=int,   default=DEFAULT_BAUD)
    p.add_argument('--hop',         type=int,   default=DEFAULT_HOP)
    p.add_argument('--ch',          type=int,   default=DEFAULT_CHANNELS)
    p.add_argument('--fs',          type=int,   default=DEFAULT_FS,
                   help="Sample rate (informativo para el header WAV)")
    args = p.parse_args()

    if not os.path.exists(args.port):
        print(f"✗ {args.port} no existe. ls /dev/ttyUSB*", file=sys.stderr)
        sys.exit(1)

    print(f"Abriendo {args.port} a {args.baud} baud...")
    ser = open_serial(args.port, args.baud)

    print("Buscando SYNC_BYTE...")
    if not find_sync(ser):
        print("✗ Timeout: ESP32 no envía SYNC_BYTE", file=sys.stderr)
        print("  → Probá: python3 diagnose_serial.py", args.port, file=sys.stderr)
        ser.close()
        sys.exit(1)

    print("Sync OK. Capturando audio...")

    target_pkts = int(args.seconds * args.fs / args.hop)
    pkts_ok      = 0
    pkts_corrupt = 0
    last_counter = None
    pkts_lost    = 0
    bar_width    = 40

    # Pre-abrimos el WAV para no perder datos en memoria si la captura es larga
    wf = wave.open(args.out, 'wb')
    wf.setnchannels(args.ch)
    wf.setsampwidth(2)        # int16
    wf.setframerate(args.fs)

    t0 = time.time()
    try:
        while pkts_ok < target_pkts:
            # Buscar siguiente SYNC
            b = ser.read(1)
            if not b:
                continue
            if b[0] != SYNC_BYTE:
                continue

            # Leer counter para tracking de drops (no se escribe al WAV)
            cnt_bytes = ser.read(2)
            if len(cnt_bytes) < 2:
                pkts_corrupt += 1
                continue
            counter = (cnt_bytes[0] << 8) | cnt_bytes[1]

            pkt_data_bytes = args.hop * args.ch * BYTES_PER_SAMPLE   # int16 = 2 bytes
            raw = ser.read(pkt_data_bytes)
            if len(raw) < pkt_data_bytes:
                pkts_corrupt += 1
                continue
            end = ser.read(1)
            if not end or end[0] != END_BYTE:
                pkts_corrupt += 1
                continue

            if last_counter is not None:
                expected = (last_counter + 1) & 0xFFFF
                if counter != expected:
                    pkts_lost += (counter - expected) & 0xFFFF
            last_counter = counter

            pkts_ok += 1
            # Las muestras ya son int16 little-endian (igual que el WAV): se
            # escriben tal cual, sin conversión.
            wf.writeframes(raw)

            # Progress bar simple cada ~10 pkts
            if pkts_ok % 10 == 0:
                pct = pkts_ok / target_pkts
                done = int(pct * bar_width)
                print(f"\r  [{'='*done}{' '*(bar_width-done)}] "
                      f"{pkts_ok}/{target_pkts} pkts "
                      f"(perdidos: {pkts_lost}, corruptos: {pkts_corrupt})",
                      end='', flush=True)

    except KeyboardInterrupt:
        print("\n  Captura interrumpida por el usuario.")
    finally:
        wf.close()
        ser.close()

    elapsed = time.time() - t0
    duration_audio = (pkts_ok * args.hop) / args.fs

    print()
    print()
    print(f"✓ WAV escrito: {args.out}")
    print(f"  Paquetes válidos : {pkts_ok}")
    print(f"  Paquetes perdidos: {pkts_lost}")
    print(f"  Paquetes corruptos: {pkts_corrupt}")
    print(f"  Duración audio   : {duration_audio:.2f}s en {elapsed:.2f}s reales")
    if pkts_ok < target_pkts:
        print(f"  ⚠ Solo {pkts_ok}/{target_pkts} paquetes capturados — algo va mal")
    print()
    print(f"  Inspección rápida con sox (si está instalado):")
    print(f"    sox {args.out} -n stat")
    print(f"    sox {args.out} -n stats")


if __name__ == '__main__':
    main()
