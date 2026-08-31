"""
diagnose_serial.py — Diagnóstico de la cadena ESP32 → RPi por USB-serial.

Herramienta independiente: prueba solo la recepcion de datos del ESP32

Etapas
    1. Sanity de entorno   — ¿existe el puerto? ¿permisos? ¿dmesg?
    2. Apertura serial     — ¿podemos abrir? ¿termios raw aplica?
    3. Tasa de bytes       — ¿está mandando algo el ESP32, y a qué ritmo?
    4. Decodificación      — ¿son paquetes válidos? ¿counter incrementa?
    5. Calidad de canales  — sobre toda la ventana, con estadísticos.
    6. Repetibilidad       — con --repeat N, mide cuánto varían los
                             estadísticos entre ventanas.

Uso:
    python3 diagnose_serial.py /dev/ttyUSB0
    python3 diagnose_serial.py /dev/ttyUSB0 --repeat 5        # variabilidad
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import termios
import time

try:
    import numpy as np
except ImportError:
    print("✗ numpy no instalado. Ejecutá: pip3 install numpy", file=sys.stderr)
    sys.exit(2)

try:
    import serial
except ImportError:
    print("✗ pyserial no instalado. Ejecutá: pip3 install pyserial", file=sys.stderr)
    sys.exit(2)

# Defaults — deben coincidir con config.py / firmware
DEFAULT_BAUD       = 921600
DEFAULT_HOP        = 256
DEFAULT_CHANNELS   = 4
DEFAULT_DURATION_S = 3.0
DEFAULT_FS         = 11025
BYTES_PER_SAMPLE   = 2       # int16 little-endian
SYNC_BYTE          = 0xAA
END_BYTE           = 0x55

# Banda de trabajo del sistema
BAND_LO = 200.0
BAND_HI = 2400.0

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

    if not os.path.exists(port):
        _bad(f"{port} no existe")
        _info("Probá: ls /dev/ttyUSB* /dev/ttyACM*")
        _info("Si no aparece nada: revisá cable USB / dmesg | tail -20")
        return False
    _ok(f"{port} existe")

    if not os.access(port, os.R_OK | os.W_OK):
        _bad(f"Sin permisos R/W sobre {port}")
        _info(f"Agregate al grupo dialout: sudo usermod -aG dialout {os.environ.get('USER','$USER')}")
        _info("Después logout/login (o newgrp dialout)")
        return False
    _ok("Permisos R/W OK")

    try:
        import subprocess
        out = subprocess.run(['fuser', port], capture_output=True, text=True, timeout=2)
        if out.stdout.strip():
            _warn(f"Otros procesos están usando {port}: {out.stdout.strip()}")
            _info("Cerrá cualquier monitor serial / Arduino IDE / minicom abierto")
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    return True


# -----------------------------------------------------------------------------
# Chequeo 2: apertura serial + termios raw
# -----------------------------------------------------------------------------

def _abrir_puerto(port, baud, verbose=True):
    """Abre el puerto y lo deja en modo binario puro."""
    try:
        ser = serial.Serial()
        ser.port     = port
        ser.baudrate = baud
        ser.timeout  = 2.0
        ser.open()
    except serial.SerialException as e:
        if verbose:
            _bad(f"No se pudo abrir {port}: {e}")
        return None
    try:
        _apply_raw_termios(ser)
    except Exception as e:
        if verbose:
            _bad(f"No se pudo aplicar termios raw: {e}")
        ser.close()
        return None
    return ser


def open_serial(port, baud, do_reset=True):
    """do_reset=False es CRÍTICO para diagnosticar el arranque en frío.

    El pulso RTS/DTR reinicia el ESP32 y lo lleva al estado 'bueno'. Si el
    problema aparece solo al energizar, cualquier chequeo que empiece con un
    reset lo borra antes de poder medirlo — que es exactamente lo que estuvo
    enmascarando este bug: correr el diagnóstico lo arreglaba.
    """
    _section("2. Apertura del puerto")

    ser = _abrir_puerto(port, baud)
    if ser is None:
        return None
    _ok(f"Abierto a {baud} baud, modo RAW termios aplicado")

    if not do_reset:
        _warn("SIN reset (--no-reset): se conserva el estado de arranque actual")
        _info("El ESP32 queda como está; no se toca RTS/DTR.")
        return ser

    # ARRANQUE VERIFICADO — el reintento REABRE EL PUERTO, no solo pulsa EN.
    #
    # Observación que lo motiva: en la PRIMERA ejecución después de energizar,
    # TRES pulsos de EN seguidos fallan; en la SEGUNDA ejecución el primero
    # funciona. Lo único que cambia entre una y otra no es el reset —es que el
    # proceso terminó, se CERRÓ el descriptor del puerto y se volvió a ABRIR.
    #
    # Eso descarta que el problema esté en el ESP32 (el mismo pulso de EN falla
    # y funciona según qué pasó del lado del host) y lo pone en la
    # inicialización del puente USB-serial: al cerrar, el driver deja caer
    # DTR/RTS y reinicia el estado de control de módem; al abrir, reenvía toda
    # la configuración de línea. Mientras el descriptor sigue abierto, repetir
    # el pulso no reconstruye ese estado, y por eso reintentar sin cerrar no
    # sirve de nada.
    #
    # flush=False: el banner del firmware sale ~500 ms después del boot y el
    # reset_input_buffer() de reset_esp32 lo descartaba antes de leerlo.
    pkt_total = 1 + 2 + DEFAULT_HOP * DEFAULT_CHANNELS * BYTES_PER_SAMPLE + 1

    for intento in range(1, 4):
        if not reset_esp32(ser, flush=False):
            _warn("RTS/DTR no soportado o falló")
            _info("No es crítico: probá apretar el botón EN manualmente")
            break

        _info(f"Capturando el arranque (intento {intento}/3, 2.5 s — incluye "
              f"el asentamiento del bias)...")
        arranque = capturar_arranque(ser, seconds=2.5)

        if _hay_framing_en(arranque, pkt_total):
            _ok("ESP32 reiniciado y emitiendo paquetes válidos"
                + (f" (hizo falta reabrir el puerto {intento-1} vez/veces)"
                   if intento > 1 else ""))
            _buscar_banner(arranque, avisar_si_falta=False)
            break

        _warn(f"Intento {intento}: sin paquetes válidos "
              f"({len(arranque)} bytes). Reabriendo el puerto...")
        if intento == 3:
            _bad("Tras 3 aperturas sigue sin enganchar")
            _info("→ Los chequeos que siguen van a fallar. Diagnosticá con:")
            _info(f"      python3 diagnose_serial.py {port} --raw")
            break

        # LA PARTE QUE IMPORTA: cerrar de verdad y volver a abrir.
        ser.close()
        time.sleep(0.4)
        ser = _abrir_puerto(port, baud, verbose=False)
        if ser is None:
            _bad("No se pudo reabrir el puerto")
            return None

    ser.reset_input_buffer()
    return ser


def _hay_framing_en(data, pkt_total):
    """¿Hay dos paquetes consecutivos válidos en estos bytes? (mismo criterio
    que Framer.lock, pero sobre un buffer ya capturado)."""
    b, n = bytes(data), pkt_total
    for i in range(len(b) - 2 * n + 1):
        if b[i] != SYNC_BYTE or b[i + n - 1] != END_BYTE:
            continue
        if b[i + n] != SYNC_BYTE or b[i + 2 * n - 1] != END_BYTE:
            continue
        c1 = (b[i + 1] << 8) | b[i + 2]
        c2 = (b[i + n + 1] << 8) | b[i + n + 2]
        if ((c2 - c1) & 0xFFFF) == 1:
            return True
    return False


def reset_esp32(ser, flush=True):
    """Reinicia el ESP32 a modo RUN por RTS/DTR (secuencia esptool: EN←!RTS,
    GPIO0←!DTR; DTR bajo mantiene GPIO0 alto durante el pulso RTS. Un toggle de
    DTR solo puede dejar el chip en bootloader según el adaptador).

    flush=False deja intacto lo que el ESP32 emite al arrancar. Es necesario
    para capturar el banner del firmware: se imprime ~500 ms después del boot,
    y el reset_input_buffer() de esta función lo tiraba a la basura antes de
    que nadie lo leyera. Ese era el motivo real de "el banner no aparece".
    """
    try:
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.reset_input_buffer()    # limpiar ANTES de soltar el reset
        ser.rts = False             # desde acá el ESP32 arranca y habla
        if flush:
            time.sleep(0.7)
            ser.reset_input_buffer()
        return True
    except Exception:
        return False


def capturar_arranque(ser, seconds=2.5):
    """Lee TODO lo que el ESP32 emite justo después de un reset, y busca el
    banner. Devuelve los bytes capturados (que el llamador descarta)."""
    old, ser.timeout = ser.timeout, 0.05
    data = bytearray()
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            c = ser.read(8192)
            if c:
                data += c
    finally:
        ser.timeout = old
    return data


# -----------------------------------------------------------------------------
# Chequeo 3: tasa de bytes (sin parsear nada)
# -----------------------------------------------------------------------------

def measure_byte_rate(ser, duration_s, expected_byterate, baud):
    _section(f"3. Tasa de bytes recibidos (ventana de {duration_s:.1f}s)")

    ser.reset_input_buffer()

    # Lectura BLOQUEANTE en loop cerrado, sin sleep. Esto es lo que arregla el
    # sub-conteo sistemático: nunca dejamos crecer backlog en el n_tty.
    old_timeout = ser.timeout
    ser.timeout = 0.2
    t0 = time.time()
    total_bytes = 0
    try:
        while True:
            elapsed = time.time() - t0
            if elapsed >= duration_s:
                break
            chunk = ser.read(4096)
            total_bytes += len(chunk)
    finally:
        ser.timeout = old_timeout

    elapsed = time.time() - t0
    rate_Bs = total_bytes / elapsed
    expected_kBs = expected_byterate / 1024.0
    link_capacity = baud / 10.0   # 8N1 → 10 bits por byte

    if total_bytes == 0:
        _bad("Cero bytes recibidos en toda la ventana")
        _info("→ El ESP32 no está mandando nada. Causas más comunes:")
        _info("    a) Firmware no flasheado o corrupto")
        _info("    b) ESP32 colgado en boot (revisá GPIO 0/2/15)")
        _info("    c) Cable USB sin línea de datos (cable solo de carga)")
        _info("    d) Mismatch de baud entre firmware y este script")
        return False, 0.0

    _ok(f"Recibidos {total_bytes} bytes ({rate_Bs/1024:.1f} KB/s)")
    _info(f"Esperado del firmware : {expected_kBs:.1f} KB/s")
    _info(f"Capacidad del enlace  : {link_capacity/1024:.1f} KB/s "
          f"({baud} baud 8N1) → el firmware pide el "
          f"{100*expected_byterate/link_capacity:.0f}% del enlace")

    ratio = rate_Bs / expected_byterate if expected_byterate > 0 else 0
    if ratio < 0.4:
        _bad(f"Tasa MUY baja ({ratio*100:.0f}% del esperado)")
        _info("→ El ESP32 no está enviando a ritmo normal. Causas:")
        _info("    a) Baud mismatch (firmware vs script)")
        _info("    b) ESP32 atascado entre frames (i2s_read bloqueado)")
    elif ratio < 0.90:
        _warn(f"Tasa baja ({ratio*100:.0f}% del esperado)")
        _info("→ Antes de culpar al hardware: mirá los pkt/s y el counter del")
        _info("   chequeo 4. Ese par es el veredicto real (ver nota abajo).")
    elif ratio > 1.10:
        _warn(f"Tasa más alta de lo esperado ({ratio*100:.0f}%)")
        _info("→ Revisá HOP_SIZE / SAMPLE_RATE en firmware vs config.py")
    else:
        _ok(f"Tasa coherente con el firmware ({ratio*100:.0f}% del teórico)")

    return True, rate_Bs


# -----------------------------------------------------------------------------
# Volcado crudo — qué está mandando realmente el ESP32
# -----------------------------------------------------------------------------

# TIOCGICOUNT: contadores de errores del driver de tty en Linux.
# struct serial_icounter_struct { int cts,dsr,rng,dcd,rx,tx,frame,overrun,
#                                 parity,brk,buf_overrun; int reserved[9]; }
_TIOCGICOUNT = 0x545D
_ICOUNT_CAMPOS = ('cts', 'dsr', 'rng', 'dcd', 'rx', 'tx',
                  'frame', 'overrun', 'parity', 'brk', 'buf_overrun')


def _icounts(ser):
    """Contadores de errores del PUERTO, leídos del kernel.

    Es el dato que separa las dos explicaciones que quedan cuando llega un
    stream continuo que no se puede decodificar:

      · BAUD EQUIVOCADO → el receptor pierde el bit de stop constantemente y el
        kernel cuenta FRAME ERRORS. Los bytes con error se descartan, así que
        la tasa observada baja a una fracción de la real y ningún paquete llega
        completo. Encaja con "recibo el 22% y cero paquetes válidos".

      · RECEPCIÓN LIMPIA → cero errores de frame. Entonces los bytes llegan bien
        y sencillamente NO son el protocolo: el firmware manda otra cosa.

    Sin esto las dos hipótesis producen el mismo síntoma visto desde arriba, y
    no hay forma de distinguirlas mirando los bytes.
    """
    try:
        import fcntl
        buf = struct.pack('20i', *([0] * 20))
        r = fcntl.ioctl(ser.fileno(), _TIOCGICOUNT, buf)
        vals = struct.unpack('20i', r)
        return dict(zip(_ICOUNT_CAMPOS, vals[:len(_ICOUNT_CAMPOS)]))
    except Exception:
        return None      # no todos los drivers lo implementan


def _reportar_marcas_de_frame(res):
    """Reporte del método PARMRK (ver medir_errores_de_frame)."""
    if res is None:
        _warn("No se pudo activar el marcado de errores (PARMRK) en este puerto")
        return
    buenos, marcas = res
    total = buenos + marcas
    if total == 0:
        _bad("Cero bytes durante la medición de errores de framing")
        return
    pct = 100.0 * marcas / total
    print()
    _info(f"Errores de framing medidos con PARMRK (no depende del driver):")
    _info(f"      bytes OK = {buenos}   con error de framing = {marcas}   "
          f"({pct:.1f}%)")

    if pct > 2.0:
        _bad(f"{pct:.1f}% de los bytes llegan con ERROR DE FRAMING")
        _info("→ El baud del emisor NO coincide con el del receptor. El kernel")
        _info("  descarta esos bytes, y por eso llega una fracción de lo enviado")
        _info("  y ningún paquete queda completo.")
        _info("  Como el barrido de velocidades estándar tampoco engancha, el")
        _info("  emisor está CERCA de 921600 pero no exactamente ahí — la firma")
        _info("  de un UART configurado con la frecuencia de APB equivocada.")
        _info("  Que el reset por EN lo corrija apunta a que en el arranque en")
        _info("  frío Serial.begin() corre antes de que el reloj se estabilice.")
        _info("  Arreglo en el firmware: delay(200) ANTES de Serial.begin(), y")
        _info("  setCpuFrequencyMhz(240) antes de ese delay.")
    else:
        _ok(f"Solo {pct:.1f}% de errores de framing: los bytes llegan LIMPIOS")
        _info("→ Descarta el baud como causa. El ESP32 transmite datos correctos")
        _info("  a 921600 que NO son el protocolo del firmware.")
        _info("  Lo único que queda es verificar QUÉ binario está flasheado:")
        _info("      esptool.py --port <puerto> flash_id")
        _info("      esptool.py --port <puerto> verify_flash 0x10000 <bin>")
        _info("  Si esptool habla con el chip, el enlace y el ROM están sanos y")
        _info("  el problema es la imagen de aplicación.")


def _reportar_errores_puerto(antes, despues, n_bytes):
    if antes is None or despues is None:
        _info("El driver no expone contadores de error (TIOCGICOUNT); se mide "
              "por PARMRK, que no depende del driver")
        return
    d = {k: despues.get(k, 0) - antes.get(k, 0) for k in _ICOUNT_CAMPOS}
    print()
    _info(f"Errores del puerto durante la captura (kernel, TIOCGICOUNT):")
    _info(f"      frame={d['frame']}   overrun={d['overrun']}   "
          f"parity={d['parity']}   brk={d['brk']}   "
          f"buf_overrun={d['buf_overrun']}")
    _info(f"      bytes recibidos por el driver: rx={d['rx']}  "
          f"(entregados a este proceso: {n_bytes})")

    if d['frame'] > 0.02 * max(d['rx'], 1):
        _bad(f"{d['frame']} FRAME ERRORS: el baud del emisor NO coincide con el "
             f"del receptor")
        _info("→ El receptor pierde el bit de stop una y otra vez. Los bytes")
        _info("  malos se descartan, y por eso llega una fracción de lo enviado")
        _info("  y ningún paquete queda completo.")
        _info("  Como el barrido de baudios estándar no engancha, el emisor está")
        _info("  a una velocidad CERCANA a 921600 pero no igual — típico de un")
        _info("  UART configurado con la frecuencia de APB equivocada.")
        _info("  El reset por EN lo corrige, lo que apunta a que en el arranque")
        _info("  en frío Serial.begin() se ejecuta antes de que el reloj se")
        _info("  estabilice. Arreglo: delay() ANTES de Serial.begin(), no después.")
    elif d['overrun'] + d['buf_overrun'] > 0.02 * max(d['rx'], 1):
        _bad(f"{d['overrun'] + d['buf_overrun']} OVERRUNS: la Pi no drena el "
             f"puerto a tiempo")
        _info("→ Los bytes llegan bien pero se pierden en el buffer del kernel.")
    elif d['rx'] > 0:
        _ok("Cero errores de frame: los bytes se están recibiendo LIMPIOS")
        _info("→ Descarta el baud como causa. El emisor manda datos correctos a")
        _info("  921600, pero NO son el protocolo del firmware. O sea: el ESP32")
        _info("  está corriendo, transmitiendo, y mandando otra cosa.")
        _info("  Verificá qué binario está flasheado — es lo único que queda.")


BANNER_TAG = b"[SSL-FW]"


def _buscar_banner(data, avisar_si_falta=True):
    """Busca el banner que el firmware imprime al arrancar.

    Es la vía más directa para saber QUÉ binario corre y POR QUÉ se reinició, y
    no depende ni del descriptor de la flash (que en arduino-esp32 3.x reporta
    el core y no el sketch) ni del log de la ROM (que GPIO15 puede silenciar).
    """
    if BANNER_TAG not in bytes(data):
        if avisar_si_falta:
            _warn(f"No aparece el banner {BANNER_TAG.decode()} en la captura")
            _info("→ El banner se imprime UNA VEZ, ~500 ms después de arrancar.")
            _info("  Si esta captura no incluye un arranque, es normal que falte:")
            _info("  con --no-reset el chip arrancó cuando lo enchufaste, mucho")
            _info("  antes de que empezáramos a escuchar. Para verlo en frío hay")
            _info("  que estar escuchando ANTES de energizar:")
            _info("      python3 diagnose_serial.py <puerto> --wait-boot")
            _info("  Si en cambio SÍ hubo un reset en esta captura, entonces el")
            _info("  firmware no llega a Serial.begin() en condiciones.")
            print()
        return None

    txt = bytes(data).decode('latin-1')
    lineas = []
    for i in range(len(txt)):
        if txt.startswith(BANNER_TAG.decode(), i):
            fin = txt.find('\n', i)
            lineas.append(txt[i:fin if fin > 0 else i + 160].strip())
    _ok(f"Banner del firmware encontrado ({len(lineas)} vez/veces):")
    for l in dict.fromkeys(lineas):          # sin repetir
        _info(f"    {l}")

    reinicios = sum(1 for l in lineas if 'reset=' in l)
    if reinicios > 1:
        _bad(f"El banner de ARRANQUE aparece {reinicios} veces: el chip se "
             f"reinicia solo")
    for l in lineas:
        u = l.upper()
        if 'RESET=BROWNOUT' in u:
            _bad("reset=BROWNOUT — la alimentación cae por debajo del umbral")
            _info("→ Problema de FUENTE. Alimentá el ESP32 desde un cargador o")
            _info("  hub con fuente propia y agregá 100-470 uF en el 3.3V.")
        elif 'RESET=TASK_WDT' in u or 'RESET=INT_WDT' in u:
            _bad("reset=WDT — el firmware se colgó")
        elif 'RESET=PANIC' in u:
            _bad("reset=PANIC — excepción no atrapada en el firmware")
        elif 'RESET=POWERON' in u:
            _info("reset=POWERON: arranque en frío legítimo.")
        elif 'RESET=EXT' in u:
            _info("reset=EXT: lo reinició el pin EN (RTS/DTR), o sea nosotros.")
    print()
    return lineas


def _repeticion_exacta(arr, max_p=8192):
    """Busca si el stream REPITE un bloque idéntico, y de qué tamaño.

    Es el chequeo que distingue "audio real sin framing" de "el DMA devuelve
    siempre el mismo buffer". Si i2s_read() no bloquea pero tampoco hay datos
    nuevos, el firmware retransmite el contenido viejo del buffer y el stream
    queda perfectamente periódico byte a byte — cosa que el audio real nunca es.
    """
    n = len(arr)
    ventana = min(4096, n // 3)
    if ventana < 256:
        return None
    a = arr[:ventana].astype(np.int16)
    mejor = None
    for p in range(2, min(max_p, n - ventana)):
        if np.array_equal(arr[p:p + ventana], a):
            mejor = p
            break
    if mejor is None:
        _ok("El stream no repite ningún bloque: los datos son nuevos en cada "
            "muestra")
        return None

    # ¿Qué fracción de la captura respeta esa repetición?
    m = n - mejor
    frac = float(np.mean(arr[mejor:] == arr[:m]))
    _bad(f"El stream REPITE un bloque de {mejor} bytes ({100*frac:.0f}% de la "
         f"captura es copia exacta)")
    if mejor % 8 == 0:
        _info(f"→ {mejor} bytes = {mejor//8} muestras de 4 canales int16.")
    _info("  Audio real jamás se repite bit a bit. Esto es el mismo buffer")
    _info("  retransmitido: i2s_read() devuelve datos viejos porque el DMA no")
    _info("  se está llenando. El micrófono no entrega o el clock no llega.")
    return mejor


def _probar_alineaciones(arr, num_ch=4, hay_framing=False):
    """¿El stream sin framing son muestras de audio? ¿De qué endianness?

    QUÉ SE PUEDE DETERMINAR Y QUÉ NO
    ---------------------------------------------------------------------------
    Sin el byte de SYNC no hay forma de saber qué canal es el 0: correr el
    origen en 2, 4 o 6 bytes da exactamente la misma señal con los canales
    ROTADOS, y las cuatro interpretaciones son estadísticamente idénticas. La
    rotación de canales es indeterminable, punto.

    Lo que SÍ se determina bien es si el contenido tiene ESTRUCTURA DE AUDIO:
    las muestras vecinas están muy correlacionadas (r≈0.99) mientras que el
    ruido o los datos de otro protocolo dan r≈0. Ese contraste es de dos
    órdenes de magnitud y no admite discusión.

    EL ORDEN DE BYTES NO SE PUEDE DETERMINAR ASÍ, y se dice explícitamente en
    vez de arriesgar un veredicto. Sobre audio de baja frecuencia el byte alto
    es casi constante, así que leerlo invertido conserva casi toda la
    correlación: medido sobre señales sintéticas, la lectura correcta da 0.998
    y la invertida 0.955 — y con audio realmente big-endian, 0.998 contra
    0.979. Los dos márgenes (0.043 y 0.019) se solapan, o sea que el criterio
    no separa los casos. Se reportan los dos números como información y nada
    más; para resolver endianness hay que mirar el firmware, no el stream.
    """
    paso = 2 * num_ch
    if len(arr) < paso * 64:
        return

    def evaluar(dt, off):
        cuerpo = arr[off:]
        m = (len(cuerpo) // paso) * paso
        if m < paso * 32:
            return None, None
        x = np.frombuffer(cuerpo[:m].tobytes(), dtype=dt).reshape(-1, num_ch)
        x = x.astype(np.float64)
        x -= x.mean(axis=0)
        s = x.std(axis=0)
        if np.any(s < 1e-9):
            return None, None
        r1 = float(np.mean([np.mean(x[1:, c] * x[:-1, c]) / (s[c] ** 2)
                            for c in range(num_ch)]))
        return r1, x

    resultados = {}
    for endian, dt in (('little', '<i2'), ('big', '>i2')):
        mejor_r, mejor_x = -2.0, None
        for off in range(paso):          # se barre, pero solo para no perder el
            r1, x = evaluar(dt, off)     # máximo; el off en sí no se reporta
            if r1 is not None and r1 > mejor_r:
                mejor_r, mejor_x = r1, x
        resultados[endian] = (mejor_r, mejor_x)

    r_le = resultados['little'][0]
    r_be = resultados['big'][0]
    if mejor_x is None and r_le < -1:
        return

    print()
    _info(f"Interpretado como {num_ch} canales int16 "
          f"(correlación entre muestras vecinas, 1.0 = audio perfecto):")
    _info(f"      little-endian: {r_le:+.3f}       big-endian: {r_be:+.3f}")
    _info("      (estos dos números NO sirven para decidir el endianness: sobre")
    _info("       audio de baja frecuencia se diferencian menos de 0.05)")

    r1, x = resultados['little'] if r_le >= r_be else resultados['big']
    _info("  rms por canal (el ORDEN de canales es indeterminable sin SYNC): " +
          "  ".join(f"{x[:, c].std():.0f}" for c in range(num_ch)))

    # Además del agrupado por canales, la secuencia CRUDA de int16. Agrupar de a
    # 4 mezcla canales distintos y puede enmascarar que los valores son
    # perfectamente razonables. Se muestran los números para poder juzgarlos a
    # ojo: audio en silencio son enteros chicos y suaves; basura son valores de
    # decenas de miles saltando de signo.
    mejor_cruda = (-2.0, None, None)
    for dt, nom in (('<i2', 'little'), ('>i2', 'big')):
        for off in (0, 1):
            cuerpo = arr[off:]
            m = (len(cuerpo) // 2) * 2
            v = np.frombuffer(cuerpo[:m].tobytes(), dtype=dt).astype(np.float64)
            if v.std() < 1e-9 or len(v) < 64:
                continue
            r = float(np.corrcoef(v[1:], v[:-1])[0, 1])
            if r > mejor_cruda[0]:
                mejor_cruda = (r, nom, off)
    if mejor_cruda[1]:
        r, nom, off = mejor_cruda
        cuerpo = arr[off:]
        m = (len(cuerpo) // 2) * 2
        v = np.frombuffer(cuerpo[:m].tobytes(),
                          dtype='<i2' if nom == 'little' else '>i2')
        _info(f"  Secuencia cruda de int16 (sin agrupar por canal), mejor caso: "
              f"{nom}-endian, offset {off}, r={r:+.3f}")
        _info(f"    primeros valores: {v[:12].tolist()}")
        _info(f"    |max|={int(np.abs(v.astype(np.int32)).max())}   "
              f"mediana |v|={int(np.median(np.abs(v.astype(np.int32))))}")
        _info("    OJO: leer big-endian en offset par da los MISMOS valores que")
        _info("    little-endian en offset impar. Los dos casos no se distinguen")
        _info("    acá; lo que sí se ve es si los números son plausibles.")

    if hay_framing:
        _ok(f"Muestras de audio válidas (r={r1:.3f}) DENTRO de paquetes con "
            f"framing correcto")
        return
    if r1 < 0.3:
        _warn(f"Agrupado de a {num_ch} canales, la correlación es baja "
              f"({r1:.2f})")
        _info("→ No concluye por sí solo: si el offset o el número de canales no")
        _info("  son los correctos, la correlación se destruye aunque los datos")
        _info("  sean audio perfecto. Mirá los valores crudos de arriba.")
    else:
        _ok(f"Son muestras de audio válidas (r={r1:.3f}), pero SIN la cabecera "
            f"del protocolo")
        _info("→ Los DATOS llegan bien; lo que falta es el SYNC+counter+END.")
        _info("  Con el enlace al 96% de ocupación, el sospechoso es que")
        _info("  Serial.write() se corte cuando el buffer de TX está lleno.")
        _info("  El firmware nuevo ya manda el paquete en UNA sola escritura y")
        _info("  chequea el retorno; si esto aparece, es firmware viejo.")


def dump_raw(ser, pkt_total_bytes, n_bytes=8192, n_show=192):
    """Muestra los bytes tal como llegan y los clasifica.

    Cuando la decodificación falla, "no se pudo enganchar el framing" no dice
    nada: puede ser el log del bootloader, silencio, basura, o paquetes válidos
    con otro tamaño. Son cuatro problemas distintos y se distinguen mirando los
    bytes. Es más rápido que cualquier inferencia sobre estadísticos.
    """
    _section("Volcado crudo del stream")

    old = ser.timeout
    ser.timeout = 0.3
    ic_antes = _icounts(ser)
    data = bytearray()
    t0 = time.time()
    while len(data) < n_bytes and time.time() - t0 < 4.0:
        c = ser.read(n_bytes - len(data))
        if not c:
            break
        data += c
    ic_despues = _icounts(ser)
    ser.timeout = old

    if not data:
        _bad("Cero bytes. El ESP32 no transmite nada en este estado.")
        _reportar_errores_puerto(ic_antes, ic_despues, 0)
        return

    # Sin avisar si falta: acá la captura arranca mucho después del boot, así
    # que la ausencia del banner es lo esperable y no significa nada.
    _buscar_banner(data, avisar_si_falta=False)

    # Esto va PRIMERO: distingue "baud equivocado" de "datos limpios que no son
    # el protocolo", y esa bifurcación decide qué significa todo lo que sigue.
    _reportar_errores_puerto(ic_antes, ic_despues, len(data))
    if ic_antes is None or ic_despues is None:
        _reportar_marcas_de_frame(medir_errores_de_frame(ser))
    print()

    dump_raw_bytes(data, pkt_total_bytes, n_show=n_show, seccion=False)


def dump_raw_bytes(data, pkt_total_bytes, n_show=192, seccion=True):
    """Análisis del volcado, separado de la captura para poder reutilizarlo
    sobre bytes ya capturados (p.ej. los del arranque en frío)."""
    if seccion:
        _section("Volcado crudo del stream")
    if not data:
        _bad("Sin bytes para analizar")
        return

    _info(f"{len(data)} bytes capturados. Primeros {min(n_show,len(data))}:")
    print()
    for off in range(0, min(n_show, len(data)), 16):
        chunk = data[off:off + 16]
        hexs  = " ".join(f"{b:02x}" for b in chunk)
        asci  = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"     {off:04x}  {hexs:<47}  |{asci}|")
    print()

    # --- Estadística de bytes ---
    # Mirar 192 bytes en hexadecimal induce a ver patrones que no están (o a
    # perderse los que sí). Estos tres números se calculan sobre TODA la
    # captura y separan las causas que el hexdump confunde.
    arr = np.frombuffer(bytes(data), dtype=np.uint8)
    hist = np.bincount(arr, minlength=256) / len(arr)
    nz = hist[hist > 0]
    entropia = float(-(nz * np.log2(nz)).sum())          # 8.0 = uniforme
    frac_div4 = float(np.mean((arr & 0x03) == 0))        # 0.25 si es uniforme
    frac_div16 = float(np.mean((arr & 0x0F) == 0))       # 0.0625 si es uniforme
    # Racha media de bytes iguales: alta = stream leído a un baud más rápido
    cambios = int(np.count_nonzero(arr[1:] != arr[:-1]))
    racha_media = len(arr) / max(cambios, 1)

    print(f"    Estadística sobre los {len(arr)} bytes:")
    print(f"      entropía          {entropia:5.2f} bits/byte  (8.00 = uniforme)")
    print(f"      múltiplos de 4    {100*frac_div4:5.1f} %          (25.0 % si es uniforme)")
    print(f"      múltiplos de 16   {100*frac_div16:5.1f} %          ( 6.2 % si es uniforme)")
    print(f"      racha media       {racha_media:5.2f} bytes iguales seguidos")
    print()

    # ORDEN IMPORTA: primero los casos degenerados (texto, línea muda). Una
    # línea de puros 0x00 cumple "100% múltiplos de 4" y "racha altísima", así
    # que si se evalúan antes dispara dos alarmas que no vienen al caso.
    printable = sum(1 for b in data if 32 <= b < 127 or b in (10, 13))
    frac_txt  = printable / len(data)
    zeros     = float(np.mean(arr == 0))

    if frac_txt > 0.85:
        _bad(f"El {100*frac_txt:.0f}% son caracteres imprimibles: esto es TEXTO, "
             f"no audio")
        _info("Casi seguro es el log del bootloader de la ROM del ESP32. Ese log")
        _info("sale a 115200 baud fijo, así que a 921600 se ve como basura; si")
        _info("acá se lee legible, el firmware ni siquiera arrancó.")
        _info("Volvé a leerlo a 115200 con:  --probe-boot")
        return
    if zeros > 0.9:
        _bad(f"El {100*zeros:.0f}% son 0x00: la línea está muda o el mic no entrega")
        _info("→ El ESP32 transmite (hay bytes) pero todos nulos. Típico de")
        _info("  i2s_read() devolviendo buffers vacíos: el clock no llega a los")
        _info("  micrófonos o el pin de SD no está conectado.")
        return

    if frac_div4 > 0.60:
        _bad(f"El {100*frac_div4:.0f}% de los bytes son múltiplos de 4: los 2 bits "
             f"bajos están casi siempre en cero")
        _info("→ Los datos vienen CORRIDOS A LA IZQUIERDA 2 posiciones. Es la")
        _info("  contracara del desbalance de ganancia que venías midiendo: en")
        _info("  vez de dividir por 4, acá el word del I2S quedó multiplicado.")
        _info("  Confirma que el problema es alineación de bits del I2S y no las")
        _info("  cápsulas, y que el arranque en frío la deja peor.")

    # OJO: el discriminante acá es la RACHA, no la entropía. Un stream leído a
    # un baud más alto repite cada byte varias veces, pero los bytes en sí
    # siguen siendo variados: la entropía se mantiene alta (~7.9) y solo la
    # racha se dispara. Audio real y paquetes dan racha ~1.0.
    if racha_media > 2.5:
        _bad(f"Rachas largas: {racha_media:.1f} bytes idénticos seguidos en promedio")
        _info("→ Firma típica de leer un stream a un baud MÁS ALTO que el real:")
        _info(f"  cada byte del emisor se muestrea ~{racha_media:.0f} veces, o sea")
        _info(f"  que el baud real rondaría los {DEFAULT_BAUD/racha_media:,.0f}.")
        _info("  Confirmalo con --scan-baud.")

    _repeticion_exacta(arr)

    # ORDEN: la estructura de paquetes va ANTES que la interpretación como
    # audio. Si el framing está presente, la conclusión "son muestras válidas
    # pero SIN cabecera" es falsa y no debe imprimirse. Con el orden invertido
    # el reporte se contradecía a sí mismo con tres líneas de diferencia.
    hay_framing = _estructura_paquetes(arr, pkt_total_bytes)
    _probar_alineaciones(arr, num_ch=4, hay_framing=hay_framing)
    return


def _estructura_paquetes(b, pkt_total_bytes):
    """¿El stream tiene el framing del firmware? Devuelve True/False."""
    sync = (b == SYNC_BYTE)
    n = len(b)

    # --- ¿Hay estructura de paquetes? ---
    # NO sirve mirar la distancia entre 0xAA consecutivos: en datos de audio
    # aleatorios ~1 de cada 256 bytes vale 0xAA, o sea ~8 falsos por paquete, y
    # la moda de las distancias se va al ruido. Se busca el PERÍODO por
    # autocorrelación del indicador de SYNC, que es inmune a esos falsos.
    if sync.sum() < 4:
        _warn("Casi no aparece el byte de SYNC (0xAA): el stream no tiene el "
              "framing del firmware")
        return False

    s  = sync.astype(np.float64)
    s -= s.mean()
    # Se limita el lag a n/MIN_REP para que cualquier período candidato tenga al
    # menos MIN_REP repeticiones en la muestra. Sin esa cota, con 2 repeticiones
    # el "mejor offset" de 6880 posibles siempre encuentra dos 0xAA alineados
    # por puro azar y el ruido aleatorio se clasifica como estructura.
    MIN_REP = 6
    ac = np.fft.irfft(np.abs(np.fft.rfft(s, 2 * n)) ** 2)[:n // 2]
    lag_min, lag_max = 16, n // MIN_REP
    if lag_max <= lag_min:
        _warn("Muestra demasiado corta para estimar el período")
        return False
    periodo = int(np.argmax(ac[lag_min:lag_max]) + lag_min)

    endb = (b == END_BYTE)

    def calidad(p):
        """En el mejor offset del período p: fracción de paquetes con SYNC al
        principio y END al final. 0 si no hay repeticiones suficientes."""
        nrep = n // p
        if nrep < MIN_REP:
            return 0.0, 0.0
        m   = sync[:nrep * p].reshape(nrep, p)
        off = int(m.sum(axis=0).argmax())
        e   = endb[:nrep * p].reshape(nrep, p)
        return float(m[:, off].mean()), float(e[:, (off - 1) % p].mean())

    # La autocorrelación también pica en los MÚLTIPLOS del período real. Si el
    # firmware manda paquetes de 1028 bytes, el pico más alto puede caer en
    # 2056 y el diagnóstico quedaría mal (diría hop=256 en vez de 128). Se baja
    # al fundamental: el divisor más chico que conserva la periodicidad.
    frac_sync, frac_end = calidad(periodo)
    if frac_sync >= 0.9:
        for k in range(8, 1, -1):
            if periodo % k:
                continue
            fs, fe = calidad(periodo // k)
            if fs >= 0.9 and fe >= 0.9:
                periodo = periodo // k
                frac_sync, frac_end = fs, fe
                break

    _info(f"Período detectado por autocorrelación: {periodo} bytes "
          f"(esperado {pkt_total_bytes})")
    _info(f"En el mejor offset: SYNC presente en {100*frac_sync:.0f}% de los "
          f"paquetes, END en {100*frac_end:.0f}%")

    if frac_sync < 0.6 or frac_end < 0.6:
        _warn("No hay periodicidad real en los 0xAA: es ruido, no paquetes")
        _info("→ El ESP32 transmite algo pero no con este protocolo. Verificá")
        _info("  qué firmware está flasheado y que el baud coincida.")
        return False
    if periodo == pkt_total_bytes:
        _ok("El framing ESTÁ: período correcto, SYNC y END en su lugar")
        if frac_end < 0.99:
            _warn(f"END aparece solo en el {100*frac_end:.0f}% de los paquetes")
            _info("→ Paquetes truncados. Con el enlace al 96% de ocupación, es")
            _info("  Serial.write() cortando cuando el buffer de TX se llena.")
        return True
    _bad(f"Los paquetes miden {periodo} bytes, no {pkt_total_bytes}")
    hop_real = (periodo - 4) / (BYTES_PER_SAMPLE * 4)
    _info(f"→ Con 4 canales int16, eso corresponde a hop = {hop_real:.0f} "
          f"muestras (el script asume {(pkt_total_bytes-4)//8}).")
    _info("  Alineá HOP_SIZE y número de canales entre el firmware, "
          "config.py y este script.")
    return False


def _apply_raw_termios(ser):
    """Modo binario puro. Crítico: sin esto los 0xAA/0x11/0x13 se reinterpretan.

    Se limpian también ISTRIP, INPCK, PARMRK, IGNPAR y BRKINT, que la versión
    anterior no tocaba. No es paranoia: ISTRIP enmascara cada byte a 7 bits y
    PARMRK inserta bytes de marca en el stream. Cualquiera de los dos destruye
    audio binario en silencio, y como no se seteaban explícitamente quedaban a
    merced de lo que hubiera dejado configurado el proceso anterior en el
    puerto.
    """
    fd = ser.fileno()
    a = termios.tcgetattr(fd)
    a[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY |
              termios.INLCR | termios.IGNCR | termios.ICRNL |
              termios.ISTRIP | termios.INPCK | termios.PARMRK |
              termios.IGNPAR | termios.BRKINT | termios.IGNBRK)
    a[1] &= ~termios.OPOST
    a[3] &= ~(termios.ECHO | termios.ECHOE | termios.ICANON |
              termios.ISIG | termios.IEXTEN)
    termios.tcsetattr(fd, termios.TCSANOW, a)


def medir_errores_de_frame(ser, seconds=2.0):
    """Cuenta errores de framing SIN depender de TIOCGICOUNT.

    Varios drivers USB-serial (CH340, algunos CP210x) no implementan ese ioctl.
    Pero termios ofrece un camino equivalente y portable: con INPCK activado,
    PARMRK activado e IGNPAR desactivado, el kernel entrega cada byte que llegó
    con error de framing o paridad precedido por la secuencia \\377 \\0. Un byte
    legítimo de valor \\377 se duplica para no confundirse.

    Contando esas marcas se obtiene exactamente el dato que hace falta:

      · muchas marcas → el baud del emisor NO coincide con el del receptor. Los
        bytes malos se descartan, por eso llega una fracción de lo enviado y
        ningún paquete queda completo.
      · cero marcas   → los bytes se reciben limpios y sencillamente NO son el
        protocolo del firmware.

    Devuelve (n_bytes_buenos, n_marcas) o None si no se pudo configurar.
    """
    fd = ser.fileno()
    try:
        original = termios.tcgetattr(fd)
    except termios.error:
        return None
    try:
        a = termios.tcgetattr(fd)
        a[0] |= (termios.INPCK | termios.PARMRK)
        a[0] &= ~termios.IGNPAR
        termios.tcsetattr(fd, termios.TCSANOW, a)
        ser.reset_input_buffer()

        old_to, ser.timeout = ser.timeout, 0.2
        data = bytearray()
        t0 = time.time()
        while time.time() - t0 < seconds:
            c = ser.read(8192)
            if c:
                data += c
        ser.timeout = old_to
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, original)

    # Contar secuencias \377\0 (error) distinguiéndolas de \377\377 (0xFF real)
    marcas, buenos, i, n = 0, 0, 0, len(data)
    while i < n:
        if data[i] == 0xFF and i + 1 < n:
            if data[i + 1] == 0x00:
                marcas += 1
                i += 3          # \377 \0 <byte con error>
                continue
            if data[i + 1] == 0xFF:
                buenos += 1
                i += 2          # 0xFF legítimo, duplicado
                continue
        buenos += 1
        i += 1
    return buenos, marcas


BAUDS_COMUNES = [921600, 460800, 230400, 115200, 1000000, 500000, 74880, 57600]


def scan_bauds(port, hop, num_ch, bauds=None, do_reset=True):
    """Prueba varios baudios y reporta en cuál aparece el framing del firmware.

    Responde de una vez la pregunta que el volcado crudo deja abierta: ¿el
    stream es basura, o son paquetes perfectamente válidos leídos al baud
    equivocado? Son cosas muy distintas y a ojo, en hexadecimal, no se
    distinguen.

    Se incluye 74880 a propósito: es el baud al que la ROM del ESP32 imprime
    cuando el módulo trae cristal de 26 MHz en vez de 40 MHz (115200*26/40).
    Si el firmware engancha ahí, todos los baudios del sistema están corridos
    por el factor 26/40 y eso explicaría un enlace que "a veces" funciona.
    """
    _section("Barrido de baudios")
    pkt_total = 1 + 2 + hop * num_ch * BYTES_PER_SAMPLE + 1
    bauds = bauds or BAUDS_COMUNES
    if not do_reset:
        _warn("SIN reset: se conserva el estado de arranque en frío")
        _info("Es lo correcto para diagnosticar el fallo al energizar, pero el")
        _info("barrido tarda más porque no se puede reiniciar entre baudios.")

    print(f"    {'baud':>9} {'KB/s':>8} {'% cap.':>8} {'framing':>9}   nota")
    print("    " + "─" * 62)
    enganches = []
    medidas = []
    for baud in bauds:
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
        except (serial.SerialException, ValueError):
            print(f"    {baud:>9} {'-':>8} {'no abre':>10}")
            continue
        try:
            _apply_raw_termios(ser)
            if do_reset:
                reset_esp32(ser)
                time.sleep(1.5)
            ser.reset_input_buffer()

            t0, nb = time.time(), 0
            while time.time() - t0 < 1.0:
                nb += len(ser.read(4096))
            kBs = nb / (time.time() - t0) / 1024.0

            ser.reset_input_buffer()
            fr = Framer(ser, pkt_total)
            ok = fr.lock(timeout_s=2.0)
        finally:
            ser.close()

        cap = baud / 10.0 / 1024.0          # capacidad del receptor, KB/s
        pct = 100.0 * kBs / cap if cap else 0.0
        medidas.append((baud, kBs, pct))

        nota = ""
        if ok:
            enganches.append(baud)
            nota = "← paquetes válidos acá"
        elif kBs < 1.0:
            nota = "casi sin datos"
        print(f"    {baud:>9} {kBs:>8.1f} {pct:>7.0f}% {'SÍ' if ok else 'no':>9}   {nota}")

    print()
    if enganches:
        if enganches == [DEFAULT_BAUD] or enganches == [921600]:
            _ok(f"Solo engancha a {enganches[0]} — el baud configurado es el correcto")
        else:
            _bad(f"Engancha a {enganches}, no al baud configurado")
            _info("→ El firmware habla a otra velocidad que la que asume el")
            _info("  sistema. Corregí SERIAL_BAUD en firmware y config.py.")
        return

    _bad("El framing no aparece a ningún baud probado")

    # ¿Hay un stream REAL a una velocidad no probada, o no hay UART válido?
    # La clave: no se pueden RECIBIR más bytes de los que el emisor MANDÓ.
    #   · Si existe un stream real, su tasa en bytes/s es la misma se lea al
    #     baud que se lea: la columna KB/s sale CONSTANTE y el % de capacidad
    #     cae al subir el baud.
    #   · Si la línea no lleva UART válido (ruido, TX sin manejar, o basura de
    #     un emisor mucho más rápido), el receptor engancha falsos bits de
    #     arranque a su propio ritmo: KB/s CRECE con el baud y el % queda
    #     parecido en todos.
    activos = [(b, k, p) for b, k, p in medidas if k > 0.5]
    if len(activos) >= 3:
        kbs = [k for _, k, _ in activos]
        pcts = [p for _, _, p in activos]
        cv_kbs = statistics.pstdev(kbs) / statistics.fmean(kbs)
        cv_pct = statistics.pstdev(pcts) / statistics.fmean(pcts)
        kmax = max(activos, key=lambda t: t[1])

        _info(f"Tasa recibida: CV entre baudios = {100*cv_kbs:.0f}% en KB/s "
              f"y {100*cv_pct:.0f}% en % de capacidad")
        if cv_kbs < cv_pct:
            _bad("La tasa en BYTES es parecida a todos los baudios")
            _info(f"→ Hay un stream real de ~{statistics.fmean(kbs)*1024:,.0f} B/s.")
            _info("  Como no engancha a ninguna velocidad probada, el baud real")
            _info("  es otro. Estimalo: baud ≈ bytes/s × 10 ≈ "
                  f"{statistics.fmean(kbs)*1024*10:,.0f}.")
        else:
            _bad("La tasa CRECE con el baud del receptor: no hay UART válido")
            _info("→ Esto NO es 'un stream a otra velocidad'. Si existiera un")
            _info("  emisor real, no se podrían recibir más bytes de los que")
            _info("  manda, y la tasa en bytes sería la misma a todos los")
            _info(f"  baudios. Acá va de {min(kbs):.1f} a {kmax[1]:.1f} KB/s")
            _info(f"  siguiendo al receptor ({min(pcts):.0f}-{max(pcts):.0f}% de")
            _info("  su capacidad en todos los casos).")
            _info("  El receptor está enganchando flancos de algo que no es UART")
            _info("  a esta velocidad: línea de TX sin manejar (el firmware nunca")
            _info("  llegó a Serial.begin), chip en reset, o un emisor mucho más")
            _info("  rápido que todo lo probado.")
            _info("")
            _info(f"  Dato duro: a {kmax[0]:,} baud entraron {kmax[1]:.1f} KB/s. Si")
            _info("  fuera un emisor lento, eso sería imposible — descarta que el")
            _info("  reloj del ESP32 esté corrido hacia abajo por 2x o 4x.")
            _info("  Siguiente medición, sin resetear:")
            _info("      python3 diagnose_serial.py <puerto> --raw --no-reset")
            _info("  La longitud de racha y la entropía de ese volcado separan")
            _info("  'ruido en una línea sin manejar' de 'datos reales'.")
    else:
        _info("→ El ESP32 casi no transmite. Firmware colgado, en bootloader, o")
        _info("  la línea de TX no llega. Probá --raw --no-reset.")


def _periodicidad(tasa, bin_s, esperada):
    """¿La tasa baja es un flujo continuo, o un CICLO rápido de arranques?

    Un reinicio por brownout se repite cada ~100 ms. Con ventanas de 100 ms el
    ciclo completo (ráfaga del log de arranque + silencio del reset) cae dentro
    de UNA ventana y el promedio se ve como un flujo continuo a tasa baja.
    Mirando a 10 ms el ciclo aparece, y su PERÍODO identifica la causa.
    """
    x = np.asarray(tasa, dtype=np.float64)
    if len(x) < 64:
        return
    x = x - x.mean()
    if x.std() < 1e-9:
        _info("La tasa es perfectamente plana: no hay ciclo de arranques.")
        return
    n = len(x)
    ac = np.fft.irfft(np.abs(np.fft.rfft(x, 2 * n)) ** 2)[:n // 2]
    ac /= ac[0]
    lo = max(2, int(0.02 / bin_s))              # ignorar lags < 20 ms
    hi = max(lo + 3, n // 3)                    # exigir >=3 ciclos en la captura
    hi = min(hi, len(ac) - 1)
    if hi <= lo + 2:
        return

    # Solo MÁXIMOS LOCALES. La autocorrelación de cualquier señal suave es alta
    # cerca del lag 0 y decae; tomar el argmax directo devuelve lags cortos sin
    # ningún significado. Un ciclo real deja un pico, no una pendiente.
    cand = [j for j in range(lo + 1, hi)
            if ac[j] > ac[j - 1] and ac[j] >= ac[j + 1]]
    if not cand:
        print()
        _info("La tasa no tiene picos de autocorrelación: sin ciclo de arranques.")
        return

    mejor = max(cand, key=lambda j: ac[j])
    # La autocorrelación pica también en los MÚLTIPLOS del período real, y el
    # más alto puede ser un armónico (un ciclo de 120 ms daba 840 = 7x120). Se
    # baja al fundamental: el candidato más chico del que el pico máximo sea
    # múltiplo entero.
    periodo_k = mejor
    for j in sorted(cand):
        if ac[j] < 0.7 * ac[mejor]:
            continue
        m = max(1, round(mejor / j))
        if abs(mejor - m * j) <= max(1, 0.05 * mejor):
            periodo_k = j
            break

    pico = float(ac[periodo_k])
    periodo = periodo_k * bin_s

    print()
    _info(f"Autocorrelación de la tasa (ventanas de {bin_s*1000:.0f} ms): "
          f"pico {pico:.2f} en {periodo*1000:.0f} ms"
          + (f"  (el máximo estaba en {mejor*bin_s*1000:.0f} ms, "
             f"armónico {round(mejor/periodo_k)}×)" if periodo_k != mejor else ""))

    # LÍMITE DE ESTA MEDICIÓN — leer antes de sacar conclusiones de un pico
    # débil. Los bytes no llegan del ESP32 en tiempo continuo: el puente
    # USB-serial los entrega en paquetes USB de 64 bytes, y el host los sondea
    # cada ~1 ms. A 20 KB/s eso es un paquete cada ~3 ms, así que a resolución
    # de 10 ms una parte de las ventanas vacías y de la varianza las produce el
    # USB, no el ESP32. Por debajo de ~50 ms este chequeo mide el patrón de
    # entrega del bus, no el del emisor.
    if periodo < 0.05:
        _warn(f"El período detectado ({periodo*1000:.0f} ms) está en el rango "
              f"donde manda la granularidad del USB")
        _info("→ No es concluyente: el puente entrega paquetes de 64 bytes y el")
        _info("  host sondea cada ~1 ms, lo que por sí solo genera ventanas")
        _info("  vacías a esta escala. Hace falta otra medición, no más análisis")
        _info("  de esta serie.")
        return

    if pico < 0.25:
        _bad("La tasa es baja pero SIN periodicidad clara")
        _info("→ No hay un ciclo de arranques identificable en esta serie. Ojo")
        _info("  que un pico apenas por debajo del umbral no prueba lo")
        _info("  contrario: la entrega por USB ensucia la señal a esta escala.")
        _info("  La medición que sí decide, sin depender de esto:")
        _info("      python3 diagnose_serial.py <puerto> --probe-boot --no-reset")
        _info("  Si el chip se está reiniciando, el log de la ROM sale a 115200")
        _info("  UNA VEZ POR ARRANQUE: verlo repetido lo demuestra directamente.")
        return

    _bad(f"CICLO PERIÓDICO de {periodo*1000:.0f} ms (autocorrelación {pico:.2f})")
    _info("→ El ESP32 arranca, emite una ráfaga y se reinicia, una y otra vez.")
    _info("  Con ventanas de 100 ms esto se veía como 'tasa baja constante' —")
    _info("  el ciclo entero entraba en una sola ventana.")
    _info("")
    _info(f"  El período de {periodo*1000:.0f} ms dice cuál es la causa:")
    if periodo < 0.4:
        _info("   · < 400 ms → BROWNOUT. El detector de bajo voltaje resetea el")
        _info("     chip apenas arranca, vuelve a arrancar, vuelve a caer. Los 4")
        _info("     INMP441 más el ESP32 tirando del mismo 3.3V en el instante")
        _info("     del power-up es el caso clásico, sobre todo alimentando por")
        _info("     el USB de la Pi, que es una fuente floja.")
        _info("     Qué probar, en orden y sin tocar la placa:")
        _info("       1. Alimentar el ESP32 desde un cargador o hub CON fuente,")
        _info("          no desde el puerto USB de la Pi. Es la prueba de un")
        _info("          minuto que confirma o descarta todo esto.")
        _info("       2. Un electrolítico de 100-470 uF entre 3.3V y GND, cerca")
        _info("          del regulador de la placa del ESP32.")
        _info("       3. Cable USB más corto y más grueso (la caída en un cable")
        _info("          largo y fino alcanza para disparar el brownout).")
        _info("     Encaja con TODO lo observado: falla solo al energizar, se")
        _info("     arregla sola tras unos segundos (los condensadores terminan")
        _info("     de cargar y el consumo se estabiliza), y un reset por EN no")
        _info("     cambia el consumo pero para entonces la fuente ya se asentó.")
    elif periodo < 2.0:
        _info("   · 0.4-2 s → el firmware arranca y falla temprano (panic o")
        _info("     watchdog RTC durante setup()). Mirá si el reinicio ocurre")
        _info("     antes o después de instalar el I2S moviendo el delay(500).")
    else:
        _info("   · > 2 s → Task Watchdog: loop() bloqueado. Con el timeout")
        _info("     finito de i2s_read esto ya no debería pasar; si aparece,")
        _info("     el bloqueo está en Serial.write().")


def watch_stream(ser, hop, num_ch, fs, seconds):
    """Vigila el stream y detecta REINICIOS del ESP32 sin depender del log.

    Por qué no alcanza con --probe-boot: el log de la ROM sale a 115200 y dura
    ~30 ms, y enseguida el firmware reconfigura el UART a 921600 y lo tapa.
    Peor: GPIO15 es pin de STRAPPING y el firmware sync lo usa como entrada de
    WS del slave; si esa red queda en LOW al arrancar, la ROM directamente NO
    imprime el log. O sea que "no veo el log" no distingue entre 'no hubo
    reinicio' y 'el log está silenciado'. Es un camino que no puede concluir.

    Este chequeo usa el enlace que SÍ funciona. El firmware arranca
    frame_counter en 0 y lo incrementa por paquete, así que un reinicio deja
    una huella inconfundible: el counter SALTA HACIA ATRÁS hasta cerca de cero.
    Y mientras el chip rebootea no transmite, así que también queda un HUECO
    en la tasa de bytes. Dos evidencias independientes del mismo evento.
    """
    _section(f"Vigilancia del stream ({seconds:.0f} s)")

    pkt_total = 1 + 2 + hop * num_ch * BYTES_PER_SAMPLE + 1
    # BIN de 10 ms, no 100. Un reinicio por brownout puede repetirse cada
    # ~100 ms; con ventanas de 100 ms el ciclo queda promediado adentro de cada
    # ventana y el resultado parece un flujo continuo a tasa baja. Esa fue
    # exactamente la lectura equivocada de la primera versión de este chequeo.
    BIN = 0.01

    ser.reset_input_buffer()
    old, ser.timeout = ser.timeout, 0.002
    trozos, buf = [], bytearray()          # (timestamp, nbytes) por lectura
    t0 = time.time()
    try:
        while True:
            now = time.time()
            if now - t0 >= seconds:
                break
            c = ser.read(4096)
            if c:
                buf += c
                trozos.append((now - t0, len(c)))
    finally:
        ser.timeout = old

    if not trozos:
        _bad("No se capturó nada")
        return

    # Binning offline a partir de los timestamps: más preciso que acumular
    # dentro del lazo de lectura.
    nb = int(seconds / BIN)
    bins = np.zeros(nb)
    for t, k in trozos:
        i = int(t / BIN)
        if 0 <= i < nb:
            bins[i] += k / BIN
    bins = list(bins)

    b = np.frombuffer(bytes(buf), dtype=np.uint8)
    tasa = np.array(bins) / 1024.0
    esperada = (fs * num_ch * BYTES_PER_SAMPLE + 4 * fs / hop) / 1024.0

    # --- Perfil temporal de la tasa ---
    # Hay que separar dos cosas que un solo umbral confunde:
    #   · HUECOS      → tasa normal interrumpida por silencios. Eso es reinicio.
    #   · TASA BAJA   → tasa reducida pero CONSTANTE. Eso NO es reinicio: el
    #                   ESP32 transmite todo el tiempo, más lento. Apunta a que
    #                   el reloj del chip no es el que el firmware supone.
    _info(f"Tasa media {tasa.mean():.1f} KB/s sobre {len(tasa)} ventanas de "
          f"{BIN*1000:.0f} ms (esperada {esperada:.1f} KB/s)")
    mudas  = np.flatnonzero(tasa < 0.05 * esperada)      # silencio de verdad
    cv     = float(tasa.std() / tasa.mean()) if tasa.mean() > 0 else 0.0
    ratio  = float(tasa.mean() / esperada)
    _info(f"Dispersión de la tasa entre ventanas: CV = {100*cv:.0f}%")

    if len(mudas) > 0.05 * len(tasa):
        _bad(f"{len(mudas)} ventanas SIN transmisión "
             f"({BIN*len(mudas):.1f} s de silencio en total)")
        # El PERÍODO de los huecos es lo que identifica la causa, así que la
        # periodicidad se calcula también acá (antes esta rama cortaba y solo
        # listaba los instantes, que es el dato menos útil de los dos).
        _periodicidad(tasa, BIN, esperada)
    elif ratio < 0.75:
        _bad(f"Tasa baja: {100*ratio:.0f}% de lo esperado (CV {100*cv:.0f}%)")
        _periodicidad(tasa, BIN, esperada)
    else:
        _ok("Transmisión continua y a la tasa esperada")

    # --- Saltos del counter hacia atrás = reinicios ---
    counters, i, n = [], 0, len(b)
    while i + pkt_total <= n:
        if b[i] == SYNC_BYTE and b[i + pkt_total - 1] == END_BYTE:
            counters.append(((int(b[i + 1]) << 8) | int(b[i + 2]), i))
            i += pkt_total
        else:
            i += 1                              # perdimos alineación: rebuscar

    if len(counters) < 10:
        _warn(f"Solo {len(counters)} paquetes válidos en {seconds:.0f} s: no se "
              f"puede seguir el counter")
        _info("→ Con el framing roto, mirá los huecos de arriba: si hay huecos")
        _info("  periódicos, el chip está reiniciándose igual.")
        return

    reinicios = []
    for (c1, _), (c2, p2) in zip(counters[:-1], counters[1:]):
        if c2 < c1 and c2 < 200:                # saltó atrás y arrancó de cero
            reinicios.append((p2 / max(b.size, 1) * seconds, c1, c2))

    print()
    _info(f"{len(counters)} paquetes válidos, counter de {counters[0][0]} "
          f"a {counters[-1][0]}")
    if reinicios:
        _bad(f"BOOT LOOP CONFIRMADO: {len(reinicios)} reinicios en "
             f"{seconds:.0f} s")
        for t, c1, c2 in reinicios[:10]:
            _info(f"   t≈{t:5.1f}s   el counter cayó de {c1} a {c2}")
        periodo = seconds / len(reinicios)
        _info(f"→ Se reinicia cada ~{periodo:.1f} s en promedio. Ese período es")
        _info("  el dato clave: si ronda los 5 s, es el Task Watchdog (TWDT), y")
        _info("  la causa es loop() bloqueado. i2s_read() con portMAX_DELAY hace")
        _info("  exactamente eso si el DMA no se llena.")
        _info("  Arreglos, en orden:")
        _info("   1. Timeout finito en i2s_read (p.ej. pdMS_TO_TICKS(100)) y")
        _info("      reintentar en vez de bloquear para siempre.")
        _info("   2. i2s_stop() tras instalar, esperar ~200 ms con los mics ya")
        _info("      alimentados, y recién ahí i2s_start().")
        _info("   3. Mover el WS del slave fuera de GPIO15 (pin de strapping).")
    else:
        _ok(f"Sin reinicios: el counter avanzó monótonamente durante "
            f"{seconds:.0f} s")
        _info("→ En este estado el firmware es estable. El problema del arranque")
        _info("  en frío no se reproduce con el chip ya caliente; hay que correr")
        _info("  esto JUSTO después de enchufar.")


def probe_boot_log(port, n_bytes=2048, do_reset=True, seconds=6.0):
    """Lee el puerto a 115200 para capturar el log de la ROM del ESP32.

    El bootloader de la ROM imprime SIEMPRE a 115200 (derivado del cristal),
    independientemente del baud del firmware. Su primera línea dice la CAUSA del
    reset, que es exactamente lo que hace falta saber cuando "la primera vez
    después de perder alimentación no funciona":

        rst:0x1  (POWERON_RESET)      arranque en frío
        rst:0x3  (SW_RESET)           reinicio por software
        rst:0x7  (TG0WDT_SYS_RESET)   watchdog: el firmware se colgó
        rst:0x10 (RTCWDT_RTC_RESET)   watchdog RTC
        rst:0xc  (SW_CPU_RESET)       reset por EN / RTS-DTR

    Y la línea 'boot:0x..' dice el modo de arranque: si termina en
    '(DOWNLOAD_BOOT...)' el chip quedó en modo bootloader y nunca corre el
    firmware — que es una causa clásica de "la primera vez no anda".
    """
    _section("Log del bootloader (115200 baud)")
    try:
        s = serial.Serial(port, 115200, timeout=1.0)
    except serial.SerialException as e:
        _bad(f"No se pudo abrir a 115200: {e}")
        return
    try:
        _apply_raw_termios(s)
        # NO se usa reset_esp32(): esa función hace reset_input_buffer() después
        # de soltar RTS, y el log de la ROM sale en los primeros ~100-300 ms —
        # o sea que el flush TIRABA justamente lo que queremos capturar
        # (incluida la línea 'rst:', que es el dato central de este chequeo).
        if do_reset:
            s.dtr = False
            s.rts = True
            time.sleep(0.1)
            s.reset_input_buffer()  # limpiar ANTES de soltar el reset
            s.rts = False           # a partir de acá el ESP32 arranca y habla
            ventana = 0.60
        else:
            # Sin resetear no se sabe cuándo arranca el chip, así que se escucha
            # varios segundos. Si está en un ciclo de reinicios, el log de la
            # ROM va a aparecer REPETIDO — y eso lo demuestra directamente, sin
            # depender de estadística sobre la tasa de bytes (que a resolución
            # fina queda contaminada por la granularidad de entrega del USB).
            _warn("SIN reset: se escucha el estado actual del chip")
            ventana = seconds
            n_bytes = max(n_bytes, int(ventana * 12000))

        # VENTANA CORTA cuando se resetea, a propósito. El log de la ROM son
        # ~300 bytes y sale en ~25 ms a 115200; inmediatamente después el
        # firmware hace Serial.begin(921600) y todo lo que sigue, leído a
        # 115200, es basura que tapa el log. Leer 3 s (como hacía la primera
        # versión de esto) devuelve 2 KB de ruido y ni una línea legible.
        data = bytearray()
        t0 = time.time()
        while len(data) < n_bytes and time.time() - t0 < ventana:
            c = s.read(min(4096, n_bytes - len(data)))
            if not c and do_reset:
                break            # sin reset, un hueco es normal: seguir oyendo
            data += c
    finally:
        s.close()

    # ¿Aparece más de un arranque en la captura? El log de la ROM sale UNA vez
    # por reset. Verlo repetido significa que el ESP32 se está reiniciando solo.
    txt_all = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
    n_entry = txt_all.count("entry 0x")
    n_rst   = txt_all.count("rst:0x")
    n_ets   = txt_all.count("ets ")          # primera línea de la ROM
    n_boot  = max(n_entry, n_rst, n_ets)

    _info(f"Marcas de arranque en la captura: 'ets '={n_ets}  "
          f"'rst:0x'={n_rst}  'entry 0x'={n_entry}")

    if n_boot > 1:
        _bad(f"BOOT LOOP: el log de arranque aparece {n_boot} veces")
        _info("→ El log de la ROM se imprime UNA VEZ POR RESET. Verlo repetido")
        _info("  demuestra que el ESP32 se reinicia solo, sin depender de")
        _info("  ninguna estadística sobre la tasa de bytes.")
        _info("  Mirá la línea 'rst:' de abajo: ahí está la causa.")
        print()
    elif n_boot == 1 and not do_reset:
        _ok("Un solo arranque en toda la ventana: el chip NO está reiniciándose")
        _info("→ Descarta boot loop, brownout y watchdog. El firmware arrancó una")
        _info("  vez y sigue corriendo; el problema está en lo que hace después.")
        print()
    elif n_boot == 0 and not do_reset:
        _warn("Ningún log de arranque en toda la ventana")
        _info("→ El chip no se reinició durante la captura. Combinado con que")
        _info("  tampoco emite el protocolo, apunta a un firmware corriendo pero")
        _info("  atascado, o a que la línea de TX no la maneja nadie.")
        print()

    # Extraer TODOS los tramos legibles, no solo el del principio. Con --no-reset
    # los arranques pueden caer en cualquier momento de la ventana, intercalados
    # con el stream binario del firmware.
    txt = "".join(chr(b) if 32 <= b < 127 or b in (10, 13) else "\x00"
                  for b in data)
    lineas = []
    for tramo in txt.split("\x00" * 8):          # separar por rachas binarias
        limpio = tramo.replace("\x00", "")
        for l in limpio.splitlines():
            if len(l.strip()) >= 6:              # descartar restos sueltos
                lineas.append(l.strip())
    if not lineas:
        _warn("No se capturó texto legible a 115200")
        _info(f"(se descartaron {len(data)-len(cab)} bytes de stream binario)")
        _info("Explicaciones posibles:")
        _info(" · GPIO15 en LOW al arrancar → la ROM silencia el log. Es un pin")
        _info("   de strapping y el firmware sync lo usa como entrada de WS del")
        _info("   slave; conviene moverlo a un GPIO que no sea de strapping.")
        _info(" · El módulo tiene cristal de 26 MHz → la ROM imprime a 74880.")
        _info("   Probá:  --scan-baud")
        return
    for l in lineas[:25]:
        print(f"     {l}")
    print()
    # Las causas se resumen UNA vez cada una, aunque el log aparezca repetido.
    causas = []
    for l in lineas:
        if 'rst:' in l and l.strip() not in causas:
            causas.append(l.strip())
    for c in causas:
        _info(f"Causa del reset: {c}")
        u = c.upper()
        if 'POWERON' in u:
            _ok("Arranque en frío legítimo (el chip acaba de recibir alimentación)")
        elif 'BROWNOUT' in u:
            _bad("BROWNOUT: la alimentación cae por debajo del umbral y el chip "
                 "se resetea solo")
            _info("→ Es un problema de FUENTE, no de firmware. Los 4 INMP441 más")
            _info("  el ESP32 tirando del mismo 3.3V en el instante del arranque.")
            _info("  Qué probar, en orden:")
            _info("   1. Alimentar el ESP32 desde un cargador o un hub CON fuente")
            _info("      propia, no desde el puerto USB de la Pi.")
            _info("   2. Electrolítico de 100-470 uF entre 3.3V y GND, cerca del")
            _info("      regulador de la placa del ESP32.")
            _info("   3. Cable USB más corto y grueso.")
        elif 'WDT' in u:
            _bad("WATCHDOG: el firmware se colgó y el watchdog lo reseteó")
            _info("→ Con el timeout finito en i2s_read esto no debería pasar; si")
            _info("  aparece igual, el bloqueo está en Serial.write().")
        elif 'SW_CPU' in u or 'SW_RESET' in u:
            _info("Reset por software o por EN (RTS/DTR). Es el esperado cuando")
            _info("el diagnóstico resetea el chip a propósito.")
        elif 'PANIC' in u or 'RTCWDT_RTC' in u:
            _bad("PANIC o watchdog RTC durante el arranque: el firmware se cae "
                 "antes de estabilizarse")
    for l in lineas:
        if 'boot:' in l and 'DOWNLOAD' in l.upper():
            _bad("El chip quedó en MODO BOOTLOADER: no corre el firmware")
            _info("→ GPIO0 quedó en LOW al arrancar. Revisá el circuito de")
            _info("  auto-reset del adaptador USB y que nada tire de GPIO0.")
            break


# -----------------------------------------------------------------------------
# Chequeo 4: sync, decodificación, counter
# -----------------------------------------------------------------------------

class Framer:
    """Lector de paquetes con enganche VERIFICADO y sin descartar datos.

    Por qué no alcanza "buscar un 0xAA y leer el paquete":
      · Cualquier byte de audio puede valer 0xAA (~8 falsos candidatos por
        paquete de 2052 bytes). Hay que confirmar el candidato.
      · Y sobre todo: si al confirmar se CONSUMEN los bytes del puerto, cada
        candidato falso se come 2051 bytes y se pasa de largo el límite real
        de paquete. Un enganche ingenuo puede tirar varios paquetes buenos
        (o, en un enlace al 96% de capacidad, desbordar el buffer del kernel
        mientras busca).

    Solución: se lee un bloque a memoria y se BUSCA AHÍ el offset, sin
    consumir. Se confirma exigiendo SYNC/END en su lugar en DOS paquetes
    consecutivos y counters que difieran en 1 (falso positivo ~2^-32).
    """

    def __init__(self, ser, pkt_total_bytes):
        self.ser = ser
        self.n   = pkt_total_bytes
        self.buf = bytearray()

    def _fill(self, need):
        while len(self.buf) < need:
            chunk = self.ser.read(need - len(self.buf))
            if not chunk:
                return False
            self.buf += chunk
        return True

    def lock(self, timeout_s=2.0):
        n = self.n
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            # 3 paquetes garantizan que haya un período completo de offsets
            # candidatos (0..n) con 2 paquetes enteros por delante para validar.
            if not self._fill(3 * n):
                return False
            for i in range(len(self.buf) - 2 * n + 1):
                b = self.buf
                if b[i] != SYNC_BYTE or b[i + n - 1] != END_BYTE:
                    continue
                if b[i + n] != SYNC_BYTE or b[i + 2 * n - 1] != END_BYTE:
                    continue
                c1 = (b[i + 1] << 8) | b[i + 2]
                c2 = (b[i + n + 1] << 8) | b[i + n + 2]
                if ((c2 - c1) & 0xFFFF) != 1:
                    continue
                del self.buf[:i]          # buffer alineado al primer paquete
                return True
            # Ningún offset válido: conservar solo la cola que aún podría
            # contener el inicio de un paquete y volver a llenar.
            del self.buf[:len(self.buf) - (2 * n - 1)]
        return False

    def read_packet(self):
        if not self._fill(self.n):
            return None
        pkt = bytes(self.buf[:self.n])
        del self.buf[:self.n]
        return pkt


def decode_packets(ser, hop_size, num_ch, duration_s, fs, quiet=False):
    """Devuelve (data[N, num_ch] int16, peak_per_frame, stats) o None."""
    if not quiet:
        _section(f"4. Decodificación de paquetes (ventana de {duration_s:.1f}s)")

    pkt_data_bytes  = hop_size * num_ch * BYTES_PER_SAMPLE
    pkt_total_bytes = 1 + 2 + pkt_data_bytes + 1   # SYNC + counter + data + END

    ser.reset_input_buffer()
    fr = Framer(ser, pkt_total_bytes)

    if not fr.lock():
        if not quiet:
            _bad("No se pudo enganchar el framing (SYNC+END+counter) en 2 s")
            # No dejar al usuario a ciegas: mostrar QUÉ está llegando.
            dump_raw(ser, pkt_total_bytes)
        return None
    if not quiet:
        _ok("Framing enganchado y verificado (2 paquetes consecutivos)")

    pkts_ok      = 0
    pkts_corrupt = 0
    pkts_lost    = 0
    last_counter = None
    first_counter = None
    blocks       = []

    t0 = time.time()
    while time.time() - t0 < duration_s:
        pkt = fr.read_packet()
        if pkt is None:
            break
        if pkt[0] != SYNC_BYTE or pkt[-1] != END_BYTE:
            pkts_corrupt += 1
            # Perdimos alineación: re-enganchar en vez de seguir leyendo basura
            if not fr.lock(timeout_s=1.0):
                break
            last_counter = None
            continue

        counter = (pkt[1] << 8) | pkt[2]
        pkts_ok += 1
        if first_counter is None:
            first_counter = counter
        if last_counter is not None:
            gap = (counter - last_counter) & 0xFFFF
            if gap != 1:
                pkts_lost += gap - 1
        last_counter = counter

        blocks.append(np.frombuffer(pkt[3:3 + pkt_data_bytes], dtype='<i2'))

    elapsed = time.time() - t0

    if pkts_ok == 0:
        if not quiet:
            _bad("Ningún paquete completo decodificado tras enganchar")
        return None

    data = np.concatenate(blocks).reshape(-1, num_ch)   # (n_frames*hop, num_ch)
    peak_per_frame = np.array([
        np.abs(b.reshape(-1, num_ch)).max(axis=0) for b in blocks
    ])                                                   # (n_pkts, num_ch)

    rate          = pkts_ok / elapsed
    expected_rate = fs / hop_size
    rate_ratio    = rate / expected_rate

    # Tasa de PRODUCCIÓN del ESP32, independiente de lo que perdamos nosotros:
    # el counter avanza una vez por paquete transmitido, así que
    # (counter_final - counter_inicial) / tiempo = pkt/s que el firmware generó.
    counter_span  = ((last_counter - first_counter) & 0xFFFF) + 1
    prod_rate     = counter_span / elapsed
    prod_ratio    = prod_rate / expected_rate

    stats = dict(pkts_ok=pkts_ok, pkts_corrupt=pkts_corrupt, pkts_lost=pkts_lost,
                 rate=rate, prod_rate=prod_rate, expected_rate=expected_rate,
                 elapsed=elapsed, first_counter=first_counter,
                 last_counter=last_counter)

    if quiet:
        return data, peak_per_frame, stats

    _info(f"Tasa esperada a {fs} Hz / hop {hop_size}: {expected_rate:.1f} pkt/s")
    print()

    # --- El veredicto: producción vs recepción -------------------------------
    if prod_ratio < 0.95:
        _bad(f"El ESP32 PRODUCE {prod_rate:.1f} pkt/s ({prod_ratio*100:.0f}% del teórico)")
        _info("→ El counter avanza más lento que el reloj: el firmware no genera")
        _info("  frames al ritmo del I2S. Como el DMA sigue corriendo mientras")
        _info("  Serial.write bloquea, el audio recibido tiene GAPS TEMPORALES:")
        _info("  cada frame es internamente correcto, pero entre frames")
        _info("  consecutivos falta señal. Eso invalida toda continuidad de fase")
        _info("  entre frames — y MUSIC depende de fase.")
        _info("  Acción: bajar SAMPLE_RATE a 10000 (firmware Y config.py), o")
        _info("  subir dma_buf_count, o mandar 3 canales en vez de 4.")
    else:
        _ok(f"El ESP32 produce {prod_rate:.1f} pkt/s ({prod_ratio*100:.0f}% del teórico)")

    if rate_ratio < 0.95:
        _warn(f"Recibidos {rate:.1f} pkt/s ({rate_ratio*100:.0f}%) — menos de los producidos")
        _info("→ La pérdida es del lado de la Pi (buffer que se desborda).")
    else:
        _ok(f"Recibidos {pkts_ok} paquetes ({rate:.1f} pkt/s)")

    if pkts_corrupt > 0:
        pct = 100 * pkts_corrupt / (pkts_ok + pkts_corrupt)
        (_bad if pct > 5 else _warn)(f"Paquetes corruptos: {pkts_corrupt} ({pct:.1f}%)")
        if pct > 5:
            _info("→ >5% es preocupante. Revisá cable, baud, HOP_SIZE")

    if pkts_lost > 0:
        pct = 100 * pkts_lost / (pkts_ok + pkts_lost)
        (_bad if pct > 5 else _warn)(f"Paquetes perdidos (gaps de counter): {pkts_lost} ({pct:.1f}%)")
    else:
        _ok("Counter sin gaps: no se perdió ningún paquete transmitido")

    return data, peak_per_frame, stats


# -----------------------------------------------------------------------------
# Chequeo 5: calidad por canal
# -----------------------------------------------------------------------------

SAT_THRESHOLD  = 32000   # |sample| >= esto cuenta como saturado (int16 max 32767)
DEAD_THRESHOLD = 20      # std < esto = canal realmente mudo (nada, ni ruido propio)
DC_THRESHOLD   = 2000    # |DC offset| > esto = sospechoso
MISMATCH_DB    = 4.0     # desbalance de ganancia tolerado contra la mediana


def band_rms(x, fs, lo, hi, nfft=1024):
    """RMS de x restringido a [lo, hi) Hz, en cuentas int16.

    Periodograma promediado (Welch sin solape) con ventana de Hann, normalizado
    por la potencia de la ventana para que la suma sobre TODOS los bins
    reproduzca la varianza de x. Se usa numpy puro: no requiere scipy, que no
    siempre está en la RPi.
    """
    x = np.asarray(x, dtype=np.float64)
    x = x - x.mean()
    nseg = len(x) // nfft
    if nseg == 0:
        return 0.0
    win  = np.hanning(nfft)
    segs = x[:nseg * nfft].reshape(nseg, nfft) * win
    P    = (np.abs(np.fft.rfft(segs, axis=1)) ** 2).mean(axis=0) / (nfft ** 2)
    P[1:-1] *= 2.0                      # espectro de un solo lado
    P /= (win ** 2).mean()              # corrección de potencia de la ventana
    freqs = np.fft.rfftfreq(nfft, 1.0 / fs)
    m = (freqs >= lo) & (freqs < hi)
    return float(np.sqrt(P[m].sum()))


def channel_metrics(data, peak_per_frame, ch, fs):
    """Todos los estadísticos de un canal, en un dict."""
    x = data[:, ch].astype(np.float64)
    n = len(x)
    dc  = float(x.mean())
    std = float(x.std())                       # RMS con DC removido
    p999 = float(np.percentile(np.abs(x), 99.9))
    peak = int(peak_per_frame[:, ch].max())
    n_sat = int(np.count_nonzero(np.abs(x) >= SAT_THRESHOLD))
    return dict(
        dc=dc, std=std, p999=p999, peak=peak,
        pct_sat=100.0 * n_sat / n,
        frames_clip=int(np.count_nonzero(peak_per_frame[:, ch] >= SAT_THRESHOLD)),
        n_frames=peak_per_frame.shape[0],
        rms_lf   = band_rms(x, fs, 0.0,     BAND_LO),
        rms_band = band_rms(x, fs, BAND_LO, BAND_HI),
        rms_hf   = band_rms(x, fs, BAND_HI, fs / 2.0),
    )


def analyze_channels(data, peak_per_frame, num_ch, fs):
    _section("5. Calidad de los canales")

    n_per_ch = data.shape[0]
    n_frames = peak_per_frame.shape[0]
    _info(f"Analizando {n_per_ch} muestras/canal sobre {n_frames} frames "
          f"({n_per_ch / fs:.2f} s de audio)")
    _info(f"rms = desviación estándar (DC removido). Bandas: "
          f"LF <{BAND_LO:.0f} Hz | MUSIC {BAND_LO:.0f}-{BAND_HI:.0f} Hz | HF >{BAND_HI:.0f} Hz")
    print()

    hdr = (f"    {'ch':>3} {'rms':>8} {'p99.9':>8} {'peak':>8} {'dc':>8} "
           f"{'rms_lf':>8} {'rms_band':>9} {'rms_hf':>8} {'gan.dB':>7} {'%sat':>6}  estado")
    print(hdr)
    print("    " + "─" * (len(hdr) - 4))

    all_m = [channel_metrics(data, peak_per_frame, ch, fs) for ch in range(num_ch)]

    # Referencia de ganancia: mediana de rms_band entre canales. Los 4 mics ven
    # el MISMO campo acústico (5 cm de separación, ruido ambiente difuso), así
    # que sus niveles deben coincidir dentro de pocos dB. Una diferencia grande
    # NO es acústica: es ganancia digital, L/R, o alineación del word I2S.
    med_band = statistics.median(m['rms_band'] for m in all_m) or 1.0

    any_dead = any_clip_cont = any_clip_burst = any_lf = False
    any_mismatch = False

    for ch in range(num_ch):
        m = all_m[ch]
        m['gain_db'] = 20.0 * np.log10(max(m['rms_band'], 1e-9) / med_band)

        status = []
        if m['std'] < DEAD_THRESHOLD:
            status.append("\033[31mMUDO\033[0m (cable/L-R/firmware)")
            any_dead = True
        elif abs(m['gain_db']) > MISMATCH_DB:
            status.append(f"\033[31mGANANCIA {m['gain_db']:+.1f} dB\033[0m")
            any_mismatch = True
        elif m['pct_sat'] > 1.0:
            status.append(f"\033[31mSATURADO CONTINUO\033[0m ({m['pct_sat']:.1f}%)")
            any_clip_cont = True
        elif m['frames_clip'] > 0:
            status.append(f"\033[33mSATURADO ESPORÁDICO\033[0m "
                          f"({m['frames_clip']}/{m['n_frames']} frames)")
            any_clip_burst = True
        elif m['rms_lf'] > 3.0 * m['rms_band']:
            status.append("\033[33mDOMINADO POR LF\033[0m")
            any_lf = True
        elif abs(m['dc']) > DC_THRESHOLD:
            status.append("\033[33mDC offset alto\033[0m")
        else:
            status.append("\033[32mOK\033[0m")

        print(f"    {ch:>3} {m['std']:>8.0f} {m['p999']:>8.0f} {m['peak']:>8d} "
              f"{m['dc']:>+8.0f} {m['rms_lf']:>8.0f} {m['rms_band']:>9.0f} "
              f"{m['rms_hf']:>8.0f} {m['gain_db']:>+7.1f} {m['pct_sat']:>5.2f}%  "
              f"{' '.join(status)}")

    # --- ¿La atenuación es digital o acústica? ---
    # Un corrimiento de bits divide señal Y ruido por igual, así que la RELACIÓN
    # SEÑAL-RUIDO del canal no cambia. Una atenuación acústica (puerto tapado,
    # cápsula menos sensible) baja la señal y deja intacto el piso de ruido
    # electrónico, así que la SNR SÍ empeora. Eso se lee en dos lugares:
    #   · el cociente entre canales por banda: si en HF se aplana respecto de
    #     LF/banda, hay un piso aditivo que no escala.
    #   · la correlación entre canales (chequeo 6): un canal con menos SNR
    #     correlaciona peor con el resto.
    if any_mismatch and num_ch >= 2:
        fuertes = [m for m in all_m if m['gain_db'] > 0]
        debiles = [m for m in all_m if m['gain_db'] <= -MISMATCH_DB]
        if fuertes and debiles:
            f = statistics.fmean(m['rms_band'] for m in fuertes)
            d = statistics.fmean(m['rms_band'] for m in debiles)
            f_hf = statistics.fmean(m['rms_hf'] for m in fuertes)
            d_hf = statistics.fmean(m['rms_hf'] for m in debiles)
            r_band = f / d if d else 0
            r_hf = f_hf / d_hf if d_hf else 0
            _info(f"Cociente fuertes/débiles por banda: "
                  f"MUSIC {r_band:.2f}×  |  HF {r_hf:.2f}×")
            if r_hf > 0 and r_band / r_hf > 1.5:
                _bad("El cociente SE APLANA en HF: la atenuación NO es un "
                     "corrimiento de bits")
                _info("→ Un corrimiento de bits divide señal y ruido por igual y")
                _info("  daría el MISMO cociente en todas las bandas. Que en HF")
                _info("  baje quiere decir que hay un piso de ruido aditivo que")
                _info("  no se atenúa: la señal acústica llega debilitada, pero")
                _info("  el ruido electrónico del canal es el mismo.")
                # Estimar el piso aditivo y la atenuación real
                g = r_band
                sig_hf = f_hf / g
                nu = (max(d_hf ** 2 - sig_hf ** 2, 0.0)) ** 0.5
                _info(f"  Modelo: señal atenuada {20*np.log10(g):.1f} dB más un")
                _info(f"  piso fijo de ~{nu:.0f} cuentas rms, igual en los 4 canales.")
                _info("  Causas compatibles: puerto acústico tapado o sellado en")
                _info("  esos micrófonos, cápsula de menor sensibilidad, o el")
                _info("  módulo montado con la abertura contra la placa.")
                _info("  NO compatible: desalineación de bits del I2S.")
            else:
                _info("El cociente se mantiene en todas las bandas: compatible")
                _info("con un factor digital (corrimiento de bits o ganancia).")

    print()
    if any_mismatch:
        _info("DESBALANCE DE GANANCIA entre canales. Los 4 mics están a 5 cm en el")
        _info("  mismo campo difuso: sus niveles TIENEN que coincidir dentro de ~3 dB.")
        _info("  Una diferencia mayor no es acústica. Sospechosos, en orden:")
        _info("   1. Corrimiento de bits del word I2S en un canal (un factor 2 exacto")
        _info("      = 1 bit; 4 = 2 bits). Mirá si el cociente entre grupos da 2^k.")
        _info("   2. Orden L/R invertido en UNO de los buses: el patrón de canales")
        _info("      afectados queda 'cruzado' (0 y 3, o 1 y 2) en vez de por bus.")
        _info("   3. GAIN_SHIFT distinto por canal en el firmware (la variante NO-sync")
        _info("      usa GAIN_SHIFT_L = GAIN_SHIFT+1; la sync usa el mismo para todos).")
        _info("  IMPORTANTE para MUSIC: el steering a(θ) no modela ganancias por")
        _info("  canal. Con x = G·(A·s + n) y G diagonal, el subespacio de ruido de")
        _info("  G·R·Gᴴ ya no es ortogonal a a(θ): el pico se ensancha y se sesga.")
        _info("  Se PUEDE corregir por software con una calibración diagonal (dividir")
        _info("  cada canal por su ganancia medida acá) y es práctica estándar. Lo que")
        _info("  el software NO recupera es el rango dinámico perdido: un canal 12 dB")
        _info("  abajo llega al piso de cuantización 12 dB antes. Conviene arreglar")
        _info("  el origen y dejar la calibración diagonal solo para el residuo.")
    if any_dead:
        _info("Canal mudo → revisar L/R wiring (GND vs VDD), cable SD del bus, o el INMP441")
    if any_clip_cont:
        _info("Saturación CONTINUA: subí GAIN_SHIFT en el firmware. Ojo: para MUSIC")
        _info("  la ganancia absoluta es IRRELEVANTE (el algoritmo es invariante a")
        _info("  escala y además hay DIGITAL_GAIN aguas abajo). Lo único que")
        _info("  importa es NO recortar. Ante la duda, subí GAIN_SHIFT.")
    if any_clip_burst:
        _info("Saturación ESPORÁDICA: eventos puntuales, o bit-corruption del bus.")
        _info("  Si p99.9 es mucho menor que peak, son outliers aislados.")
    if any_lf:
        _info("Canal DOMINADO POR LF: la energía <200 Hz supera 3× la de la banda de")
        _info("  trabajo. Es lo que hace bailar el rms de banda ancha entre corridas.")
        _info("  Fuentes típicas: HVAC, puertas, manipular el array, deriva del bias")
        _info("  del INMP441, o masa/cable largo en ESE mic. NO afecta a MUSIC")
        _info("  (que filtra a 200-2400 Hz), pero SÍ al detector de energía si el")
        _info("  piso se calibró en otro momento.")
    if not (any_dead or any_clip_cont or any_clip_burst or any_lf):
        _ok("Los canales presentan señal limpia, sin clipping ni dominancia LF")

    return all_m


# -----------------------------------------------------------------------------
# Chequeo 6: coherencia y desfase entre canales
# -----------------------------------------------------------------------------

def _bandpass_fft(x, fs, lo, hi):
    """Filtro ideal por FFT (fase cero). x: (N, ch)."""
    n = x.shape[0]
    X = np.fft.rfft(x, axis=0)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    X[(f < lo) | (f >= hi), :] = 0.0
    return np.fft.irfft(X, n=n, axis=0)


def _lag_samples(a, b, maxlag=32):
    """Desfase entero (muestras) que maximiza la correlación cruzada.

    Convención: lag > 0 significa que b va RETRASADO respecto de a.
    """
    n = len(a)
    nfft = 1 << (2 * n - 1).bit_length()
    A = np.fft.rfft(a, nfft)
    B = np.fft.rfft(b, nfft)
    cc = np.fft.irfft(A * np.conj(B), nfft)
    cc = np.concatenate([cc[-maxlag:], cc[:maxlag + 1]])
    return int(np.argmax(cc) - maxlag)


def analyze_coherence(data, fs, num_ch):
    """¿Son 4 micrófonos reales, independientes y sincronizados?

    A menos de 200 Hz la longitud de onda supera 1.7 m contra 5 cm de apertura:
    los 4 mics ven prácticamente la MISMA presión, así que la correlación entre
    canales debe ser ALTA si los cuatro son micrófonos reales en el mismo campo.
    Un canal con r ≈ 0 contra todos los demás no está captando acústica.

    OJO: la correlación NO sirve para detectar canales duplicados. En LF dos
    mics sanos a 5 cm dan r > 0.999 igual que una copia literal — no hay margen
    para separarlos. La duplicación se detecta comparando las MUESTRAS: si un
    slot del bus no se llena y repite el word del otro canal, los int16 son
    idénticos bit a bit, cosa que dos micrófonos reales jamás producen.

    En la banda MUSIC la correlación baja (el ruido difuso decorrelaciona) pero
    el DESFASE debe seguir siendo de pocas muestras: a 7.07 cm (la diagonal) el
    retardo máximo físico es 0.0707/343 = 206 µs ≈ 2.3 muestras a 11025 Hz. Un
    desfase mayor entre canales de buses distintos = los buses NO están
    sincronizados, que es justo lo que el firmware master/slave debe garantizar.
    """
    _section("6. Coherencia y sincronismo entre canales")

    # --- Duplicación: comparación bit a bit, no correlación ------------------
    raw = np.asarray(data, dtype=np.int32)
    dup_found = False
    for i in range(num_ch):
        for j in range(i + 1, num_ch):
            frac = float(np.mean(raw[:, i] == raw[:, j]))
            if frac > 0.99:
                _bad(f"ch{i} y ch{j} son la MISMA señal ({100*frac:.1f}% de muestras idénticas)")
                _info("     → No son dos micrófonos. Un slot del bus no se llena y")
                _info("       repite el word del otro canal, o el de-interleave está mal.")
                _info("       Con canales duplicados la covarianza pierde rango y")
                _info("       MUSIC no puede separar señal de ruido.")
                dup_found = True
                continue
            # Mismo dato con un corrimiento de bits (ganancia 2^k exacta)
            for k in (1, 2, 3, 4):
                if float(np.mean(raw[:, i] == (raw[:, j] >> k))) > 0.95:
                    _bad(f"ch{i} == ch{j} >> {k}: misma señal con {k} bit(s) de corrimiento")
                    _info(f"     → Diferencia de ganancia de exactamente {6.02*k:.0f} dB por")
                    _info("       alineación del word I2S, no por acústica.")
                    dup_found = True
    if not dup_found:
        _ok("Ningún par de canales comparte muestras: son 4 señales distintas")
    print()

    x = data.astype(np.float64)
    x -= x.mean(axis=0)

    C_lf = C_band = None
    for label, lo, hi in (("LF  <200 Hz", 1.0, BAND_LO),
                          ("banda MUSIC", BAND_LO, BAND_HI)):
        y = _bandpass_fft(x, fs, lo, hi)
        C = np.corrcoef(y.T)
        if lo < BAND_LO:
            C_lf = C
        else:
            C_band = C
        print(f"    correlación en {label}:")
        print("        " + "".join(f"{c:>8d}" for c in range(num_ch)))
        for i in range(num_ch):
            row = "".join(f"{C[i, j]:>8.3f}" for j in range(num_ch))
            print(f"     ch{i}{row}")
        print()

    for i in range(num_ch):
        others = [C_lf[i, j] for j in range(num_ch) if j != i]
        if max(others) < 0.3:
            _bad(f"ch{i} no correlaciona con ninguno en LF (max r={max(others):.2f})")
            _info("     → Ese canal no está captando el campo acústico común:")
            _info("       SD flotante, mic muerto, o slot que nunca se llena.")

    # Desfase entre canales, medido sobre la banda de trabajo.
    # La referencia NO puede ser el canal de mayor nivel (si el canal roto es el
    # más ruidoso, todo lo demás aparece desfasado). Se usa el canal mejor
    # correlacionado con el resto: el que con más certeza es un mic sano.
    y = _bandpass_fft(x, fs, BAND_LO, BAND_HI)
    med_corr = [statistics.median(abs(C_band[i, j]) for j in range(num_ch) if j != i)
                for i in range(num_ch)]
    ref = int(np.argmax(med_corr))
    max_fisico = MIC_MAX_DELAY_SAMPLES(fs)
    print(f"    desfase contra ch{ref} (referencia = mejor correlacionado con el resto):")
    bad_lag = False
    for c in range(num_ch):
        lag = 0 if c == ref else _lag_samples(y[:, ref], y[:, c])
        flag = ""
        if abs(lag) > max_fisico:
            flag = f"  ← supera el máximo físico ({max_fisico} muestras)"
            bad_lag = True
        print(f"     ch{c}: {lag:+d} muestras{flag}")
    print()
    if bad_lag:
        _bad("Hay canales desfasados más de lo que la geometría permite")
        _info("→ Si el desfase agrupa a ch0/ch1 contra ch2/ch3, los DOS BUSES I2S")
        _info("  no están sincronizados: revisá los jumpers GPIO26→14 y GPIO25→15,")
        _info("  y que el slave se instale ANTES que el master en setup().")
        _info("  Con los buses desfasados, la elevación (que sale de comparar la")
        _info("  fila de arriba con la de abajo) es directamente inutilizable.")
    else:
        _ok(f"Todos los desfases están dentro del máximo físico ({max_fisico} muestras)")
        _info("Ojo: esto valida el sincronismo GRUESO. Con ruido difuso el pico de")
        _info("correlación es ancho; no confunde con calibración fina de fase.")


def MIC_MAX_DELAY_SAMPLES(fs, aperture_m=0.0707, c=343.0):
    """Retardo máximo físico entre dos mics del array, en muestras (redondeado
    hacia arriba y con 1 muestra de margen). Apertura = diagonal del cuadrado."""
    return int(np.ceil(aperture_m / c * fs)) + 1


# -----------------------------------------------------------------------------
# Chequeo 7: repetibilidad entre ventanas
# -----------------------------------------------------------------------------

def analyze_repeatability(runs, num_ch):
    """runs: lista de listas de dicts de channel_metrics (una por ventana)."""
    _section(f"7. Repetibilidad entre {len(runs)} ventanas")

    _info("Coeficiente de variación (CV = desvío/media entre ventanas).")
    _info("CV < 10% = estable | 10-30% = ruidoso | >30% = algo cambia de verdad.")
    _info("La columna que IMPORTA para el sistema es cv(rms_band).")
    print()

    hdr = (f"    {'ch':>3} {'rms medio':>10} {'cv(rms)':>9} "
           f"{'rms_band medio':>15} {'cv(rms_band)':>13} {'cv(rms_lf)':>11} "
           f"{'dc min..max':>14} {'peak min..max':>18}")
    print(hdr)
    print("    " + "─" * (len(hdr) - 4))

    verdicts = []
    dcs_por_ch = []
    for ch in range(num_ch):
        stds  = [r[ch]['std']      for r in runs]
        bands = [r[ch]['rms_band'] for r in runs]
        lfs   = [r[ch]['rms_lf']   for r in runs]
        peaks = [r[ch]['peak']     for r in runs]
        dcs   = [r[ch]['dc']       for r in runs]
        dcs_por_ch.append(dcs)

        def cv(v):
            mu = statistics.fmean(v)
            if mu == 0:
                return 0.0
            sd = statistics.stdev(v) if len(v) > 1 else 0.0
            return 100.0 * sd / mu

        cv_std, cv_band, cv_lf = cv(stds), cv(bands), cv(lfs)
        verdicts.append((ch, cv_std, cv_band, cv_lf, stds))

        print(f"    {ch:>3} {statistics.fmean(stds):>10.0f} {cv_std:>8.1f}% "
              f"{statistics.fmean(bands):>15.0f} {cv_band:>12.1f}% "
              f"{cv_lf:>10.1f}% {min(dcs):>+6.0f}..{max(dcs):<+6.0f} "
              f"{min(peaks):>8d}..{max(peaks):<8d}")

    # --- El DC como tercera evidencia sobre el origen del desbalance ---------
    # Un factor de escala DIGITAL multiplica todo, incluida la continua. Por eso:
    #   · si los canales débiles estuvieran DIVIDIDOS, tendrían el DC más CHICO;
    #   · si los fuertes estuvieran MULTIPLICADOS, tendrían el DC más GRANDE.
    # Observar lo contrario (débiles con más |DC|) descarta las dos versiones
    # digitales y apoya el mismo modelo que el aplanamiento en HF: la señal
    # acústica llega atenuada mientras que un término aditivo del canal (offset
    # y ruido electrónico) no se atenúa.
    medias_band = [statistics.fmean(r[ch]['rms_band'] for r in runs)
                   for ch in range(num_ch)]
    ref_band = statistics.median(medias_band) or 1.0
    debiles = [c for c in range(num_ch)
               if 20 * np.log10(medias_band[c] / ref_band) < -MISMATCH_DB]
    fuertes = [c for c in range(num_ch)
               if 20 * np.log10(medias_band[c] / ref_band) > MISMATCH_DB]
    if debiles and fuertes:
        dc_deb = statistics.fmean(abs(statistics.fmean(dcs_por_ch[c]))
                                  for c in debiles)
        dc_fue = statistics.fmean(abs(statistics.fmean(dcs_por_ch[c]))
                                  for c in fuertes)
        disp_deb = statistics.fmean(max(dcs_por_ch[c]) - min(dcs_por_ch[c])
                                    for c in debiles)
        print()
        _info(f"|DC| medio — canales débiles {dc_deb:.0f}, fuertes {dc_fue:.0f} "
              f"(dispersión del DC entre ventanas en los débiles: {disp_deb:.0f})")
        if dc_deb > 2 * dc_fue and dc_deb > 3 * 0 + 10:
            if disp_deb > dc_deb:
                _warn("Los débiles tienen MÁS |DC|, pero el DC varía tanto entre "
                      "ventanas como su propio valor")
                _info("→ Apunta en la misma dirección que el aplanamiento en HF")
                _info("  (término aditivo que no se atenúa), pero por sí solo no")
                _info("  alcanza: puede ser deriva del bias del INMP441.")
            else:
                _bad("Los canales débiles tienen MÁS |DC| que los fuertes, y de "
                     "forma estable")
                _info("→ Descarta las dos versiones digitales del desbalance: un")
                _info("  factor de escala multiplica también la continua, así que")
                _info("  canales DIVIDIDOS tendrían MENOS DC y canales")
                _info("  MULTIPLICADOS tendrían MÁS. Se observa lo contrario.")
                _info("  Confirma el modelo: señal acústica atenuada, con el")
                _info("  offset y el ruido del canal intactos.")

    print()
    for ch, cv_std, cv_band, cv_lf, stds in verdicts:
        if cv_band < 10.0 and cv_std >= 20.0:
            _ok(f"ch{ch}: la banda de trabajo es ESTABLE (cv {cv_band:.0f}%); "
                f"la variación del rms ancho ({cv_std:.0f}%) es LF/DC.")
            _info("     → No es un problema del sistema. Ignorá el rms de banda ancha.")
        elif cv_band >= 30.0:
            _bad(f"ch{ch}: la banda de trabajo VARÍA en serio (cv {cv_band:.0f}%)")
            _info("     → Acá sí hay que investigar. Chequeo útil: relación entre")
            _info("       ventanas. Si los rms son ~múltiplos de 2 entre sí, es un")
            _info("       corrimiento de bit/word del I2S (típico del bus SLAVE si")
            _info("       el jumper de WS/SCK engancha con un flanco distinto en")
            _info("       cada reset). Si varían de forma continua, es acústico o")
            _info("       de contacto (soldadura fría en SD, masa marginal).")
            ratios = sorted(v / min(stds) for v in stds if min(stds) > 0)
            if ratios:
                _info(f"       rms relativos entre ventanas: "
                      f"{', '.join(f'{r:.2f}×' for r in ratios)}")
        elif cv_band >= 10.0:
            _warn(f"ch{ch}: banda de trabajo algo ruidosa (cv {cv_band:.0f}%) — "
                  f"normal si el ambiente no es un cuarto anecoico.")

    # --- Vector de calibración diagonal, listo para pegar en config.py -------
    medias = [statistics.fmean(r[ch]['rms_band'] for r in runs)
              for ch in range(num_ch)]
    ref = statistics.median(medias) or 1.0
    cvs = [100 * (statistics.stdev([r[ch]['rms_band'] for r in runs]) /
                  statistics.fmean(r[ch]['rms_band'] for r in runs))
           if len(runs) > 1 else 0.0 for ch in range(num_ch)]
    if max(abs(20 * np.log10(m / ref)) for m in medias) > MISMATCH_DB:
        print()
        _info("Calibración diagonal sugerida (compensa el desbalance por "
              "software):")
        _info("  CHANNEL_GAINS = [" +
              ", ".join(f"{ref/m:.4f}" for m in medias) + "]")
        _info(f"  Medido sobre {len(runs)} ventanas; dispersión por canal: " +
              ", ".join(f"{c:.1f}%" for c in cvs))
        _info("  Pegalo en config.py. Corrige el SESGO del DOA (el steering de")
        _info("  MUSIC asume ganancias iguales), pero NO recupera la relación")
        _info("  señal-ruido: amplificar un canal débil amplifica también su")
        _info("  ruido. Es un parche válido para seguir midiendo, no el arreglo.")


# -----------------------------------------------------------------------------
# Chequeo 8: repetibilidad ENTRE REINICIOS  (--reboots N)
# -----------------------------------------------------------------------------

def _reboot_verdict(rows, num_ch, alcance):
    """Tabla + veredicto compartido por --reboots y --cold-boots.

    alcance: 'esp32' (reset por EN: los mics NO se reinicializan) o
             'frio'  (corte de alimentación: los mics TAMBIÉN arrancan de cero).
    """
    print()
    print(f"    rms en banda útil, y entre paréntesis log2(ch/max) del arranque:")
    hdr = "    boot " + "".join(f"{'ch%d' % c:>18}" for c in range(num_ch))
    print(hdr)
    print("    " + "─" * (len(hdr) - 4))

    todos_log2, patrones = [], []
    for b, r in enumerate(rows):
        mx = max(r) or 1.0
        l2 = [np.log2(v / mx) if v > 0 else -99 for v in r]
        todos_log2 += [v for v in l2 if v > -90]
        patrones.append(tuple(1 if v > -0.5 else 0 for v in l2))
        print(f"    {b+1:>4} " + "".join(f"{r[c]:>10.0f} ({l2[c]:>+5.2f})"
                                         for c in range(num_ch)))
    print()

    err = [abs(v - round(v)) for v in todos_log2]
    err_med = statistics.median(err)
    n_grid  = sum(1 for e in err if e < 0.25)
    # Sesgo: un corrimiento de bits REAL da log2 centrado en el entero. Si todos
    # los desvíos caen del mismo lado, hay un término extra que no es de bits.
    sesgo = [v - round(v) for v in todos_log2 if abs(v - round(v)) > 1e-3]
    sesgo_med = statistics.median(sesgo) if sesgo else 0.0

    print(f"    Desvío de log2(ch/max) contra el entero más cercano:")
    print(f"      mediana |desvío| {err_med:.3f}  |  {n_grid}/{len(err)} valores "
          f"a menos de 0.25 (±1.5 dB) de una potencia de 2")
    print(f"      desvío CON SIGNO (mediana): {sesgo_med:+.3f} "
          f"({6.02*sesgo_med:+.1f} dB)")

    patron_estable = len(set(patrones)) == 1
    print(f"    Patrón de canales fuertes: "
          f"{'CONSTANTE' if patron_estable else 'CAMBIA'} entre arranques "
          f"({len(set(patrones))} patrones distintos en {len(rows)})")
    print()

    en_grilla   = err_med < 0.25 and n_grid >= 0.7 * len(err)
    sesgo_fuerte = abs(sesgo_med) > 0.10          # >0.6 dB sistemático

    if en_grilla and not patron_estable:
        _bad("ALINEACIÓN DE BITS DEL I2S — no es un problema de los módulos")
        _info("Los niveles caen sobre potencias de 2 y el patrón se re-sortea en")
        _info("cada arranque. Ninguna falla física hace eso.")
        _sugerencias_bitslip()
        return
    if en_grilla and sesgo_fuerte:
        _warn("Cerca de potencias de 2, pero con un SESGO sistemático "
              f"de {6.02*sesgo_med:+.1f} dB")
        _info("Un corrimiento de bits puro da log2 centrado en el entero, con")
        _info("desvíos a ambos lados. Que TODOS caigan del mismo lado dice que")
        _info("además del factor 2^k hay un término continuo — o sea, mezcla de")
        _info("corrimiento de bits Y diferencia analógica de sensibilidad.")
        _info("No lo trates como una sola causa.")
    if patron_estable and alcance == 'esp32':
        print()
        _bad("OJO: este test NO reinicializa los micrófonos")
        _info("El pulso RTS/DTR baja EN del ESP32, pero los INMP441 siguen")
        _info("ALIMENTADOS todo el tiempo: su contador interno de bits NO se")
        _info("reinicia. Si el corrimiento está en el micrófono (que es la")
        _info("hipótesis), este test no lo puede mover, y un patrón constante")
        _info("acá NO descarta nada.")
        _info("Para exigirlo de verdad hace falta cortar la alimentación:")
        _info("    python3 diagnose_serial.py <puerto> --cold-boots 8")
        _info("(desenchufás y volvés a enchufar el USB entre medición y medición)")
    elif patron_estable and alcance == 'frio':
        _ok("Patrón CONSTANTE con corte real de alimentación")
        _info("Los micrófonos arrancaron de cero 10 veces y siempre dio lo mismo:")
        _info("el desbalance es DETERMINISTA. Eso saca del banco la hipótesis del")
        _info("bit-slip aleatorio y deja: sensibilidad real de esos módulos,")
        _info("cableado/soldadura, o un corrimiento fijo por formato I2S.")
        _info("Siguiente paso: swap físico, pero comparando 5 arranques fríos")
        _info("ANTES contra 5 DESPUÉS — una sola observación no alcanza.")
    elif not en_grilla and not patron_estable:
        _warn("El patrón cambia pero los niveles NO caen en potencias de 2")
        _info("No es corrimiento de bits limpio. Sospechá de alimentación:")
        _info("4 INMP441 + ESP32 desde el mismo 3.3V, con caídas en el arranque.")
        _info("Poné 100 nF por mic pegado a su pin de VDD y medí VDD en el boot.")
    elif not en_grilla:
        _ok("Ganancias estables entre arranques y fuera de la grilla de 2^k")
        _info("El desbalance es físico: módulos, soldaduras o cableado.")


def _sugerencias_bitslip():
    _info("Lo que se reinicializa en cada boot es el contador de bits del")
    _info("INMP441. Causas, en orden de probabilidad:")
    _info("")
    _info(" 1. BCLK con divisor FRACCIONARIO. A 11025 Hz y 32 bits/slot el")
    _info("    BCLK es 64*11025 = 705600 Hz, y desde PLL_D2 (160 MHz) el")
    _info("    divisor da 226.757 — no entero. El periférico lo sintetiza")
    _info("    alternando períodos de distinta duración, y esos flancos")
    _info("    irregulares son justo lo que hace patinar el contador del mic.")
    _info("      a) use_apll = true SOLO en el bus MASTER (el slave recibe el")
    _info("         clock por jumper, así que la limitación no aplica).")
    _info("      b) SAMPLE_RATE = 10000: BCLK = 640000 y 160e6/640000 = 250")
    _info("         EXACTO. Baja además el enlace de 96% a 89% de ocupación.")
    _info("")
    _info(" 2. INTEGRIDAD DE SEÑAL en SCK/WS: una salida maneja 4 mics MÁS dos")
    _info("    jumpers, sin terminación. Poné 33-100 ohm en SERIE pegados al")
    _info("    pin del ESP32 y acortá los cables.")
    _info("")
    _info(" 3. SECUENCIA DE ARRANQUE: instalar los drivers, i2s_stop(), esperar")
    _info("    ~200 ms con los mics ya alimentados, y recién ahí i2s_start().")


def analyze_reboots(ser, hop, num_ch, duration_s, fs, n_boots):
    """Ganancia por canal en N resets del ESP32 (pulso EN por RTS/DTR).

    LIMITACIÓN IMPORTANTE: este reset NO corta la alimentación de los INMP441.
    Reinicia el ESP32 y su periférico I2S, pero los micrófonos siguen con VDD y
    conservan su estado interno. Sirve para exigir el lado ESP32; para exigir el
    lado micrófono hace falta --cold-boots.
    """
    _section(f"8. Repetibilidad entre RESETS DEL ESP32 ({n_boots} arranques)")
    _info("Reset por EN (RTS/DTR): reinicia el ESP32, NO los micrófonos.")

    rows, first_counters = [], []
    for b in range(n_boots):
        print(f"    arranque {b+1}/{n_boots}...", flush=True)
        reset_esp32(ser)
        time.sleep(2.0)                  # asentamiento del bias
        ser.reset_input_buffer()
        res = decode_packets(ser, hop, num_ch, duration_s, fs, quiet=True)
        if res is None:
            _warn(f"arranque {b+1}: no se pudo decodificar, se descarta")
            continue
        data, peaks, st = res
        rows.append([channel_metrics(data, peaks, c, fs)['rms_band']
                     for c in range(num_ch)])
        first_counters.append(st['first_counter'])

    if len(rows) < 2:
        _bad("No se juntaron suficientes arranques válidos")
        return

    # ¿El reset ocurrió de verdad? El firmware arranca frame_counter en 0, así
    # que tras un reset real el primer counter debe ser chico y NO acumularse.
    # Si RTS/DTR no está cableado a EN (pasa en varios adaptadores), la llamada
    # no lanza excepción y el test mide 10 veces el MISMO arranque.
    print()
    print(f"    primer counter de cada arranque: "
          f"{', '.join(str(c) for c in first_counters)}")
    creciente = all(b > a for a, b in zip(first_counters[:-1], first_counters[1:]))
    if creciente or min(first_counters) > 2000:
        _bad("El ESP32 NO se está reiniciando: el counter sigue acumulando")
        _info("RTS/DTR no está llegando a EN en este adaptador. Todo lo que")
        _info("sigue mide UN SOLO arranque repetido y no significa nada.")
        _info("Usá --cold-boots, que no depende de RTS/DTR.")
        return
    _ok("Reset confirmado: el counter reinicia en cada arranque")

    _reboot_verdict(rows, num_ch, alcance='esp32')


def esperar_arranque(port, baud, seconds=6.0):
    """Escucha el arranque EN FRÍO desde el primer byte.

    Con --no-reset el chip ya arrancó cuando lo enchufaste, así que el banner
    (que sale una sola vez, ~500 ms después del boot) se perdió mucho antes de
    que abriéramos el puerto. La única forma de verlo en frío es estar
    escuchando ANTES de energizar: se espera a que el nodo del puerto
    desaparezca (USB desenchufado) y vuelva a aparecer, y se abre de inmediato.
    """
    _section("Captura del arranque en frío")
    print(f"    >>> DESENCHUFÁ el USB del ESP32...", flush=True)
    t0 = time.time()
    while os.path.exists(port):
        if time.time() - t0 > 120:
            _bad("Timeout esperando que se desenchufe")
            return
        time.sleep(0.1)
    print("        desconectado. Ahora volvé a enchufarlo...", flush=True)
    t0 = time.time()
    while not os.path.exists(port):
        if time.time() - t0 > 120:
            _bad("Timeout esperando la reconexión")
            return
        time.sleep(0.05)

    # Abrir lo antes posible: el banner sale ~500 ms después del boot y el nodo
    # tarda un instante en tener permisos.
    ser = None
    t0 = time.time()
    while ser is None and time.time() - t0 < 10.0:
        try:
            ser = serial.Serial(port, baud, timeout=0.05)
        except (serial.SerialException, PermissionError, OSError):
            time.sleep(0.05)
    if ser is None:
        _bad("No se pudo abrir el puerto tras la reconexión")
        return
    print(f"        puerto abierto {1000*(time.time()-t0):.0f} ms después de "
          f"aparecer; escuchando {seconds:.0f} s...", flush=True)

    try:
        _apply_raw_termios(ser)
        # NO se resetea ni se vacía el buffer: se quiere justamente lo primero
        # que emite el chip al energizarse.
        data = capturar_arranque(ser, seconds=seconds)
    finally:
        ser.close()

    tasa = len(data) / seconds / 1024.0
    _info(f"{len(data)} bytes capturados desde el arranque ({tasa:.1f} KB/s "
          f"promedio, incluye el tiempo de boot en que no transmite)")
    print()

    if _buscar_banner(data, avisar_si_falta=False) is None:
        _warn("Sin banner en el arranque en frío")
        _info("→ NO significa necesariamente que el firmware falle. El banner")
        _info("  sale por el UART ~500 ms después de energizar, pero el puente")
        _info("  USB-serial tarda en enumerarse y el nodo /dev/ttyUSB0 recién")
        _info("  aparece después. Todo lo que el ESP32 emita ANTES de que el")
        _info("  host esté escuchando se pierde en el aire, y eso incluye el")
        _info("  banner. Es una carrera que no se puede ganar desde la Pi.")
        _info("  Para leerlo, forzá un reset con el puerto ya abierto:")
        _info("      python3 diagnose_serial.py <puerto> --raw")
        _info("  Lo que SÍ vale de esta captura es si los paquetes que siguen")
        _info("  tienen framing — que es la pregunta de fondo.")
        print()

    dump_raw_bytes(data, 1 + 2 + 256 * 4 * BYTES_PER_SAMPLE + 1)


def analyze_cold_boots(port, baud, hop, num_ch, duration_s, fs, n_boots):
    """Ganancia por canal en N arranques EN FRÍO (corte de alimentación).

    Es la única forma de re-sortear el estado interno de los INMP441: hay que
    quitarles VDD. El script espera a que el nodo del puerto desaparezca (USB
    desenchufado) y vuelva a aparecer, y ahí mide.
    """
    _section(f"8. Repetibilidad entre ARRANQUES EN FRÍO ({n_boots} ciclos)")
    _info("A diferencia del reset por EN, esto SÍ reinicializa los micrófonos.")
    print()

    rows = []
    for b in range(n_boots):
        print(f"    >>> ciclo {b+1}/{n_boots}: DESENCHUFÁ el USB del ESP32...",
              flush=True)
        t0 = time.time()
        while os.path.exists(port):
            if time.time() - t0 > 120:
                _bad("Timeout esperando que se desenchufe"); return
            time.sleep(0.2)
        print("        desconectado. Volvé a enchufarlo...", flush=True)
        t0 = time.time()
        while not os.path.exists(port):
            if time.time() - t0 > 120:
                _bad("Timeout esperando la reconexión"); return
            time.sleep(0.2)
        time.sleep(1.5)          # que el driver termine de crear el nodo

        ser = None
        for _ in range(10):      # los permisos del nodo tardan un instante
            try:
                ser = serial.Serial(port, baud, timeout=2.0)
                break
            except (serial.SerialException, PermissionError):
                time.sleep(0.5)
        if ser is None:
            _warn(f"ciclo {b+1}: no se pudo abrir el puerto, se descarta")
            continue
        try:
            fd = ser.fileno()
            a = termios.tcgetattr(fd)
            a[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY |
                      termios.INLCR | termios.IGNCR | termios.ICRNL)
            a[1] &= ~termios.OPOST
            a[3] &= ~(termios.ECHO | termios.ECHOE | termios.ICANON)
            termios.tcsetattr(fd, termios.TCSANOW, a)
            time.sleep(2.0)      # asentamiento del bias de los INMP441
            ser.reset_input_buffer()
            res = decode_packets(ser, hop, num_ch, duration_s, fs, quiet=True)
            if res is None:
                _warn(f"ciclo {b+1}: no se pudo decodificar, se descarta")
                continue
            data, peaks, _ = res
            rows.append([channel_metrics(data, peaks, c, fs)['rms_band']
                         for c in range(num_ch)])
            print(f"        ok: " + "  ".join(f"ch{c}={rows[-1][c]:.0f}"
                                              for c in range(num_ch)))
        finally:
            ser.close()

    if len(rows) < 2:
        _bad("No se juntaron suficientes arranques válidos")
        return

    _reboot_verdict(rows, num_ch, alcance='frio')

    print()
    print(f"    rms en banda útil, y entre paréntesis log2(ch/max) del arranque:")
    hdr = "    boot " + "".join(f"{'ch%d' % c:>18}" for c in range(num_ch))
    print(hdr)
    print("    " + "─" * (len(hdr) - 4))

    todos_log2 = []
    patrones   = []
    for b, r in enumerate(rows):
        mx = max(r) or 1.0
        l2 = [np.log2(v / mx) if v > 0 else -99 for v in r]
        todos_log2 += [v for v in l2 if v > -90]
        # patrón = qué canales quedaron "arriba" (dentro de 3 dB del máximo)
        patrones.append(tuple(1 if v > -0.5 else 0 for v in l2))
        print(f"    {b+1:>4} " + "".join(f"{r[c]:>10.0f} ({l2[c]:>+5.2f})"
                                         for c in range(num_ch)))
    print()

    # ¿Los niveles relativos caen sobre la grilla de potencias de 2?
    err = [abs(v - round(v)) for v in todos_log2]
    err_med = statistics.median(err)
    n_grid  = sum(1 for e in err if e < 0.25)
    print(f"    Desvío de log2(ch/max) contra el entero más cercano:")
    print(f"      mediana {err_med:.3f}   |   {n_grid}/{len(err)} valores "
          f"a menos de 0.25 (±1.5 dB) de una potencia de 2")

    patron_estable = len(set(patrones)) == 1
    print(f"    Patrón de canales fuertes: "
          f"{'CONSTANTE' if patron_estable else 'CAMBIA'} entre arranques "
          f"({len(set(patrones))} patrones distintos en {len(rows)})")
    print()

    en_grilla = err_med < 0.25 and n_grid >= 0.7 * len(err)

    if en_grilla and not patron_estable:
        _bad("ALINEACIÓN DE BITS DEL I2S — no es un problema de los módulos")
        _info("Los niveles caen sobre potencias de 2 y el patrón se re-sortea en")
        _info("cada arranque. Ninguna falla física hace eso: una soldadura fría o")
        _info("una cápsula dañada dan el MISMO resultado todos los arranques.")
        _info("Lo que se reinicializa en cada boot es el contador de bits del")
        _info("INMP441. Causas, en orden de probabilidad:")
        _info("")
        _info(" 1. BCLK con divisor FRACCIONARIO. A 11025 Hz y 32 bits/slot el")
        _info("    BCLK es 64*11025 = 705600 Hz, y desde PLL_D2 (160 MHz) el")
        _info("    divisor da 226.757 — no entero. El periférico lo sintetiza")
        _info("    alternando períodos de distinta duración, y esos flancos")
        _info("    irregulares son justo lo que hace patinar el contador del mic.")
        _info("    Arreglos (los dos son de una línea):")
        _info("      a) use_apll = true SOLO en el bus MASTER. En la versión")
        _info("         master/slave el slave recibe el clock por jumper, así que")
        _info("         la limitación de 'el APLL alimenta un solo I2S' no aplica.")
        _info("      b) SAMPLE_RATE = 10000. BCLK = 640000 Hz y 160e6/640000 = 250")
        _info("         EXACTO. Ademas baja el enlace de 96% a 89% de ocupación.")
        _info("         No cuesta nada al algoritmo: el aliasing espacial ya topa")
        _info("         la localización en ~2426 Hz, muy por debajo del nuevo")
        _info("         Nyquist de 5000 Hz.")
        _info("")
        _info(" 2. INTEGRIDAD DE SEÑAL en SCK/WS. Una sola salida maneja 4 mics")
        _info("    MÁS dos jumpers a GPIO14/15, sin terminación. A 705 kHz con")
        _info("    flancos rápidos, la reflexión en las puntas puede dar un doble")
        _info("    flanco que el mic cuenta y el ESP32 no. Poné 33-100 ohm en")
        _info("    SERIE a la salida del ESP32 (uno por línea, pegado al pin),")
        _info("    acortá los cables y evitá topología en cadena larga.")
        _info("")
        _info(" 3. SECUENCIA DE ARRANQUE. Si el clock ya está corriendo mientras")
        _info("    VDD de los mics todavía sube, el contador arranca en cualquier")
        _info("    lado. En setup(): instalar los drivers, i2s_stop(), esperar")
        _info("    ~200 ms con los mics ya alimentados, y recién ahí i2s_start().")
    elif en_grilla and patron_estable:
        _warn("Niveles en potencias de 2 pero con patrón ESTABLE")
        _info("Sigue oliendo a alineación de bits, pero determinista. Repetí con")
        _info("más arranques (--reboots 10): si nunca cambia, mirá el cableado")
        _info("de esos mics en particular antes que el clock.")
    elif not en_grilla and not patron_estable:
        _warn("El patrón cambia pero los niveles NO caen en potencias de 2")
        _info("No es corrimiento de bits limpio. Sospechá de alimentación:")
        _info("4 INMP441 + ESP32 desde el 3.3V del regulador de la placa, con")
        _info("caídas en el arranque. Medí VDD con osciloscopio durante el boot y")
        _info("poné 100 nF por mic pegado al pin de VDD.")
    else:
        _ok("Ganancias estables entre arranques y fuera de la grilla de 2^k")
        _info("Ahí sí el desbalance es físico: módulos, soldaduras o cableado.")


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
    p.add_argument('--repeat',   type=int,   default=1,
                   help="Ventanas de decodificación seguidas. >1 activa el "
                        "chequeo 7 (repetibilidad). Usá 5 para diagnosticar "
                        "'el rms me cambia en cada corrida'.")
    p.add_argument('--reboots',  type=int,   default=0,
                   help="Resetea el ESP32 N veces (pulso EN por RTS/DTR) y "
                        "tabula la ganancia por canal. OJO: no corta la "
                        "alimentación de los micrófonos.")
    p.add_argument('--raw', action='store_true',
                   help="Vuelca los bytes crudos y los clasifica (texto del "
                        "bootloader / silencio / basura / paquetes de otro "
                        "tamaño) en vez de intentar decodificar. Es lo primero "
                        "a correr cuando 'no decodifica'.")
    p.add_argument('--no-reset', action='store_true', dest='no_reset',
                   help="NO reiniciar el ESP32 por RTS/DTR. Imprescindible para "
                        "diagnosticar el arranque en frío: el reset lleva al "
                        "chip al estado 'bueno' y borra el síntoma antes de "
                        "poder medirlo.")
    p.add_argument('--watch', type=float, default=0.0, metavar='SEGUNDOS',
                   help="Vigila el stream N segundos y detecta REINICIOS del "
                        "ESP32 (saltos del counter hacia cero + huecos de "
                        "transmisión). No depende del log de la ROM, que puede "
                        "estar silenciado por GPIO15. Corrélo justo después de "
                        "enchufar: --watch 30")
    p.add_argument('--scan-baud', action='store_true', dest='scan_baud',
                   help="Prueba varios baudios y dice en cuál aparece el "
                        "framing. Separa 'el stream es basura' de 'son paquetes "
                        "válidos leídos a la velocidad equivocada'.")
    p.add_argument('--wait-boot', action='store_true', dest='wait_boot',
                   help="Espera a que desenchufes y vuelvas a enchufar el USB, "
                        "y abre el puerto de inmediato para capturar el "
                        "arranque EN FRÍO desde el primer byte (incluido el "
                        "banner del firmware, que sale una sola vez).")
    p.add_argument('--probe-boot', action='store_true', dest='probe_boot',
                   help="Lee el puerto a 115200 para capturar el log de la ROM "
                        "del ESP32: dice la CAUSA del reset (POWERON / watchdog "
                        "/ software) y si quedó en modo bootloader.")
    p.add_argument('--cold-boots', type=int, default=0, dest='cold_boots',
                   help="Igual pero con arranque EN FRÍO: el script espera que "
                        "desenchufes y vuelvas a enchufar el USB en cada ciclo. "
                        "Es el único que reinicializa los INMP441, y por lo "
                        "tanto el único que puede mover un corrimiento de bits "
                        "originado en el micrófono. Usá 6-8 ciclos.")
    args = p.parse_args()

    print(f"\n\033[1mDiagnóstico ESP32 → RPi\033[0m")
    print(f"  Puerto   : {args.port}")
    print(f"  Baud     : {args.baud}")
    print(f"  Hop size : {args.hop} muestras")
    print(f"  Canales  : {args.ch}")
    print(f"  fs       : {args.fs} Hz")

    data_byterate     = args.fs * args.ch * BYTES_PER_SAMPLE
    framing_byterate  = 4 * (args.fs / args.hop)
    expected_byterate = data_byterate + framing_byterate

    if not check_environment(args.port):
        sys.exit(1)

    # Estos modos manejan el puerto por su cuenta (abren/cierran a otro baud).
    if args.wait_boot:
        esperar_arranque(args.port, args.baud)
        print()
        return

    if args.scan_baud:
        scan_bauds(args.port, args.hop, args.ch, do_reset=not args.no_reset)
        print()
        return

    # El log de la ROM sale a 115200, no al baud del firmware: puerto aparte.
    if args.probe_boot:
        probe_boot_log(args.port, do_reset=not args.no_reset)
        print()
        return

    # El modo en frío gestiona el puerto por su cuenta (lo abre y lo cierra en
    # cada ciclo), así que no se abre acá.
    if args.cold_boots > 0:
        analyze_cold_boots(args.port, args.baud, args.hop, args.ch,
                           args.duration, args.fs, args.cold_boots)
        print()
        return

    ser = open_serial(args.port, args.baud, do_reset=not args.no_reset)
    if ser is None:
        sys.exit(1)

    try:
        if args.raw:
            dump_raw(ser, 1 + 2 + args.hop * args.ch * BYTES_PER_SAMPLE + 1)
            return

        if args.watch > 0:
            watch_stream(ser, args.hop, args.ch, args.fs, args.watch)
            return

        ok, _ = measure_byte_rate(ser, args.duration, expected_byterate, args.baud)
        if not ok:
            sys.exit(1)

        if args.reboots > 0:
            analyze_reboots(ser, args.hop, args.ch, args.duration, args.fs,
                            args.reboots)
            return

        runs = []
        for i in range(args.repeat):
            quiet = (i > 0)
            if quiet:
                print(f"    ventana {i+1}/{args.repeat}...", flush=True)
            result = decode_packets(ser, args.hop, args.ch, args.duration,
                                    args.fs, quiet=quiet)
            if result is None:
                if i == 0:
                    sys.exit(1)
                _warn(f"Ventana {i+1} falló, se descarta")
                continue
            data, peaks, _stats = result
            if i == 0:
                metrics = analyze_channels(data, peaks, args.ch, args.fs)
                analyze_coherence(data, args.fs, args.ch)
            else:
                metrics = [channel_metrics(data, peaks, c, args.fs)
                           for c in range(args.ch)]
            runs.append(metrics)

        if len(runs) > 1:
            analyze_repeatability(runs, args.ch)
        elif args.repeat == 1:
            print()
            _info("Sugerencia: corré con --repeat 5 para medir cuánto varían estos")
            _info("números entre ventanas antes de sacar conclusiones de una sola.")

    finally:
        ser.close()
        print()
        _info("Puerto cerrado.")
    print()


if __name__ == '__main__':
    main()
