"""
audio_input.py — Captura de audio desde el ESP32 por USB-serial.

Implementación:
    - Modo RAW via termios: CRÍTICO para recibir datos binarios correctamente.
      Sin esto, el driver tty de Linux intercepta bytes como 0x11 (XON),
      0x13 (XOFF), 0x0D (CR→LF), etc., corrompiendo el stream.
    - Reset RTS/DTR al abrir: resetea el ESP32 a modo RUN automáticamente
      (secuencia estilo esptool: DTR bajo + pulso RTS), evitando tener que
      presionar el botón físico y sin riesgo de caer en bootloader.
    - Sincronización por SYNC_BYTE/END_BYTE: el lector siempre busca el
      byte de inicio antes de leer un paquete completo. Esto permite
      recuperarse de paquetes truncados o corruptos sin perder sincronía.
    - Tracking de paquetes perdidos: compara el contador del ESP32 con el
      esperado para detectar drops de paquetes.

También incluye SimulatedAudioInput para probar el pipeline sin hardware.
"""

import struct
import time
import threading
import queue
import termios
import numpy as np

SYNC_BYTE = 0xAA
END_BYTE  = 0x55


class SerialAudioInput:
    """
    Lee frames de 4 canales desde el ESP32 por serial en un hilo dedicado.
    Pone los frames en una cola thread-safe para el hilo de procesamiento.
    """

    def __init__(self, port, baud, hop_size, num_channels=4, bits_per_sample=16):
        assert bits_per_sample in (8, 16), "bits_per_sample debe ser 8 o 16"
        self.port      = port
        self.baud      = baud
        self.hop_size  = hop_size
        self.num_ch    = num_channels
        self.bits      = bits_per_sample
        bytes_per_samp = bits_per_sample // 8
        self.pkt_bytes = hop_size * num_channels * bytes_per_samp

        self._queue    = queue.Queue(maxsize=8)
        self._running  = False
        self._thread   = None
        self._ser      = None

        # Estadísticas
        self.pkts_received = 0
        self.pkts_lost     = 0
        self.pkts_corrupt  = 0
        self._last_counter = None

    def start(self):
        import serial
        self._ser = serial.Serial()
        self._ser.port     = self.port
        self._ser.baudrate = self.baud
        self._ser.timeout  = 2.0
        self._ser.open()

        # Modo RAW — sin esto los bytes de control se pierden o se reinterpretan.
        # IXON/IXOFF/IXANY: desactiva XON/XOFF (0x11/0x13 corromperían el stream).
        # ECHO/ICANON: sin eco ni modo canónico.
        # INLCR/IGNCR/ICRNL: sin traducción CR↔LF (0x0D en una muestra int16
        #   se convertiría a 0x0A o se descartaría, corrompiendo el framing).
        # OPOST: sin post-procesado de salida.
        fd = self._ser.fileno()
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
        attrs[3] &= ~(termios.ECHO | termios.ECHOE | termios.ICANON)
        attrs[0] &= ~(termios.INLCR | termios.IGNCR | termios.ICRNL)
        attrs[1] &= ~termios.OPOST
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

        # Reset del ESP32 a modo RUN vía RTS/DTR. En las placas DevKit el
        # circuito de auto-reset (dos transistores) mapea EN←!RTS y GPIO0←!DTR,
        # y pyserial levanta DTR/RTS al abrir el puerto: un toggle de DTR solo
        # puede dejar el chip en BOOTLOADER según el adaptador (CP2102/CH340).
        # Secuencia segura (la misma que usa esptool para "hard reset"):
        # DTR desactivado (GPIO0=alto → modo RUN) y pulso de RTS (EN).
        self._ser.dtr = False
        self._ser.rts = True
        time.sleep(0.1)
        self._ser.rts = False
        time.sleep(0.5)                  # esperar boot del ESP32
        self._ser.reset_input_buffer()   # descartar la salida del boot ROM

        self._running = True
        self._thread  = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            self._ser.close()

    def read_frame(self, timeout=1.0):
        """
        Retorna un frame (ndarray float64, shape (hop_size, 4)) o None si timeout.
        Los valores están normalizados a [-1, 1].
        """
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _reader_loop(self):
        while self._running:
            try:
                self._sync()
                pkt = self._read_packet()
                if pkt is not None:
                    try:
                        self._queue.put_nowait(pkt)
                    except queue.Full:
                        pass  # descarta frame si el procesador no da abasto
            except Exception:
                time.sleep(0.01)

    def _sync(self):
        """Avanza en el stream hasta encontrar SYNC_BYTE."""
        while self._running:
            b = self._ser.read(1)
            if b and b[0] == SYNC_BYTE:
                return
        raise StopIteration

    def _read_packet(self):
        # Leer contador de paquete (2 bytes big-endian)
        cnt_bytes = self._ser.read(2)
        if len(cnt_bytes) < 2:
            self.pkts_corrupt += 1
            return None

        counter = (cnt_bytes[0] << 8) | cnt_bytes[1]

        # Detectar paquetes perdidos
        if self._last_counter is not None:
            expected = (self._last_counter + 1) & 0xFFFF
            if counter != expected:
                lost = (counter - expected) & 0xFFFF
                self.pkts_lost += lost
        self._last_counter = counter

        # Leer datos (hop_size * 4 canales * 2 bytes)
        raw = self._ser.read(self.pkt_bytes)
        if len(raw) < self.pkt_bytes:
            self.pkts_corrupt += 1
            return None

        # Leer END_BYTE
        end = self._ser.read(1)
        if not end or end[0] != END_BYTE:
            self.pkts_corrupt += 1
            return None

        self.pkts_received += 1

        # Decodificar muestras interleaved → (hop_size, 4) float64 normalizado [-1,1)
        # El tipo y la escala dependen de la resolución configurada:
        #   8-bit  → int8,  escala /128.0   (firmware legado, 22050 Hz)
        #   16-bit → int16, escala /32768.0  (firmware actual, 11025 Hz)
        # ORDEN DE BYTES (16-bit): LITTLE-ENDIAN ('<i2'). El ESP32 es
        # little-endian y el firmware hace Serial.write del arreglo int16 SIN
        # byte-swap, así que los bytes salen LSB primero. (El contador de paquete
        # del header sí va big-endian, pero se decodifica aparte con shifts, así
        # que el endianness de las MUESTRAS es independiente de ese.)
        if self.bits == 16:
            samples = np.frombuffer(raw, dtype='<i2').reshape(self.hop_size, self.num_ch)
            return samples.astype(np.float64) / 32768.0
        else:
            samples = np.frombuffer(raw, dtype=np.int8).reshape(self.hop_size, self.num_ch)
            return samples.astype(np.float64) / 128.0


class SimulatedAudioInput:
    """
    Genera audio sintético de una onda plana desde una dirección conocida.
    Útil para verificar el algoritmo sin hardware.
    """

    def __init__(self, mic_positions, sample_rate, hop_size,
                 azimuth_deg=45.0, elevation_deg=0.0,
                 speed_of_sound=343.0, snr_db=20.0):
        self.mic_pos  = np.array(mic_positions)
        self.fs       = sample_rate
        self.hop_size = hop_size
        self.c        = speed_of_sound
        self.snr      = 10 ** (snr_db / 20.0)
        self._t       = 0

        # Calcular vector de dirección y retardos entre micrófonos
        az = np.radians(azimuth_deg)
        el = np.radians(elevation_deg)
        # Vector unitario apuntando hacia la fuente (onda plana → fuente en infinito)
        d = np.array([np.sin(az) * np.cos(el),
                      np.cos(az) * np.cos(el),
                      np.sin(el)])
        # Retardo de propagación de cada micrófono (segundos). Coincide con la
        # convención del MUSICEngine: el micrófono m recibe s(t - _delays[m]).
        self._delays = self.mic_pos @ d / self.c  # (M,)

        # Tonos fijos dentro de la banda de análisis. Una suma de senoides
        # produce una señal de banda ancha COHERENTE entre micrófonos, con
        # frecuencias acotadas por debajo de Nyquist (sin riesgo de aliasing).
        self._tones = np.linspace(350.0, 3300.0, 9)

    def read_frame(self, timeout=None):
        # Tiempo absoluto: avanza entre frames para mantener la continuidad
        # de fase de la señal simulada.
        t = np.arange(self._t, self._t + self.hop_size) / self.fs
        self._t += self.hop_size

        M = self.mic_pos.shape[0]
        frame = np.zeros((self.hop_size, M))
        for m in range(M):
            # Onda plana: el micrófono m recibe la señal retardada
            # _delays[m] segundos. El retardo se aplica de forma ANALÍTICA
            # (dentro del argumento del seno) → exacto, fraccionario, sin
            # efectos de borde.
            sig = np.zeros(self.hop_size)
            for f in self._tones:
                sig += np.sin(2 * np.pi * f * (t - self._delays[m]))
            noise = np.random.randn(self.hop_size) / self.snr
            frame[:, m] = sig + noise

        return frame

    def start(self): pass
    def stop(self):  pass
