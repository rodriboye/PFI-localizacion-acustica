"""
audio_input.py — Fuentes de audio, todas con la misma interfaz
(start / read_frame / stop / stats / eof):

    SerialAudioInput    — captura desde el ESP32 por USB-serial.
    WavAudioInput       — reproduce un WAV de capture_wav.py (diagnóstico).
    SimulatedAudioInput — onda plana sintética, sin hardware.
"""

import time
import threading
import queue
import termios
import numpy as np

SYNC_BYTE = 0xAA
END_BYTE  = 0x55


class SerialAudioInput:
    """Lee frames de 4 canales desde el ESP32 en un hilo dedicado."""

    def __init__(self, port, baud, hop_size, num_channels=4, bits_per_sample=16,
                 queue_size=8, drop_policy='newest'):
        assert bits_per_sample in (8, 16), "bits_per_sample debe ser 8 o 16"
        assert drop_policy in ('newest', 'oldest')
        self.port      = port
        self.baud      = baud
        self.hop_size  = hop_size
        self.num_ch    = num_channels
        self.bits      = bits_per_sample
        bytes_per_samp = bits_per_sample // 8
        self.pkt_bytes = hop_size * num_channels * bytes_per_samp

        # Descarte cuando el consumidor no da abasto:
        #   'newest' → se tira el recién llegado; la latencia queda clavada en
        #              queue_size hops.
        #   'oldest' → se tira el más viejo; el consumidor ve lo más reciente.
        # Default 'newest'
        self.drop_policy = drop_policy

        self._queue    = queue.Queue(maxsize=queue_size)
        self._running  = False
        self._thread   = None
        self._ser      = None

        # Estadísticas
        self.pkts_received = 0
        self.pkts_lost     = 0
        self.pkts_corrupt  = 0
        self._last_counter = None

        # Contrapresión: frames LEÍDOS BIEN y tirados por cola llena, o sea
        # porque el procesamiento no sostiene el tiempo real. `pkts_lost` no lo
        # cubre (mide huecos del contador del ESP32, que acá queda consecutivo):
        # un sistema que tira un tercio de sus frames reporta lost = 0.
        self.pkts_dropped    = 0
        self.queue_high_water = 0

    # Intentos de arranque antes de darse por vencido (ver _arranque_verificado).
    MAX_INTENTOS_ARRANQUE = 4

    def start(self, verbose=True):
        import serial
        self._ser = serial.Serial()
        self._ser.port     = self.port
        self._ser.baudrate = self.baud
        self._ser.timeout  = 2.0
        self._ser.open()

        self._configurar_termios()
        self._arranque_verificado(verbose=verbose)

        self._running = True
        self._thread  = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    # ARRANQUE VERIFICADO. El primer arranque tras conectar la alimentación no
    # produce paquetes decodificables: la línea transmite pero sin protocolo. La
    # causa raíz no está identificada, pero la recuperación
    # —reabrir el puerto— es determinista. Por eso el arranque no se da por bueno
    # hasta CONFIRMAR paquetes válidos, en vez de lanzar el lector a ciegas.

    def _configurar_termios(self):
        """Modo binario puro sobre el descriptor actual: sin control de flujo
        por software, sin traducción CR↔LF, sin post-procesado, sin eco.

        ISTRIP, INPCK, PARMRK, IGNPAR y BRKINT se fijan EXPLÍCITAMENTE: cualquiera
        rompe audio binario en silencio (ISTRIP enmascara a 7 bits, PARMRK inserta
        bytes de marca) y si no quedarían como los dejó el proceso anterior.
        """
        fd = self._ser.fileno()
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY |
                      termios.INLCR | termios.IGNCR | termios.ICRNL |
                      termios.ISTRIP | termios.INPCK | termios.PARMRK |
                      termios.IGNPAR | termios.BRKINT | termios.IGNBRK)
        attrs[1] &= ~termios.OPOST
        attrs[3] &= ~(termios.ECHO | termios.ECHOE | termios.ICANON |
                      termios.ISIG | termios.IEXTEN)
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

    def _pulso_reset(self):
        """Reset del ESP32 a modo RUN vía RTS/DTR.

        En las DevKit el auto-reset mapea EN←!RTS y GPIO0←!DTR, y pyserial levanta
        ambos al abrir: togglear DTR puede dejar el chip en BOOTLOADER. La
        secuencia segura (el "hard reset" de esptool) es DTR bajo (GPIO0 alto →
        modo RUN) más un pulso de RTS.
        """
        self._ser.dtr = False
        self._ser.rts = True
        time.sleep(0.1)
        self._ser.rts = False
        time.sleep(0.5)                  # esperar boot del ESP32
        self._ser.reset_input_buffer()   # descartar la salida del boot ROM

    def _hay_framing(self, timeout_s=2.0):
        """¿Llegan paquetes válidos? Exige SYNC y END en su lugar en DOS
        paquetes consecutivos con counters que difieran en 1: un solo 0xAA no
        alcanza, en audio ~1 de cada 256 bytes vale 0xAA."""
        n = 1 + 2 + self.pkt_bytes + 1        # SYNC + counter + datos + END
        buf = bytearray()
        t0 = time.time()
        while len(buf) < 3 * n and time.time() - t0 < timeout_s:
            c = self._ser.read(3 * n - len(buf))
            if not c:
                break
            buf += c
        if len(buf) < 3 * n:
            return False
        for i in range(len(buf) - 2 * n + 1):
            if buf[i] != SYNC_BYTE or buf[i + n - 1] != END_BYTE:
                continue
            if buf[i + n] != SYNC_BYTE or buf[i + 2 * n - 1] != END_BYTE:
                continue
            c1 = (buf[i + 1] << 8) | buf[i + 2]
            c2 = (buf[i + n + 1] << 8) | buf[i + n + 2]
            if ((c2 - c1) & 0xFFFF) == 1:
                return True
        return False

    def _reabrir(self):
        """Cierra y reabre el puerto, reaplicando termios.

        Esto es lo que arregla el fallo del primer arranque, no el pulso de EN:
        varios pulsos sobre el mismo descriptor fallan y reabrir engancha al
        primer intento. Lo que se reconstruye es el estado del puente USB-serial,
        que con el descriptor abierto no se reenvía.
        """
        import serial
        try:
            self._ser.close()
        except Exception:
            pass
        time.sleep(0.4)
        self._ser = serial.Serial()
        self._ser.port     = self.port
        self._ser.baudrate = self.baud
        self._ser.timeout  = 2.0
        self._ser.open()
        self._configurar_termios()

    def _arranque_verificado(self, verbose=True):
        for intento in range(1, self.MAX_INTENTOS_ARRANQUE + 1):
            self._pulso_reset()
            if self._hay_framing():
                if verbose and intento > 1:
                    print(f"[audio] ESP32 enganchado tras reabrir el puerto "
                          f"{intento-1} vez/veces (el primer arranque después "
                          f"de energizar suele fallar)", flush=True)
                self._ser.reset_input_buffer()
                return
            if verbose:
                print(f"[audio] intento {intento}/{self.MAX_INTENTOS_ARRANQUE}: "
                      f"sin paquetes válidos, reabriendo el puerto...",
                      flush=True)
            if intento < self.MAX_INTENTOS_ARRANQUE:
                self._reabrir()

        raise RuntimeError(
            f"El ESP32 en {self.port} no entregó paquetes válidos tras "
            f"{self.MAX_INTENTOS_ARRANQUE} aperturas del puerto.\n"
            f"  Diagnosticá con:  python3 diagnose_serial.py {self.port} --raw\n"
            f"  Si la línea transmite pero sin framing, es el fallo de arranque "
            f"en frío conocido: desconectá la alimentación unos segundos y "
            f"volvé a probar.")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            self._ser.close()

    def read_frame(self, timeout=1.0):
        """Frame (hop_size, 4) float64 normalizado a [-1, 1), o None si timeout."""
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    @property
    def eof(self):
        """El serial nunca termina (existe por simetría con WavAudioInput)."""
        return False

    @property
    def drop_ratio(self):
        """Fracción de frames válidos tirados por contrapresión (0.0 - 1.0)."""
        total = self.pkts_received + self.pkts_dropped
        return (self.pkts_dropped / total) if total else 0.0

    def stats(self):
        return {
            'received':   self.pkts_received,
            'lost':       self.pkts_lost,
            'corrupt':    self.pkts_corrupt,
            'dropped':    self.pkts_dropped,
            'drop_ratio': self.drop_ratio,
            'q_high':     self.queue_high_water,
            'q_max':      self._queue.maxsize,
        }

    def _reader_loop(self):
        while self._running:
            try:
                self._sync()
                pkt = self._read_packet()
                if pkt is not None:
                    self._enqueue(pkt)
            except Exception:
                time.sleep(0.01)

    def _enqueue(self, pkt):
        """Encola el frame aplicando la política de descarte, y contabiliza."""
        try:
            self._queue.put_nowait(pkt)
        except queue.Full:
            self.pkts_dropped += 1
            if self.drop_policy == 'oldest':
                try:
                    self._queue.get_nowait()     # descartar el más viejo
                    self._queue.put_nowait(pkt)  # y meter el nuevo
                except (queue.Empty, queue.Full):
                    pass
            # 'newest': el frame recién leído se pierde acá.
        q = self._queue.qsize()
        if q > self.queue_high_water:
            self.queue_high_water = q

    def _sync(self):
        """Avanza en el stream hasta encontrar SYNC_BYTE."""
        while self._running:
            b = self._ser.read(1)
            if b and b[0] == SYNC_BYTE:
                return
        raise StopIteration

    def _read_packet(self):
        # Contador de paquete: 2 bytes BIG-endian (shifts explícitos, así que es
        # independiente del endianness de las muestras).
        cnt_bytes = self._ser.read(2)
        if len(cnt_bytes) < 2:
            self.pkts_corrupt += 1
            return None

        counter = (cnt_bytes[0] << 8) | cnt_bytes[1]

        if self._last_counter is not None:
            expected = (self._last_counter + 1) & 0xFFFF
            if counter != expected:
                lost = (counter - expected) & 0xFFFF
                self.pkts_lost += lost
        self._last_counter = counter

        raw = self._ser.read(self.pkt_bytes)
        if len(raw) < self.pkt_bytes:
            self.pkts_corrupt += 1
            return None

        end = self._ser.read(1)
        if not end or end[0] != END_BYTE:
            self.pkts_corrupt += 1
            return None

        self.pkts_received += 1

        # Muestras interleaved → (hop_size, 4) float64 normalizado.
        #   8-bit  → int8,  /128.0    (firmware legado)
        #   16-bit → int16, /32768.0  (firmware actual)
        # ORDEN DE BYTES (16-bit): LITTLE-endian ('<i2'). El ESP32 es
        # little-endian y el firmware escribe el arreglo int16 sin byte-swap.
        if self.bits == 16:
            samples = np.frombuffer(raw, dtype='<i2').reshape(self.hop_size, self.num_ch)
            return samples.astype(np.float64) / 32768.0
        else:
            samples = np.frombuffer(raw, dtype=np.int8).reshape(self.hop_size, self.num_ch)
            return samples.astype(np.float64) / 128.0


class WavAudioInput:
    """
    Reproduce un WAV de `capture_wav.py` por la misma interfaz que
    SerialAudioInput, para separar dos preguntas que en vivo están mezcladas:
    ¿el ALGORITMO detecta esta señal?, ¿la PLATAFORMA lo sostiene?

      realtime=False (default) — offline: sin hilos, sin cola, sin reloj. Es
          imposible perder un frame.
      realtime=True — un hilo productor empuja frames al ritmo real hacia una
          cola acotada con la misma política de descarte que el serial:
          contrapresión real con una entrada determinista.

    Normalización idéntica a SerialAudioInput (int16 / 32768.0): capture_wav.py
    escribe los mismos bytes que llegan por serial.
    """

    def __init__(self, path, hop_size, num_channels=4, expected_fs=None,
                 realtime=False, queue_size=8, drop_policy='newest', loop=False):
        import wave

        self.path      = path
        self.hop_size  = hop_size
        self.num_ch    = num_channels
        self.realtime  = realtime
        self.loop      = loop
        self.drop_policy = drop_policy

        w = wave.open(path, 'rb')
        ch = w.getnchannels()
        fs = w.getframerate()
        sw = w.getsampwidth()
        raw = w.readframes(w.getnframes())
        w.close()

        if sw != 2:
            raise ValueError(f"{path}: se esperaban muestras int16 (2 bytes), "
                             f"el WAV tiene {sw} bytes/muestra.")
        if ch != num_channels:
            raise ValueError(f"{path}: el WAV tiene {ch} canales, el pipeline "
                             f"espera {num_channels}.")

        self.sample_rate = fs
        self.fs_mismatch = (expected_fs is not None and fs != expected_fs)
        if self.fs_mismatch:
            # No se resamplea: cambiaría los retardos entre micrófonos.
            print(f"[wav] ADVERTENCIA: el WAV declara {fs} Hz y config.py usa "
                  f"{expected_fs} Hz. NO se resamplea — verificá que el header "
                  f"sea correcto, porque el fs afecta directamente al DOA.")

        x = np.frombuffer(raw, dtype='<i2').reshape(-1, ch)
        n_hops = x.shape[0] // hop_size          # el hop parcial final se descarta
        self._data = (x[:n_hops * hop_size].astype(np.float64) / 32768.0
                      ).reshape(n_hops, hop_size, ch)

        self.total_frames   = n_hops
        self.duration_s     = n_hops * hop_size / float(fs)
        self._pos           = 0
        self._producer_done = False

        # Mismos nombres que SerialAudioInput: el display y el resumen final no
        # distinguen la fuente.
        self.pkts_received    = 0
        self.pkts_lost        = 0
        self.pkts_corrupt     = 0
        self.pkts_dropped     = 0
        self.queue_high_water = 0

        self._queue   = queue.Queue(maxsize=queue_size) if realtime else None
        self._running = False
        self._thread  = None

    # --- interfaz común ---------------------------------------------------
    def start(self):
        if self.realtime:
            self._running = True
            self._thread  = threading.Thread(target=self._producer_loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def read_frame(self, timeout=1.0):
        if not self.realtime:
            # Camino offline: sincrónico, sin cola, cero pérdidas.
            if self._pos >= self.total_frames:
                if self.loop:
                    self._pos = 0
                else:
                    self._producer_done = True
                    return None
            frame = self._data[self._pos]
            self._pos += 1
            self.pkts_received += 1
            return frame

        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    @property
    def eof(self):
        """True cuando no queda audio por entregar (corta el lazo de main.py)."""
        if self.loop:
            return False
        if not self.realtime:
            return self._pos >= self.total_frames
        return self._producer_done and self._queue.empty()

    @property
    def drop_ratio(self):
        total = self.pkts_received + self.pkts_dropped
        return (self.pkts_dropped / total) if total else 0.0

    def stats(self):
        return {
            'received':   self.pkts_received,
            'lost':       self.pkts_lost,
            'corrupt':    self.pkts_corrupt,
            'dropped':    self.pkts_dropped,
            'drop_ratio': self.drop_ratio,
            'q_high':     self.queue_high_water,
            'q_max':      self._queue.maxsize if self._queue else 0,
        }

    # --- productor a ritmo real -------------------------------------------
    def _producer_loop(self):
        """Empuja frames al ritmo del firmware (hop/fs por frame).

        Agendado por DEADLINE ABSOLUTO (t0 + k*periodo) y no por sleep(periodo):
        dormir el período en cada vuelta acumula el overhead de la iteración y el
        reloj se atrasa, que es justo el error que haría parecer que el consumidor
        va más rápido de lo que va.
        """
        period = self.hop_size / float(self.sample_rate)
        t0 = time.perf_counter()
        k  = 0
        while self._running and self._pos < self.total_frames:
            target = t0 + k * period
            dt = target - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
            frame = self._data[self._pos]
            self._pos += 1
            k += 1
            self.pkts_received += 1
            try:
                self._queue.put_nowait(frame)
            except queue.Full:
                self.pkts_dropped += 1
                if self.drop_policy == 'oldest':
                    try:
                        self._queue.get_nowait()
                        self._queue.put_nowait(frame)
                    except (queue.Empty, queue.Full):
                        pass
            q = self._queue.qsize()
            if q > self.queue_high_water:
                self.queue_high_water = q
        self._producer_done = True


class SimulatedAudioInput:
    """Onda plana sintética desde una dirección conocida, para verificar el
    algoritmo sin hardware."""

    def __init__(self, mic_positions, sample_rate, hop_size,
                 azimuth_deg=45.0, elevation_deg=0.0,
                 speed_of_sound=343.0, snr_db=20.0):
        self.mic_pos  = np.array(mic_positions)
        self.fs       = sample_rate
        self.hop_size = hop_size
        self.c        = speed_of_sound
        self.snr      = 10 ** (snr_db / 20.0)
        self._t       = 0

        az = np.radians(azimuth_deg)
        el = np.radians(elevation_deg)
        # Unitario HACIA la fuente (onda plana → fuente en el infinito)
        d = np.array([np.sin(az) * np.cos(el),
                      np.cos(az) * np.cos(el),
                      np.sin(el)])
        # El mic m recibe s(t - delays[m]). SIGNO NEGATIVO: d apunta hacia la
        # fuente, así que el mic con mayor (r_m . d) es el más cercano y tiene el
        # retardo MENOR. Es el modelo FÍSICO, independiente del steering de
        # doa_engine.py — replicar aquel signo hacía que --simulate pareciera
        # funcionar porque el error se cancelaba solo en simulación.
        self._delays = -(self.mic_pos @ d) / self.c  # (M,)

        # Suma de senoides: señal de banda ancha COHERENTE entre micrófonos.
        # Techo en 2300 Hz: por encima de los 2426 del aliasing espacial de la
        # diagonal (y de FREQ_MAX) el array no puede localizar. MUSIC los
        # enmascaraba con su ROI, pero SRP corre la GCC-PHAT sobre toda la banda
        # y los tonos aliaseados le degradaban el mapa.
        self._tones = np.linspace(350.0, 2300.0, 9)

    def read_frame(self, timeout=None):
        # Tiempo absoluto: continuidad de fase entre frames.
        t = np.arange(self._t, self._t + self.hop_size) / self.fs
        self._t += self.hop_size

        M = self.mic_pos.shape[0]
        frame = np.zeros((self.hop_size, M))
        for m in range(M):
            # Retardo ANALÍTICO (dentro del seno): exacto, fraccionario y sin
            # efectos de borde.
            sig = np.zeros(self.hop_size)
            for f in self._tones:
                sig += np.sin(2 * np.pi * f * (t - self._delays[m]))
            noise = np.random.randn(self.hop_size) / self.snr
            frame[:, m] = sig + noise

        return frame

    def start(self): pass
    def stop(self):  pass

    @property
    def eof(self):
        """La simulación es un generador infinito."""
        return False
