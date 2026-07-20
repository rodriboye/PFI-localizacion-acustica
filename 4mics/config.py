"""
config.py — Parámetros centrales del sistema DOA (4 micrófonos INMP441 + MUSIC).

Todos los valores ajustables están aquí. El resto del código los importa desde
este módulo. Para cambiar el comportamiento del sistema, solo hay que editar
este archivo.

CONFIGURACIÓN POR DEFECTO DEL SISTEMA
--------------------------------------------------------------------------------
Sin ningún flag, `python3 main.py` corre:
    seguimiento (servo tracking) + motor MUSIC + audio serial del ESP32 +
    gate espectral armónico (confirma firma de dron antes de localizar).
Las demás opciones se explicitan por línea de comandos (ver main.py --help).

CADENA DE AUDIO — 11.025 kHz / 16-bit, COHERENTE EN TODAS LAS ETAPAS
--------------------------------------------------------------------------------
fs = 11025 Hz, 16 bits/muestra (BYTES_PER_SAMPLE=2), little-endian (nativo ESP32).
Estos valores deben coincidir en: firmware (.ino), config.py, audio_input.py,
capture_wav.py, diagnose_serial.py y check_capture.py.

Por qué 11 kHz/16-bit y no 22 kHz: el aliasing espacial de la diagonal del array
(7.07 cm) topa la localización en ~2425 Hz; por encima no se puede localizar a
NINGUNA tasa de muestreo, así que el ancho de banda extra de 22 kHz es inútil
para DOA. Lo que sí importa es el rango dinámico: 16-bit (~96 dB) vs 8-bit
(~48 dB) ayuda a detectar los armónicos débiles de un dron lejano. Además, a
921600 baud el enlace serial solo entra al 96% con 11025×4ch×2B≈88 kB/s;
22 kHz/16-bit (≈176 kB/s) NO entra (~1.9× la capacidad). Ver
docs/notes/deteccion_espectral.md §6.
"""

import numpy as np

# =============================================================================
# GEOMETRÍA DEL ARRAY
# =============================================================================

# Distancia entre micrófonos adyacentes (lado del cuadrado), en metros.
# Afecta: resolución angular, frecuencia máxima sin aliasing espacial.
#
# Aliasing espacial: NO lo fija el lado del cuadrado sino la MAYOR distancia
# entre cualquier par de micrófonos — en un cuadrado, la diagonal d·√2.
#   f_max = c / (2 · d·√2) = 343 / (2 · 0.0707) ≈ 2426 Hz   (con d=0.05 m)
# El valor c/(2·d) ≈ 3430 Hz aplica solo a pares adyacentes y NO es el
# límite práctico del array.
MIC_DISTANCE = 0.05   # lado del cuadrado, 5 cm (placa soldada).
                      # Aliasing espacial lo fija la diagonal d*sqrt(2)=7.07 cm:
                      #   f_max = c/(2*d*sqrt(2)) = 343/(2*0.0707) ~ 2426 Hz
                      # -> FREQ_MAX se fija en 2400 (ver abajo).

# Posiciones 3D de los 4 micrófonos en el array vertical (plano XZ).
# El array está montado perpendicular al suelo, mirando hacia adelante (+Y).
# Coordenadas [x, y, z] en metros.
#
#   Mic 2 (0, 0, d) ─────── Mic 3 (d, 0, d)
#        │                        │
#        │          (+Y)          │
#        │           ↑            │
#   Mic 0 (0, 0, 0) ─────── Mic 1 (d, 0, 0)
#
d = MIC_DISTANCE
MIC_POSITIONS = np.array([
    [0, 0, 0],
    [d, 0, 0],
    [0, 0, d],
    [d, 0, d],
], dtype=np.float64)

# Inversión del signo de la ELEVACIÓN en los cálculos. Es independiente de
# SERVO_EL_INVERT (que solo invierte la dirección MECÁNICA del servo): esto
# corrige el VALOR estimado que se muestra en la terminal y se guarda en el log.
#
# La elevación sale del eje Z (comparar la fila de arriba mic2/mic3 con la de
# abajo mic0/mic1). Si el montaje físico tiene el Z al revés del modelo, una
# fuente hacia ARRIBA aparece como hacia abajo (elevación invertida). Se corrige
# negando la coordenada Z de los micrófonos —el origen del signo— en un solo
# lugar, así queda consistente en ambos motores DOA y en check_capture.
#   True  → fuente arriba = elevación positiva (corrige el montaje invertido)
#   False → convención original del modelo
DOA_EL_INVERT = True
if DOA_EL_INVERT:
    MIC_POSITIONS = MIC_POSITIONS.copy()
    MIC_POSITIONS[:, 2] *= -1.0

# =============================================================================
# AUDIO  (11.025 kHz / 16-bit, little-endian — coherente en toda la cadena)
# =============================================================================

SAMPLE_RATE = 11025  # Hz — DEBE coincidir con el firmware del ESP32.
                     # Elegido 11025 (no 22050): el aliasing espacial topa la
                     # localización en ~2425 Hz, así que el ancho de banda de
                     # 22 kHz es inútil para DOA; y a 921600 baud solo entra
                     # 11025×4ch×2B≈88 kB/s (~96%). Con 16-bit:
                     #   · 49 bins en [200-2400 Hz] (R mejor condicionada)
                     #   · σ_q ≈ 9e-6 (16-bit) vs 2.3e-3 (8-bit) → 4 órdenes
                     #     menos de ruido de cuantización en la covarianza.
HOP_SIZE    = 256     # muestras por frame — debe coincidir con el firmware
SPEED_OF_SOUND = 343.0  # m/s — ajustar según temperatura ambiente (~343 a 20°C)

# Bytes por muestra del protocolo serial ESP32→Pi.
#   1 → int8  (firmware legado, 22050 Hz / 8-bit)
#   2 → int16 (firmware actual, 11025 Hz / 16-bit, little-endian)
# Debe coincidir con lo que envía el firmware. Si no coincide, el framing
# de paquetes se rompe silenciosamente (bytes desfasados en cada paquete).
# ORDEN DE BYTES: little-endian (LSB primero), que es el nativo del ESP32
# (Serial.write del arreglo int16 sin byte-swap). audio_input.py / capture_wav.py
# leen con dtype '<i2' en coherencia con esto.
BYTES_PER_SAMPLE = 2

# Ganancia digital aplicada por software a cada frame tras la normalización.
# Permite compensar micrófonos débiles o ajustar el nivel de entrada al
# detector sin recompilar el firmware del ESP32.
#   1.0  → sin cambio   |   2.0 → +6 dB   |   0.5 → −6 dB
# Advertencia: valores > 4.0 pueden saturar digitalmente la señal (muestras que
# superan ±1.0 en float). Verificar con diagnose_serial.py si hay dudas.
DIGITAL_GAIN = 3.0

# =============================================================================
# ALGORITMO MUSIC
# =============================================================================

# Rango de frecuencias a incluir en el análisis broadband (Hz).
# Límite inferior: evitar DC y componentes de muy baja frecuencia.
# Límite superior: el aliasing espacial de la diagonal del array fija el techo
# duro en f ≈ c/(2·d·√2) ≈ 2426 Hz (ver comentario de MIC_DISTANCE).
#
# FREQ_MAX=2400 queda justo por debajo de ese límite y concentra el análisis
# donde está la energía de un dron (BPF y armónicos, ~100-2400 Hz).
FREQ_MIN = 200    # Drones de ala rotatoria tienen BPF (blade pass frequency)
                  # típicamente en 80-300 Hz (n_palas × RPM/60). FREQ_MIN=200
                  # captura el fundamental de drones de rotación media/rápida y
                  # la energía de vibración de motores, sin meter tanto ruido
                  # ambiental sub-200 Hz en MUSIC. (El gate espectral SÍ mira
                  # desde más abajo para clasificar; ver sección ESPECTRAL.)
FREQ_MAX = 2400   # techo de aliasing del array de 5 cm (diagonal) ~ 2426 Hz.

# Resolución angular del escaneo (grados).
# Menor = más preciso pero más lento. 5° es un buen compromiso en RPi 3A+.
ANGLE_RESOLUTION = 5

# Rango de escaneo azimut y elevación (grados).
# La grilla escanea en espacio-u (u = sin(az)) para distribuir los puntos donde
# el array tiene resolución real; la zona >±75° es ambigua por apertura
# insuficiente, por eso se limita. Para elevar este límite: aumentar MIC_DISTANCE.
AZIMUTH_MIN   = -75
AZIMUTH_MAX   =  75
# Elevación: rango REAL (mundo) que se reporta en pantalla/log. El array está
# inclinado ARRAY_TILT_DEG hacia arriba para centrar el rango útil en el
# broadside (mejor resolución). El MOTOR escanea en el marco del array:
#   el_array ∈ [ELEVATION_MIN - tilt, ELEVATION_MAX - tilt]
# y el valor reportado se convierte a real sumando el tilt: real = array + tilt.
ELEVATION_MIN =  0
ELEVATION_MAX =  70

# Inclinación física del array hacia arriba (grados). El array apunta a
# ARRAY_TILT_DEG de elevación; su "broadside" coincide con esa elevación real.
#   real_elevation = array_elevation + ARRAY_TILT_DEG
# Poner 0 si el array está vertical (sin inclinar).
ARRAY_TILT_DEG = 20.0

# Número de fuentes sonoras asumido. Con 4 mics: máximo 3 fuentes (M-1).
# Con 1 fuente el pseudoespectro tiene picos más definidos.
NUM_SOURCES = 1

# Factor de promediado temporal exponencial de la matriz de covarianza.
# R[t] = alpha * R[t-1] + (1 - alpha) * R_snapshot_frame  (una vez por frame)
# tau = -1 / log(alpha) frames.
COV_ALPHA = 0.85  # Para fuentes SOSTENIDAS (drone): tau ≈ 6.5 frames de memoria
                  # → ~150 ms a 11025/256=43 fps. El drone es cuasi-estacionario:
                  # más memoria → R más estable → pico MUSIC más nítido. Contra:
                  # responde más lento a cambios de dirección (~300 ms). Para
                  # impulsivas, el perfil 'evento' lo baja a 0.5.

# Carga diagonal (fracción de la traza de R). Previene singularidad de la
# covarianza ante señales correlacionadas o pocas muestras.
DIAGONAL_LOADING = 0.005  # Con 16-bit (σ_q ≈ 9e-6) el piso de cuantización es 4
                          # órdenes menor que con 8-bit: se puede reducir la
                          # carga diagonal sin riesgo de singularidad.

# =============================================================================
# MOTOR DE DOA
# =============================================================================
# Qué algoritmo usa main.py para estimar la dirección. Se sobreescribe por línea
# de comandos con --engine.
#
#   'music' → subespacio (MUSIC broadband incoherente). Pico más fino con alta
#             coherencia; frágil ante reverberación/SNR moderado.
#   'srp'   → SRP-PHAT. Integra la GCC-PHAT de los 6 pares; robusto a
#             reverberación y picos espurios. Pico más ancho (confianza en
#             escala más baja; ver umbrales SRP_* abajo).
#
# Ambos comparten interfaz (process(frame) -> DOAResult), detector, tracker,
# logger y servo: se pueden intercambiar para comparar.
DOA_ENGINE = 'music'  # Objetivo: tracking continuo de drone (fuente sostenida,
                      # alta coherencia) → MUSIC da pico más fino que SRP.

# Modo de combinación entre frames del motor SRP:
#   'onset'  → frame del CAMINO DIRECTO (pico SRP del evento, antes de los ecos).
#              Robusto a reverberación. Para fuentes IMPULSIVAS.
#   'accum'  → promedia el mapa sobre frames activos (EMA). Para SOSTENIDAS.
SRP_MODE = 'accum'  # Drone = fuente sostenida.

# =============================================================================
# PERFILES POR MODO DE SERVO
# =============================================================================
# Cada modo de servo (--evento / --seguimiento) afina TODA la cadena (motor DOA
# + detector) para su caso de uso. Los valores acá SOBREESCRIBEN los defaults
# base cuando se elige ese modo. PRECEDENCIA: flag CLI > perfil del modo >
# default base.
#
#   --evento      → fuentes IMPULSIVAS (aplausos, golpes). SRP 'onset', corta
#                   rápido (poca histéresis), reacción en 1-2 frames.
#   --seguimiento → fuentes SOSTENIDAS (dron, DEFAULT). SRP 'accum', más
#                   histéresis para que las fluctuaciones del dron no corten.
MODE_PROFILES = {
    'evento': {
        'SRP_MODE':               'onset',  # camino directo, robusto a reverb
        'DETECTOR_SILENCE_RATIO': 0.3,      # corta rápido tras el impulso
        'COV_ALPHA':              0.5,      # menos memoria: reacciona en 1-2 frames
        'EVENT_MIN_FRAMES':       3,        # confirmación rápida (~70 ms a 43 fps)
    },
    'seguimiento': {
        'SRP_MODE':               'accum',  # promedia mapa sobre frames activos
        # DETECTOR_SILENCE_RATIO: usa default (0.75) — alta histéresis
        # COV_ALPHA: usa default (0.85) — máxima memoria para drone
    },
}

# Modo de servo por defecto cuando no se pasa ningún flag de modo en main.py.
# (main.py implementa la precedencia; este valor documenta la intención.)
DEFAULT_SERVO_MODE = 'seguimiento'

# =============================================================================
# TRACKER DE SALIDA DOA
# =============================================================================

# EMA aplicado al ángulo de salida (az y el por separado). Solo se actualiza si
# la confianza supera DOA_MIN_CONF_UPDATE.
DOA_SMOOTH_ALPHA = 0.85  # ~150 ms de inercia a 43 fps: filtra saltos
                         # frame-a-frame de MUSIC sin lag perceptible.

# Confianza mínima (dB pico/mediana del pseudoespectro) para aceptar una nueva
# estimación y actualizar el tracker.
#   <1 dB → casi seguro ruido | 2 dB → señal débil/reverberante | >4 dB → clara
DOA_MIN_CONF_UPDATE = 2.5  # Drone a distancia puede tener confianza moderada.

# Equivalente para SRP-PHAT. Su escala de confianza NO es comparable a la de
# MUSIC (mapa desplazado a no-negativos, pico de otra forma), por eso se fija
# por separado. Conviene reajustarlo según el escenario, sobre todo en 'accum'.
SRP_MIN_CONF_UPDATE = 3.0

# EMA del mapa SRP en modo 'accum' (sin efecto en 'onset').
SRP_ACCUM_ALPHA = 0.6

# =============================================================================
# DETECTOR DE EVENTOS (gate de ENERGÍA — primer gate de la cascada)
# =============================================================================
#   umbral_evento   = ruido_piso × (1 + DETECTOR_K)
#   umbral_silencio = ruido_piso × (1 + DETECTOR_K × DETECTOR_SILENCE_RATIO)
#
# DETECTOR_K: sensibilidad.
#   Bajo (~1.0) detecta fuentes débiles/lejanas (más falsos); alto (2-3)
#   solo eventos fuertes.
# DETECTOR_SILENCE_RATIO (0 - 1): histéresis inicio/fin. Más bajo corta antes;
#   más alto extiende el evento. Nunca 1.0 (oscila ONSET↔ACTIVE).
DETECTOR_K             = 1.2   # Drone no es tan fuerte como un aplauso vs ruido
                               # ambiente. Subir (2-3) si hay falsos positivos.
DETECTOR_SILENCE_RATIO = 0.75  # Alta histéresis: el dron fluctúa en amplitud
                               # (efecto de pala, multipath); aguanta bajones
                               # transitorios sin cortar ACTIVE.

# Duración mínima de un evento (frames). Más cortos se descartan.
EVENT_MIN_FRAMES = 8   # A 43 fps → ~185 ms. El dron necesita confirmación más
                       # larga que un aplauso para no arrancar con ruido ambiente.

# Cooldown tras un evento (frames). Corto para re-enganchar rápido al dron.
COOLDOWN_FRAMES = 3    # ~70 ms — ignora el eco del final del evento.

# PISO DE RUIDO FIJO (sin adaptación). Se mide UNA vez en la calibración de
# arranque y queda constante (el adaptativo "dejaba de escuchar" en seguimiento:
# el piso trepaba hacia la fuente sostenida). Costo: RECALIBRAR al cambiar de
# escenario (reiniciar main.py o detector.recalibrate()).
#
# Override manual: si se fija un número, se usa ese y se saltea la calibración.
DETECTOR_NOISE_FLOOR = None   # None → auto-calibrar al arranque.

# Frames de calibración al arranque. Durante esta fase el detector NO detecta:
# promedia la energía de los primeros frames (SILENCIO AMBIENTE asumido).
# REQUISITO: mantené silencio (sin la fuente de interés) durante este lapso.
DETECTOR_CALIB_FRAMES = 86   # ~2 s a 43 fps (11025/256).

# =============================================================================
# GATE ESPECTRAL ARMÓNICO (segundo gate — confirma "es un dron" por su firma)
# =============================================================================
# Tras el gate de energía, valida que la fuente tenga la estructura armónica
# (peine de BPF) de un multirrotor antes de habilitar localización + servo +
# registro. Reduce falsos positivos (viento, voces, tráfico). Se desactiva con
# --sin-espectral para probar la cadena con otros sonidos (dispara por energía
# sola). Ver docs/notes/deteccion_espectral.md y src/processing/spectral_gate.py.
#
# SOLO aplica en los modos seguimiento y sin-servo (los pensados para drones).
# El modo EVENTO lo saltea SIEMPRE: es para fuentes impulsivas sin firma
# armónica, y su disparo rápido ocurre antes de que la ventana del gate
# pueda llenarse (main.py implementa esta exclusión).

SPECTRAL_ENABLED = True   # default ON (se puede anular con --sin-espectral)

# Ventana de análisis (muestras). Larga → mejor resolución en frecuencia
# (df = fs/window) para resolver el peine. 2048 @ 11025 Hz → df ≈ 5.4 Hz.
# Se llena en window/hop = 2048/256 = 8 hops (~185 ms), similar a EVENT_MIN_FRAMES.
SPECTRAL_WINDOW = 2048

# Rango de búsqueda de la BPF (fundamental) por HPS. Multirrotores chicos/medios:
# BPF ~80-300 Hz. Se da margen hasta 400.
SPECTRAL_BPF_MIN = 80.0
SPECTRAL_BPF_MAX = 400.0

# Número de armónicos del peine a inspeccionar (BPF, 2·BPF, ... n·BPF).
SPECTRAL_N_HARMONICS = 8

# Factor de decimación del Harmonic Product Spectrum (cuántas versiones ÷d se
# multiplican para realzar la fundamental). 4-5 es típico.
SPECTRAL_HPS_DOWNSAMPLE = 5

# Banda útil de localización (Hz) para exigir armónicos con SNR ahí: garantiza
# que MUSIC tendrá energía tonal real en su banda. Coincide con el techo de
# aliasing (~2425) y el piso blando emergente (~300, ver deteccion_espectral §5).
SPECTRAL_MUSIC_BAND_LO = 300.0
SPECTRAL_MUSIC_BAND_HI = 2425.0

# SNR mínimo (dB, pico sobre el MÁXIMO entre la mediana local y el piso global)
# para contar un armónico. El piso global evita que la fuga de un tono puro
# finja un peine (ver spectral_gate.py).
SPECTRAL_HARMONIC_SNR_DB = 8.0

# Armónicos del peine con SNR suficiente requeridos para confirmar dron.
SPECTRAL_MIN_HARMONICS = 3
# De esos, cuántos deben caer DENTRO de la banda MUSIC (habilita localización).
SPECTRAL_MIN_HARMONICS_IN_BAND = 1

# Harmonic-to-Noise Ratio global mínimo del peine (dB) para confirmar.
SPECTRAL_SCORE_MIN = 6.0

# Fracción mínima de energía en el peine sobre la energía total de la banda.
# Separa el dron (peine concentrado, fracción alta) del RUIDO de banda ancha
# (energía repartida, fracción baja). Referencia en sintético: dron ~0.44,
# ruido blanco ~0.01; 0.10 queda holgadamente entre ambos.
SPECTRAL_HARMONIC_FRACTION_MIN = 0.10

# Ventanas positivas CONSECUTIVAS requeridas para confirmar (anti-espurios).
# Las ventanas se solapan mucho (hop=256 << window=2048), así que 2 cuestan
# ~1 hop de latencia extra.
SPECTRAL_CONFIRM_WINDOWS = 2

# Tolerancia (Hz) alrededor de cada armónico: la BPF no es estacionaria (el
# control de vuelo varía las RPM). ±18 Hz ≈ ±3 bins a df=5.4 Hz.
SPECTRAL_HARMONIC_TOL_HZ = 18.0

# Histéresis del veredicto (frames). Una vez confirmado el dron, se MANTIENE
# estos frames aunque algún frame puntual no valide (fluctuaciones de amplitud).
# ~10 frames ≈ 230 ms a 43 fps.
SPECTRAL_HOLD_FRAMES = 10

# =============================================================================
# SERIAL (ESP32)
# =============================================================================

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 921600

# =============================================================================
# MODO DE OPERACIÓN
# =============================================================================
# El modo de servo se elige por línea de comandos (flags mutuamente excluyentes).
# DEFAULT = seguimiento (sin flag). --evento = apuntado puntual. --sin-servo =
# solo DOA + registro. Ver main.py para la descripción completa de cada modo.
#
# Por qué el disparo no es instantáneo: MUSIC promedia exponencialmente R con
# COV_ALPHA; en el frame 1 del evento, R todavía contiene info del silencio
# previo y el ángulo es pobre. El detector recién emite 'event' tras
# EVENT_MIN_FRAMES sobre umbral, cuando R ya absorbió la señal nueva.
#   - Modo EVENTO: apunta en ese frame 'event' (sin gate espectral).
#   - Modo SEGUIMIENTO: el snap-and-hold queda ARMADO en 'event' y se dispara
#     en el primer frame activo con la firma espectral confirmada — el gate
#     necesita llenar su ventana (2048 muestras ≈ 9 hops, más que
#     EVENT_MIN_FRAMES) más SPECTRAL_CONFIRM_WINDOWS. Con --sin-espectral
#     dispara directamente en 'event'.
# EVENT_MIN_CONFIDENCE es permisivo: vale más apuntar con menos precisión que
# perder el evento por décimas de dB.
MODE = 'continuous'

SERVO_EVENT_LOCK_DURATION = 1.0   # s que el servo queda fijo tras evento (seguimiento)

# Confianza mínima (dB) para DISPARAR un evento (apuntar el servo).
EVENT_MIN_CONFIDENCE     = 2.0    # MUSIC
SRP_EVENT_MIN_CONFIDENCE = 2.0    # SRP-PHAT (escala más baja)

# =============================================================================
# MUSIC — CONTROL DE RESET DE COVARIANZA
# =============================================================================
# Solo resetear R (y la ventana del gate espectral) tras silencio SOSTENIDO, no
# en cada bajón breve del dron (efecto de pala, multipath). Si R se reseteara en
# cada bajón, MUSIC necesitaría ~6 frames para reconstruirla → parpadeo del
# tracking. 30 frames × ~23 ms ≈ 690 ms de silencio → dron fuera o apagado.
# Para no resetear nunca: poner un valor muy alto (p.ej. 9999).
MUSIC_RESET_IDLE_FRAMES = 30

# =============================================================================
# SERVOMOTORES (opcional)
# =============================================================================

SERVO_ENABLED = False   # main.py lo habilita al elegir un modo de servo

# Pines GPIO BCM
SERVO_AZ_PIN = 12
SERVO_EL_PIN = 13

# Rango mecánico NOMINAL del servo (grados). Se usa en _deg_to_pulse para mapear
# grados mecánicos a µs de pulso PWM (500-2500 µs ↔ 0-180°). NO TOCAR — son las
# características del SG90, no del montaje.
SERVO_AZ_MIN = 0;   SERVO_AZ_MAX = 180
SERVO_EL_MIN = 0;   SERVO_EL_MAX = 180

# Rango mecánico USABLE de cada servo en el montaje real (medido con el script de
# calibración). El DOA se mapea LINEALMENTE a este rango. Para preservar
# 1° DOA = 1° servo, el rango USABLE debe tener la misma magnitud que el DOA.
#   Azimut:    DOA ±75° (150°) → USABLE 150° centrado en 90° → [15°, 165°].
#   Elevación: DOA [0,70] (70°) → USABLE 70°; con SERVO_EL_INVERT el mapeo se
#              invierte (DOA=0 → cerca del tope; DOA=70 → cerca del piso).
SERVO_AZ_USABLE_MIN = 15
SERVO_AZ_USABLE_MAX = 165
SERVO_EL_USABLE_MIN = 5
SERVO_EL_USABLE_MAX = 75

# Inversión de sentido de giro por eje (montaje físico al revés). Con INVERT=True
# el mapeo lineal se invierte: DOA_MIN → USABLE_MAX y DOA_MAX → USABLE_MIN.
SERVO_AZ_INVERT = True
SERVO_EL_INVERT = True

# Zona muerta (grados): si la diferencia es menor, no mover. Bajar a 3.0 si la
# fuente queda cerca de la horizontal y el servo de elevación no se mueve; subir
# si tiembla.
SERVO_DEAD_ZONE = 7.0

# Paso máximo por actualización (grados) — limita velocidad de giro.
SERVO_MAX_STEP = 10.0

# Estimaciones a promediar antes de mover. BATCH=1 = máxima frecuencia de
# seguimiento. Subir a 2-3 si el servo tiembla por jitter del pseudoespectro.
SERVO_BATCH = 1

# Confianza mínima para mover (dB del pico MUSIC sobre el piso).
SERVO_MIN_CONFIDENCE = 2.0

# Detach del PWM tras posicionar (s). 0 = no detach. Corre en hilo aparte, no
# bloquea el loop. <0.15 s puede no dejar al SG90 llegar a posición.
SERVO_DETACH_DELAY = 0.4

# Retorno al centro tras silencio prolongado (s). 0 = nunca.
SERVO_SILENCE_RETURN = 5.0

# =============================================================================
# SALIDA
# =============================================================================

LOG_FILE         = "events.csv"   # UNA fila por evento (ver src/utils/logger.py)
DISPLAY_INTERVAL = 0.15   # segundos entre actualizaciones de pantalla
VERBOSITY        = 1      # 0=errores, 1=eventos, 2=procesamiento, 3=debug
