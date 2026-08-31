"""
config.py — Configuración de parámetros del sistema.

el resto del código importa desde acá.

SAMPLE_RATE, HOP_SIZE y BYTES_PER_SAMPLE deben coincidir con el firmware .ino del ESP32.
"""

import numpy as np

# =============================================================================
# GEOMETRÍA DEL ARRAY
# =============================================================================

# Lado del cuadrado en metros
MIC_DISTANCE = 0.05

# Posiciones [x, y, z] (m). Array vertical en el plano XZ, mirando a +Y.
#
#   Mic 2 (0, 0, d) ─────── Mic 3 (d, 0, d)
#        │                        │
#        │                        │
#        │                        │
#   Mic 0 (0, 0, 0) ─────── Mic 1 (d, 0, 0)
#
d = MIC_DISTANCE
MIC_POSITIONS = np.array([
    [0, 0, 0],
    [d, 0, 0],
    [0, 0, d],
    [d, 0, d],
], dtype=np.float64)

# =============================================================================
# AUDIO
# =============================================================================

SAMPLE_RATE = 11025     # Hz
HOP_SIZE    = 256       # muestras por frame
SPEED_OF_SOUND = 343.0  # m/s — 331 + 0.6xT [°C], ajustar en temperaturas lejanas a 25°C

#2 = int16 little-endian.
BYTES_PER_SAMPLE = 2

# Ganancia digital sobre el frame ya normalizado, en veces (2.0 = +3 dB).
DIGITAL_GAIN = 1.0

# Ganancia por canal, None = sin corrección.
# Se mide un valor recomendado con `diagnose_serial.py <puerto> --repeat 5`.
# Corrige el sesgo si los microfonos tienen ganancia distinta.
CHANNEL_GAINS = None   # ej. [2.3, 1.0, 1.04, 1.8]

# =============================================================================
# ALGORITMO MUSIC
# =============================================================================

# Banda de trabajo del algoritmo DOA.
FREQ_MIN = 200    # Hz 
FREQ_MAX = 2400   # Hz

# FFT de análisis, desacoplada del hop (debe ser multiplo entero de HOP_SIZE):
# HOP_SIZE fija latencia y fps, MUSIC_FFT_SIZE fija df = fs/N.
MUSIC_FFT_SIZE = HOP_SIZE * 2   #512 -> df (tamaño de bin) = 21.53 

# Submuestreo de los bins. Aumentar a reduce el costo de MUSIC, pero empeora el rendimiento
# aumentar solo si no entra en tiempo real
MUSIC_BIN_STRIDE = 1

# Resolucion en grados para la grilla en ACIMUT
# Define cuantos puntos entran dentro del rango, los pasos reales no son lineales (espacio-u)
AZIMUTH_RESOLUTION = 5 

# Semi-apertura del escaneo en acimut, para rango simétrico.
AZIMUTH_HALF_SPAN = 75

# Rango acimut.
AZIMUTH_MIN   = - AZIMUTH_HALF_SPAN
AZIMUTH_MAX   =  AZIMUTH_HALF_SPAN

# Resolución del eje de ELEVACIÓN.
ELEVATION_RESOLUTION = 6

# Semi-apertura del escaneo en elevación. 
# Rango desde el MARCO DEL ARRAY: -HALF_SPAN a +HALF_SPAN 
ELEVATION_HALF_SPAN = 35.0

# Inclinación física del array (grados): elevación_real = array + tilt.
ARRAY_TILT_DEG = 15.0

# Rango de elevación REAL, considerando la inclinacion del arreglo.
ELEVATION_MIN = ARRAY_TILT_DEG - ELEVATION_HALF_SPAN
ELEVATION_MAX = ARRAY_TILT_DEG + ELEVATION_HALF_SPAN

NUM_SOURCES = 1   # máximo M-1 = 3 con 4 micrófonos

# Alfa del promediado temporal de la covarianza: R = alpha·R + (1-alpha)·R_frame.
# entre 0 y 1. Más alto = mas estable, menos alto = mas reactivo.
# El perfil 'evento' lo baja para que varíe más rápidamente.
COV_ALPHA = 0.8

# Carga diagonal (fracción de la traza). Solo estabiliza, no cambia los
# autovectores ni el pseudoespectro.
DIAGONAL_LOADING = 0.005

# =============================================================================
# MOTOR DE DOA
# =============================================================================
# 'music' o 'srp'. Intercambiables.
DOA_ENGINE = 'music' #default, se puede elegir al ejecutar main.py con --engine

# Combinación entre frames del motor SRP:
#   'onset' -> frame del camino directo (fuentes impulsivas)
#   'accum' -> EMA del mapa sobre frames activos (fuentes sostenidas)
SRP_MODE = 'onset'

# =============================================================================
# PERFILES POR MODO DE SERVO
# =============================================================================

MODE_PROFILES = {
    'evento': {
        'SRP_MODE':               'onset',
        'DETECTOR_SILENCE_RATIO': 0.3,      # corta rápido tras el impulso
        'COV_ALPHA':              0.5,      # reacciona en 1-2 frames
        'EVENT_MIN_FRAMES':       3,
    },
    'seguimiento': {
        'SRP_MODE':               'accum',
        # el resto usa los defaults base
    },
}

DEFAULT_SERVO_MODE = 'seguimiento'   # main.py implementa la precedencia

# =============================================================================
# TRACKER DE SALIDA DOA
# =============================================================================

DOA_SMOOTH_ALPHA = 0.85   # EMA del ángulo de salida, suaviza estimaciones ruidosas

# Confianza mínima (dB pico/mediana) para actualizar el tracker.
#   <1 dB ruido | 2 dB señal débil o reverberante | >4 dB estimación clara
DOA_MIN_CONF_UPDATE = 2.5

# Equivalente para SRP: su escala de confianza NO es comparable a la de MUSIC.
SRP_MIN_CONF_UPDATE = 2.0

SRP_ACCUM_ALPHA = 0.6     # EMA del mapa SRP en modo 'accum'

# =============================================================================
# DETECTOR DE EVENTOS (gate 1 — energía)
# =============================================================================
# umbral_evento   = energía_piso_ruido × (1 + K)
# umbral_silencio = energía_piso_ruido × (1 + K × SILENCE_RATIO)
# SILENCE_RATIO es la histeresis, debe ser menor a 1.0.
DETECTOR_K             = 2.0
DETECTOR_SILENCE_RATIO = 0.7

EVENT_MIN_FRAMES = 8   # duración mínima del evento para ser considerado válido
COOLDOWN_FRAMES  = 3   # para ignorar algún eco luego del final del evento

# Piso de ruido fijo. Recalibrar segun el escenario.
# None calibra al arrancar, un valor saltea (~1e-6 interiores silenciosos, ~1e-5 exteriores)
DETECTOR_NOISE_FLOOR = 1.5e-6   

# Frames de calibración de piso de ruido. 
# Durante esa fase el detector no funciona y se asume SILENCIO AMBIENTE.
DETECTOR_CALIB_FRAMES = 100

# Para que un ruido durante la estimacion no mueva el piso demasiado arriba.
DETECTOR_CALIB_PERCENTILE = 20.0

# Medir la energía solo en [FREQ_MIN, FREQ_MAX], la misma banda que el motor DOA. 
DETECTOR_BAND_LIMITED = True

# =============================================================================
# GATE ESPECTRAL ARMÓNICO (gate 2 — firma de dron)
# =============================================================================
# Valida el peine de BPF antes de habilitar localización,
# servo y registro. Solo en SEGUIMIENTO, el modo EVENTO lo saltea

SPECTRAL_ENABLED = True   # se anula con --sin-espectral 

# Ventana de análisis (muestras)
SPECTRAL_WINDOW = 2048

SPECTRAL_BPF_MIN = 80.0    # Hz — búsqueda de la fundamental por HPS
SPECTRAL_BPF_MAX = 400.0

SPECTRAL_N_HARMONICS = 8       # armónicos a inspeccionar (BPF … n·BPF)
SPECTRAL_HPS_DOWNSAMPLE = 5    # decimaciones del Harmonic Product Spectrum

# Banda útil de localización: coincidir con DOA garantiza tener energía tonal en la banda.
SPECTRAL_MUSIC_BAND_LO = 200.0
SPECTRAL_MUSIC_BAND_HI = 2400.0

# SNR mínimo para validar armónico
SPECTRAL_HARMONIC_SNR_DB = 8.0

SPECTRAL_MIN_HARMONICS = 3          # armónicos CONSECUTIVOS para confirmar
SPECTRAL_MIN_HARMONICS_IN_BAND = 2  # de esos, cuántos en la banda MUSIC

SPECTRAL_SCORE_MIN = 6.0   # HNR global mínimo del peine (dB)

# Energía del peine / energía de banda: separa el dron (peine concentrado) del
# ruido de banda ancha (energía repartida).
SPECTRAL_HARMONIC_FRACTION_MIN = 0.10

# Ventanas positivas consecutivas para confirmar el veredicto.
SPECTRAL_CONFIRM_WINDOWS = 2

# Tolerancia por armónico: la BPF no es estacionaria porque las RPM del rotor varían.
SPECTRAL_HARMONIC_TOL_HZ = 20.0   #Hz

# Histéresis: una vez confirmado se mantiene aunque algún frame no valide.
SPECTRAL_HOLD_FRAMES = 10

# =============================================================================
# SERIAL (ESP32)
# =============================================================================

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 921600

# =============================================================================
# MODO DE OPERACIÓN
# =============================================================================

MODE = 'continuous'

SERVO_EVENT_LOCK_DURATION = 0.5   # s de snap-and-hold tras un evento

# Confianza mínima para disparar un evento.
EVENT_MIN_CONFIDENCE     = 2.0    # MUSIC
SRP_EVENT_MIN_CONFIDENCE = 1.5    # SRP-PHAT (escala más baja)

# Frames IDLE consecutivos antes de limpiar R y la ventana del gate.
MUSIC_RESET_IDLE_FRAMES = 30

# =============================================================================
# SERVOMOTORES 
# =============================================================================

SERVO_ENABLED = False   # main.py lo habilita al elegir un modo de servo

SERVO_AZ_PIN = 12   # GPIO, numeración BCM
SERVO_EL_PIN = 13

# Rango mecánico nominal del servo, diferente al rango de montaje y de escaneo.
SERVO_AZ_MIN = 0;   SERVO_AZ_MAX = 180
SERVO_EL_MIN = 0;   SERVO_EL_MAX = 180

# Rango mecánico USABLE en el montaje real. El DOA se mapea linealmente acá, así
# que para preservar 1° DOA = 1° servo debe tener la misma magnitud que el DOA.
SERVO_AZ_USABLE_MIN = 15
SERVO_AZ_USABLE_MAX = 165
SERVO_EL_USABLE_MIN = 5
SERVO_EL_USABLE_MAX = 75

# Dominio de elevación que el servo puede APUNTAR, desacoplado del rango de
# ESCANEO: el montaje hace tope, el log y el display siguen reportando la
# elevación real mientras el servo queda saturado en el tope.
SERVO_EL_DOA_MIN = 0.0
SERVO_EL_DOA_MAX = 70.0

# Tasa del hilo escritor a pigpio. Queda por encima del ancho de banda mecánico
# del servo (~5-10 Hz) y por debajo de la tasa de frames (43 fps).
SERVO_WRITE_HZ = 30

# Montaje físico al revés: manda DOA_MIN → USABLE_MAX y DOA_MAX → USABLE_MIN.
SERVO_AZ_INVERT = False
SERVO_EL_INVERT = True

SERVO_DEAD_ZONE = 5.0    # grados. Si nueva estimacion difiere menos que esto, no mover
SERVO_MAX_STEP  = 25.0   # grados por actualización, limita la velocidad
SERVO_BATCH     = 1      # estimaciones a promediar antes de mover, 1 para mov rapido
SERVO_MIN_CONFIDENCE = 2.0   # dB del pico sobre el piso para mover el servo

SERVO_DETACH_DELAY   = 0.2   # s tras posicionar; detach para que no tiemble, 0 = no detach
SERVO_SILENCE_RETURN = 5.0   # s de silencio antes de volver al centro; 0 = nunca

# =============================================================================
# SALIDA
# =============================================================================

LOG_FILE         = "events.csv"   # una fila por evento (src/utils/logger.py)
DISPLAY_INTERVAL = 0.15   # s entre refrescos de pantalla

# Muestras de los percentiles del display en vivo: acota el costo del refresco,
# que si no crece con la duración de la corrida. ≈7 s de historia a 43 fps.
DISPLAY_STATS_WINDOW = 300
VERBOSITY        = 0      # 0=errores, 1=eventos, 2=procesamiento, 3=debug