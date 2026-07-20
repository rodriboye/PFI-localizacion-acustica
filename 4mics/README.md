# Sistema DOA con Array de 4 Micrófonos INMP441

Sistema de estimación de Dirección de Arribo (DOA) en tiempo real usando el algoritmo MUSIC con un array cuadrado de 4 micrófonos MEMS INMP441. Desarrollado sobre Raspberry Pi 3A+ con ESP32 como frontend de audio.

## Arquitectura

```
                    USB Serial (921600 baud)
INMP441 ×4 → ESP32 ──────────────────────→ Raspberry Pi 3A+
              (I2S ×2)                       MUSIC/SRP + Detector + Display
                                             └─ opcional: 2× Servo SG90
```

El ESP32 captura los 4 micrófonos con sus 2 buses I2S nativos y transmite las muestras crudas. No hace procesamiento de señal. La Raspberry Pi ejecuta MUSIC frame a frame, detecta eventos acústicos y opcionalmente mueve dos servomotores para seguimiento físico de la fuente.


## Hardware

|Componente|Cantidad|Notas|
|-|-|-|
|INMP441 (módulo breakout)|4|Micrófono MEMS I2S|
|ESP32 DevKit v1|1|El clásico 30-pin; no S2/S3|
|Raspberry Pi 3A+|1|También: 3B, 4, Zero 2W|
|Cable USB A-microB|1|ESP32 → RPi|
|Servo SG90|2|Opcional, seguimiento físico|
|Fuente 5V / 2A|1|Para RPi + servos|

### Array de micrófonos

Cuadrado de 5 cm de lado, montado verticalmente (plano XZ).
La normal del array apunta hacia adelante (+Y).

```
Mic 2 (0, 0, d) ─────── Mic 3 (d, 0, d)
      │         frontal          │
      │            ↑ (+Y)        │
Mic 0 (0, 0, 0) ─────── Mic 1 (d, 0, 0)
```

### Cableado ESP32 → INMP441 (firmware canónico: `esp32_audio_frontend_sync`)

Los dos buses I2S comparten el clock del bus 0 (master); el bus 1 corre como
slave. Así los 4 mics muestrean en el mismo flanco (sin deriva de fase entre
buses — necesario para que MUSIC resuelva la elevación).

|Señal|GPIO ESP32|Mic 0|Mic 1|Mic 2|Mic 3|
|-|-|-|-|-|-|
|SCK (master)|26|SCK|SCK|SCK|SCK|
|WS  (master)|25|WS|WS|WS|WS|
|SD bus 0|22|SD|SD|—|—|
|SD bus 1|32|—|—|SD|SD|
|SCK bus 1 (entrada slave)|14|—|—|—|—|
|WS  bus 1 (entrada slave)|15|—|—|—|—|
|L/R|(fijo)|GND|VDD|GND|VDD|
|VDD|3.3V|VDD|VDD|VDD|VDD|
|GND|GND|GND|GND|GND|GND|

**Jumpers de sincronía** (el clock del master entra al periférico slave):

- GPIO26 → GPIO14 (SCK)
- GPIO25 → GPIO15 (WS)

Los 4 micrófonos comparten SCK/WS del master. Mic 0 y 1 comparten SD en
GPIO22 (movido de GPIO27: adyacente a las líneas de clock, captaba crosstalk);
Mic 2 y 3 comparten SD en GPIO32. El INMP441 tri-statea su SD en la mitad del
frame que no le corresponde (según L/R).

## Instalación

### En la Raspberry Pi

```bash
sudo apt update && sudo apt install -y python3-pip python3-numpy
pip3 install -r requirements.txt

# Solo si usás servos:
sudo apt install -y pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### Firmware ESP32

Abrir `firmware/esp32_audio_frontend_sync/esp32_audio_frontend_sync.ino` en
Arduino IDE (el firmware **canónico**: master+slave con clock compartido).
`firmware/esp32_audio_frontend/` (dos masters independientes) queda solo como
referencia/diagnóstico — tiene deriva de fase entre buses y no localiza bien
la elevación.

Configuración en el IDE:

* Placa: `ESP32 Dev Module`
* Partición: `Default 4MB with spiffs`
* Velocidad upload: 921600

Parámetro clave: `GAIN_SHIFT` (default 9). Apuntar a std ~2000–8000 con clip
~0% en `diagnose_serial.py`: si la señal es débil (std<500) bajarlo (más
ganancia); si hay clipping subirlo.

Flashear normalmente. Al conectar el USB a la RPi, el sistema resetea el ESP32
automáticamente vía RTS/DTR.

## Uso

**Por defecto** (sin ningún flag) el sistema corre: **seguimiento + MUSIC + audio serial + gate espectral armónico**. Es decir, arranca a rastrear un dron por su firma acústica sobre el puerto serial por defecto. Cualquier otra opción se explicita.

```bash
# DEFAULT: seguimiento de dron (MUSIC + serial + gate espectral):
python3 main.py
python3 main.py --serial /dev/ttyUSB0

# Probar la cadena con palmadas/otros sonidos (el modo evento nunca usa el
# gate espectral, no hace falta ningún flag extra):
python3 main.py --evento

# Solo DOA + registro, sin mover el servo:
python3 main.py --sin-servo

# Simulación (sin hardware; los tonos sim no son un peine de dron → --sin-espectral):
python3 main.py --simulate --sim-az 45 --sim-el 0 --sin-espectral

# Ajustar sensibilidad del detector de energía (k más bajo = más sensible):
python3 main.py --evento --k 1.0
```

### Detección: dos gates en cascada

```
audio → [gate ENERGÍA] → [gate ESPECTRAL armónico] → [MUSIC]
        ¿vale la pena?     ¿es un dron (peine BPF)?    localizar
```

El **gate de energía** decide si hay actividad. El **gate espectral** confirma que la fuente tiene la estructura armónica de un multirrotor (fundamental BPF + armónicos regularmente espaciados) antes de localizar/mover el servo — reduce falsos positivos (viento, voces, tráfico). Aplica solo en los modos `--seguimiento` y `--sin-servo`; el modo `--evento` lo saltea siempre (fuentes impulsivas, sin firma de dron). Se desactiva con `--sin-espectral`. Ver `docs/notes/deteccion_espectral.md` y `src/processing/spectral_gate.py`.

**Registro:** `events.csv` guarda **una fila por evento**: en `--evento`, la primera estimación (el ángulo del apuntado); en seguimiento, el **rango de ángulos** (min/max de azimut y elevación) que la fuente ocupó durante todo el seguimiento, con duración y máximos de confianza/energía.

### Modos de servo (mutuamente excluyentes)

| Flag | Comportamiento |
|-|-|
| (ninguno) / `--seguimiento` | **DEFAULT.** El servo sigue la fuente en tiempo real; ante un evento fuerte salta y la fija unos segundos (snap-and-hold), luego retoma. |
| `--evento` | El servo apunta una vez por evento y queda fijo hasta el próximo (localización puntual, reacción rápida). |
| `--sin-servo` | No mueve el servo: solo DOA + detección + registro + display. |

## Estructura del repositorio

```
4mics/
├── config.py                          # Todos los parámetros del sistema
├── main.py                            # Punto de entrada
├── requirements.txt
├── capture_wav.py                     # Captura WAV crudo 4 canales del ESP32
├── check_capture.py                   # Diagnóstico de salud de una captura
├── check_spectral.py                  # ¿Un WAV pasa el gate espectral?
├── diagnose_serial.py                 # Diagnóstico de la cadena serial
├── bench_engines.py                   # Benchmark MUSIC vs SRP-PHAT
├── firmware/
│   ├── esp32_audio_frontend_sync/     # CANÓNICO: master+slave, clock compartido
│   └── esp32_audio_frontend/          # Legado: dos masters (solo diagnóstico)
├── src/
│   ├── acquisition/
│   │   └── audio_input.py             # Lector serial + simulador
│   ├── processing/
│   │   ├── doa_engine.py              # Algoritmo MUSIC
│   │   ├── srp_doa_engine.py          # Motor SRP-PHAT (alternativo)
│   │   ├── detector.py                # Gate 1: detector de eventos por energía
│   │   └── spectral_gate.py           # Gate 2: confirmación armónica de dron
│   └── utils/
│       ├── display.py                 # Visualización terminal
│       ├── logger.py                  # Log CSV: una fila por evento
│       └── servo_control.py           # Control servomotores
├── docs/notes/                        # Notas de diseño propias del 4mics
├── tests/                             # Regresión: motor DOA y gate espectral
└── data/
    ├── samples/                       # Audios cortos de prueba (en repo)
    └── recordings/                    # Grabaciones largas (ignoradas por git)
```

## Parámetros principales

Todos en `config.py`. Los más relevantes:

|Parámetro|Default|Descripción|
|-|-|-|
|`MIC_DISTANCE`|0.05 m|Lado del array cuadrado (placa soldada)|
|`SAMPLE_RATE`|11025 Hz|16-bit little-endian; debe coincidir con firmware|
|`HOP_SIZE`|256|Muestras por frame|
|`FREQ_MIN/MAX`|200–2400 Hz|Rango de análisis MUSIC (techo = aliasing diagonal)|
|`ANGLE_RESOLUTION`|5°|Paso de la grilla de escaneo|
|`NUM_SOURCES`|1|Fuentes sonoras asumidas|
|`COV_ALPHA`|0.85|Promediado temporal de covarianza (dron sostenido)|
|`DETECTOR_K`|1.5|Factor de umbral del detector de energía|
|`SPECTRAL_ENABLED`|True|Gate espectral armónico (anular con `--sin-espectral`)|
|`SPECTRAL_BPF_MIN/MAX`|80–400 Hz|Banda de búsqueda de la fundamental (BPF) del dron|
|`DOA_ENGINE`|`music`|Motor de DOA (`--engine srp` para impulsivos)|

## Limitaciones conocidas

* **Resolución angular**: \~5–10° en condiciones normales (SNR > 10 dB).
* **Solo azimut + elevación 2D**: sin ambigüedad frente/atrás en campo lejano.
* **Campo lejano**: MUSIC asume onda plana. A menos de \~50 cm del array, el modelo falla.
* **Aliasing espacial**: lo fija la diagonal del array (d·√2 = 7.07 cm con d=5 cm) → frecuencia máxima útil ≈ c/(2·d·√2) ≈ 2426 Hz. Por eso `FREQ_MAX=2400` y por eso 22 kHz de muestreo no aportaría a la localización (ver `docs/notes/deteccion_espectral.md`).
* **Reverberación**: reflexiones fuertes generan picos espurios en el pseudoespectro.

## Referencias

Ver `docs/references/` para los papers completos.

* Schmidt (1986) — Algoritmo MUSIC
* Van Trees (2002) — Optimum Array Processing
* Friedlander (2009) — Estimación clásica y moderna de DOA
* Risoud et al. (2018) — Survey de localización de fuente de sonido
* Belloch et al. (2019) — Consideraciones prácticas en plataformas IoT
* Riabko et al. (2024) — Array MEMS lineal para detección de UAV

