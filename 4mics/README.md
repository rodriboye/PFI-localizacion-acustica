# Sistema de Estimación de Dirección de Arribo (DOA) — 4 Micrófonos

Estimación en tiempo real de azimut y elevación de una fuente sonora usando un array cuadrado de 4 micrófonos I2S. Corre sobre Raspberry Pi 3A+ con un ESP32 como frontend de captura de audio. Puede utilizar el algoritmo MUSIC para detección y seguimiento de drones o el algoritmo SRP-PHAT para eventos impulsivos.

**Autor:** Rodrigo Boyé
**Proyecto Final Integrador** — Ingeniería en Telecomunicaciones
Universidad Nacional de Río Negro — Trabajo realizado con Invap
Bariloche, Argentina, 2026


## Qué hace

```
                    USB Serial (921600 baud)
INMP441 ×4 → ESP32 ──────────────────────→ Raspberry Pi 3A+
              (I2S ×2)                       MUSIC + Detector + Display
                                             └─ opcional: 2× Servo SG90
```

El ESP32 captura los 4 micrófonos (2 buses I2S) y transmite las muestras por serial, sin procesarlas. La Raspberry Pi ejecuta los algoritmos frame a frame para estimar azimut/elevación, la señal pasa por dos gates de detección (energía + firma espectral armónica (opcional)) y opcionalmente mueve dos servomotores para apuntar físicamente hacia la fuente.

Por defecto (sin flags) el sistema corre en modo **seguimiento de dron**: MUSIC + gate espectral + servos activos.


## Hardware

| Componente | Cantidad | Notas |
|---|---|---|
| INMP441 (módulo breakout) | 4 | Micrófono MEMS I2S |
| ESP32 DevKit v1 | 1 | El clásico de 30 pines; no S2/S3 |
| Raspberry Pi 3A+ | 1 | También sirve 3B, 4, Zero 2W |
| Cable USB A–microB | 1 | ESP32 → RPi |
| Servo SG90 | 2 | Opcional, seguimiento físico |
| Fuente 5V / 2A | 1 | Para RPi + servos |

### Array de micrófonos

Cuadrado de 5 cm de lado.

```
Mic 2 (0, 0, d) ─────── Mic 3 (d, 0, d)
      │                          │
      │                          │
Mic 0 (0, 0, 0) ─────── Mic 1 (d, 0, 0)
```

### Cableado ESP32 → INMP441 (firmware: `esp32_audio_frontend_sync`)

| Señal | GPIO ESP32 | Mic 0 | Mic 1 | Mic 2 | Mic 3 |
|---|---|---|---|---|---|
| SCK (master) | 26 | SCK | SCK | SCK | SCK |
| WS (master) | 25 | WS | WS | WS | WS |
| SD bus 0 | 22 | SD | SD | — | — |
| SD bus 1 | 32 | — | — | SD | SD |
| SCK bus 1 (entrada slave) | 14 | — | — | — | — |
| WS bus 1 (entrada slave) | 15 | — | — | — | — |
| L/R | (fijo) | GND | VDD | GND | VDD |
| VDD | 3.3V | VDD | VDD | VDD | VDD |
| GND | GND | GND | GND | GND | GND |

Jumpers de sincronía (clock del master hacia el periférico slave): GPIO26→GPIO14 (SCK), GPIO25→GPIO15 (WS). Mic 0/1 comparten SD en GPIO22, Mic 2/3 en GPIO32.


## Instalación

### Raspberry Pi

```bash
sudo apt update && sudo apt install -y python3-pip python3-numpy
pip3 install -r requirements.txt

# para los servos:
sudo apt install -y pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### Firmware ESP32

Abrir `firmware/esp32_audio_frontend_sync/esp32_audio_frontend_sync.ino` en Arduino IDE.

Parámetro clave: `GAIN_SHIFT`. Apuntar a std ~2000–8000 con clip ~0% en `diagnose_serial.py`: si la señal es débil (std<500) bajarlo; si hay clipping subirlo.

Flashear normalmente. Al conectar el USB a la RPi, el ESP32 se resetea automáticamente vía RTS/DTR.

## Uso y Opciones (CLI)

El sistema incluye múltiples parámetros y modos de operación configurables vía línea de comandos.

```bash
# DEFAULT: seguimiento de dron (MUSIC + serial + gate espectral)
python3 main.py --serial /dev/ttyUSB0 

# palmadas/otros sonidos (modo evento, algoritmo SRP-PHAT, sin gate espectral)
python3 main.py --evento

# Solo DOA + registro, sin mover el servo
python3 main.py --sin-servo

# Simulación (sin hardware)
python3 main.py --simulate --sim-az 45 --sim-el 20 --sin-espectral

# Reproducir una captura WAV grabada previamente, se incluyen grabaciones en el repositorio
python3 main.py --wav captura.wav
python3 main.py --wav captura.wav --wav-realtime   # forzar ritmo de tiempo real
```

### Flags y Parámetros Disponibles

**Modos de Servo (mutuamente excluyentes):**
* `--seguimiento` (Default): El servo sigue la fuente en tiempo real. Por default con MUSIC
* `--evento`: Apunta una sola vez por evento de inmediato, y queda fijo hasta el próximo evento. Ideal para fuentes impulsivas. Por default con SRP-PHAT
* `--sin-servo`: No mueve el servo. Corre el resto del pipeline (DOA, detección, registro, display). Por default con MUSIC

**Opciones de Motor y Audio:**
* `--serial PORT`: Define el puerto serie del ESP32 (default en config.py).
* `--engine {srp,music}`: Motor de DOA a utilizar.
* `--srp-mode {onset,accum}`: Override del modo SRP (impulsivas o sostenidas).
* `--gain VALUE`: Ganancia digital multiplicativa (ej. `2.0` suma +6 dB).

**Detector y Filtros (Gates):**
* `--k VALUE`: Factor de umbral del detector de energía. Un valor menor lo hace más sensible.
* `--silence-ratio VALUE`: Histéresis del detector (ratio de silencio).
* `--noise-floor VALUE`: Fija el piso de ruido a mano y saltea la calibración inicial. Útil para entornos que no inician en silencio.
* `--sin-espectral`: Desactiva el gate espectral armónico. El sistema dispara solo por energía (para fuentes que no son drones).

**Tiempos y Comportamiento:**
* `--servo-lock-time VALUE`: Segundos que el servo queda fijo tras un evento en modo seguimiento (snap-and-hold).

**Diagnóstico, Simulación y Output:**
* `--wav FILE.wav`: Corre el pipeline completo sobre un archivo de audio (captura) en vez del serial.
* `--wav-realtime`: Con `--wav`, entrega los frames al ritmo real de la tasa de muestreo.
* `--drop-policy {newest,oldest}`: Qué frame descartar cuando se llena la cola de procesamiento.
* `--simulate`: Utiliza entrada simulada sin requerir hardware.
    * `--sim-az VALUE` / `--sim-el VALUE`: Fija el azimut y elevación para el audio simulado.
* `--no-log`: Desactiva el registro CSV.
* `--sin-display`: No renderiza el panel de estadísticas en la terminal (mejora rendimiento).
* `--verbosity LEVEL`: Nivel de detalle en los logs de la consola (0, 1, 2, 3).


### Registro

`events.csv`: una fila por evento. En `--evento`, la primera estimación (ángulo del apuntado). En seguimiento, el rango de ángulos (min/max de azimut y elevación) ocupado durante todo el evento, con duración y máximos de confianza/energía.


## Estructura de la carpeta

```
4mics/
├── config.py                          # Todos los parámetros del sistema
├── main.py                            # Programa principal
├── setup_rpi_i2s.sh                   # Configura la RPi para poder correr
├── requirements.txt                   # Requerimientos que instala setup_rpi_i2s.sh
├── capture_wav.py                     # Captura WAV crudo 4 canales del ESP32
├── diagnose_serial.py                 # Diagnóstico de la cadena serial
├── firmware/
│   ├── esp32_audio_frontend_sync/     # master+slave, clock compartido
├── src/
│   ├── acquisition/
│   │   └── audio_input.py             # Lector serial + simulador
│   ├── processing/
│   │   ├── doa_engine.py              # Algoritmo MUSIC
│   │   ├── srp_doa_engine.py          # Motor SRP-PHAT (alternativo)
│   │   ├── detector.py                # Gate 1: detector de eventos por energía
│   │   └── spectral_gate.py           # Gate 2: confirmación armónica de dron
│   └── utils/
│       ├── display.py                 # Visualización en terminal
│       ├── logger.py                  # Log CSV: una fila por evento
│       └── servo_control.py           # Control de servomotores
```


## Parámetros principales

Todos en `config.py`.

| Parámetro | Default | Descripción |
|---|---|---|
| `MIC_DISTANCE` | 0.05 m | Lado del array cuadrado |
| `SAMPLE_RATE` | 11025 Hz | debe coincidir con el firmware |
| `HOP_SIZE` | 256 | Muestras por frame |
| `FREQ_MIN/MAX` | 200–2400 Hz | Rango de análisis de MUSIC |
| `AZIMUTH_RESOLUTION` | 5° | Paso promedio de la grilla de escaneo |
| `NUM_SOURCES` | 1 | Fuentes sonoras asumidas |
| `COV_ALPHA` | 0.85 | Promediado temporal de la matriz de covarianza |
| `DETECTOR_K` | 1.5 | Factor de umbral del detector de energía |
| `SPECTRAL_ENABLED` | True | Gate espectral armónico (anular con `--sin-espectral`) |
| `SPECTRAL_BPF_MIN/MAX` | 80–400 Hz | Banda de búsqueda de la fundamental (BPF) del dron |
| `DOA_ENGINE` | `music` | Motor de DOA (`--engine srp` para fuentes impulsivas) |


## Herramientas de diagnóstico

| Script | Uso | 
|---|---|
| `diagnose_serial.py` | Verifica la cadena serial ESP32 → RPi y ayuda a calibrar `GAIN_SHIFT`. |
| `capture_wav.py` | Graba un WAV crudo de 4 canales desde el ESP32. |

### Comandos

```bash
python3 diagnose_serial.py /dev/ttyUSB0 --repeat 5  
python3 capture_wav.py /dev/ttyUSB0 --seconds 10 --out nombre.wav
```
