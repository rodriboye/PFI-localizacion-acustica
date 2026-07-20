# Sistema de Estimación de Dirección de Arribo (DOA) — 4 Micrófonos

Estimación en tiempo real de azimut y elevación de una fuente sonora con el algoritmo MUSIC, usando un array cuadrado de 4 micrófonos MEMS INMP441. Corre sobre Raspberry Pi 3A+ con un ESP32 como frontend de captura de audio. Pensado para seguimiento de drones por su firma acústica, con detección de eventos genéricos como modo alternativo.

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

El ESP32 captura los 4 micrófonos (2 buses I2S) y transmite las muestras crudas por serial, sin procesarlas. La Raspberry Pi ejecuta MUSIC frame a frame para estimar azimut/elevación, pasa la señal por dos gates de detección (energía + firma espectral armónica) y opcionalmente mueve dos servomotores para apuntar físicamente hacia la fuente.

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

Cuadrado de 5 cm de lado, montado verticalmente (plano XZ), normal apuntando hacia adelante (+Y).

```
Mic 2 (0, 0, d) ─────── Mic 3 (d, 0, d)
      │         frontal          │
      │            ↑ (+Y)        │
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

# Solo si usás servos:
sudo apt install -y pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### Firmware ESP32

Abrir `firmware/esp32_audio_frontend_sync/esp32_audio_frontend_sync.ino` en el Arduino IDE (firmware **canónico**: master+slave con clock compartido). `firmware/esp32_audio_frontend/` (dos masters independientes) es solo referencia/diagnóstico.

Configuración en el IDE:
- Placa: `ESP32 Dev Module`
- Partición: `Default 4MB with spiffs`
- Velocidad de upload: 921600

Parámetro clave: `GAIN_SHIFT` (default 9). Apuntar a std ~2000–8000 con clip ~0% en `diagnose_serial.py`: si la señal es débil (std<500) bajarlo; si hay clipping subirlo.

Flashear normalmente. Al conectar el USB a la RPi, el ESP32 se resetea automáticamente vía RTS/DTR.


## Uso

```bash
# DEFAULT: seguimiento de dron (MUSIC + serial + gate espectral)
python3 main.py
python3 main.py --serial /dev/ttyUSB0

# Probar la cadena con palmadas/otros sonidos (modo evento, sin gate espectral)
python3 main.py --evento

# Solo DOA + registro, sin mover el servo
python3 main.py --sin-servo

# Simulación (sin hardware)
python3 main.py --simulate --sim-az 45 --sim-el 0 --sin-espectral

# Ajustar sensibilidad del detector de energía (k más bajo = más sensible)
python3 main.py --evento --k 1.0
```

### Modos de servo (mutuamente excluyentes)

| Flag | Comportamiento |
|---|---|
| (ninguno) / `--seguimiento` | **Default.** El servo sigue la fuente en tiempo real; ante un evento fuerte salta y fija unos segundos (snap-and-hold), luego retoma. |
| `--evento` | El servo apunta una vez por evento y queda fijo hasta el próximo. |
| `--sin-servo` | No mueve el servo: solo DOA + detección + registro + display. |

### Cadena de detección

```
audio → [gate ENERGÍA] → [gate ESPECTRAL armónico] → [MUSIC]
```

El gate de energía decide si hay actividad; el gate espectral confirma la estructura armónica de un multirrotor antes de localizar/mover el servo. Aplica en `--seguimiento` y `--sin-servo`; `--evento` lo saltea siempre. Se desactiva con `--sin-espectral`. Detalle en `docs/notes/deteccion_espectral.md` y `src/processing/spectral_gate.py`.

### Registro

`events.csv`: una fila por evento. En `--evento`, la primera estimación (ángulo del apuntado). En seguimiento, el rango de ángulos (min/max de azimut y elevación) ocupado durante todo el evento, con duración y máximos de confianza/energía.


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
│   ├── esp32_audio_frontend_sync/     # Canónico: master+slave, clock compartido
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
│       ├── display.py                 # Visualización en terminal
│       ├── logger.py                  # Log CSV: una fila por evento
│       └── servo_control.py           # Control de servomotores
├── docs/notes/                        # Notas de diseño propias del 4mics
├── tests/                             # Regresión: motor DOA y gate espectral
└── data/
    ├── samples/                       # Audios cortos de prueba (en repo)
    └── recordings/                    # Grabaciones largas (ignoradas por git)
```


## Parámetros principales

Todos en `config.py`.

| Parámetro | Default | Descripción |
|---|---|---|
| `MIC_DISTANCE` | 0.05 m | Lado del array cuadrado |
| `SAMPLE_RATE` | 11025 Hz | 16-bit little-endian; debe coincidir con el firmware |
| `HOP_SIZE` | 256 | Muestras por frame |
| `FREQ_MIN/MAX` | 200–2400 Hz | Rango de análisis de MUSIC |
| `ANGLE_RESOLUTION` | 5° | Paso de la grilla de escaneo |
| `NUM_SOURCES` | 1 | Fuentes sonoras asumidas |
| `COV_ALPHA` | 0.85 | Promediado temporal de la matriz de covarianza |
| `DETECTOR_K` | 1.5 | Factor de umbral del detector de energía |
| `SPECTRAL_ENABLED` | True | Gate espectral armónico (anular con `--sin-espectral`) |
| `SPECTRAL_BPF_MIN/MAX` | 80–400 Hz | Banda de búsqueda de la fundamental (BPF) del dron |
| `DOA_ENGINE` | `music` | Motor de DOA (`--engine srp` para fuentes impulsivas) |


## Herramientas de diagnóstico

| Script | Uso |
|---|---|
| `diagnose_serial.py` | Verifica la cadena serial ESP32 → RPi y ajuda a calibrar `GAIN_SHIFT`. |
| `capture_wav.py` | Graba un WAV crudo de 4 canales desde el ESP32. |
| `check_capture.py` | Diagnóstico de salud de una captura (clipping, nivel, sincronía). |
| `check_spectral.py` | Corre el gate espectral sobre un WAV existente. |
| `bench_engines.py` | Compara desempeño de MUSIC vs SRP-PHAT. |
| `diagnose_audio.sh` | Diagnóstico rápido de la captura de audio. |


## Limitaciones conocidas

- **Resolución angular**: ~5–10° en condiciones normales (SNR > 10 dB).
- **Solo azimut + elevación 2D**: sin ambigüedad frente/atrás en campo lejano.
- **Campo lejano**: MUSIC asume onda plana; a menos de ~50 cm del array el modelo falla.
- **Aliasing espacial**: limitado por la diagonal del array (d·√2 = 7.07 cm con d=5 cm) → frecuencia máxima útil ≈ 2426 Hz. Por eso `FREQ_MAX=2400`.
- **Reverberación**: reflexiones fuertes generan picos espurios en el pseudoespectro.


## Referencias

Papers completos en `docs/references/` (a nivel raíz del repo).

- Schmidt (1986) — Algoritmo MUSIC
- Van Trees (2002) — Optimum Array Processing
- Friedlander (2009) — Estimación clásica y moderna de DOA
- Risoud et al. (2018) — Survey de localización de fuente de sonido
- Belloch et al. (2019) — Consideraciones prácticas en plataformas IoT
- Riabko et al. (2024) — Array MEMS lineal para detección de UAV
