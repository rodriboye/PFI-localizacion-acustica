# Sistema de Estimación de Dirección de Arribo (DOA) — 2 Micrófonos

Estimación en tiempo real del ángulo de llegada de sonido usando dos micrófonos MEMS INMP441, algoritmo GCC-PHAT, seguimiento con servomotor, y registro de eventos acústicos.

**Autor:** Rodrigo Boyé
**Proyecto Final Integrador** — Ingeniería en Telecomunicaciones
Universidad Nacional de Río Negro — Trabajo realizado con Invap
Bariloche, Argentina, 2026


## Qué hace

Captura audio estéreo desde dos micrófonos I2S, estima la dirección de la fuente sonora cuadro a cuadro (GCC-PHAT), y opcionalmente mueve un servomotor para apuntar hacia la fuente. Cuando la energía supera un umbral, registra el evento (ángulo, duración, confianza) en un CSV.

Salida angular: 0° a 180°, donde 90° es el frente del array, 0° el extremo derecho y 180° el izquierdo. Con dos micrófonos solo se resuelve azimut en un plano — no hay elevación ni distinción arriba/abajo.


## Archivos

| Archivo | Descripción |
|---|---|
| `gccphat_servo_2mics.py` | Programa principal. Todo el sistema en un solo archivo. |
| `setup_rpi_i2s.sh` | Configuración de I2S en la Raspberry Pi. Ejecutar una vez. |
| `calibrate_servo_endpoints.py` | Calibración de los extremos de recorrido del servo. |
| `diagnose_audio.sh` | Diagnóstico de la captura de audio (ALSA/PortAudio). |
| `eventos_doa.csv` | Log de eventos (se crea automáticamente al ejecutar). |


## Hardware necesario

- Raspberry Pi (probado en 3A+) con Raspberry Pi OS
- 2× micrófonos INMP441 (MEMS, salida I2S, omnidireccionales)
- 1× servomotor estándar (SG90 o similar) — opcional
- Cables, protoboard, resistor 100 kΩ pull-down en la línea SD


## Conexiones

### Micrófonos I2S

Ambos micrófonos comparten SCK, WS y SD. Se distinguen por el pin L/R.

| INMP441 Pin | RPi GPIO (BCM) | Notas |
|---|---|---|
| SCK | GPIO 18 | Bit Clock |
| WS | GPIO 19 | Word Select (LRCLK) |
| SD | GPIO 20 | Data — unir SD de ambos mics |
| VDD | 3.3V | |
| GND | GND | |
| L/R (mic izq) | GND | Canal izquierdo |
| L/R (mic der) | 3.3V | Canal derecho |

### Servomotor

| Cable | RPi GPIO (BCM) | Notas |
|---|---|---|
| Señal (naranja/blanco) | GPIO 12 | PWM0. También GPIO 13 (PWM1). No usar GPIO 18 (ocupado por I2S). |
| Alimentación (rojo) | 5V | Para SG90. Servos de mayor consumo (MG996R): fuente externa 5V. |
| Tierra (negro/marrón) | GND | Tierra común con la RPi |


## Instalación

### 1. Configurar I2S en la Raspberry Pi

```bash
sudo bash setup_rpi_i2s.sh
sudo reboot
```

Verificar que la tarjeta de audio aparece:

```bash
arecord -l
```

Si no aparece: revisar que `/boot/config.txt` (o `/boot/firmware/config.txt` en Bookworm) tenga `dtparam=i2s=on` y el overlay correspondiente.

### 2. Instalar dependencias Python

```bash
pip install numpy sounddevice --break-system-packages
```

`gpiozero` viene preinstalada en Raspberry Pi OS. Si no está:

```bash
pip install gpiozero --break-system-packages
```

### 3. Verificar la captura de audio

```bash
arecord -D plughw:CARD=<nombre>,DEV=0 -c 2 -r 48000 -f S32_LE -d 3 /tmp/test.wav
```

Reemplazar `<nombre>` por el que muestra `arecord -l`. Si da error de sample rate, probar 16000 o 32000.

### 4. Verificar el dispositivo de audio para el script

```bash
python3 -m sounddevice
```

Lista los dispositivos con su índice. Usar ese índice con `--device` si el default no funciona.


## Uso

```bash
# Solo DOA en terminal
python3 gccphat_servo_2mics.py

# Con servomotor
python3 gccphat_servo_2mics.py --servo

# Modo simulación (sin hardware)
python3 gccphat_servo_2mics.py --simulate --sim-angle 120
python3 gccphat_servo_2mics.py --simulate --servo --sim-angle 75

# Especificar dispositivo de audio
python3 gccphat_servo_2mics.py --device 1
```

Detener con Ctrl+C: cierra el stream de audio, desactiva el servo, escribe los eventos pendientes y muestra un resumen.


## Parámetros configurables

Todos vía línea de comandos. Defaults en `DEFAULT_CONFIG` al inicio del código.

### Audio y DOA

| Parámetro | Default | Descripción |
|---|---|---|
| `--sample-rate` | 48000 | Frecuencia de muestreo (Hz). Si falla, probar 16000 o 32000. |
| `--block-size` | 2048 | Muestras por bloque FFT. Más alto = más estable, más latencia (~43 ms a 48 kHz). |
| `--mic-distance` | 0.03 | Separación entre micrófonos (m). |
| `--device` | (auto) | Índice o nombre del dispositivo de audio. Listar con `python3 -m sounddevice`. |
| `--umbral-ruido` | 1e-4 | Energía mínima para estimar DOA. |
| `--umbral-evento` | umbral_ruido × 30 | Energía mínima para registrar un evento. |
| `--log-path` | eventos_doa.csv | Ruta del archivo de eventos. |

### Servo

| Parámetro | Default | Descripción |
|---|---|---|
| `--servo` | (desactivado) | Activa el control del servomotor. |
| `--servo-pin` | 12 | Pin GPIO (BCM) de la señal PWM. |
| `--servo-batch` | 15 | Estimaciones acumuladas antes de decidir movimiento (~640 ms a 48 kHz / bloques 2048). |
| `--servo-min-conf` | 0.2 | Confianza mínima del pico GCC-PHAT para entrar al buffer del servo. |
| `--servo-dead-zone` | 8.0 | Grados mínimos de diferencia para mover el servo. |
| `--servo-max-step` | 10.0 | Grados máximos por movimiento. |
| `--servo-lock-time` | 3.0 | Segundos que el servo queda fijo tras un evento. |
| `--servo-invert` | (no) | Invierte la dirección (servo montado al revés). |

### Simulación

| Parámetro | Default | Descripción |
|---|---|---|
| `--simulate` | (no) | Genera señales sintéticas, sin micrófonos ni servo. |
| `--sim-angle` | 60 | Ángulo de la fuente simulada (°). |
| `--sim-freq` | 1000 | Frecuencia del tono simulado (Hz). |
| `--sim-snr` | 20 | SNR de la simulación (dB). |


## Registro de eventos

Formato del CSV (`eventos_doa.csv`):

```
evento,timestamp_inicio,timestamp_fin,duracion_ms,angulo_promedio,angulo_std,confianza_promedio,energia_pico,num_frames
1,1773084145.184,1773084145.900,716.0,120.3,1.2,0.854,6.05e-01,17
```

Timestamps en formato Unix. Conversión a hora legible:

```python
from datetime import datetime
datetime.fromtimestamp(1773084145.184)
```


## Visualización en terminal

```
180°[──────────────●──────────────│─────────────────────────────]0°  θ= 136.2° srv=130.0° buf[▮▮▮▯▯] ✓ conf[████████░░]
```

- **Barra**: posición angular. `│` = centro (90°), `●` = ángulo estimado
- **θ**: ángulo DOA estimado (promedio móvil de las últimas 10 estimaciones)
- **srv**: ángulo actual del servo
- **buf**: llenado del buffer del servo
- **✓/⚠**: estimación válida o clampeada
- **conf**: confianza del pico GCC-PHAT (10 barras = 1.0)

Sin señal suficiente: `[SILENCIO]`. Al detectar un evento: `[EVENTO #3] θ=120.3°±1.2° | dur=716ms | E_pico=6.05e-01 | frames=17`.


## Límites del sistema

- **Resolución angular**: ~2–5° con interpolación parabólica (SNR razonable); ~21° sin interpolación (d=3cm, fs=48kHz).
- **Frecuencia máxima útil**: f_max = c/(2·d) ≈ 5717 Hz con d=3cm. Por encima hay aliasing espacial.
- **Ambigüedad arriba/abajo**: con 2 micrófonos en línea existe cono de confusión (resolverlo requiere ≥3 mics fuera del plano).
- **Reverberación**: GCC-PHAT es robusto ante reverberación moderada; en ambientes muy reverberantes puede haber picos espurios (subir `--servo-min-conf` ayuda).
- **Velocidad del sonido**: default 343 m/s (aire, ~20°C). Ajustable en `speed_of_sound` dentro de `DEFAULT_CONFIG`.


## Ajuste de umbrales

1. Ejecutar sin servo y observar el valor de energía (`E=`) en la terminal.
2. En silencio, anotar el nivel de energía y setear `--umbral-ruido` un poco por encima.
3. Generar el sonido a detectar (aplauso, voz fuerte) y anotar su energía.
4. Setear `--umbral-evento` entre ambos niveles.

```bash
python3 gccphat_servo_2mics.py --umbral-ruido 1e-5 --umbral-evento 1e-3
```


## Troubleshooting

| Problema | Solución |
|---|---|
| `PortAudioError: Invalid sample rate` | Probar `--sample-rate 48000`, `16000` o `32000`. |
| `arecord -l` no muestra tarjeta de captura | Overlay I2S no cargó. Revisar `cat /boot/config.txt \| grep -E "i2s\|google\|dtoverlay"` y `dmesg \| grep -i i2s`. |
| El servo tiembla | Aumentar `--servo-dead-zone`, usar `pigpiod` como backend, o un driver I2C (PCA9685). |
| El ángulo estimado oscila mucho | Subir `--servo-batch` y/o `--servo-min-conf`; verificar distancia y soldadura de los micrófonos. |
| `AttributeError: module 'numpy' has no attribute 'fftshift'` | En numpy 2.x usar `np.fft.fftshift` en vez de `np.fftshift` (función `gcc_phat`). |


## Dependencias

| Paquete | Uso |
|---|---|
| numpy | FFT, operaciones numéricas |
| sounddevice | Captura de audio (PortAudio/ALSA) |
| gpiozero | Control del servo (solo con `--servo`), preinstalada en RPi OS |
