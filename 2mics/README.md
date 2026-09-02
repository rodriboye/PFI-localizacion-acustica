# Sistema de Estimación de Dirección de Arribo (DOA) — 2 Micrófonos

Estimación en tiempo real del ángulo de llegada de sonido usando dos micrófonos MEMS INMP441, algoritmo GCC-PHAT, seguimiento con servomotor, y registro de eventos acústicos.

## Qué hace

Captura audio estéreo desde dos micrófonos I2S, estima la dirección de la fuente sonora cuadro a cuadro (GCC-PHAT), y opcionalmente mueve un servomotor para apuntar hacia la fuente. Cuando la energía supera un umbral, registra el evento (ángulo, duración, confianza) en un CSV.


## Archivos

| Archivo | Descripción |
|---|---|
| `gccphat_servo_2mics.py` | Programa principal. Todo el sistema en un solo archivo. |
| `setup_rpi_i2s.sh` | Configuración de I2S en la Raspberry Pi. Ejecutar una vez. |
| `calibrate_servo_endpoints.py` | Calibración de los extremos de recorrido del servo. |
| `diagnose_audio.sh` | Diagnóstico de la captura de audio (ALSA/PortAudio). |
| `eventos_doa.csv` | Log de eventos (se crea automáticamente al ejecutar). |


## Hardware necesario

- Raspberry Pi
- 2× micrófonos INMP441 (MEMS, salida I2S, omnidireccionales)
- 1× servomotor estándar (SG90 o similar) — opcional


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

Para usar el servo (`--seguimiento` o `--evento`), instalar `pigpio` y correr el daemon antes de ejecutar el script:

```bash
pip install pigpio --break-system-packages
sudo pigpiod -t 0   # -t 0 evita conflicto con el reloj usado por I2S
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

El servo tiene dos modos, mutuamente excluyentes (`--seguimiento` o `--evento`); sin ninguno de los dos el sistema corre solo DOA + registro de eventos, sin mover nada.

```bash
# Solo DOA en terminal, sin servo
python3 gccphat_servo_2mics.py

# Servo sigue el DOA en tiempo real
python3 gccphat_servo_2mics.py --seguimiento

# Servo se mueve solo al detectar un evento y se queda fijo unos segundos
python3 gccphat_servo_2mics.py --evento

# Modo simulación (sin hardware)
python3 gccphat_servo_2mics.py --simulate --sim-angle 120
python3 gccphat_servo_2mics.py --simulate --seguimiento --sim-angle 75

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
| `--block-size` | 128 | Muestras por bloque FFT (~2.7 ms a 48 kHz). Bloques cortos + `--acc-blocks` dan baja latencia sin perder SNR (ver más abajo). |
| `--acc-blocks` | 3 | Bloques de cross-spectrum PHAT que se promedian antes de resolver el TDOA (~8 ms de latencia total con los defaults). Más bloques = mejor SNR con mics muy cercanos, más latencia. |
| `--mic-distance` | 0.035 | Separación entre micrófonos (m). |
| `--bandpass-low` | 200 | Corte inferior del pasabanda aplicado al cross-spectrum PHAT (Hz). Filtra ruido de baja frecuencia (viento, vibración estructural). |
| `--bandpass-high` | (auto) | Corte superior del pasabanda (Hz). Si no se especifica, se calcula como 90% de f_max espacial = 0.9 · c/(2·d), recalculado según `--mic-distance`. Evita que bins con aliasing espacial degraden el pico de correlación. |
| `--device` | (auto) | Índice o nombre del dispositivo de audio. Listar con `python3 -m sounddevice`. |
| `--umbral-ruido` | 1e-5 | Energía mínima para estimar DOA. |
| `--umbral-evento` | umbral_ruido × 100 | Energía mínima para registrar un evento. |
| `--log-path` | eventos_doa.csv | Ruta del archivo de eventos. |

### Servo

| Parámetro | Default | Descripción |
|---|---|---|
| `--seguimiento` | (desactivado) | Modo seguimiento: el servo persigue el DOA en tiempo real (batching + zona muerta + paso máximo). Mutuamente excluyente con `--evento`. |
| `--evento` | (desactivado) | Modo evento: el servo solo se mueve al detectar un evento y se queda fijo `--servo-lock-time` segundos. Mutuamente excluyente con `--seguimiento`. |
| `--servo-pin` | 12 | Pin GPIO (BCM) de la señal PWM. |
| `--servo-batch` | 8 | Estimaciones acumuladas antes de decidir movimiento (solo modo seguimiento; ~8 ms cada una con los defaults de bloque/acc-blocks). |
| `--servo-min-conf` | 0.2 | Confianza mínima del pico GCC-PHAT para entrar al buffer del servo (solo modo seguimiento). |
| `--servo-dead-zone` | 5.0 | Grados mínimos de diferencia para mover el servo (solo modo seguimiento). |
| `--servo-max-step` | 20.0 | Grados máximos por movimiento (solo modo seguimiento). |
| `--servo-lock-time` | 3.0 | Segundos que el servo queda fijo tras un evento (ambos modos). |
| `--servo-invert` | (no) | Invierte la dirección (servo montado al revés). |

### Simulación

| Parámetro | Default | Descripción |
|---|---|---|
| `--simulate` | (no) | Genera señales sintéticas, sin micrófonos ni servo. |
| `--sim-angle` | 60 | Ángulo de la fuente simulada (°). |
| `--sim-freq` | 1000 | Frecuencia del tono simulado (Hz). |
| `--sim-snr` | 20 | SNR de la simulación (dB). |


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
| pigpio | Control del servo (con `--seguimiento` o `--evento`). Requiere `sudo pigpiod -t 0` corriendo antes de ejecutar el script. |

`gpiozero` (en `requirements.txt`) es para `doa2_gpiozero.py`, la variante alternativa — `gccphat_servo_2mics.py` no la usa.
