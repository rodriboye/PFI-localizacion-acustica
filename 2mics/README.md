Estimación en tiempo real del ángulo de llegada de sonido usando dos micrófonos MEMS INMP441, algoritmo GCC-PHAT, seguimiento con servomotor, y registro de eventos acústicos.

## Qué hace el sistema

El sistema captura audio estéreo desde dos micrófonos I2S, estima la dirección de la fuente sonora cuadro a cuadro usando correlación cruzada generalizada (GCC-PHAT), y puede opcionalmente controlar un servomotor para que apunte hacia la fuente. Cuando la energía del sonido supera un umbral configurable, el evento se registra en un archivo CSV con su ángulo, duración y confianza.

El ángulo de salida va de 0° a 180°, donde 90° es el frente del array (broadside), 0° es el extremo derecho y 180° el extremo izquierdo. Con dos micrófonos solo se resuelve el azimut en un plano — no hay elevación ni distinción arriba/abajo.


## Archivos

| Archivo | Descripción |
|---|---|
| `main.py` | Programa principal. Todo el sistema en un solo archivo. |
| `setup_rpi_i2s.sh` | Script de configuración de I2S en la Raspberry Pi. Ejecutar una única vez antes de usar. |
| `eventos_doa.csv` | Log de eventos (se crea automáticamente al ejecutar). |


## Hardware necesario

- Raspberry Pi (probado en 3A+) con Raspberry Pi OS
- 2x micrófonos INMP441 (MEMS, salida I2S, omnidireccionales)
- 1x servomotor estándar (SG90 o similar) — opcional
- Cables, protoboard, resistor 100 kΩ pull-down en la línea SD


## Conexiones

### Micrófonos I2S

Ambos micrófonos comparten las líneas SCK, WS y SD. Se distinguen por el pin L/R.

| INMP441 Pin | RPi GPIO (BCM) | Notas |
|---|---|---|
| SCK | GPIO 18 | Bit Clock |
| WS | GPIO 19 | Word Select (LRCLK) |
| SD | GPIO 20 | Data — unir SD de ambos mics |
| VDD | 3.3V | Alimentación |
| GND | GND | |
| L/R (mic izq) | GND | Canal izquierdo |
| L/R (mic der) | 3.3V | Canal derecho |

### Servomotor

| Cable | RPi GPIO (BCM) | Notas |
|---|---|---|
| Señal (naranja/blanco) | GPIO 12 | PWM0. También se puede usar GPIO 13 (PWM1). No usar GPIO 18, está ocupado por I2S. |
| Alimentación (rojo) | 5V | Para SG90. Para servos de mayor consumo (MG996R), usar fuente externa de 5V. |
| Tierra (negro/marrón) | GND | Tierra común con la RPi |


## Instalación

### 1. Configurar I2S en la Raspberry Pi

```bash
sudo bash setup_rpi_i2s.sh
sudo reboot
```

Después del reinicio, verificar que la tarjeta de audio aparece:

```bash
arecord -l
```

Debe mostrar alguna tarjeta de captura I2S. Si no aparece, revisar que `/boot/config.txt` (o `/boot/firmware/config.txt` en Bookworm) tenga `dtparam=i2s=on` y el overlay correspondiente.

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

Reemplazar `<nombre>` con el nombre que muestra `arecord -l`. Si da error de sample rate, probar con 16000 o 32000.

### 4. Verificar el dispositivo de audio para el script

```bash
python3 -m sounddevice
```

Esto lista los dispositivos disponibles con su índice. Usar ese índice con `--device` si el default no funciona.


## Uso

### Ejecución básica (solo DOA en terminal)

```bash
python3 gccphat_servo_2mics.py
```

### Con servomotor

```bash
python3 gccphat_servo_2mics.py --servo
```

### Modo simulación (sin hardware)

```bash
python3 gccphat_servo_2mics.py --simulate --sim-angle 120
python3 gccphat_servo_2mics.py --simulate --servo --sim-angle 75
```

### Especificar dispositivo de audio

```bash
python3 gccphat_servo_2mics.py --device 1
```

### Detener

Ctrl+C. El programa cierra el stream de audio, desactiva el servo, escribe los eventos pendientes al log, y muestra un resumen.


## Parámetros configurables

Todos se pueden cambiar desde la línea de comandos. Los defaults están en `DEFAULT_CONFIG` al inicio del código.

### Audio y DOA

| Parámetro | Default | Descripción |
|---|---|---|
| `--sample-rate` | 48000 | Frecuencia de muestreo en Hz. El driver I2S de la RPi suele soportar 48000. Si da error, probar 16000 o 32000. |
| `--block-size` | 2048 | Muestras por bloque FFT. Más alto = estimación más estable pero más latencia. 2048 a 48 kHz = ~43 ms por bloque. |
| `--mic-distance` | 0.03 | Separación entre micrófonos en metros. Afecta la resolución angular y la frecuencia máxima útil. |
| `--device` | (auto) | Dispositivo de audio. Puede ser un índice numérico o un nombre. Listar con `python3 -m sounddevice`. |
| `--umbral-ruido` | 1e-4 | Energía mínima para considerar que hay señal. Por debajo de esto no se estima DOA. |
| `--umbral-evento` | umbral_ruido × 30 | Energía mínima para registrar un evento. El factor multiplicador `k` se ajusta en el código (default: 30). |
| `--log-path` | eventos_doa.csv | Ruta del archivo donde se guardan los eventos. |

### Servo

| Parámetro | Default | Descripción |
|---|---|---|
| `--servo` | (desactivado) | Flag para activar el control del servomotor. |
| `--servo-pin` | 12 | Pin GPIO (numeración BCM) para la señal PWM del servo. |
| `--servo-batch` | 15 | Cuántas estimaciones acumular antes de decidir si mover el servo. A 48 kHz con bloques de 2048: 15 batches ≈ 640 ms. |
| `--servo-min-conf` | 0.2 | Confianza mínima del pico GCC-PHAT para que la estimación entre al buffer del servo. Estimaciones con menor confianza se descartan. |
| `--servo-dead-zone` | 8.0 | Grados mínimos de diferencia para que el servo se mueva. Evita micro-movimientos por ruido. |
| `--servo-max-step` | 10.0 | Grados máximos por movimiento. Limita la velocidad para que el servo no salte bruscamente. |
| `--servo-lock-time` | 3.0 | Segundos que el servo se queda fijo apuntando al ángulo de un evento detectado. Durante este tiempo ignora nuevas estimaciones. |
| `--servo-invert` | (no) | Invierte la dirección: servo_angle = 180 - doa_angle. Usar si el servo está montado al revés. |

### Simulación

| Parámetro | Default | Descripción |
|---|---|---|
| `--simulate` | (no) | Activa el modo simulación. Genera señales sintéticas sin necesidad de micrófonos ni servo. |
| `--sim-angle` | 60 | Ángulo de la fuente simulada (grados). |
| `--sim-freq` | 1000 | Frecuencia del tono simulado (Hz). |
| `--sim-snr` | 20 | Relación señal a ruido de la simulación (dB). |


## Cómo funciona cada parte

### Algoritmo GCC-PHAT

Implementado desde cero usando solo NumPy. No usa librerías de procesamiento de señales externas.

El flujo por cada bloque de audio es:

1. **Ventana de Hanning** sobre ambos canales para reducir leakage espectral
2. **FFT** de cada canal (con zero-padding al doble para evitar aliasing circular)
3. **Cross-spectrum**: G₁₂(f) = X₁(f) · X₂*(f) — la fase contiene la información de retardo
4. **Normalización PHAT**: dividir por la magnitud, dejando solo la fase. Produce un pico más agudo que la correlación cruzada pura y es más robusto ante reverberación moderada
5. **IFFT** para obtener la función de correlación en el dominio temporal
6. **Búsqueda del pico** restringida al rango de retardos físicamente posibles (±d/c × fs muestras)
7. **Interpolación parabólica** alrededor del pico para resolución sub-muestra
8. **Conversión a ángulo**: θ = arccos(τ · c / d), usando el modelo de onda plana (far-field)

### Control del servo

El servo no se mueve en cada frame. Acumula `batch_size` estimaciones de DOA, descarta las de baja confianza y los outliers (fuera de ±2σ de la mediana), promedia las restantes, y solo mueve si la diferencia supera la zona muerta. El movimiento está limitado a `max_step` grados por batch para evitar saltos bruscos.

Después de cada movimiento, el PWM se mantiene activo durante 0.3 segundos y luego se apaga (`servo.angle = None`). El servo mantiene su posición por fricción mecánica, pero deja de recibir los pulsos de software PWM que causan temblor (jitter).

Cuando hay silencio sostenido (~1.3 segundos), el servo vuelve gradualmente al centro (90°).

Cuando se detecta un evento (energía > umbral_evento), el servo va directo al ángulo del evento y se bloquea ahí durante `servo-lock-time` segundos, ignorando todas las estimaciones intermedias.

### Registro de eventos

Siempre activo. Cada vez que la energía supera `umbral_evento` se abre un evento. Mientras la energía se mantenga alta, se acumulan las estimaciones de ángulo. Cuando la energía baja (o hay silencio), el evento se cierra y se escribe una línea al CSV.

Eventos de un solo frame se descartan como espurios (requiere al menos 2 frames consecutivos).

El archivo CSV tiene este formato:

```
evento,timestamp_inicio,timestamp_fin,duracion_ms,angulo_promedio,angulo_std,confianza_promedio,energia_pico,num_frames
1,1773084145.184,1773084145.900,716.0,120.3,1.2,0.854,6.05e-01,17
```

Los timestamps son Unix (segundos desde epoch). Se pueden convertir a hora legible en postprocesamiento:

```python
from datetime import datetime
datetime.fromtimestamp(1773084145.184)
```


## Visualización en terminal

Mientras corre, el programa muestra una línea actualizada en tiempo real:

```
180°[──────────────●──────────────│─────────────────────────────]0°  θ= 136.2° srv=130.0° buf[▮▮▮▯▯] ✓ conf[████████░░]
```

- **Barra**: posición angular. `│` es el centro (90°), `●` es el ángulo estimado
- **θ**: ángulo DOA estimado (promedio móvil de las últimas 10 estimaciones, solo para display)
- **srv**: ángulo actual del servo (puede diferir del DOA por zona muerta, steps, o lock)
- **buf**: llenado del buffer del servo (cuántas estimaciones acumuladas del batch actual)
- **✓/⚠**: si la estimación es válida (cos θ dentro de [-1, 1]) o si hubo que clampear
- **conf**: confianza del pico GCC-PHAT (10 barras = confianza 1.0)

Cuando no hay señal suficiente muestra `[SILENCIO]`.

Cuando se detecta un evento, se imprime una línea aparte:

```
[EVENTO #3] θ=120.3°±1.2° | dur=716ms | E_pico=6.05e-01 | frames=17
```


## Límites del sistema

### Resolución angular

Con d = 3 cm y fs = 48000 Hz, el retardo máximo entre micrófonos es ~4.2 muestras. Sin interpolación la resolución sería de ~21°. La interpolación parabólica la mejora a ~2-5° en condiciones de SNR razonable.

### Frecuencia máxima útil (aliasing espacial)

f_max = c / (2·d). Con d = 3 cm: f_max ≈ 5717 Hz. Las frecuencias por encima de esto producen ambigüedad espacial. Para voz (fundamental ~100-300 Hz, armónicos hasta ~4 kHz) la separación de 3 cm es adecuada.

### Ambigüedad

Con dos micrófonos en línea hay ambigüedad arriba/abajo (cono de confusión). Una fuente a 60° desde arriba produce el mismo retardo que una a 60° desde abajo. Para resolverlo se necesitan micrófonos fuera del plano (mínimo 3 micrófonos en triángulo).

### Reverberación

GCC-PHAT es robusto ante reverberación moderada, pero en ambientes muy reverberantes (salas chicas con paredes duras) las reflexiones pueden producir picos espurios en la correlación. En esos casos subir `--servo-min-conf` ayuda a descartar estimaciones malas.

### Velocidad del sonido

El default es 343 m/s (aire a ~20°C). La velocidad varía con la temperatura: c ≈ 331 + 0.6·T (°C). En Bariloche en invierno (~0°C) sería ~331 m/s. Para ajustar, modificar `speed_of_sound` en `DEFAULT_CONFIG`.


## Ajuste de umbrales

Los dos umbrales más importantes son `umbral_ruido` y `umbral_evento`. Los valores correctos dependen del ambiente y de la ganancia de los micrófonos.

Para encontrar los valores adecuados:

1. Ejecutar el programa sin servo y observar el valor de energía (`E=`) que muestra en la terminal
2. En silencio, tomar nota del nivel de energía. Setear `umbral_ruido` ligeramente por encima de ese valor
3. Generar un sonido representativo del evento que se quiere detectar (por ejemplo aplaudir, hablar fuerte). Tomar nota de la energía
4. Setear `umbral_evento` entre el ruido de fondo y el nivel del evento. El factor `k` en el código (default: 30) controla la relación entre ambos umbrales

```bash
# probar con umbrales diferentes
python3 gccphat_servo_2mics.py --umbral-ruido 1e-5 --umbral-evento 1e-3
```


## Troubleshooting

### `PortAudioError: Invalid sample rate`

El driver I2S no soporta la tasa de muestreo pedida. Probar con `--sample-rate 48000`, `--sample-rate 16000`, o `--sample-rate 32000`.

### `arecord -l` no muestra tarjeta de captura

El overlay I2S no cargó. Verificar:

```bash
cat /boot/config.txt | grep -E "i2s|google|dtoverlay"
dmesg | grep -i i2s
```

### El servo tiembla

Si el servo vibra estando quieto, es por jitter del software PWM de gpiozero. El programa ya aplica detach del PWM después de mover, pero si sigue pasando, las opciones son:

- Aumentar `--servo-dead-zone` para que no intente corregir diferencias chicas
- Usar `pigpiod` como backend (requiere instalar pigpio y correr `sudo pigpiod` antes de ejecutar)
- Usar un driver de servo por I2C como PCA9685

### El ángulo estimado oscila mucho

- Subir `--servo-batch` (más estimaciones antes de mover, más estable pero más lento)
- Subir `--servo-min-conf` (descarta más estimaciones ruidosas)
- Verificar que los micrófonos estén bien soldados y que la distancia entre ellos sea precisa

### `AttributeError: module 'numpy' has no attribute 'fftshift'`

En numpy 2.x, `fftshift` se movió a `np.fft.fftshift`. Reemplazar `np.fftshift(gcc)` por `np.fft.fftshift(gcc)` en la función `gcc_phat`.


## Dependencias

| Paquete | Versión | Uso |
|---|---|---|
| numpy | cualquiera | FFT, operaciones numéricas |
| sounddevice | cualquiera | Captura de audio via PortAudio/ALSA |
| gpiozero | (preinstalada en RPi OS) | Control del servo — solo si se usa `--servo` |
