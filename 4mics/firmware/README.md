# Firmware del ESP32 — compilar, flashear y verificar

Captura el audio de los cuatro micrófonos de forma sincronizada, los empaqueta y los envía por USB a la Pi

## Verificar qué está flasheado

El firmware se identifica solo, se ve desde la Pi con:

```bash
python3 diagnose_serial.py /dev/ttyUSB0 --raw
```

y debe mostrar una línea como:

```
[SSL-FW] build Aug  5 2026 14:22:01 | reset=EXT | fs=11025 hop=256 ch=4 shift=9 cpu=240MHz
```