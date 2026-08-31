# tesis-ssl — Sistema de Localización de Fuente de Sonido

**Autor:** Rodrigo Boyé
**Proyecto Final Integrador** — Ingeniería en Telecomunicaciones
Universidad Nacional de Río Negro — Trabajo realizado con Invap
Bariloche, Argentina, 2026

Estimación de la dirección de arribo (DOA) de una fuente sonora en tiempo real sobre Raspberry Pi. El repositorio contiene dos sistemas independientes y el material de la tesis (documento, referencias).

## Sistemas

| Carpeta | Qué hace | Hardware |
|---|---|---|
| [`2mics/`](2mics/README.md) | DOA 1D (azimut, 0–180°) con GCC-PHAT. Seguimiento con un servo. | 2× INMP441 + RPi + 1 servo (opcional) |
| [`4mics/`](4mics/README.md) | DOA 2D (azimut + elevación) con MUSIC. Detección de dron por firma armónica, seguimiento con 2 servos. | 4× INMP441 + ESP32 (frontend I2S) + RPi + 2 servos (opcional) |

Cada carpeta es autocontenida: tiene su propio README con instalación, uso y parámetros. Ver esos archivos para el detalle de cada sistema.

## Estructura del repositorio

```
tesis-ssl/
├── 2 microfonos/               # Sistema DOA de 2 micrófonos (GCC-PHAT)
├── 4 microfonos/               # Sistema DOA de 4 micrófonos (MUSIC / SRP-PHAT)
├── Informe PFI.pdf		# Documento que desarrolla el proyecto
├── referencias/		# Papers citados (PDF)
├── requirements.txt            # Dependencias Python unificadas (ambos sistemas)
└── setup_venv.sh               # Script para crear el entorno virtual
```

## Instalación (entorno unificado)

```bash
git clone <repo>
cd PFI-localizacion-acustica
bash setup_venv.sh
source venv/bin/activate
```

Esto crea `venv/` en la raíz e instala todo lo necesario para ambos sistemas (`requirements.txt`). Cada sistema tiene además pasos propios de configuración de hardware — ver su README.
