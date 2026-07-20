#!/bin/bash
#
# Script de diagnóstico para verificar la configuración de audio I2S
# Útil para detectar problemas antes de ejecutar el sistema
#
# Ejecutar: bash diagnose_audio.sh

set -e

echo "=================================================="
echo "  Diagnóstico de Configuración de Audio I2S"
echo "=================================================="
echo ""

# Color para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

ERRORS=0
WARNINGS=0

# Función para verificar
check() {
    local msg=$1
    local cmd=$2
    echo -n "$msg ... "
    if eval "$cmd" > /dev/null 2>&1; then
        echo -e "${GREEN}✓${NC}"
        return 0
    else
        echo -e "${RED}✗${NC}"
        ((ERRORS++))
        return 1
    fi
}

warn() {
    local msg=$1
    echo -e "${YELLOW}⚠ $msg${NC}"
    ((WARNINGS++))
}

echo "1. VERIFICACIÓN DE CONFIGURACIÓN DE BOOTLOADER"
echo "==============================================="
CONFIG="/boot/config.txt"
if [ ! -f "$CONFIG" ]; then
    CONFIG="/boot/firmware/config.txt"
fi

if [ ! -f "$CONFIG" ]; then
    echo -e "${RED}✗ No encontrado: $CONFIG${NC}"
    exit 1
fi

echo "Archivo: $CONFIG"
echo ""

# Verificar dtparam=i2s=on
if grep -q "^dtparam=i2s=on" "$CONFIG"; then
    echo -e "${GREEN}✓ I2S habilitado${NC}"
else
    echo -e "${RED}✗ I2S NO habilitado${NC}"
    echo "  Agregar a $CONFIG: dtparam=i2s=on"
    ((ERRORS++))
fi

# Verificar overlay correcto
if grep -q "^dtoverlay=adau7002-simple" "$CONFIG"; then
    echo -e "${GREEN}✓ Overlay CORRECTO: adau7002-simple${NC}"
elif grep -q "^dtoverlay=googlevoicehat-soundcard" "$CONFIG"; then
    echo -e "${RED}✗ Overlay INCORRECTO: googlevoicehat-soundcard${NC}"
    echo "  Este overlay NO es compatible con ADAU7002"
    echo "  Cambiar a: dtoverlay=adau7002-simple"
    ((ERRORS++))
elif grep -q "^dtoverlay=" "$CONFIG"; then
    OVERLAY=$(grep "^dtoverlay=" "$CONFIG")
    warn "Overlay encontrado pero NO es adau7002-simple: $OVERLAY"
else
    echo -e "${RED}✗ No hay overlay I2S configurado${NC}"
    echo "  Agregar a $CONFIG: dtoverlay=adau7002-simple"
    ((ERRORS++))
fi

echo ""
echo "2. VERIFICACIÓN DE HARDWARE"
echo "============================"

# Verificar si es Raspberry Pi
if [ -f /proc/device-tree/model ]; then
    MODEL=$(cat /proc/device-tree/model | tr -d '\0')
    echo "Modelo: $MODEL"
else
    warn "No se pudo detectar el modelo de RPi"
fi

# Listar tarjetas de sonido
echo ""
echo "Tarjetas de audio disponibles (arecord -l):"
if command -v arecord &> /dev/null; then
    arecord -l
else
    warn "arecord no disponible"
fi

echo ""
echo "3. VERIFICACIÓN DE DEPENDENCIAS"
echo "================================"

check "Python 3 instalado" "python3 --version"
check "NumPy disponible" "python3 -c 'import numpy; print(numpy.__version__)'"
check "sounddevice disponible" "python3 -c 'import sounddevice; print(sounddevice.__version__)'"
check "gpiozero disponible" "python3 -c 'import gpiozero; print(gpiozero.__version__)'"

echo ""
echo "4. VERIFICACIÓN DE PERMISOS"
echo "============================"

# Verificar si puede acceder a GPIO (solo si no es root)
if [ "$EUID" -ne 0 ]; then
    if [ -c /dev/gpiomem ]; then
        if [ -r /dev/gpiomem ] && [ -w /dev/gpiomem ]; then
            echo -e "${GREEN}✓ Acceso a /dev/gpiomem: OK${NC}"
        else
            echo -e "${YELLOW}⚠ Sin permisos a /dev/gpiomem${NC}"
            echo "  Solución: sudo usermod -a -G gpio $USER"
            echo "  Luego reiniciar sesión"
            ((WARNINGS++))
        fi
    else
        warn "/dev/gpiomem no encontrado (normal en emulación)"
    fi
fi

echo ""
echo "5. RECOMENDACIONES"
echo "==================="

if [ "$ERRORS" -eq 0 ] && [ "$WARNINGS" -eq 0 ]; then
    echo -e "${GREEN}✓ Todo parece estar bien${NC}"
    echo ""
    echo "Próximos pasos:"
    echo "  1. Si cambió el config.txt, reiniciar: sudo reboot"
    echo "  2. Después, verificar capture: arecord -D plughw:CARD=sndrpiaudios,DEV=0 -c 4 -r 48000 -f S32_LE -d 3 /tmp/test.wav"
    echo "  3. Ejecutar el script principal"
else
    echo -e "${RED}Se encontraron $ERRORS error(s) y $WARNINGS advertencia(s)${NC}"
    if [ "$ERRORS" -gt 0 ]; then
        echo ""
        echo "PASOS PARA CORREGIR:"
        echo "  1. Editar: sudo nano $CONFIG"
        echo "  2. Asegurarse que está: dtparam=i2s=on"
        echo "  3. Asegurarse que está: dtoverlay=adau7002-simple"
        echo "  4. Guardar (Ctrl+X, Y, Enter)"
        echo "  5. Reiniciar: sudo reboot"
        echo "  6. Después del reinicio, ejecutar nuevamente: bash diagnose_audio.sh"
    fi
fi

echo ""
echo "=================================================="
