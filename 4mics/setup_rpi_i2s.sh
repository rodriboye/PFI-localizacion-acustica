#
# prepara el Raspberry Pi para capturar audio
# desde cuatro micrófonos INMP441 vía I2S.
#
# Ejecutar como root: sudo bash setup_rpi_i2s.sh
# Requiere reinicio después de ejecutar.

set -e

echo "========================================="
echo "  Setup I2S para INMP441 en Raspberry Pi"
echo "========================================="

# Verificar que se ejecuta como root
if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Ejecutar con sudo"
    exit 1
fi

#-------------------------------------------------------------------------
# 1. Habilitar I2S en config.txt

CONFIG="/boot/config.txt"
# puede variar en otras distribuciones

echo "[1/4] Habilitando I2S en $CONFIG"

# Habilitar i2s si no está ya
if ! grep -q "^dtparam=i2s=on" "$CONFIG"; then
    echo "dtparam=i2s=on" >> "$CONFIG"
    echo "  → Agregado dtparam=i2s=on"
else
    echo "  → Ya habilitado"
fi

#-------------------------------------------------------------------------
# 2. Cargar driver I2S

# ADAU7002 es el overlay correcto para micrófonos INMP441 con ese codec
# NO usar googlevoicehat-soundcard (error común - ese es para otros hardware)

echo "[2/4] Configurando overlay de audio I2S para ADAU7002"

# El overlay CORRECTO para este proyecto
OVERLAY_LINE="dtoverlay=adau7002-simple"

if grep -q "^dtoverlay=adau7002-simple" "$CONFIG"; then
    echo "  → Overlay ADAU7002 ya configurado ✓"
elif grep -q "^dtoverlay=googlevoicehat-soundcard" "$CONFIG"; then
    echo "  [ERROR] Encontrado overlay INCORRECTO: googlevoicehat-soundcard"
    echo "  Reemplazando por el correcto: $OVERLAY_LINE"
    sed -i '/^dtoverlay=googlevoicehat-soundcard/d' "$CONFIG"
    echo "$OVERLAY_LINE" >> "$CONFIG"
    echo "  → Overlay reemplazado ✓"
elif grep -q "^dtoverlay=" "$CONFIG"; then
    echo "  [WARN] Hay otro overlay I2S. Verifica que sea compatible con ADAU7002:"
    grep "^dtoverlay=" "$CONFIG"
else
    echo "  → Agregando $OVERLAY_LINE"
    echo "$OVERLAY_LINE" >> "$CONFIG"
    echo "  → Overlay configurado ✓"
fi

#------------------------------------------------------------------------------
# 3. Instalar dependencias Python

echo "[3/4] Instalando dependencias Python"

# Verificar si estamos en un entorno con pip restringido (Bookworm)
if python3 -c "import sounddevice" 2>/dev/null; then
    echo "  → sounddevice ya instalado"
else
    # Intentar con --break-system-packages (necesario en Bookworm)
    pip3 install numpy sounddevice --break-system-packages 2>/dev/null || \
    pip3 install numpy sounddevice || \
    echo "  [WARN] No se pudo instalar automáticamente. Ejecutar manualmente:"
    echo "         pip3 install numpy sounddevice --break-system-packages"
fi

#-----------------------------------------------------------------------------
# 4. Configuración ALSA

echo "[4/4] Configuración ALSA"

# Crear .asoundrc para el usuario si no existe
ASOUNDRC="/home/$(logname)/.asoundrc"
if [ ! -f "$ASOUNDRC" ]; then
    cat > "$ASOUNDRC" << 'EOF'
# Configuración ALSA para micrófonos I2S con ADAU7002
# Este archivo mapea el dispositivo I2S como default de captura.

pcm.i2s_input {
    type hw
    card sndrpiaudios     # Nombre para ADAU7002 (verificar con: arecord -l)
    device 0
}

pcm.!default {
    type asym
    capture.pcm "i2s_input"
}
EOF
    chown "$(logname):$(logname)" "$ASOUNDRC"
    echo "  → Creado $ASOUNDRC para ADAU7002"
    echo "  → Verificar el nombre exacto con: arecord -l"
    echo "    Ajustar 'card sndrpiaudios' si es diferente"
else
    echo "  → $ASOUNDRC ya existe"
fi

echo ""
echo "========================================="
echo "  Setup completado. REINICIAR el RPi:"
echo "  sudo reboot"
echo ""
echo "  Después del reinicio, verificar con:"
echo "    arecord -l"
echo "    arecord -D plughw:0,0 -c 2 -r 44100 -f S32_LE -d 3 /tmp/test.wav"
echo "========================================="
