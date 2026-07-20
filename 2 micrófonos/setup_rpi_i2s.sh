#
# prepara el Raspberry Pi para capturar audio
# estéreo desde dos micrófonos INMP441 vía I2S.
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

# googlevoicehat-soundcard (funciona en muchas RPi)
# Usamos la opción más compatible. Si no funciona, probar las alternativas.

echo "[2/4] Configurando overlay de audio I2S"

# Verificar si ya hay un overlay de audio I2S
OVERLAY_LINE="dtoverlay=googlevoicehat-soundcard"
OVERLAY_ALT="dtoverlay=i2s-mmap"

if grep -q "^dtoverlay=googlevoicehat-soundcard" "$CONFIG" || \
   grep -q "^dtoverlay=i2s-mmap" "$CONFIG"; then
    echo "  → Overlay I2S ya configurado"
else
    echo "  → Agregando $OVERLAY_LINE"
    echo "  → Si no funciona, reemplazar por: $OVERLAY_ALT"
    echo "$OVERLAY_LINE" >> "$CONFIG"
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
# Configuración ALSA para micrófonos I2S INMP441
# Este archivo mapea el dispositivo I2S como default de captura.

pcm.i2s_input {
    type hw
    card sndrpigooglevoi    # Ajustar al nombre real (ver arecord -l)
    device 0
}

pcm.!default {
    type asym
    capture.pcm "i2s_input"
}
EOF
    chown "$(logname):$(logname)" "$ASOUNDRC"
    echo "  → Creado $ASOUNDRC"
    echo "  → IMPORTANTE: verificar el nombre de la tarjeta con 'arecord -l'"
    echo "    y ajustar 'card' en .asoundrc si es diferente"
else
    echo "  → $ASOUNDRC ya existe, no se modifica"
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
