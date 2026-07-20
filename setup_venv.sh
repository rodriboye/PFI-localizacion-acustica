#!/bin/bash
# Script de Setup: Crear entorno virtual unificado
# Uso: bash setup_venv.sh

echo "═══════════════════════════════════════════════════════════"
echo "  Setup Entorno Virtual Unificado - Sistema DOA"
echo "═══════════════════════════════════════════════════════════"
echo ""

# Verificar que estamos en la raíz del proyecto
if [ ! -f "requirements.txt" ]; then
    echo "❌ Error: Ejecutar desde la raíz del proyecto, donde esta requirements.txt"
    exit 1
fi

# Crear venv en la raíz
echo "📦 Creando entorno virtual en ./venv ..."
python3 -m venv venv

if [ $? -ne 0 ]; then
    echo "❌ Error al crear venv"
    echo "   Intenta: python3 -m pip install --upgrade pip"
    exit 1
fi

echo "✓ Venv creado"
echo ""

# Activar venv
echo "🔄 Activando venv..."
source venv/bin/activate

# Actualizar pip
echo "📥 Actualizando pip, setuptools, wheel..."
pip install --upgrade pip setuptools wheel

# Instalar dependencias
echo ""
echo "📦 Instalando dependencias unificadas desde requirements.txt..."
pip install -r requirements.txt

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Setup completado exitosamente!"
    echo ""
    echo "═══════════════════════════════════════════════════════════"
    echo "  PRÓXIMOS PASOS:"
    echo "═══════════════════════════════════════════════════════════"
    echo ""
    echo "Para activar el venv en futuras sesiones:"
    echo "  source venv/bin/activate"
    echo ""
    echo "Para desactivar:"
    echo "  deactivate"
    echo ""
    echo "Para instalar nuevas dependencias:"
    echo "  pip install <nombre-paquete>"
    echo ""
    echo "Para actualizar requirements.txt:"
    echo "  pip freeze > requirements.txt"
    echo ""
    echo "Ejecutar scripts del proyecto (dentro del venv):"
    echo "  cd 4mics && python main.py"
    echo "  cd 2mics && python gccphat_servo_2mics.py"
    echo ""
else
    echo ""
    echo "❌ Error durante la instalación"
    echo "   Revisa los errores arriba"
    exit 1
fi
