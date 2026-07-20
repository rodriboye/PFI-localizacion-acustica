#!/usr/bin/env python3
"""
Calibración de 2 puntos para servo: encuentra los pulsos exactos para 0° y 180°
Esto es más preciso que asumir un rango estándar.
"""

import time
import sys

def calibrate_endpoints(gpio_pin=12):
    try:
        import pigpio
        print(f"Conectando a pigpiod (GPIO pin {gpio_pin})...\n")
        pi = pigpio.pi()
        
        if not pi.connected:
            print("ERROR: no se puede conectar a pigpiod")
            print("Ejecutar primero: sudo pigpiod -t 0")
            return
        
        print("✓ Conectado a pigpiod\n")
        print("=" * 60)
        print("CALIBRACIÓN DE SERVO - Método de 2 puntos (0° y 180°)")
        print("=" * 60)
        print()
        
        # PASO 1: Encontrar el pulso para 0° (extremo izquierdo)
        print("PASO 1: Encontrar pulso para 0° (extremo izquierdo)")
        print("-" * 60)
        print("Ingresa valores de pulso hasta que el servo llegue al extremo")
        print("izquierdo (0°). Valores típicos: 1000-1200 μs")
        print()
        
        pulse_0 = None
        while pulse_0 is None:
            try:
                pulse_str = input("Pulso para 0°: ").strip()
                pulse = int(pulse_str)
                if 100 <= pulse <= 2500:
                    pi.set_servo_pulsewidth(gpio_pin, pulse)
                    print(f"  → Enviado: {pulse} μs")
                    time.sleep(1.5)
                    response = input("  ¿Es este el extremo izquierdo (0°)? (s/n): ").strip().lower()
                    if response == 's':
                        pulse_0 = pulse
                        print(f"✓ Guardado: 0° = {pulse_0} μs\n")
                else:
                    print("Valor fuera de rango (100-2500), intenta de nuevo")
            except ValueError:
                print("Valor inválido, intenta de nuevo")
        
        # PASO 2: Encontrar el pulso para 180° (extremo derecho)
        print("PASO 2: Encontrar pulso para 180° (extremo derecho)")
        print("-" * 60)
        print("Ingresa valores de pulso hasta que el servo llegue al extremo")
        print("derecho (180°). Valores típicos: 1800-2000 μs")
        print()
        
        pulse_180 = None
        while pulse_180 is None:
            try:
                pulse_str = input("Pulso para 180°: ").strip()
                pulse = int(pulse_str)
                if 100 <= pulse <= 2500:
                    pi.set_servo_pulsewidth(gpio_pin, pulse)
                    print(f"  → Enviado: {pulse} μs")
                    time.sleep(1.5)
                    response = input("  ¿Es este el extremo derecho (180°)? (s/n): ").strip().lower()
                    if response == 's':
                        pulse_180 = pulse
                        print(f"✓ Guardado: 180° = {pulse_180} μs\n")
                else:
                    print("Valor fuera de rango (100-2500), intenta de nuevo")
            except ValueError:
                print("Valor inválido, intenta de nuevo")
        
        # PASO 3: Mostrar el resultado
        print("=" * 60)
        print("RESULTADO DE CALIBRACIÓN")
        print("=" * 60)
        print()
        print(f"  0°  → {pulse_0} μs")
        print(f"  180° → {pulse_180} μs")
        print()
        print("Actualiza el código con estos valores:")
        print()
        print(f"    MIN_PULSE = {pulse_0}   # microsegundos para 0°")
        print(f"    MAX_PULSE = {pulse_180}  # microsegundos para 180°")
        print()
        
        # Verificación: probar con 90°
        pulse_90 = (pulse_0 + pulse_180) // 2
        print(f"Verificación: Enviando pulso para 90° = {pulse_90} μs")
        pi.set_servo_pulsewidth(gpio_pin, pulse_90)
        time.sleep(1.5)
        response = input("¿Está en el centro (90°)? (s/n): ").strip().lower()
        
        pi.set_servo_pulsewidth(gpio_pin, 0)
        pi.stop()
        
        if response == 's':
            print("\n✓ CALIBRACIÓN EXITOSA")
        else:
            print("\n⚠ Podría haber un desplazamiento. Intenta con:")
            print(f"  MIN_PULSE = {pulse_0 + 50}")
            print(f"  MAX_PULSE = {pulse_180 + 50}")
        
    except ImportError:
        print("ERROR: pigpio no disponible")
        print("Instalar con: pip install pigpio")
    except Exception as e:
        print(f"ERROR: {e}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--pin", type=int, default=12, help="GPIO pin (default: 12)")
    args = parser.parse_args()
    calibrate_endpoints(gpio_pin=args.pin)
