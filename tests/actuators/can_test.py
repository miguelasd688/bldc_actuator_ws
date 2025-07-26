import os
import subprocess
import sys

# sudo ip link set can0 down
# sudo ifconfig can0 txqueuelen 65536
# ip link show can0
# ifconfig
# candump can0
# Configuración
can_interface = 'can0'


def check_can_interface(interface="can0"):
    try:
        output = subprocess.check_output(["ip", "link", "show", interface], stderr=subprocess.STDOUT)
        output = output.decode()
        if "state DOWN" in output or "NO-CARRIER" in output:
            print(f"[ERROR] {interface} está definida pero no está activa (DOWN).")
            return False
        print(f"[OK] {interface} está activa.")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] {interface} no existe o no está configurada.\n{e.output.decode()}")
        return False

if not check_can_interface(can_interface):
    print("[FATAL] CAN no disponible.")
    sys.exit(1)
