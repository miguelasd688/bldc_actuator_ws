import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import time
import math

# Conecta al ODrive
print("Buscando ODrive...")
odrv = odrive.find_any()
axis = odrv.axis0  # Cambiar a axis1 si corresponde

dump_errors(odrv)
odrv.clear_errors()
# 1. Espera a que el encoder esté listo (por si es ABS)
print("Esperando que el encoder esté listo...")
while axis.encoder.is_ready == False:
    time.sleep(0.1)

# 2. Leer posición actual y establecerla como 0
current_pos = axis.encoder.pos_estimate
axis.encoder.config.index_offset = current_pos
print(f"Encoder lineal reiniciado. Posición anterior: {current_pos:.3f} vueltas")

# 3. Guarda configuración
print("Guardando configuración...")
try:
    odrv.save_configuration()
    #odrv.reboot()
    print("Reiniciando ODrive...")
except Exception as e:
    if "DeviceLostException" in str(type(e)):
        print("ODrive reiniciado correctamente (se perdió la conexión, esperado).")
    else:
        raise e