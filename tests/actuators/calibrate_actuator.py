import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import time
import math

# Conecta al ODrive
print("Buscando ODrive...")
odrv = odrive.find_any()
axis = odrv.axis0  # Cambiar a axis1 si corresponde

dump_errors(odrv, clear=True)
# 1. Espera a que el encoder esté listo (por si es ABS)
print("Esperando que el encoder esté listo...")
while axis.encoder.is_ready == False:
    time.sleep(0.1)

# 2. Leer posición actual y establecerla como 0
current_pos = axis.encoder.pos_estimate
axis.encoder.config.index_offset 
print(f"Encoder lineal reiniciado. Posición anterior: {current_pos:.3f} vueltas")

# 3. Guarda configuración
print("Guardando configuración...")
try:
    odrv.save_configuration()
    print("Reiniciando ODrive...")
    odrv.reboot()
except Exception as e:
    if "DeviceLostException" in str(type(e)):
        print("ODrive reiniciado correctamente (se perdió la conexión, esperado).")
    else:
        raise e

print("Esperando que se reinicie...")
time.sleep(5)

print("Buscando ODrive...")
odrv = odrive.find_any()
axis = odrv.axis0

# Configura modo de control
print("Configurando trap_traj...")
axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
axis.controller.config.input_mode = INPUT_MODE_POS_FILTER

# Establece límites de velocidad y aceleración bajos para suavidad
axis.controller.config.vel_limit = 0.1
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Homing")
axis.controller.config.vel_limit = 0.1
axis.controller.input_pos = 0.0
time.sleep(1)
axis.controller.config.vel_limit = 10
# Ejecuta el movimiento
print("Iniciando movimiento lento con trap_traj...")
time.sleep(1)
def deg_to_rad(deg):
    return deg * math.pi / 180

waypoints_deg = [0, 5, -5, 10, -10, 20, -20, 45, -45, 90, -90, 0]
waypoints_rad = [deg_to_rad(deg) for deg in waypoints_deg]
for pos in waypoints_rad:
    axis.controller.input_pos = pos
    time.sleep(1)  # espera más que el tiempo de trayectoria para evitar solapamiento

print("Movimiento completado.")
axis.requested_state = AXIS_STATE_IDLE