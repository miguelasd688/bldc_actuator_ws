import time
import math
import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import json

with open("actuator_config.json", "r") as f:
    cfg = json.load(f)

print("Buscando ODrive...")
odrv = odrive.find_any()
axis = odrv.axis0
axis.requested_state = AXIS_STATE_IDLE
# Configura modo de control
print("Configurando trap_traj...")

# Establece límites de velocidad y aceleración bajos para suavidad
axis.controller.config.vel_limit = 0.5


axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
axis.controller.config.input_mode = INPUT_MODE_POS_FILTER
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Homing")
axis.controller.input_pos = 0.0
time.sleep(5)
axis.requested_state = AXIS_STATE_IDLE

axis.controller.config.vel_limit = 10

# Ejecuta el movimiento
print("Iniciando movimiento lento con trap_traj...")
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
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
try:
    odrv.reboot()
    print("Reiniciando ODrive...")
except Exception as e:
    if "DeviceLostException" in str(type(e)):
        print("ODrive reiniciado correctamente (se perdió la conexión, esperado).")
    else:
        raise e