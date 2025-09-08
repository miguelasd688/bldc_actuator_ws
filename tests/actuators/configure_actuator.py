import argparse
import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import time
import json


# ================================
# Argumentos CLI
# ================================
parser = argparse.ArgumentParser(description="Configura un actuador ODrive con parámetros desde JSON")
parser.add_argument("--node_id", type=int, default=0, help="Node ID para CAN (por defecto: 0)")
def str2bool(v: str) -> bool:
    if isinstance(v, bool):
        return v
    v = v.lower()
    if v in ("yes", "y", "true", "t", "1", "on"):
        return True
    if v in ("no", "n", "false", "f", "0", "off"):
        return False
    raise argparse.ArgumentTypeError("Valor booleano esperado (true/false)")

parser.add_argument("--r120", type=str2bool, nargs="?", const=True, default=True,
                    help="Enable built-in 120Ω terminator (true/false, por defecto: true)")
parser.add_argument("--config", type=str, default="actuator_config.json", help="Archivo JSON de configuración")
args = parser.parse_args()

# ================================
# Carga configuración JSON
# ================================
with open(args.config, "r") as f:
    cfg = json.load(f)

# Sobrescribir node_id si se pasa por argumento
if args.node_id is not None:
    cfg["can"]["node_id"] = args.node_id
if args.r120 is not None:
    cfg["can"]["enable_r120"] = args.r120

# ================================
# Conexión ODrive
# ================================
print("Buscando ODrive...")
print("=============================================================")
odrive.find_any()
print(odrive.find_any())
dump_errors(odrive.find_any(), clear=True)
odrv = odrive.find_any()
axis = odrv.axis0
dump_errors(odrv)
odrv.clear_errors()
axis.requested_state = AXIS_STATE_IDLE

axis.motor.config.current_lim = cfg["motor"]["current_lim"]
axis.motor.config.calibration_current = cfg["motor"]["calibration_current"]
axis.motor.config.pole_pairs = cfg["motor"]["pole_pairs"]
axis.motor.config.torque_constant = cfg["motor"]["torque_constant"]

axis.controller.config.vel_limit=cfg["controller"]["vel_limit"]
axis.controller.config.enable_torque_mode_vel_limit=cfg["controller"]["enable_torque_mode_vel_limit"]
axis.controller.config.pos_gain=cfg["controller"]["pos_gain"]
axis.controller.config.vel_gain=cfg["controller"]["vel_gain"]
axis.controller.config.vel_integrator_gain=cfg["controller"]["vel_integrator_gain"]

axis.motor.motor_thermistor.config.enabled = cfg["thermistor"]["enabled"]
axis.motor.motor_thermistor.config.temp_limit_lower = cfg["thermistor"]["temp_limit_lower"]
axis.motor.motor_thermistor.config.temp_limit_upper = cfg["thermistor"]["temp_limit_upper"]

odrv.config.dc_bus_undervoltage_trip_level = cfg["dc_bus"]["undervoltage_trip_level"]
odrv.config.dc_bus_overvoltage_trip_level = cfg["dc_bus"]["overvoltage_trip_level"]
odrv.config.dc_max_positive_current = cfg["dc_bus"]["max_positive_current"]
odrv.config.dc_max_negative_current = cfg["dc_bus"]["max_negative_current"]

axis.trap_traj.config.vel_limit = cfg["trap_traj"]["vel_limit"]
axis.trap_traj.config.accel_limit = cfg["trap_traj"]["accel_limit"]
axis.trap_traj.config.decel_limit = cfg["trap_traj"]["decel_limit"]
axis.controller.config.inertia = cfg["trap_traj"].get("inertia", 0.0)

odrv.config.enable_brake_resistor = cfg["brake"]["enable_brake_resistor"]
odrv.config.brake_resistance = cfg["brake"]["brake_resistance"]
odrv.config.max_regen_current = cfg["brake"]["max_regen_current"]
odrv.config.enable_dc_bus_overvoltage_ramp = cfg["brake"]["enable_dc_bus_overvoltage_ramp"]
odrv.config.dc_bus_overvoltage_ramp_start = cfg["brake"]["dc_bus_overvoltage_ramp_start"]
odrv.config.dc_bus_overvoltage_ramp_end = cfg["brake"]["dc_bus_overvoltage_ramp_end"]

odrv.config.enable_can_a = cfg["can"]["enable_can_a"]
axis.config.can.node_id = cfg["can"]["node_id"]
odrv.can.config.enable_r120 = cfg["can"]["enable_r120"]
odrv.can.config.baud_rate = cfg["can"]["baud_rate"]
print("")
print("=============================================================")
print("CONFIGURATION")
print("")
print("DC bus status:")
print(f"  - vbus_voltage = {odrv.vbus_voltage}")
print(f"  - dc_bus_undervoltage_trip_level = {odrv.config.dc_bus_undervoltage_trip_level}")
print(f"  - dc_bus_overvoltage_trip_level = {odrv.config.dc_bus_overvoltage_trip_level}")
print(f"  - dc_max_positive_current = {odrv.config.dc_max_positive_current}")
print(f"  - dc_max_negative_current =  {odrv.config.dc_max_negative_current}")  
print("")
print("Motor values:")
print(f"  - current_lim = {axis.motor.config.current_lim}")
print(f"  - calibration_current = {axis.motor.config.calibration_current}")
print(f"  - pole_pairs = {axis.motor.config.pole_pairs}")
print(f"  - torque_constant = {axis.motor.config.torque_constant}")
print("")
print(f"  - temp_limit_lower = {axis.motor.motor_thermistor.config.temp_limit_lower}")
print(f"  - temp_limit_upper = {axis.motor.motor_thermistor.config.temp_limit_upper}")
print(f"  - temperature = {axis.motor.motor_thermistor.temperature}")
print(f"  - board_temperature = {axis.motor.fet_thermistor.temperature}")
print("")
print("Controller values:")
print(f"  - vel_limit = {axis.controller.config.vel_limit}") 
print(f"  - enable_torque_mode_vel_limit = {axis.controller.config.enable_torque_mode_vel_limit}")
print("") 
print("PID settings:")
print(f"  - pos_gain = {axis.controller.config.pos_gain}")
print(f"  - vel_gain = {axis.controller.config.vel_gain}")
print(f"  - vel_integrator_gain = {axis.controller.config.vel_integrator_gain}")
print("")
print("CAN interface status:")
print(f"  - enable_can_a = {odrv.config.enable_can_a}")
print(f"  - can.node_id = {axis.config.can.node_id}")
print(f"  - can.enable_r120 = {odrv.can.config.enable_r120}")
print(f"  - can.baud_rate = {odrv.can.config.baud_rate}")
print("")
print("=============================================================")
print("Starting Initial Calibration")
print("")
axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
dump_errors(odrv, clear=True)
print("Doing motor calibration")
print("|----------------------------------------¬")
print('|', end='', flush=True)
for i in range(40):
    print('=', end='', flush=True)
    time.sleep(6/40)    
print("|")
print("")
axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION # AXIS_STATE_FULL_CALIBRATION_SEQUENCE
dump_errors(odrv, clear=True)
print("Doing encoder calibration")
print("|----------------------------------------¬")
print('|', end='', flush=True)
for i in range(40):
    print('=', end='', flush=True)
    time.sleep(80/40)    
print("|")
current_pos = axis.encoder.pos_estimate
axis.encoder.config.index_offset = current_pos
axis.motor.config.pre_calibrated=True
axis.encoder.config.pre_calibrated=True
print(f"Encoder lineal reiniciado. Posición anterior: {current_pos:.3f} vueltas")
print("")
print("Setting index_offset")
print(f" - phase_resistance = {axis.motor.config.phase_resistance}")
print(f" - phase_inductance = {axis.motor.config.phase_inductance}")
print(f" - index_offset = {current_pos}")

print("")
print("SAVING CONFIGURATION AND RESETING...")
print(f"{axis.encoder}")

try:
    odrv.save_configuration()
    print("Reiniciando ODrive...")
except Exception as e:
    if "DeviceLostException" in str(type(e)):
        print("    Actuator ready")
    else:
        raise e

