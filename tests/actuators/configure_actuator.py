import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import time
import json

with open("actuator_config.json", "r") as f:
    cfg = json.load(f)


# Conecta al ODrive
print("Buscando ODrive...")
print("=============================================================")
odrive.find_any()
print(odrive.find_any())
dump_errors(odrive.find_any(), clear=True)
odrv = odrive.find_any()
axis = odrv.axis0
axis.motor.config.current_lim = cfg["motor"]["current_lim"]
axis.motor.config.calibration_current = cfg["motor"]["calibration_current"]
axis.motor.config.pole_pairs = cfg["motor"]["pole_pairs"]
axis.motor.config.torque_constant = cfg["motor"]["torque_constant"]

axis.controller.config.vel_limit=cfg["controller"]
axis.controller.config.enable_torque_mode_vel_limit=10
axis.controller.config.pos_gain=20.0
axis.controller.config.vel_gain=0.16
axis.controller.config.vel_integrator_gain=0.32

axis.motor.motor_thermistor.config.enabled = cfg["thermistor"]["enabled"]
axis.motor.motor_thermistor.config.temp_limit_lower = cfg["thermistor"]["temp_limit_lower"]
axis.motor.motor_thermistor.config.temp_limit_upper = cfg["thermistor"]["temp_limit_upper"]

odrv.config.dc_bus_undervoltage_trip_level = cfg["dc_bus"]["undervoltage_trip_level"]
odrv.config.dc_bus_overvoltage_trip_level = cfg["dc_bus"]["overvoltage_trip_level"]
odrv.config.dc_max_positive_current = cfg["dc_bus"]["max_positive_current"]
odrv.config.dc_max_negative_current = cfg["dc_bus"]["max_negative_current"]

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
print("")
dump_errors(odrv, clear=True)
axis.motor.config.pre_calibrated=True
axis.encoder.config.pre_calibrated=True
current_pos = axis.encoder.pos_estimate
axis.encoder.config.index_offset = current_pos
print("")
print("Setting index_offset")
print(f" - phase_resistance = {axis.motor.config.phase_resistance}")
print(f" - phase_inductance = {axis.motor.config.phase_inductance}")
print(f" - index_offset = {current_pos}")

print("")
print("SAVING CONFIGURATION AND RESETING...")
try:
    odrv.save_configuration()
    print("Reiniciando ODrive...")
except Exception as e:
    if "DeviceLostException" in str(type(e)):
        print("    Actuator ready")
    else:
        raise e

