import time
import math
import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import json

# ========== utilidades ==========
TAU = 2.0 * math.pi  # 2π

def rad_to_turns(rad_out: float, gear_ratio: float) -> float:
    """rad del eje -> turns del motor"""
    return (rad_out / TAU) * gear_ratio

def turns_to_rad(turns_motor: float, gear_ratio: float) -> float:
    """turns del motor -> rad del eje"""
    return (turns_motor / gear_ratio) * TAU

def get_pos_rad(axis, gear_ratio: float) -> float:
    return turns_to_rad(axis.encoder.pos_estimate, gear_ratio)

def set_input_pos_rad(axis, target_rad_out: float, gear_ratio: float,
                      vel_ff_rad_s: float = 0.0, torque_ff_motor_Nm: float = 0.0):
    axis.controller.input_pos = rad_to_turns(target_rad_out, gear_ratio)
    if vel_ff_rad_s:
        axis.controller.input_vel = rad_to_turns(vel_ff_rad_s, gear_ratio)
    if torque_ff_motor_Nm:
        axis.controller.input_torque = torque_ff_motor_Nm  # motor-side

def wait_until_reached(axis, target_rad_out: float, gear_ratio: float,
                       tol_rad: float = math.radians(0.5),
                       settle_time: float = 0.05,
                       timeout: float = 5.0,
                       use_traj_done: bool = True):
    t0 = time.monotonic()
    ok_since = None
    target_turns_motor = rad_to_turns(target_rad_out, gear_ratio)
    while True:
        now = time.monotonic()
        if now - t0 > timeout:
            err_rad = turns_to_rad(axis.controller.input_pos - axis.encoder.pos_estimate, gear_ratio)
            raise TimeoutError(f"No alcanzó el objetivo en {timeout:.2f}s (err={abs(err_rad):.4f} rad)")

        traj_done = False
        try:
            traj_done = bool(axis.controller.trajectory_done) if use_traj_done else False
        except Exception:
            pass

        err_rad = turns_to_rad(target_turns_motor - axis.encoder.pos_estimate, gear_ratio)
        within = abs(err_rad) <= tol_rad or traj_done
        if within:
            ok_since = ok_since or now
            if (now - ok_since) >= settle_time:
                return
        else:
            ok_since = None
        time.sleep(0.002)

# ========== carga config ==========
with open("actuator_config.json", "r") as f:
    cfg = json.load(f)

POS_TOL_RAD = cfg.get("controller", {}).get("pos_tolerance_rad", math.radians(0.5))
TIMEOUT_S   = cfg.get("controller", {}).get("move_timeout_s", 10.0)

# ========== conexión ==========
print("Buscando ODrive...")
odrv = odrive.find_any()
axis = odrv.axis0

print("Errores actuales:")
dump_errors(odrv)
odrv.clear_errors()

# Descubre gear_ratio: ODrive -> JSON -> 1.0
try:
    GEAR = float(axis.motor.config.gear_ratio)
    if GEAR <= 0:
        raise ValueError
except Exception:
    GEAR = float(cfg.get("motor", {}).get("gear_ratio", 1.0))

print(f"Gear ratio usado = {GEAR:.3f} (motor:ejE)")

# Estado IDLE para configurar
axis.requested_state = AXIS_STATE_IDLE

# Control de posición con TRAP_TRAJ (valores en rad/s del EJE)
axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
axis.controller.config.input_mode   = INPUT_MODE_TRAP_TRAJ

traj_vel_limit_rad_s = cfg["controller"].get("vel_limit", 2.0)
accel_limit_rad_s2   = cfg["controller"].get("accel_limit", 0.2)
decel_limit_rad_s2   = cfg["controller"].get("decel_limit", 0.2)

axis.trap_traj.config.vel_limit   = rad_to_turns(traj_vel_limit_rad_s, GEAR)
axis.trap_traj.config.accel_limit = rad_to_turns(accel_limit_rad_s2, GEAR)
axis.trap_traj.config.decel_limit = rad_to_turns(decel_limit_rad_s2, GEAR)

# Ganancias PID desde tu JSON
axis.controller.config.pos_gain            = cfg["controller"].get("pos_gain", 20.0)
axis.controller.config.vel_gain            = cfg["controller"].get("vel_gain", 0.1)
axis.controller.config.vel_integrator_gain = cfg["controller"].get("vel_integrator_gain", 5.0)

# Límite de velocidad global del controlador (rev/s, así que convertimos algo razonable en rad/s eje)
axis.controller.config.enable_vel_limit = True
axis.controller.config.vel_limit = max(axis.trap_traj.config.vel_limit,
                                       rad_to_turns(5, GEAR))  # 2 rad/s eje mínimo

# Entrar en CLOSED_LOOP
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# ========== movimientos ==========
def move_and_hold(axis, target_deg, tol_rad=POS_TOL_RAD, timeout=TIMEOUT_S):
    target_rad = math.radians(target_deg)
    print(f"→ Moviendo a {target_deg:.1f}° ({target_rad:.3f} rad). Actual: {get_pos_rad(axis, GEAR):.3f} rad")
    set_input_pos_rad(axis, target_rad, GEAR)
    wait_until_reached(axis, target_rad, GEAR, tol_rad=tol_rad, timeout=timeout)
    pos_rad = get_pos_rad(axis, GEAR)
    err = pos_rad - target_rad
    print(f"   Llegado. pos={pos_rad:.4f} rad  err={err:.4f} rad")

try:
    # Opcional: fija cero de usuario aquí (si ya estás mecánicamente en tu “cero”)
    # axis.encoder.set_linear_count(0)

    for deg in [0, 5, -5, 10, -10, 20, -20, 45, -45, 90, -90, 0]:
        move_and_hold(axis, deg)

    print("Movimiento completado.")
finally:
    axis.requested_state = AXIS_STATE_IDLE
