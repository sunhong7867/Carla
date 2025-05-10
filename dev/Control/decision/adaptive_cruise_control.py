# Python version of ACC module implementing:
# - acc_mode_selection
# - calculate_accel_for_distance_pid
# - calculate_accel_for_speed_pid
# - acc_output_selection

from enum import Enum

# ===== ENUMS =====
class ACCMode(Enum):
    SPEED = 0
    DISTANCE = 1
    STOP = 2

class ACCTargetStatus(Enum):
    MOVING = 0
    STOPPED = 1
    STATIONARY = 2
    ONCOMING = 3

class ACCTargetSituation(Enum):
    NORMAL = 0
    CUT_IN = 1
    CUT_OUT = 2

# ===== STATE =====
speed_integral = 0.0
speed_prev_error = 0.0
dist_integral = 0.0
dist_prev_error = 0.0
prev_time_distance = 0.0

# ===== FUNCTIONS =====
def acc_mode_selection(target_data, ego_data, lane_data):
    if target_data is None or ego_data is None or lane_data is None:
        return ACCMode.SPEED
    if target_data['id'] < 0:
        return ACCMode.SPEED

    dist = target_data['distance']
    status = target_data['status']
    situation = target_data['situation']

    if dist > 55:
        return ACCMode.SPEED
    elif dist < 45:
        if status == ACCTargetStatus.STOPPED and ego_data['velocity_x'] < 0.5:
            return ACCMode.STOP
        return ACCMode.DISTANCE
    else:
        if status == ACCTargetStatus.STOPPED and ego_data['velocity_x'] < 0.5:
            return ACCMode.STOP
        if situation == ACCTargetSituation.CUT_IN:
            return ACCMode.DISTANCE
        return ACCMode.SPEED

def calculate_accel_for_distance_pid(mode, target_data, ego_data, current_time):
    global dist_integral, dist_prev_error, prev_time_distance

    if target_data is None or ego_data is None:
        return 0.0
    if mode not in [ACCMode.DISTANCE, ACCMode.STOP]:
        return 0.0

    delta_time = (current_time - prev_time_distance) / 1000.0
    if delta_time <= 0.0:
        delta_time = 0.01
    prev_time_distance = current_time

    target_distance = 40.0
    error = target_data['distance'] - target_distance

    Kp, Ki, Kd = 0.4, 0.05, 0.1
    dist_integral += error * delta_time
    d_err = (error - dist_prev_error) / delta_time
    dist_prev_error = error

    accel = Kp * error + Ki * dist_integral + Kd * d_err
    accel = max(min(accel, 10.0), -10.0)

    if mode == ACCMode.STOP:
        if target_data['status'] == ACCTargetStatus.STOPPED and ego_data['velocity_x'] < 0.5:
            if target_data['velocity_x'] > 0.5:
                accel = 1.2  # restart
            else:
                accel = -3.0  # stay stopped
    return accel

def calculate_accel_for_speed_pid(ego_data, lane_data, delta_time):
    global speed_integral, speed_prev_error

    if ego_data is None or lane_data is None or delta_time <= 0.0:
        return 0.0

    target_speed = 22.22  # 80 km/h
    if lane_data['is_curved'] and target_speed > 15.0:
        target_speed = 15.0

    error = target_speed - ego_data['velocity_x']
    Kp, Ki, Kd = 0.5, 0.1, 0.05

    speed_integral += error * delta_time
    d_err = (error - speed_prev_error) / (delta_time + 1e-5)
    speed_prev_error = error

    accel = Kp * error + Ki * speed_integral + Kd * d_err
    return accel

def acc_output_selection(mode, accel_dist, accel_speed):
    if mode == ACCMode.SPEED:
        return accel_speed
    elif mode == ACCMode.DISTANCE:
        return accel_dist
    elif mode == ACCMode.STOP:
        return 0.0
    return 0.0
