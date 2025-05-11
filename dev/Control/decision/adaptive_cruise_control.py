# adaptive_cruise_control.py
# ACC 기능 구현 모듈 (Python 버전)

from decision.shared_types import ACCMode, ObjectStatus, TargetSituation, LaneSelectOutput, EgoData, ACCTarget

# ===== PID 상태 변수 =====
speed_integral = 0.0
speed_prev_error = 0.0
dist_integral = 0.0
dist_prev_error = 0.0
prev_time_distance = 0.0

# ===== FUNCTION 1: ACC 모드 결정 =====
def acc_mode_selection(target_data: ACCTarget, ego_data: EgoData, lane_data: LaneSelectOutput) -> ACCMode:
    if target_data is None or ego_data is None or lane_data is None:
        return ACCMode.SPEED
    if target_data.object_id < 0:
        return ACCMode.SPEED

    dist = target_data.distance
    status = target_data.status
    situation = target_data.situation

    if dist > 55:
        return ACCMode.SPEED
    elif dist < 45:
        if status == ObjectStatus.STOPPED and ego_data.velocity_x < 0.5:
            return ACCMode.STOP
        return ACCMode.DISTANCE
    else:
        if status == ObjectStatus.STOPPED and ego_data.velocity_x < 0.5:
            return ACCMode.STOP
        if situation == TargetSituation.CUTIN:
            return ACCMode.DISTANCE
        return ACCMode.SPEED

# ===== FUNCTION 2: 거리 기반 PID 제어 =====
def calculate_accel_for_distance_pid(mode: ACCMode, target_data: ACCTarget, ego_data: EgoData, current_time: float) -> float:
    global dist_integral, dist_prev_error, prev_time_distance

    if mode not in [ACCMode.DISTANCE, ACCMode.STOP]:
        return 0.0
    if target_data is None or ego_data is None:
        return 0.0

    delta_time = (current_time - prev_time_distance) / 1000.0
    if delta_time <= 0.0:
        delta_time = 0.01
    prev_time_distance = current_time

    target_distance = 40.0
    error = target_data.distance - target_distance

    kp, ki, kd = 0.4, 0.05, 0.1
    dist_integral += error * delta_time
    d_err = (error - dist_prev_error) / delta_time
    dist_prev_error = error

    accel = kp * error + ki * dist_integral + kd * d_err
    accel = max(min(accel, 10.0), -10.0)

    if mode == ACCMode.STOP:
        if target_data.status == ObjectStatus.STOPPED and ego_data.velocity_x < 0.5:
            if target_data.velocity_x > 0.5:
                accel = 1.2  # 재출발
            else:
                accel = -3.0  # 정지 유지
    return accel

# ===== FUNCTION 3: 속도 기반 PID 제어 =====
def calculate_accel_for_speed_pid(ego_data: EgoData, lane_data: LaneSelectOutput, delta_time: float) -> float:
    global speed_integral, speed_prev_error

    if ego_data is None or lane_data is None or delta_time <= 0.0:
        return 0.0

    target_speed = 22.22  # 80 km/h
    if lane_data.is_curved_lane:
        target_speed = min(target_speed, 15.0)

    error = target_speed - ego_data.velocity_x
    kp, ki, kd = 0.5, 0.1, 0.05

    speed_integral += error * delta_time
    d_err = (error - speed_prev_error) / (delta_time + 1e-5)
    speed_prev_error = error

    accel = kp * error + ki * speed_integral + kd * d_err
    return max(min(accel, 10.0), -10.0)

# ===== FUNCTION 4: 최종 출력 가속도 선택 =====
def acc_output_selection(mode: ACCMode, accel_dist: float, accel_speed: float) -> float:
    if mode == ACCMode.SPEED:
        return accel_speed
    elif mode == ACCMode.DISTANCE:
        return accel_dist
    elif mode == ACCMode.STOP:
        return 0.0
    else:
        print("[ACC] Warning: Unknown mode")
        return 0.0