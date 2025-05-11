# lane_following_assist.py
# Python version of LFA module implementing:
# - lfa_mode_selection
# - calculate_steer_in_low_speed_pid
# - calculate_steer_in_high_speed_stanley
# - lfa_output_selection

from decision.shared_types import EgoData, LaneSelectOutput, LFAMode, LFA_LOW_SPEED_THRESHOLD, LFA_MAX_STEERING_ANGLE
import math

# ===== PID / Stanley State =====
pid_integral = 0.0
pid_prev_error = 0.0
kp, ki, kd = 0.1, 0.01, 0.005
stanley_gain = 1.0

MIN_VEL = 0.1

# ===== 1. LFA 모드 선택 =====
def lfa_mode_selection(ego: EgoData) -> LFAMode:
    if ego is None or not math.isfinite(ego.velocity_x):
        return LFAMode.LOW_SPEED
    return LFAMode.LOW_SPEED if ego.velocity_x < LFA_LOW_SPEED_THRESHOLD else LFAMode.HIGH_SPEED

# ===== 2. 저속 PID 기반 조향 계산 =====
def calculate_steer_in_low_speed_pid(lane: LaneSelectOutput, dt: float) -> float:
    global pid_integral, pid_prev_error
    if lane is None or dt <= 0:
        return 0.0

    hdg_err = lane.heading_error
    off_err = lane.lane_offset

    if math.isnan(hdg_err): return 0.0
    if math.isnan(off_err): return float('nan')

    err = hdg_err + off_err
    if math.isinf(err):
        pid_integral = 0.0
        pid_prev_error = 0.0
        return math.copysign(LFA_MAX_STEERING_ANGLE, err)

    if abs(hdg_err) > 180.0 or abs(off_err) > 2.0:
        return math.copysign(LFA_MAX_STEERING_ANGLE, err)

    if abs(hdg_err) == 180.0 and abs(off_err) == 2.0:
        return math.copysign(LFA_MAX_STEERING_ANGLE, err)

    pid_integral += err * dt
    if math.isinf(pid_integral) or abs(pid_integral) > 1e5:
        sign = math.copysign(1.0, pid_integral)
        pid_integral = 0.0
        pid_prev_error = 0.0
        return 0.0 if abs(err) < 1e-6 else sign * LFA_MAX_STEERING_ANGLE

    d_err = (err - pid_prev_error) / (dt + 1e-6) if abs(err) > 1e-6 else 0.0
    pid_prev_error = err

    out = kp * err + ki * pid_integral + kd * d_err
    if abs(ki) < 1e-9 and abs(kd) < 1e-9 and abs(err) > 1e-9:
        out += 1e-6 if err > 0 else -1e-6

    return max(min(out, LFA_MAX_STEERING_ANGLE), -LFA_MAX_STEERING_ANGLE)

# ===== 3. 고속 Stanley 제어 기반 조향 계산 =====
def calculate_steer_in_high_speed_stanley(ego: EgoData, lane: LaneSelectOutput) -> float:
    if ego is None or lane is None:
        return 0.0

    vx = ego.velocity_x
    hdg_err = lane.heading_error
    cte = lane.lane_offset

    if any(map(math.isnan, [vx, hdg_err, cte])):
        return 0.0
    if math.isinf(hdg_err):
        return math.copysign(LFA_MAX_STEERING_ANGLE, hdg_err)
    if abs(hdg_err) >= 180.0 and abs(cte) >= 2.0:
        return math.copysign(LFA_MAX_STEERING_ANGLE, hdg_err + cte)

    if vx < MIN_VEL:
        vx = MIN_VEL

    offset_rad = math.atan((stanley_gain * cte) / vx)
    offset_deg = math.degrees(offset_rad)
    steer = hdg_err + offset_deg

    return max(min(steer, LFA_MAX_STEERING_ANGLE), -LFA_MAX_STEERING_ANGLE)

# ===== 4. 조향 최종 출력 선택 =====
def lfa_output_selection(mode: LFAMode,
                         steer_pid: float,
                         steer_stanley: float,
                         lane: LaneSelectOutput,
                         ego: EgoData) -> float:
    if not lane or not ego:
        return 0.0

    steer_out = steer_pid if mode == LFAMode.LOW_SPEED else steer_stanley

    if lane.is_changing_lane:
        steer_out *= 0.2
    if not lane.is_within_lane:
        steer_out *= 1.5
    if lane.is_curved_lane:
        curve_gain = 1.2
        if ego.yaw_rate >= 30.0:
            curve_gain = 0.8
        steer_out *= curve_gain

    return max(min(steer_out, LFA_MAX_STEERING_ANGLE), -LFA_MAX_STEERING_ANGLE)
