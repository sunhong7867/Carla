# Python version of arbitration logic
# Inputs:
# - acceleration_by_acc: ACC module output (-10 to +10)
# - acceleration_by_aeb: AEB module output (-10 to 0)
# - steer_by_lfa: LFA module output (steering angle in degrees)
# - aeb_mode: one of ['NORMAL', 'ALERT', 'BRAKE']
# Output:
# - dict with throttle (0~1), brake (0~1), steer (-1~1)

MAX_THROTTLE_ACCEL = 10.0  # m/s^2
MAX_BRAKE_DECEL = -10.0    # m/s^2
MAX_STEER_ANGLE = 540.0    # degrees

class AEBMode:
    NORMAL = 'NORMAL'
    ALERT = 'ALERT'
    BRAKE = 'BRAKE'

def arbitration(accel_acc, decel_aeb, steer_lfa, aeb_mode):
    selected_accel = 0.0

    # 1. Choose longitudinal accel
    if aeb_mode == AEBMode.BRAKE:
        selected_accel = decel_aeb  # expected: -10 ~ 0
    else:
        selected_accel = accel_acc  # expected: -10 ~ +10

    # 2. throttle / brake command
    if selected_accel > 0:
        throttle = min(max(selected_accel / MAX_THROTTLE_ACCEL, 0.0), 1.0)
        brake = 0.0
    elif selected_accel < 0:
        brake = min(max(abs(selected_accel / MAX_BRAKE_DECEL), 0.0), 1.0)
        throttle = 0.0
    else:
        throttle = 0.0
        brake = 0.0

    # 3. steer normalization
    steer_ratio = steer_lfa / MAX_STEER_ANGLE
    steer = min(max(steer_ratio, -1.0), 1.0)

    return {
        'throttle': throttle,
        'brake': brake,
        'steer': steer
    }