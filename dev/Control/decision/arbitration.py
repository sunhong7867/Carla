# arbitration.py
from shared_types import AEBMode, VehicleControl

# 상수 정의
MAX_THROTTLE_ACCEL = 10.0  # [m/s^2]
MAX_BRAKE_DECEL = -10.0  # [m/s^2]
MAX_STEER_ANGLE = 540.0  # [deg]


# ===== FUNCTION: arbitration =====
def arbitration(accel_acc: float, decel_aeb: float, steer_lfa: float, aeb_mode: AEBMode) -> VehicleControl:
    """
    Arbitration logic to resolve final vehicle control output.

    Parameters:
    - accel_acc: ACC acceleration output (m/s^2)
    - decel_aeb: AEB braking deceleration (m/s^2)
    - steer_lfa: LFA steering angle (deg)
    - aeb_mode: current AEB mode (Enum)

    Returns:
    - VehicleControl object (throttle, brake, steer)
    """
    # 1. 종방향 가속도 선택
    if aeb_mode == AEBMode.BRAKE:
        selected_accel = decel_aeb
    else:
        selected_accel = accel_acc

    # 2. Throttle & Brake 계산
    if selected_accel > 0.0:
        throttle = min(selected_accel / MAX_THROTTLE_ACCEL, 1.0)
        brake = 0.0
    elif selected_accel < 0.0:
        brake = min(abs(selected_accel) / abs(MAX_BRAKE_DECEL), 1.0)
        throttle = 0.0
    else:
        throttle = 0.0
        brake = 0.0

    # 3. 조향 각도 정규화
    steer = max(min(steer_lfa / MAX_STEER_ANGLE, 1.0), -1.0)

    return VehicleControl(
        throttle=throttle,
        brake=brake,
        steer=steer
    )
