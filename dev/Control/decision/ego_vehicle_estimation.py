import numpy as np
from dataclasses import dataclass

# ==============================
# 데이터 구조 정의 (adas_shared.h 기반)
# ==============================
@dataclass
class TimeData:
    current_time: float  # [ms]

@dataclass
class GPSData:
    velocity_x: float  # [m/s]
    velocity_y: float  # [m/s]
    timestamp: float   # [ms]

@dataclass
class IMUData:
    accel_x: float  # [m/s^2]
    accel_y: float  # [m/s^2]
    yaw_rate: float # [deg/s]

@dataclass
class EgoData:
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    acceleration_x: float = 0.0
    acceleration_y: float = 0.0
    heading: float = 0.0
    position_x: float = 0.0
    position_y: float = 0.0
    position_z: float = 0.0

@dataclass
class EgoVehicleKFState:
    last_gps_velocity_x: float = 0.0
    last_gps_velocity_y: float = 0.0
    last_gps_timestamp: float = 0.0

    previous_update_time: float = 0.0
    prev_accel_x: float = 0.0
    prev_accel_y: float = 0.0
    prev_yaw_rate: float = 0.0
    prev_gps_vel_x: float = 0.0
    prev_gps_vel_y: float = 0.0

    gps_update_enabled: bool = False

    X: np.ndarray = np.zeros(5)         # 상태벡터 [vx, vy, ax, ay, heading]
    P: np.ndarray = np.eye(5) * 100.0   # 공분산 행렬

# ==============================
# 상수 정의
# ==============================
GPS_VALID_TIME_MS = 50.0
ACCEL_SPIKE_THRESH = 3.0
YAW_SPIKE_THRESH = 30.0
GPS_VEL_SPIKE_THRESH = 10.0
Q_PROCESS = 0.01
R_GPS = 0.1

# ==============================
# 유틸리티 함수
# ==============================
def invert_2x2(S):
    det = S[0,0]*S[1,1] - S[0,1]*S[1,0]
    if abs(det) < 1e-6:
        return None
    inv_det = 1.0 / det
    return np.array([
        [ S[1,1], -S[0,1]],
        [-S[1,0],  S[0,0]]
    ]) * inv_det

def check_spike(new, old, thresh):
    return abs(new - old) > thresh

# ==============================
# 초기화 함수
# ==============================
def init_ego_vehicle_kf_state():
    return EgoVehicleKFState(
        X=np.zeros(5),
        P=np.eye(5) * 100.0
    )

# ==============================
# 메인 추정 함수
# ==============================
def ego_vehicle_estimation(time_data: TimeData, gps_data: GPSData, imu_data: IMUData, kf: EgoVehicleKFState) -> EgoData:
    ego = EgoData()
    ego.position_x = ego.position_y = ego.position_z = 0.0

    gps_dt = abs(time_data.current_time - gps_data.timestamp)
    gps_enabled = gps_dt <= GPS_VALID_TIME_MS

    delta_t = time_data.current_time - kf.previous_update_time
    delta_t = max(delta_t, 0.01)
    kf.previous_update_time = time_data.current_time

    ax = imu_data.accel_x
    ay = imu_data.accel_y
    yaw = imu_data.yaw_rate

    if check_spike(ax, kf.prev_accel_x, ACCEL_SPIKE_THRESH): ax = kf.prev_accel_x
    if check_spike(ay, kf.prev_accel_y, ACCEL_SPIKE_THRESH): ay = kf.prev_accel_y
    if check_spike(yaw, kf.prev_yaw_rate, YAW_SPIKE_THRESH): yaw = kf.prev_yaw_rate

    gps_vx = gps_data.velocity_x
    gps_vy = gps_data.velocity_y

    if check_spike(gps_vx, kf.prev_gps_vel_x, GPS_VEL_SPIKE_THRESH) or \
       check_spike(gps_vy, kf.prev_gps_vel_y, GPS_VEL_SPIKE_THRESH):
        gps_enabled = False

    kf.prev_accel_x = ax
    kf.prev_accel_y = ay
    kf.prev_yaw_rate = yaw
    if gps_enabled:
        kf.prev_gps_vel_x = gps_vx
        kf.prev_gps_vel_y = gps_vy

    # 상태 예측
    X = kf.X.copy()
    dt = delta_t / 1000.0
    X_pred = np.array([
        X[0] + dt * X[2],
        X[1] + dt * X[3],
        X[2] + ax,
        X[3] + ay,
        X[4] + dt * yaw
    ])

    A = np.eye(5)
    A[0,2] = dt
    A[1,3] = dt
    A[4,4] = 1.0

    Q = np.eye(5) * Q_PROCESS
    P_pred = A @ kf.P @ A.T + Q

    # 보정
    if gps_enabled:
        H = np.zeros((2,5))
        H[0,0] = 1.0
        H[1,1] = 1.0
        z = np.array([gps_vx, gps_vy])
        z_pred = X_pred[:2]
        y = z - z_pred

        S = H @ P_pred @ H.T + np.eye(2) * R_GPS
        S_inv = invert_2x2(S)
        if S_inv is not None:
            K = P_pred @ H.T @ S_inv
            X_pred = X_pred + K @ y
            kf.P = (np.eye(5) - K @ H) @ P_pred
        else:
            kf.P = P_pred
    else:
        kf.P = P_pred

    kf.X = X_pred

    ego.velocity_x = X_pred[0]
    ego.velocity_y = X_pred[1]
    ego.acceleration_x = X_pred[2]
    ego.acceleration_y = X_pred[3]
    ego.heading = X_pred[4]

    return ego
