import numpy as np
<<<<<<< HEAD
from shared_types import TimeData, GPSData, IMUData, EgoData, EgoVehicleKFState
=======
from decision.shared_types import TimeData, GPSData, IMUData, EgoData, EgoVehicleKFState
>>>>>>> develop

# ───────────── 상수 정의 ─────────────
GPS_VALID_TIME_MS = 50.0
ACCEL_SPIKE_THRESH = 3.0
YAW_SPIKE_THRESH = 30.0
GPS_VEL_SPIKE_THRESH = 10.0
Q_PROCESS = 0.01
R_GPS = 0.1

# ───────────── 2x2 역행렬 ─────────────
def invert_2x2(S):
    det = S[0,0]*S[1,1] - S[0,1]*S[1,0]
    if abs(det) < 1e-6:
        return None
    inv_det = 1.0 / det
    return np.array([
        [ S[1,1], -S[0,1]],
        [-S[1,0],  S[0,0]]
    ]) * inv_det

# ───────────── 스파이크 제거 ─────────────
def check_spike(new, old, threshold):
    return abs(new - old) > threshold

# ───────────── 초기화 함수 ─────────────
def init_ego_vehicle_kf_state():
    state = EgoVehicleKFState()
    state.X = np.zeros(5, dtype=np.float32)
    state.P = np.eye(5, dtype=np.float32) * 100.0
    return state

# ───────────── 메인 추정 함수 ─────────────
def ego_vehicle_estimation(time_data: TimeData,
                           gps_data: GPSData,
                           imu_data: IMUData,
                           kf: EgoVehicleKFState) -> EgoData:

    ego = EgoData(
        velocity_x=0.0, velocity_y=0.0,
        accel_x=0.0, accel_y=0.0,
        heading=0.0, yaw_rate=0.0,
        position_x=0.0, position_y=0.0, position_z=0.0
    )

    gps_dt = abs(time_data.current_time - gps_data.timestamp)
    gps_enabled = gps_dt <= GPS_VALID_TIME_MS

    delta_t = time_data.current_time - kf.previous_update_time
    if delta_t <= 0.0:
        delta_t = 0.01
    kf.previous_update_time = time_data.current_time
    dt = delta_t / 1000.0  # ms → s

    # IMU & GPS 입력값
    ax = imu_data.accel_x
    ay = imu_data.accel_y
    yaw_rate = imu_data.yaw_rate
    gps_vx = gps_data.velocity_x
    gps_vy = gps_data.velocity_y

    if check_spike(ax, kf.prev_accel_x, ACCEL_SPIKE_THRESH): ax = kf.prev_accel_x
    if check_spike(ay, kf.prev_accel_y, ACCEL_SPIKE_THRESH): ay = kf.prev_accel_y
    if check_spike(yaw_rate, kf.prev_yaw_rate, YAW_SPIKE_THRESH): yaw_rate = kf.prev_yaw_rate
    if check_spike(gps_vx, kf.prev_gps_vel_x, GPS_VEL_SPIKE_THRESH) or \
       check_spike(gps_vy, kf.prev_gps_vel_y, GPS_VEL_SPIKE_THRESH):
        gps_enabled = False

    kf.prev_accel_x = ax
    kf.prev_accel_y = ay
    kf.prev_yaw_rate = yaw_rate
    if gps_enabled:
        kf.prev_gps_vel_x = gps_vx
        kf.prev_gps_vel_y = gps_vy

    # 상태 예측
    X = kf.X.copy()
    X_pred = np.zeros(5)
    X_pred[0] = X[0] + dt * X[2]
    X_pred[1] = X[1] + dt * X[3]
    X_pred[2] = X[2] + ax
    X_pred[3] = X[3] + ay
    X_pred[4] = X[4] + dt * yaw_rate

    # A 행렬
    A = np.eye(5)
    A[0,2] = dt
    A[1,3] = dt

    # Q 공분산
    Q = np.eye(5) * Q_PROCESS
    P_pred = A @ kf.P @ A.T + Q

    # GPS 보정 (vx, vy)
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
    ego.accel_x   = X_pred[2]
    ego.accel_y   = X_pred[3]
    ego.heading   = X_pred[4]
    ego.yaw_rate  = yaw_rate
    # ego.position_x/y/z는 항상 0

    return ego
