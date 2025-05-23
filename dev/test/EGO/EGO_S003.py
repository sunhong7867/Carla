# === Step 평가용 변수 계산 ===

# 센서 주기
imu_dt = self.imu_intervals[-1] if self.imu_intervals else 0.0
gps_interval = self.gps_intervals[-1] if self.gps_intervals else 0.0
gps_dt = abs(self.time_data.current_time - self.gps_sensor_time)

# Kalman 추정 값
ego_velocity_x = ego_est.velocity_x
ego_accel_x = ego_est.accel_x

# Carla Ground Truth 값
gt_velocity = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
gt_accel = self._actor.get_acceleration().x

# Kalman 공분산 항
kf_p0 = self.kf_state.P[0][0]

# === 로그 저장 ===
self.log_data.append({
    # 시간
    "time": round((self.time_data.current_time - self.sim_start_time) / 1000.0, 2),

    # Step 1: 센서 수신 주기 및 Kalman 동기화
    "imu_interval": round(imu_dt, 2),
    "gps_interval": round(gps_interval, 2),
    "gps_dt": round(gps_dt, 2),

    # Step 2: Kalman 속도 / 가속도 정확도
    "ego_velocity_kf": round(ego_velocity_x, 2),
    "ego_velocity_gt": round(gt_velocity, 2),
    "velocity_error": round(abs(gt_velocity - ego_velocity_x), 2),

    "ego_accel_kf": round(ego_accel_x, 2),
    "ego_accel_gt": round(gt_accel, 2),
    "accel_error": round(abs(gt_accel - ego_accel_x), 2),

    # Step 3: Kalman 공분산 수렴도
    "kf_P0": round(kf_p0, 5)
})
