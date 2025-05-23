# === Step 평가용 변수 계산 ===
imu_dt = self.imu_intervals[-1] if len(self.imu_intervals) > 0 else None
ego_velocity_x = ego_est.velocity_x  # Kalman 추정 속도
gt_velocity = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)  # Ground Truth 속도

# === 로그 저장 ===
self.log_data.append({
    "time": round((self.time_data.current_time - self.sim_start_time) / 1000.0, 2),

    # Step 1: 센서 수신 주기 확인
    "imu_interval": round(self.imu_intervals[-1], 2) if len(self.imu_intervals) > 0 else 0.0,
    "gps_interval": round(self.gps_intervals[-1], 2) if len(self.gps_intervals) > 0 else 0.0,

    # Step 2: GPS 동기화 지연 확인
    "ego_velocity_kf": round(ego_velocity_x, 2),
    "ego_velocity_gt": round(gt_velocity, 2),

    # Step 3: Kalman 추정 정확도 평가
    "velocity_error": round(abs(gt_velocity - ego_velocity_x), 2)
})
