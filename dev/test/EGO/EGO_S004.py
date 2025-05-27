self.log_data.append({
    # [공통] 시간 기준
    "time": round((self.time_data.current_time - self.sim_start_time) / 1000.0, 2),

    # [Step 1] 센서 수신 주기 및 동기화 상태
    "imu_interval": round(imu_dt, 2),
    "gps_interval": round(gps_interval, 2),
    "gps_dt": round(gps_dt, 2),
    "gps_update_enabled": self.kf_state.gps_update_enabled,
    "imu_timestamp": round(self.last_imu_time, 2),
    "gps_timestamp": round(self.gps_sensor_time, 2),

    # [Step 2] 속도/가속도 추정 정확도 (Kalman vs GT)
    "ego_velocity_kf": round(ego_est.velocity_x, 2),
    "ego_velocity_gt": round(gt_velocity, 2),
    "velocity_error": round(abs(gt_velocity - ego_est.velocity_x), 2),

    "ego_accel_kf": round(ego_est.accel_x, 2),
    "ego_accel_gt": round(gt_accel, 2),
    "accel_error": round(abs(gt_accel - ego_est.accel_x), 2),

    # [Step 3] IMU 스파이크 보정 안정성 및 상태 유지
    "Filtered_Accel_X": round(self.imu_data.accel_x, 3),

})
