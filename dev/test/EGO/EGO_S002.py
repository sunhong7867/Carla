# === 로그 저장 ===
self.log_data.append({
    "time": round((self.time_data.current_time - self.sim_start_time) / 1000.0, 2),

    # STEP 1: 센서 수신 간격 (ms 단위, 최근 값)
    "imu_interval": round(self.imu_intervals[-1], 2) if len(self.imu_intervals) > 0 else 0.0,
    "gps_interval": round(self.gps_intervals[-1], 2) if len(self.gps_intervals) > 0 else 0.0,

    # STEP 2: GPS 시간 오차
    "gps_timestamp": round(self.gps_sensor_time, 2),
    "loop_time": round(self.time_data.current_time, 2),
    "gps_dt": round(abs(self.time_data.current_time - self.gps_sensor_time), 2),

    # STEP 3: 추정 속도 vs 실제 속도
    "ego_velocity_x": round(ego.velocity_x, 2),  # 칼만 필터 추정 속도
    "gt_velocity": round(math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2), 2)
})
