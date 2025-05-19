=== 센서 무효화 시뮬레이션(13~15초) ===
if 13.0 <= elapsed_sec < 15.0:
    gps_data.timestamp -= 100.0
    self.imu_data = IMUData(
        accel_x=99.9,
        accel_y=99.9,
        yaw_rate=999.0
    )

self.log_data.append({
    "time": round((self.time_data.current_time - self.sim_start_time) / 1000.0, 2),

    # Step 1: Ego 속도 추정
    "ego_velocity_x": round(ego.velocity_x, 2),
    "ego_acceleration_x": round(ego.accel_x, 2),

    # Step 2: GPS 무효화 판단
    "gps_dt": round(abs(self.time_data.current_time - self.gps_sensor_time), 2),
    "gps_update_enabled": gps_update_enabled,
    "raw_gps_velocity_x": round(gps_data.velocity_x, 2),
    "raw_gps_velocity_y": round(gps_data.velocity_y, 2),

    # Step 2: IMU 무효화 판단
    "filtered_accel_x": round(self.imu_data.accel_x, 2),
    "filtered_yaw_rate": round(self.imu_data.yaw_rate, 2),
    "raw_imu_accel_x": round(self.imu_data.accel_x, 2),
    "raw_imu_accel_y": round(self.imu_data.accel_y, 2),
    "raw_imu_yaw_rate": round(self.imu_data.yaw_rate, 2),

    # Step 3, 4: Ground Truth 비교용
    "gt_velocity": round(math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2), 2),
    "gt_acceleration": round(self._actor.get_acceleration().x, 2)
})
