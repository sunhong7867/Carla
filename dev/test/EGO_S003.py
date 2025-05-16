elapsed_sec = int((self.current_time_ms - self.start_time_ms) // 1000)
gps_dt = abs(self.current_time_ms - self.gps_data.timestamp)
gt_vel = self._actor.get_velocity().x
vel_err = abs(gt_vel - ego_data.velocity_x)
imu_valid = abs(self.imu_data.accel_x) < 9.8
gps_valid = gps_dt <= 50.0

print(f"[STEP1][{elapsed_sec} sec] 정속 주행 확인 → Velocity = {ego_data.velocity_x:.2f} m/s")
print(f"[STEP2][{elapsed_sec} sec] GPS 수신 지연 = {gps_dt:.1f} ms, gps_update_enabled = {self.kf_state.gps_update_enabled}")
print(f"[STEP3][{elapsed_sec} sec] Filtered_Accel_X = {self.kf_state.prev_accel_x:.2f}, Filtered_Yaw_Rate = {self.kf_state.prev_yaw_rate:.2f}")
print(f"[STEP4][{elapsed_sec} sec] Ego_Velocity_X = {ego_data.velocity_x:.2f}, Ego_Acceleration_X = {ego_data.accel_x:.2f}")
print(f"[STEP5][{elapsed_sec} sec] GPS valid: {gps_valid}, IMU valid: {imu_valid}")
print(f"[STEP6][{elapsed_sec} sec] Ego_Velocity_X = {ego_data.velocity_x:.2f}, GroundTruth = {gt_vel:.2f}, Error = {vel_err:.2f}")
