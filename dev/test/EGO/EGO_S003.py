gps_dt = abs(self.current_time_ms - self.gps_data.timestamp)
gps_valid = gps_dt > 200.0 and not self.kf_state.gps_update_enabled
imu_enabled = not self.imu_data.disabled
filtered_accel_x = self.kf_state.filtered_accel_x
filtered_yaw = self.kf_state.filtered_yaw_rate
gt_velocity = self._actor.get_velocity().x
prev_vel = self.kf_state.prev_velocity_x
prev_accel = self.kf_state.prev_acceleration_x
delta_vel = abs(ego_data.velocity_x - prev_vel)
delta_accel = abs(ego_data.acceleration_x - prev_accel)
error_vel = abs(ego_data.velocity_x - gt_velocity)

# Step 1: 정속 주행 확인
if 0.0 <= t_sec < 10.0:
    print(f"[STEP1][{t_sec:.2f}s] Ego_Velocity_X = {ego_data.velocity_x:.2f} m/s")
elif 10.0 <= t_sec < 13.0:
    print(f"[STEP1][{t_sec:.2f}s] 평균속도 판단용 Ego_Velocity_X = {ego_data.velocity_x:.2f} m/s")

# Step 2: 센서 무효 인식
if 13.0 <= t_sec < 15.0:
    print(f"[STEP2][{t_sec:.2f}s] gps_dt = {gps_dt:.1f} ms, gps_update_enabled = {self.kf_state.gps_update_enabled} → {'Pass' if gps_valid else 'Fail'}")
    print(f"[STEP2][{t_sec:.2f}s] imu_enabled = {imu_enabled} → {'Pass' if not imu_enabled else 'Fail'}")
    print(f"[STEP2][{t_sec:.2f}s] Filtered_Accel_X = {filtered_accel_x:.2f}, Filtered_Yaw_Rate = {filtered_yaw:.2f}")

# Step 3: 상태 추정 유지
if 13.0 <= t_sec < 15.0:
    print(f"[STEP3][{t_sec:.2f}s] ΔEgo_Velocity_X = {delta_vel:.3f} m/s → {'Pass' if delta_vel <= 1.0 else 'Fail'}")
    print(f"[STEP3][{t_sec:.2f}s] ΔEgo_Acceleration_X = {delta_accel:.3f} m/s² → {'Pass' if delta_accel <= 1.0 else 'Fail'}")

# Step 4: 센서 복구 후 수렴
if 15.0 <= t_sec <= 20.0:
    print(f"[STEP4][{t_sec:.2f}s] Ego_Velocity_X = {ego_data.velocity_x:.2f}, GroundTruth = {gt_velocity:.2f}, Error = {error_vel:.2f} → {'Pass' if error_vel <= 1.0 else 'Fail'}")
