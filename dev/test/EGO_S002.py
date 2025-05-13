# Step 0: 센서 수신 확인
print("======================================")
print(f"[EGO] Velocity: {ego_data.velocity_x:.2f} m/s | Accel: {ego_data.accel_x:.2f} m/s² | Heading: {ego_data.heading:.2f}°")
print(f"[DEBUG][S0] time={self.current_time_ms:.1f}ms gps_ts = {self.gps_data.timestamp:.1f} ms, imu = ({self.imu_data.accel_x:.2f}, {self.imu_data.accel_y:.2f})")

# Step 1: 추정 속도 vs 목표 속도
speed_error_target = abs(ego_data.velocity_x - 15.0)
print(f"[DEBUG][S1] Velocity = {ego_data.velocity_x:.2f} m/s, Target = 15.00 m/s, Error = {speed_error_target:.2f} m/s → {'OK' if speed_error_target <= 1.0 else 'FAIL'}")

# Step 2: GPS 수신 간격 확인
gps_dt = abs(self.current_time_ms - self.gps_data.timestamp)
print(f"[DEBUG][S2] gps_dt = {gps_dt:.1f} ms → {'OK' if gps_dt <= 50.0 else 'FAIL'}")

# Step 3: Ground Truth와 비교
gt_vel = self._actor.get_velocity()
gt_speed = math.hypot(gt_vel.x, gt_vel.y)
speed_error_gt = abs(ego_data.velocity_x - gt_speed)
print(f"[DEBUG][S3] EstimatedVelocity = {ego_data.velocity_x:.2f} m/s, GroundTruthVelocity = {gt_speed:.2f} m/s, Error = {speed_error_gt:.2f} m/s → {'OK' if speed_error_gt <= 1.0 else 'FAIL'}")
