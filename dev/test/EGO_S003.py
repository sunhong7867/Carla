# === Debugging Print for EGO_S003 ===
print("======================================")
print(f"[DEBUG][S0] time = {self.current_time_ms:.1f} ms")

# Step 1: GPS 무효 상태 확인 (지연 여부)
gps_dt = abs(self.current_time_ms - self.gps_data.timestamp)
print(f"[DEBUG][S1] gps_dt = {gps_dt:.1f} ms → {'FAIL' if gps_dt <= 50.0 else 'OK'}")

# Step 2: Kalman 필터 업데이트 비활성 여부 (직접 변수 없음 → Step 1으로 간접 판단)
# 추가 조건이 없으므로 생략 가능

# Step 3: 속도 추정 연속성 확인 (이전 추정값과 비교)
if hasattr(self, "prev_est_vel_x"):
    d_est_vel = abs(ego_data.velocity_x - self.prev_est_vel_x)
    print(f"[DEBUG][S2] ΔEstimatedVelocity = {d_est_vel:.3f} m/s → {'OK' if d_est_vel < 2.0 else 'FAIL'}")
else:
    print("[DEBUG][S2] ΔEstimatedVelocity = --- (초기 프레임)")
self.prev_est_vel_x = ego_data.velocity_x

# Step 4: Kalman 추정값과 Ground Truth 비교
gt_vel = self._actor.get_velocity()
gt_speed = math.hypot(gt_vel.x, gt_vel.y)
error_speed = abs(ego_data.velocity_x - gt_speed)
print(f"[DEBUG][S3] EstimatedVelocity = {ego_data.velocity_x:.2f} m/s, GroundTruthVelocity = {gt_speed:.2f} m/s, Error = {error_speed:.2f} m/s → {'OK' if error_speed <= 2.0 else 'FAIL'}")
