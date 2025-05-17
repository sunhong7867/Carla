# STEP 1: IMU 및 GPS 수신 간격 분석
if len(self.imu_intervals) >= 5:
    avg_imu_interval = sum(self.imu_intervals[-5:]) / 5.0
    imu_valid = 5.0 <= avg_imu_interval <= 15.0
    print(f"[STEP1][{elapsed_sec:.2f}s] IMU Interval = {avg_imu_interval:.1f} ms, Valid = {imu_valid}")

if len(self.gps_intervals) >= 3:
    avg_gps_interval = sum(self.gps_intervals[-3:]) / 3.0
    gps_valid = 95.0 <= avg_gps_interval <= 105.0
    print(f"[STEP1][{elapsed_sec:.2f}s] GPS Interval = {avg_gps_interval:.1f} ms, Valid = {gps_valid}")

# STEP 2: GPS 수신 시간 오차 확인
gps_dt = abs(timestamp - self.last_gps_time)
print(f"[STEP2][{elapsed_sec:.2f}s] gps_dt = {gps_dt:.1f} ms, gps_update_enabled = {gps_update_enabled}")


# STEP 3: Kalman 추정 속도 vs Ground Truth 비교
gt_vel = self._actor.get_velocity()
gt_speed = math.sqrt(gt_vel.x ** 2 + gt_vel.y ** 2 + gt_vel.z ** 2)
velocity_error = abs(ego.velocity_x - gt_speed)
print(f"[STEP3][{elapsed_sec:.2f}s] Ego_Speed = {ego.velocity_x:.2f} m/s, "
      f"Carla_Speed = {gt_speed:.2f} m/s, Error = {velocity_error:.2f} m/s")

# STEP 4: Object 상대 좌표 출력
all_vehicles = self.world.get_actors().filter("vehicle.*")
ego_transform = self._actor.get_transform()
ego_loc = ego_transform.location
ego_yaw = math.radians(ego_transform.rotation.yaw)

cos_yaw = math.cos(-ego_yaw)
sin_yaw = math.sin(-ego_yaw)

for veh in all_vehicles:
    if veh.id != self.ego_id:
        loc = veh.get_location()
        dx = loc.x - ego_loc.x
        dy = loc.y - ego_loc.y
        rel_x = dx * cos_yaw - dy * sin_yaw
        rel_y = dx * sin_yaw + dy * cos_yaw

        distance = math.sqrt(rel_x ** 2 + rel_y ** 2)
        abs_distance = math.sqrt(dx ** 2 + dy ** 2 + (loc.z - ego_loc.z) ** 2)

        print(f"[STEP4][{elapsed_sec:.2f}s] Object_ID={veh.id}, Rel_Pos=({rel_x:.2f}, {rel_y:.2f}), "
              f"Distance={distance:.2f} m, Carla_Distance={abs_distance:.2f} m")
