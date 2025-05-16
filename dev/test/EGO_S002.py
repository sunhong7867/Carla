# Step 1
ego_velocity = ego_data.velocity_x
velocity_error = abs(ego_velocity - 15.0)
elapsed_sec = int((self.current_time_ms - self.start_time_ms) // 1000)

if velocity_error <= 5.0:
    print(f"[STEP1][{elapsed_sec} sec] velocity = {ego_velocity:.2f} m/s → Pass")
else:
    print(f"[STEP1][{elapsed_sec} sec] velocity = {ego_velocity:.2f} m/s → Fail")



# Step 2
elapsed_sec = int((self.current_time_ms - self.start_time_ms) // 1000)
gps_dt = abs(self.current_time_ms - self.gps_data.timestamp)
if gps_dt <= 50.0:
     print(f"[STEP2][{elapsed_sec} sec] GPS 수신 시각 지연 = {gps_dt:.1f} ms → Pass")
else:
     print(f"[STEP2][{elapsed_sec} sec] GPS 수신 시각 지연 = {gps_dt:.1f} ms → Fail")


# Step 3
elapsed_sec = int((self.current_time_ms - self.start_time_ms) // 1000)
estimated_vx = ego_data.velocity_x
groundtruth_vx = self._actor.get_velocity().x
velocity_error = abs(estimated_vx - groundtruth_vx)
if velocity_error <= 5.0:
    print(f"[STEP3] [{elapsed_sec} sec] 추정 속도 오차 = {velocity_error:.2f} m/s → Pass")
else:
    print(f"[STEP3] [{elapsed_sec} sec] 추정 속도 오차 = {velocity_error:.2f} m/s → Fail")

