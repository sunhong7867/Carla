elapsed_sec = int((self.current_time_ms - self.start_time_ms) // 1000)
ego_velocity = ego_data.velocity_x
print(f"[STEP1][{elapsed_sec} sec] throttle = {self.control.throttle:.2f}, accel = {self.imu_data.accel_x:.2f} m/s², velocity = {ego_velocity:.2f} m/s")
print(f"[STEP2][{elapsed_sec} sec] ACC_Mode = {acc_mode.name}")
print(f"[STEP3][{elapsed_sec} sec] velocity = {ego_velocity:.2f} m/s})")