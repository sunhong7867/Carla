# === Debugging Print for ACC_S002 Scenario ===
elapsed_sec = int((self.current_time_ms - self.start_time_ms) // 1000)

# STEP 1: 전방 차량 감지 여부
if acc_target is not None and acc_target.object_id >= 0 and acc_target.distance <= 80.0:
    print(f"[STEP1][{elapsed_sec} sec] Target detected: ID = {acc_target.object_id}, Distance = {acc_target.distance:.2f} m")

# STEP 2: ACC 모드 출력 (전환 여부는 로그 흐름으로 확인)
print(f"[STEP2][{elapsed_sec} sec] ACC Mode = {acc_mode.name}, Accel_ACC_X = {acc_accel:.2f} m/s²")

# STEP 3: 거리 유지 진입 후 평균 거리 계산 (5초 이상 유지 확인)
if 39.0 <= acc_target.distance <= 41.0:
    self.dist_buffer.append(acc_target.distance)

if len(self.dist_buffer) >= int(5.0 / dt_sec):  # 5초치 데이터 확보
    avg_dist = sum(self.dist_buffer) / len(self.dist_buffer)
    print(f"[STEP3][{elapsed_sec} sec] Maintaining distance: Avg = {avg_dist:.2f} m over 5.0 s")
