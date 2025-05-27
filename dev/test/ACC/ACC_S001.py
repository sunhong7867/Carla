# ---------- 평가 로그 ----------
now_ms = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0
elapsed_sec = (now_ms - self.sim_start_ms) / 1000.0

# Step 1 – 전방 차량 유무 확인
object_count = len(rel_objs)
ego_started = ego.velocity_x > 0.1

# Step 2 – 목표 속도 도달 여부, 가속 출력 확인
target_speed_reached = 21.22 <= ego.velocity_x <= 23.22
throttle_cmd = round(self.control.throttle, 3)
brake_cmd    = round(self.control.brake,  3)

# Step 3 – 속도 유지 및 제어 안정성 평가
# → 여기선 manual_check로 기록 (후처리용)
speed_stable_check = "manual_check"

# ACC 타겟 기준 TTC 계산
ttc = -1.0
if acc_tgt and acc_tgt.velocity_x < ego.velocity_x:
    rel_speed = ego.velocity_x - acc_tgt.velocity_x
    if rel_speed > 0:
        ttc = acc_tgt.distance / rel_speed

# ACC 타겟 vs GT 비교
gt_x, gt_y = 0.0, 0.0
pos_error  = -1.0
if self.target_gt and acc_tgt:
    gt_loc = self.target_gt.get_location()
    gt_x, gt_y = gt_loc.x, gt_loc.y
    dx = gt_x - acc_tgt.position_x
    dy = gt_y - acc_tgt.position_y
    pos_error = (dx**2 + dy**2)**0.5

# 로그 저장
self.log_rows.append({
    "time": round(elapsed_sec, 2),

    # Step 1
    "ACC_Mode":        acc_mode.name,
    "Object_Count":    object_count,
    "Ego_Started":     round(ego.velocity_x, 2),

    # Step 2
    "Ego_Velocity_X":       round(ego.velocity_x, 2),
    "Accel_ACC_X":          round(acc_accel, 3),          # ← 추가
    "Target_Speed_Reached": target_speed_reached,
    "Throttle_Command":     throttle_cmd,
    "Brake_Command":        brake_cmd,

    # Step 3
    "Speed_Stable_5sec":      speed_stable_check,
    "ACC_Target_ID":          acc_tgt.object_id if acc_tgt else -1,
    "ACC_Target_Status":      acc_tgt.status.name,        # ← 추가
    "ACC_Target_Situation":   acc_tgt.situation.name,     # ← 추가
    "ACC_Target_Distance":    round(acc_tgt.distance, 2) if acc_tgt else -1.0,
    "ACC_Target_TTC":         round(ttc, 2) if ttc > 0 else -1.0,

    # GT 기준 비교용
    "GT_Target_X":               round(gt_x, 2),
    "GT_Target_Y":               round(gt_y, 2),
    "ACC_Target_Position_X":     round(acc_tgt.position_x, 2) if acc_tgt else 0.0,
    "ACC_Target_Position_Y":     round(acc_tgt.position_y, 2) if acc_tgt else 0.0,
    "Target_Position_Error":     round(pos_error, 2) if pos_error >= 0 else -1.0
})
