# ---------- 평가 로그 ----------
gt_loc = self.target_gt.get_location() if self.target_gt else None
ego_loc = self._actor.get_location()

dx_gt = (gt_loc.x - ego_loc.x) if gt_loc else 0.0
dy_gt = (gt_loc.y - ego_loc.y) if gt_loc else 0.0
gt_distance = math.hypot(dx_gt, dy_gt) if gt_loc else -1.0

aeb_id = aeb_tgt.object_id if aeb_tgt else -1
aeb_status = aeb_tgt.status.name if aeb_tgt else "None"
aeb_situation = aeb_tgt.situation.name if aeb_tgt else "None"
aeb_distance = round(aeb_tgt.distance, 2) if aeb_tgt else -1.0

ttc = ttc_res['ttc'] if ttc_res else -1.0
ttc_brake = ttc_res['ttc_brake'] if ttc_res else -1.0
ttc_alert = ttc_res['ttc_alert'] if ttc_res else -1.0
rel_speed = ttc_res['rel_speed'] if ttc_res else 0.0

ego_speed = round(ego.velocity_x, 2)
target_speed = round(aeb_tgt.velocity_x, 2) if aeb_tgt else 0.0

# Step 1 평가 요소: ID, Status, Distance
step1_valid = (aeb_id != -1) and (aeb_status == "MOVING") and (aeb_distance <= 60.0)

# Step 2 평가 요소: TTC 진입 시점 상태
step2_valid = (ttc <= 2.5) and (aeb_id != -1) and (aeb_status == "MOVING") and (aeb_situation != "ONCOMING")

# Step 3 평가 요소: 정속 + TTC ≤ 2.5 + 상태 유지
step3_valid = (
    (12.89 <= ego_speed <= 14.89) and
    (ttc <= 2.5) and
    (aeb_id != -1) and
    (aeb_situation != "ONCOMING")
)

self.log_rows.append({
    # 공통
    "time": round((now_ms - self.sim_start_ms) / 1000.0, 2),

    # Step 1 판단용
    "AEB_Target_ID": aeb_id,
    "AEB_Target_Status": aeb_status,
    "AEB_Target_Situation": aeb_situation,
    "AEB_Target_Distance": aeb_distance,

    # Step 2, 3 판단용
    "TTC": round(ttc, 2),
    "TTC_Brake": round(ttc_brake, 2),
    "TTC_Alert": round(ttc_alert, 2),
    "Relative_Speed": round(rel_speed, 2),

    # Step 3 추가 조건용
    "Ego_Speed": ego_speed,
    "Target_Speed": target_speed,
    "GT_Target_Distance": round(gt_distance, 2),

    # Step 평가 조건 플래그
    "STEP1_Valid": step1_valid,
    "STEP2_Valid": step2_valid,
    "STEP3_Valid": step3_valid
})
