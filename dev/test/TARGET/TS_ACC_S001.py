# ---------- 평가 로그 ----------
ttc = -1.0
if acc_tgt and acc_tgt.velocity_x < ego.velocity_x:
    rel_speed = ego.velocity_x - acc_tgt.velocity_x
    if rel_speed > 0:
        ttc = acc_tgt.distance / rel_speed

# Step 3용: ACC 타겟 vs GT 타겟 위치 오차 계산
gt_x, gt_y = 0.0, 0.0
pos_error = -1.0
if self.target_gt and acc_tgt:
    gt_x = self.target_gt.get_location().x
    gt_y = self.target_gt.get_location().y
    dx = gt_x - acc_tgt.position_x
    dy = gt_y - acc_tgt.position_y
    pos_error = (dx**2 + dy**2) ** 0.5

self.log_rows.append({
    "time": round((now_ms - self.sim_start_ms) / 1000.0, 2),

    # Step 3 – Ego 속도 유지 여부 (50 ± 3.6 km/h)
    "ego_speed": round(ego.velocity_x, 2),

    # Step 1, 2, 3 – ACC 타겟 정보
    "ACC_Target_ID": acc_tgt.object_id if acc_tgt else -1,
    "ACC_Target_Cell_ID": acc_tgt.cell_id if acc_tgt else -1,
    "ACC_Target_Status": acc_tgt.status.name if acc_tgt else "None",
    "ACC_Target_Type": acc_tgt.object_type.name if acc_tgt else "None",
    "ACC_Target_Distance": round(acc_tgt.distance, 2) if acc_tgt else -1.0,
    "ACC_Target_TTC": round(ttc, 2) if ttc > 0 else -1.0,
    "ACC_Target_Position_X": round(acc_tgt.position_x, 2) if acc_tgt else 0.0,
    "ACC_Target_Position_Y": round(acc_tgt.position_y, 2) if acc_tgt else 0.0,

    # Step 3 – GT 타겟 좌표 및 오차
    "Target_GT_X": round(gt_x, 2),
    "Target_GT_Y": round(gt_y, 2),
    "Target_Position_Error": round(pos_error, 2) if pos_error >= 0 else -1.0
})
