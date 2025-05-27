# ---------- 평가 로그 (Cut-out 전용) ----------
ego_loc = self._actor.get_location()

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

# 감지된 객체 중 cut-out / front 후보 추출
cutout_obj = None
front_obj = None
min_cutout_dist = 1e9
min_front_dist = 1e9

for obj in filtered:
    if obj.object_type != ObjectType.CAR or obj.status != ObjectStatus.MOVING:
        continue

    # Cut-out 후보: 조건 충족 여부 직접 확인 (TS_RQ_10 기반)
    is_cutout = abs(obj.velocity_y) >= 0.2 and abs(obj.position_y) > 0.85
    is_front = abs(obj.position_y) <= 1.0

    if is_cutout and obj.distance < min_cutout_dist:
        cutout_obj = obj
        min_cutout_dist = obj.distance
    elif is_front and obj.distance < min_front_dist:
        front_obj = obj
        min_front_dist = obj.distance

cutout_id = cutout_obj.object_id if cutout_obj else -1
front_id = front_obj.object_id if front_obj else -1
is_cutout_target = (aeb_id == cutout_id)
is_front_target = (aeb_id == front_id)

self.log_rows.append({
    # 시간
    "time": round((now_ms - self.sim_start_ms) / 1000.0, 2),

    # AEB 타겟 정보
    "AEB_Target_ID": aeb_id,
    "AEB_Target_Status": aeb_status,
    "AEB_Target_Situation": aeb_situation,
    "AEB_Target_Type": aeb_tgt.object_type.name if aeb_tgt else "None",
    "AEB_Target_Distance": aeb_distance,
    "AEB_Target_Position_X": round(aeb_tgt.position_x, 2) if aeb_tgt else 0.0,
    "AEB_Target_Position_Y": round(aeb_tgt.position_y, 2) if aeb_tgt else 0.0,
    "AEB_Target_is_Cutout": is_cutout_target,
    "AEB_Target_is_Front": is_front_target,

    # TTC
    "TTC": round(ttc, 2),
    "TTC_Brake": round(ttc_brake, 2),
    "TTC_Alert": round(ttc_alert, 2),
    "Relative_Speed": round(rel_speed, 2),

    # Ego 차량 상태
    "Ego_Speed": ego_speed,
    "Target_Speed": target_speed,

    # Cut-out 차량 정보
    "Cutout_ID": cutout_id,
    "Cutout_Distance": round(cutout_obj.distance, 2) if cutout_obj else -1.0,
    "Cutout_Speed": round(cutout_obj.velocity_x, 2) if cutout_obj else 0.0,

    # 정면 차량 정보
    "Front_ID": front_id,
    "Front_Distance": round(front_obj.distance, 2) if front_obj else -1.0,
    "Front_Speed": round(front_obj.velocity_x, 2) if front_obj else 0.0,

    # 전체 후보 수
    "AEB_Candidate_Count": len([
        obj for obj in filtered if obj.object_type == ObjectType.CAR and obj.status == ObjectStatus.MOVING
    ]),
})
