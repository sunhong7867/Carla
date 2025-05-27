# ---------- 평가 로그 ----------
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

# 감지된 객체 중 cut-in / front 후보 추출
cutin_obj = None
front_obj = None
min_cutin_dist = 1e9
min_front_dist = 1e9

for obj in filtered:
    if obj.object_type != ObjectType.CAR or obj.status != ObjectStatus.MOVING:
        continue
    # Cut-in 후보: 좌측 차선, 횡방향 속도 있음
    if obj.position_y > 1.0 and abs(obj.velocity_y) >= 0.2:
        if obj.distance < min_cutin_dist:
            cutin_obj = obj
            min_cutin_dist = obj.distance
    # Front 후보: 중앙 차선에 가까움
    elif abs(obj.position_y) < 1.0:
        if obj.distance < min_front_dist:
            front_obj = obj
            min_front_dist = obj.distance

cutin_id = cutin_obj.object_id if cutin_obj else -1
front_id = front_obj.object_id if front_obj else -1
is_cutin_target = (aeb_id == cutin_id)

prev_target_id = aeb_tgt.object_id if aeb_tgt else -1

self.log_rows.append({
    # 시간
    "time": round((now_ms - self.sim_start_ms) / 1000.0, 2),

    # AEB 타겟 정보
    "AEB_Target_ID": aeb_id,
    "AEB_Target_ID_Prev": prev_target_id if 'prev_target_id' in locals() else -2,
    "AEB_Target_is_Cutin": is_cutin_target,
    "AEB_Target_Status": aeb_status,
    "AEB_Target_Situation": aeb_situation,
    "AEB_Target_Type": aeb_tgt.object_type.name if aeb_tgt else "None",
    "AEB_Target_Distance": aeb_distance,
    "AEB_Target_Position_X": round(aeb_tgt.position_x, 2) if aeb_tgt else 0.0,
    "AEB_Target_Position_Y": round(aeb_tgt.position_y, 2) if aeb_tgt else 0.0,

    # TTC
    "TTC": round(ttc, 2),
    "TTC_Brake": round(ttc_brake, 2),
    "TTC_Alert": round(ttc_alert, 2),
    "Relative_Speed": round(rel_speed, 2),

    # Ego 차량 상태
    "Ego_Speed": ego_speed,
    "Target_Speed": target_speed,

    # Cut-in 차량 정보 (감지 기준)
    "Cutin_ID": cutin_id,
    "Cutin_Distance": round(cutin_obj.distance, 2) if cutin_obj else -1.0,
    "Cutin_Speed": round(cutin_obj.velocity_x, 2) if cutin_obj else 0.0,

    # 정면 차량 정보 (감지 기준)
    "Front_ID": front_id,
    "Front_Distance": round(front_obj.distance, 2) if front_obj else -1.0,
    "Front_Speed": round(front_obj.velocity_x, 2) if front_obj else 0.0,

    # 전체 후보 수
    "AEB_Candidate_Count": len(
        [obj for obj in filtered if obj.object_type == ObjectType.CAR and obj.status == ObjectStatus.MOVING]
    ),
})
