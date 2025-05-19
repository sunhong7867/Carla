# === 로그 저장 ===
self.log_data.append({
    "time": round((self.time_data.current_time - self.sim_start_time) / 1000.0, 2),
    "ego_position_x": round(ego_pos_x, 2),
    "ego_position_y": round(ego_pos_y, 2),
    "ego_position_z": round(ego_pos_z, 2),
    "carla_position_x": round(carla_loc.x, 2),
    "carla_position_y": round(carla_loc.y, 2),
    "object_position_x": round(target_front_x, 2),  # 타겟 앞바퀴 기준 위치
    "object_position_y": round(target_front_y, 2),
    "relative_position_x": round(rel_x, 2),
    "relative_position_y": round(rel_y, 2),
    "relative_distance": round(rel_dist, 2),  # 방향성 유지
    "carla_distance": round(abs_dist, 2),  # 절대 거리
    "ego_velocity": round(math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2), 2),
    "ego_front_offset_error": round(offset_error, 3)
})