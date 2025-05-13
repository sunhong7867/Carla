print(    f"[STEP1] Ego_Position = ({ego_data.position_x:.2f}, {ego_data.position_y:.2f}, {ego_data.position_z:.2f}) → 기준 (0.00, 0.00, 0.00)")


# 기준 좌표 변환
ego_tf = self._actor.get_transform()
ego_loc = ego_tf.location
front_offset_x = 1.5  # 차량 앞바퀴 중심 오프셋 (예시값)
ref_x = ego_loc.x - front_offset_x
ref_y = ego_loc.y
print(f"[STEP2] ego.get_location() = ({ego_loc.x:.2f}, {ego_loc.y:.2f}) → 기준점 변환 후 ({ref_x:.2f}, {ref_y:.2f})")


if hasattr(self, "prev_ego_position"):
    dx = abs(ego_data.position_x - self.prev_ego_position[0])
    dy = abs(ego_data.position_y - self.prev_ego_position[1])
    print(f"[STEP3] ΔEgo_Position = (Δx={dx:.4f}, Δy={dy:.4f}) → 변화 없음 여부 확인")
self.prev_ego_position = (ego_data.position_x, ego_data.position_y)


for obj in obj_list:
    dist_calc = math.hypot(obj.position_x, obj.position_y)
    print(f"[STEP4] Object_ID={obj.object_id} → Position=({obj.position_x:.2f}, {obj.position_y:.2f}), "
          f"Distance={obj.distance:.2f}m vs 계산된={dist_calc:.2f}m")
