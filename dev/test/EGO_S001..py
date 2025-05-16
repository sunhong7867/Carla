# Step 1
print(f"[STEP1] Ego_Position = ({ego_data.position_x:.2f}, {ego_data.position_y:.2f}, {ego_data.position_z:.2f}) → 기준 (0.00, 0.00, 0.00)")

# Step 2
# === Ego 차량 Transform 및 위치 ===
ego_tf = self._actor.get_transform()
ego_loc = ego_tf.location
front_offset_x = 1.5  # 차량 앞바퀴 기준 오프셋

# Carla 상의 절대 위치 (기준: 차량의 중심 위치)
carla_loc = ego_loc

# Ego 차량 기준 좌표계로 변환 (기준: 앞바퀴 기준)
ego_ref_x = ego_loc.x - front_offset_x
ego_ref_y = ego_loc.y
loc_x = ego_loc.x - front_offset_x - ego_ref_x  # = 0
loc_y = ego_loc.y - ego_ref_y                   # = 0

# === 디버깅 출력 ===
print(f"[DEBUG][STEP2] Carla 절대 위치 = ({carla_loc.x:.2f}, {carla_loc.y:.2f}) | Ego 기준 위치 = ({loc_x:.2f}, {loc_y:.2f})")


# Step 3
if hasattr(self, "prev_ego_position"):
    dx = abs(ego_data.position_x - self.prev_ego_position[0])
    dy = abs(ego_data.position_y - self.prev_ego_position[1])
    print(f"[STEP3] ΔEgo_Position = (Δx={dx:.4f}, Δy={dy:.4f}) → 변화 없음 여부 확인")
self.prev_ego_position = (ego_data.position_x, ego_data.position_y)


# Step 4
for obj in obj_list:
    dist_calc = math.hypot(obj.position_x, obj.position_y)
    print(f"[STEP4] Object_ID={obj.object_id} → Position=({obj.position_x:.2f}, {obj.position_y:.2f}), "
          f"Distance={obj.distance:.2f}m vs 계산된={dist_calc:.2f}m")
