# Ego 추정 위치
ego_est = EGO.ego_vehicle_estimation(self.time_data, gps_data, self.imu_data, self.kf_state)
ego_pos_x = ego_est.position_x
ego_pos_y = ego_est.position_y
ego_pos_z = ego_est.position_z

# Carla 절대 위치 + 앞바퀴 기준 보정
carla_loc = self._actor.get_location()
carla_yaw = math.radians(self._actor.get_transform().rotation.yaw)
offset = 3.1
ego_front_x = carla_loc.x + offset * math.cos(carla_yaw)
ego_front_y = carla_loc.y + offset * math.sin(carla_yaw)
carla_offset_error = math.sqrt(ego_front_x**2 + ego_front_y**2)  # 오차

# Target 차량 상대 위치 및 거리 계산
target_loc = self.target.get_location()
target_yaw = math.radians(self.target.get_transform().rotation.yaw)
target_front_x = target_loc.x + offset * math.cos(target_yaw)
target_front_y = target_loc.y + offset * math.sin(target_yaw)

rel_x = target_front_x - ego_front_x
rel_y = target_front_y - ego_front_y
object_distance = math.sqrt(rel_x**2 + rel_y**2)
carla_distance = math.sqrt((carla_loc.x - target_loc.x)**2 + (carla_loc.y - target_loc.y)**2)
distance_error = abs(object_distance - carla_distance)

# 로그 저장
self.log_data.append({
    "time": round((self.time_data.current_time - self.sim_start_time) / 1000.0, 2),

    # Step 1 & 3-1: Ego 좌표 고정 여부 확인
    "ego_position_x": round(ego_pos_x, 3),
    "ego_position_y": round(ego_pos_y, 3),
    "ego_position_z": round(ego_pos_z, 3),

    # Step 2: Carla 위치 보정 오차 확인
    "carla_position_x": round(ego_front_x, 3),
    "carla_position_y": round(ego_front_y, 3),
    "carla_offset_error": round(carla_offset_error, 3),

    # Step 3-2: Object 거리 정합성
    "object_position_x": round(rel_x, 3),
    "object_position_y": round(rel_y, 3),
    "object_distance": round(object_distance, 3),
    "carla_distance": round(carla_distance, 3),
    "distance_error": round(distance_error, 3),
})
