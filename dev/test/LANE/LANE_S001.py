self.log_data.append({
    # 공통 시간 기록
    "time": round((self.time_data.current_time - self.sim_start_time) / 1000.0, 2),

    # === Step 1: 조향 반응 평가 (오프셋 방향과 조향 방향) ===
    "ls_lane_offset": round(lane_output.lane_offset, 2),
    "steer_cmd": round(ctrl['steer'], 2),

    # === Step 2: 차선 중심 수렴 여부 판단 ===
    "ls_lane_offset_abs": round(abs(lane_output.lane_offset), 2),

    # === Step 3: 차선 내 판별 여부 (Lane_Width 기준) ===
    "ls_is_within_lane": lane_output.is_within_lane,
    "ls_lane_width": round(lane_output.lane_width, 2),

    # === Step 4: 속도 유지 여부 평가 ===
    "ego_velocity_x": round(ego.velocity_x, 2),

    # === 보조 정보 ===
    "ego_acceleration_x": round(ego.accel_x, 2),
    "ls_is_curved_lane": lane_output.is_curved_lane,
    "ls_lane_curvature": 0.0,  # 현재 시나리오에서는 고정

    # 센서 유효성 상태
    "gps_dt": round(abs(self.time_data.current_time - self.gps_sensor_time), 2),
    "gps_update_enabled": gps_update_enabled,

    # 센서 원시값 기록
    "raw_gps_velocity_x": round(gps_data.velocity_x, 2),
    "raw_gps_velocity_y": round(gps_data.velocity_y, 2),
    "raw_imu_accel_x": round(self.imu_data.accel_x, 2),
    "raw_imu_accel_y": round(self.imu_data.accel_y, 2),
    "raw_imu_yaw_rate": round(self.imu_data.yaw_rate, 2),

    # Ground Truth 비교용
    "gt_velocity": round(math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2), 2),
    "gt_acceleration": round(self._actor.get_acceleration().x, 2)
})
