self.log_data.append({
    # === 공통 시간 기록 ===
    "time": round((self.time_data.current_time - self.sim_start_time) / 1000.0, 2),

    # === Step 1: Ego 차량 속도 유지 여부 판단 ===
    "ego_velocity_x": round(ego.velocity_x, 2),
    "ego_acceleration_x": round(ego.accel_x, 2),

    # === Step 2: 감지 차량 상태 판단 (Moving 상태 여부) ===
    "acc_target_id": acc_target.object_id,
    "acc_target_status": acc_target.status.name,
    "acc_target_velocity_x": round(acc_target.velocity_x, 2),
    "acc_target_position_x": round(acc_target.position_x, 2),
    "acc_target_position_y": round(acc_target.position_y, 2),

    # === Step 3: 셀 번호 계산 정확성 확인 ===
    "acc_target_cell_id": acc_target.cell_id,
    "acc_target_distance": round(acc_target.distance, 2),

    # === Step 4: ACC 타겟 선정 여부 평가 ===
    "is_acc_target_selected": acc_target.object_id != -1,

    # === 차선 관련 정보 (보조 정보 포함) ===
    "ls_lane_offset": round(lane_output.lane_offset, 2),
    "ls_lane_offset_abs": round(abs(lane_output.lane_offset), 2),
    "ls_is_within_lane": lane_output.is_within_lane,
    "ls_lane_width": round(lane_output.lane_width, 2),
    "ls_is_curved_lane": lane_output.is_curved_lane,
    "ls_lane_curvature": lane_output.curvature,
    "ls_heading_error": round(lane_output.heading_error, 2) if hasattr(lane_output, 'heading_error') else 0.0,

    # === 제어 명령 (조향 등) ===
    "steer_cmd": round(ctrl['steer'], 2),
    "throttle_cmd": round(ctrl['throttle'], 2),
    "brake_cmd": round(ctrl['brake'], 2),

    # === 센서 유효성 평가 ===
    "gps_dt": round(abs(self.time_data.current_time - self.gps_sensor_time), 2),
    "gps_update_enabled": gps_update_enabled,

    # === 센서 원시값 ===
    "raw_gps_velocity_x": round(gps_data.velocity_x, 2),
    "raw_gps_velocity_y": round(gps_data.velocity_y, 2),
    "raw_imu_accel_x": round(self.imu_data.accel_x, 2),
    "raw_imu_accel_y": round(self.imu_data.accel_y, 2),
    "raw_imu_yaw_rate": round(self.imu_data.yaw_rate, 2),

    # === Ground Truth 비교용 ===
    "gt_velocity": round(math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2), 2),
    "gt_acceleration": round(self._actor.get_acceleration().x, 2)
})
``
