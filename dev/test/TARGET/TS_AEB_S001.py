self.log_data.append({
    # === 시간 ===
    "time": round((self.time_data.current_time - self.sim_start_time) / 1000.0, 2),

    # === Ego 차량 상태 ===
    "ego_velocity_x": round(ego.velocity_x, 2),
    "ego_acceleration_x": round(ego.accel_x, 2),

    # === AEB 타겟 정보 ===
    "aeb_target_id": aeb_target.object_id,
    "aeb_target_status": aeb_target.status.name,
    "aeb_target_situation": aeb_target.situation.name,
    "aeb_target_velocity_x": round(aeb_target.velocity_x, 2),
    "aeb_target_distance": round(aeb_target.distance, 2),

    # === TTC 계산 정보 ===
    "ttc": round(ttc_data["ttc"], 2),
    "ttc_brake": round(ttc_data["ttc_brake"], 2),
    "ttc_alert": round(ttc_data["ttc_alert"], 2),
    "ttc_rel_speed": round(ttc_data["rel_speed"], 2),

    # === 모드 정보 (ACC / AEB) ===
    "aeb_mode": aeb_mode.name,
    "acc_mode": acc_mode.name,
    "is_aeb_target_selected": aeb_target.object_id != -1,

    # === 제어 명령 ===
    "throttle_cmd": round(ctrl["throttle"], 2),
    "brake_cmd": round(ctrl["brake"], 2),
    "steer_cmd": round(ctrl["steer"], 2),

    # === 센서 상태 ===
    "gps_dt": round(abs(self.time_data.current_time - self.gps_sensor_time), 2),
    "gps_update_enabled": gps_update_enabled,

    # === 센서 원시값 ===
    "raw_gps_velocity_x": round(gps_data.velocity_x, 2),
    "raw_imu_accel_x": round(self.imu_data.accel_x, 2),
    "raw_imu_yaw_rate": round(self.imu_data.yaw_rate, 2),

    # === Ground Truth ===
    "gt_velocity": round(math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2), 2),
    "gt_acceleration": round(self._actor.get_acceleration().x, 2)
})
