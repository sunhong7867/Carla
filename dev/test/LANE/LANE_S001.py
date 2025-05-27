self.log_data.append({
    "time": round((self.time_data.current_time - self.sim_start_time) / 1000.0, 2),
    "ego_speed": round(ego.velocity_x, 2),
    "LS_Lane_Type": lane_output.lane_type.name,
    "LS_Lane_Offset": round(lane_output.lane_offset, 2),
    "LS_Lane_Width": round(lane_output.lane_width, 2),
    "LS_Is_Within_Lane": lane_output.is_within_lane,
    "LS_Is_Curved_Lane": lane_output.is_curved_lane,
    "lane_curvature": round(lane_data.curvature, 2)
    "Lane_Center_Check": abs(lane_output.lane_offset) < lane_output.lane_width * 0.5
})