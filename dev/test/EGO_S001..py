        # === Debugging Print ===
        print("======================================")
        tf = self._actor.get_transform()
        ego_loc = tf.location
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[DEBUG][S1] time={self.current_time_ms:.1f}ms Ego_Position = ({ego_data.position_x:.2f}, {ego_data.position_y:.2f}, {ego_data.position_z:.2f}), "
        f"Velocity = ({ego_data.velocity_x:.2f}, {ego_data.velocity_y:.2f})")


        front_offset_x = 1.5
        ref_x = ego_loc.x - front_offset_x
        ref_y = ego_loc.y
        print(f"[DEBUG][S2] ego.get_location() = ({ego_loc.x:.2f}, {ego_loc.y:.2f}) → 기준점 변환 후 ({ref_x:.2f}, {ref_y:.2f})")

        if hasattr(self, "prev_ego_position"):
            dx = abs(ego_data.position_x - self.prev_ego_position[0])
            dy = abs(ego_data.position_y - self.prev_ego_position[1])
            print(f"[DEBUG][S3] ΔEgo_Position = (Δx={dx:.4f}, Δy={dy:.4f})")
        self.prev_ego_position = (ego_data.position_x, ego_data.position_y)

        if len(filtered) > 0:
            target = filtered[0]
            for v in self.world.get_actors().filter("vehicle.*"):
                if v.id == self.ego_id:
                    continue
                tgt_loc = v.get_location()
                dx = tgt_loc.x - ego_loc.x
                dy = tgt_loc.y - ego_loc.y
                dist = math.hypot(dx, dy)
                err_x = abs(target.position_x - dx)
                err_y = abs(target.position_y - dy)
                err_d = abs(target.distance - dist)
                print(f"[DEBUG][S4] Target Δx={err_x:.2f}, Δy={err_y:.2f}, Δdist={err_d:.2f}")
                break

        # Carla 내 모든 차량의 위치 확인 (좌측 차선 인지 가능 여부 포함)
        ego_yaw = math.radians(tf.rotation.yaw)
        cy, sy = math.cos(-ego_yaw), math.sin(-ego_yaw)
        found_left_vehicle = False
        for v in self.world.get_actors().filter("vehicle.*"):
            if v.id == self.ego_id:
                continue
            loc = v.get_location()
            dx = loc.x - ego_loc.x
            dy = loc.y - ego_loc.y
            rx = dx * cy - dy * sy
            ry = dx * sy + dy * cy

            print(f"[DEBUG][S4-ALL] Vehicle ID={v.id}, Carla=({loc.x:.2f},{loc.y:.2f}) → Rel=({rx:.2f},{ry:.2f})")

            # 좌측 차선은 ry < 0 (Ego 기준 좌측)
            if -5.0 <= ry <= -2.5 and rx > 0:
                dist = math.hypot(rx, ry)
                print(f"[DEBUG][S4-L] Carla 좌측 차선 차량 → 상대 위치: x={rx:.2f}, y={ry:.2f}, 거리={dist:.2f}")
                print(f"[DEBUG][S4-L] Carla 실제 위치: ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
                found_left_vehicle = True

        if not found_left_vehicle:
            print("[DEBUG][S4-L] 좌측 차선 차량 인지 실패 → FAIL")