# ───────────── Debug 출력 ─────────────
        for obj_id, (rel_x, rel_y, rel_vx, rel_vy) in objects_positions.items():
            # 상대 거리 (Ego 기준 좌표계)
            distance = math.sqrt(rel_x ** 2 + rel_y ** 2)

            # Carla 절대 거리 계산
            ego_loc = self._actor.get_location()
            target_actor = self.world.get_actor(obj_id)
            if target_actor is not None:
                target_loc = target_actor.get_location()
                dx = target_loc.x - ego_loc.x
                dy = target_loc.y - ego_loc.y
                dz = target_loc.z - ego_loc.z
                carla_distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            else:
                carla_distance = -1.0  # 객체 사라졌을 경우 대비

            print(
                f"[STEP4][{elapsed_sec:.2f}s] Object_ID={obj_id}, "
                f"Rel_Position = ({rel_x:.2f}, {rel_y:.2f}), "
                f"Distance = {distance:.2f} m, Carla_Distance = {carla_distance:.2f} m"
            )


# STEP 1
print(f"[STEP1][{elapsed_sec:.2f}s] Ego_Position = ({0.0:.1f}, {0.0:.1f}, {0.0:.1f})")

# STEP 2
ego_loc = self._actor.get_location()
offset_x = -3.1  # 차량 앞바퀴 오프셋 (예: 차량 앞쪽 기준)
front_axle_x = ego_loc.x + offset_x
front_axle_y = ego_loc.y
print(f"[STEP2][{elapsed_sec:.2f}s] ego.get_location() = ({ego_loc.x:.2f}, {ego_loc.y:.2f}), FrontAxle = ({front_axle_x:.2f}, {front_axle_y:.2f})")


# STEP 3
print(f"[STEP3][{elapsed_sec:.2f}s] Ego_Position = (0.0, 0.0, 0.0)")


# STEP 4
for obj_id, (rel_x, rel_y, rel_vx, rel_vy) in objects_positions.items():
    distance = math.sqrt(rel_x**2 + rel_y**2)
    print(f"[STEP4][{elapsed_sec:.2f}s] Object_ID={obj_id}, Rel_Position = ({rel_x:.2f}, {rel_y:.2f}), Distance = {distance:.2f} m")
