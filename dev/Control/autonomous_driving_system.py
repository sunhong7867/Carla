# autonomous_driving_system_merged.py
#!/usr/bin/env python
import math, csv
from pathlib import Path
from typing import Dict, List
import numpy as np
import carla

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

# ---------- Decision / Controller ----------

from decision import ego_vehicle_estimation         as EGO
from decision import lane_selection                 as LANE
from decision import target_selection               as TS
#from decision import autonomous_emergency_brake     as AEB
from decision import adaptive_cruise_control        as ACC
# from decision import lane_following_assist          as LFA
from decision import arbitration                    as ARB

from decision.shared_types import (
    LaneData, LaneType, LaneChangeStatus,
    TimeData, GPSData, IMUData,
    ObjectData, ObjectType, ObjectStatus, ACCMode, AEBMode
)

# ---------------- 유틸리티 ----------------
def dict_to_objectdata(d: Dict) -> ObjectData:
    return ObjectData(
        object_id=d["object_id"],
        object_type=ObjectType(d["object_type"]),
        position_x=d["position_x"],  position_y=d["position_y"],
        position_z=d.get("position_z", 0.0),
        velocity_x=d["velocity_x"],  velocity_y=d["velocity_y"],
        accel_x=d.get("accel_x", 0.0), accel_y=d.get("accel_y", 0.0),
        heading=d["heading"], distance=d["distance"],
        status=ObjectStatus(d["status"]), cell_id=d.get("cell_id", 0)
    )

# -------------- ADS 클래스 --------------
class AutonomousDrivingSystem(BasicControl):
    def __init__(self, actor, args=None):
        super().__init__(actor)
        print("======= ADAS Python System (Merged + lane_selection) =======")

        self._actor = actor
        self.world  = CarlaDataProvider.get_world()
        self.map    = self.world.get_map()
        self.spectator = self.world.get_spectator()
        self.ego_id = actor.id
        self.target_gt = self._find_first_other_vehicle()

        # 상태 초기화
        self._init_control()
        self._set_imu_sensor()
        self._set_gnss_sensor()

        self.prev_step_ms = None
        self.kf_state = EGO.init_ego_vehicle_kf_state()
        self.imu_data = IMUData(0.0, 0.0, 0.0)
        self.time_data = TimeData(0.0)
        self.gnss_time_ms = 0.0

        self.sim_start_ms = None
        self.log_rows: List[Dict] = []

    # -------- 센서 설정 --------
    def _set_imu_sensor(self):
        bp = self.world.get_blueprint_library().find('sensor.other.imu')
        bp.set_attribute('sensor_tick', '0.01')  # 100 Hz
        imu_tf = carla.Transform(carla.Location(), carla.Rotation())
        imu = self.world.spawn_actor(bp, imu_tf, attach_to=self._actor,
                                     attachment_type=carla.AttachmentType.Rigid)
        imu.listen(self._on_imu)
        self.imu_sensor = imu

    def _set_gnss_sensor(self):
        bp = self.world.get_blueprint_library().find('sensor.other.gnss')
        bp.set_attribute('sensor_tick', '0.05')  # 20 Hz
        gnss_tf = carla.Transform(carla.Location(), carla.Rotation())
        gnss = self.world.spawn_actor(bp, gnss_tf, attach_to=self._actor,
                                      attachment_type=carla.AttachmentType.Rigid)
        gnss.listen(self._on_gnss)
        self.gnss_sensor = gnss

    def _on_imu(self, data):
        self.imu_data = IMUData(data.accelerometer.x,
                                data.accelerometer.y,
                                math.degrees(data.gyroscope.z))

    def _on_gnss(self, data):
        self.gnss_time_ms = data.timestamp * 1000.0

    # -------- 초기 VehicleControl --------
    def _init_control(self):
        self.control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0)

    # -------- 보조 함수 --------
    def _find_first_other_vehicle(self):
        for v in self.world.get_actors().filter('vehicle.*'):
            if v.id != self.ego_id:
                return v
        return None

    def _spectator_update(self):
        tf = self._actor.get_transform()
        tf.location.x -= 10
        tf.location.z += 5
        self.spectator.set_transform(tf)

    # --------------------------------------------------
    #                     MAIN LOOP
    # --------------------------------------------------
    def run_step(self):
        now_ms = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0
        if self.sim_start_ms is None:
            self.sim_start_ms = now_ms
        self.time_data.current_time = now_ms

        # ---------- Ego 추정 ----------
        vel = self._actor.get_velocity()
        gps = GPSData(vel.x, vel.y, self.gnss_time_ms, self.gnss_time_ms)
        ego = EGO.ego_vehicle_estimation(self.time_data, gps, self.imu_data, self.kf_state)

        # ---------- Waypoint ----------
        wp = self.map.get_waypoint(self._actor.get_location(), project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        if wp is None:
            self._actor.apply_control(self.control)
            return

        offset = self._actor.get_location().y - wp.transform.location.y
        heading_err = (ego.heading - wp.transform.rotation.yaw + 180) % 360 - 180
        steer_cmd = max(min(-0.3 * offset - 0.1 * heading_err, 1.0), -1.0)
        self.control.steer = steer_cmd

        # ---------- lane_selection 호출 ----------
        lane_output = LANE.lane_selection(
            LaneData(LaneType.STRAIGHT, 0.0, 0.0, offset,
                     wp.transform.rotation.yaw, wp.lane_width,
                     LaneChangeStatus.CHANGING if abs(steer_cmd) > 0.1 else LaneChangeStatus.KEEP),
            ego)

        # ---------- 객체 리스트 ----------
        rel_objs = []
        yaw = math.radians(self._actor.get_transform().rotation.yaw)
        cy, sy = math.cos(-yaw), math.sin(-yaw)
        for v in self.world.get_actors().filter('vehicle.*'):
            if v.id == self.ego_id:
                continue
            loc = v.get_location()
            vel_v = v.get_velocity()
            dx, dy = loc.x - self._actor.get_location().x, loc.y - self._actor.get_location().y
            rx, ry = dx * cy - dy * sy, dx * sy + dy * cy
            rvx, rvy = vel_v.x * cy - vel_v.y * sy, vel_v.x * sy + vel_v.y * cy
            rel_objs.append({
                'object_id': v.id, 'position_x': rx, 'position_y': ry,
                'velocity_x': rvx, 'velocity_y': rvy,
                'accel_x': 0.0, 'accel_y': 0.0,
                'distance': math.hypot(rx, ry),
                'heading': v.get_transform().rotation.yaw - self._actor.get_transform().rotation.yaw,
                'object_type': ObjectType.CAR.value, 'status': ObjectStatus.MOVING.value
            })
        object_datas = [dict_to_objectdata(d) for d in rel_objs]

        # ---------- Target Selection ----------
        filtered = TS.select_target_from_object_list(object_datas, ego, lane_output)
        predicted = TS.predict_object_future_path(filtered, lane_output)
        acc_tgt, aeb_tgt = TS.select_targets_for_acc_aeb(ego, predicted, lane_output)


        # ----- ACC 계산 ------------------------------------------
        delta_t = 0.05 if self.prev_step_ms is None else (now_ms - self.prev_step_ms) / 1000.0
        self.prev_step_ms = now_ms

        acc_mode = ACC.acc_mode_selection(acc_tgt, ego, lane_output)
        acc_dist = ACC.calculate_accel_for_distance_pid(acc_mode, acc_tgt, ego, now_ms)
        acc_speed = ACC.calculate_accel_for_speed_pid(ego, lane_output, delta_t)
        acc_accel = ACC.acc_output_selection(acc_mode, acc_dist, acc_speed)

        # ----- AEB·LFA 미사용 → 0 값 전달 --------------------------
        decel_aeb = 0.0
        aeb_mode = AEBMode.NORMAL  # 항상 Normal
        steer_lfa = 0.0  # LFA 비활성

        # ----- Arbitration ----------------------------------------
        ctrl = ARB.arbitration(acc_accel, 0.0, 0.0, AEBMode.NORMAL)
        self.control.throttle = ctrl['throttle']
        self.control.brake = ctrl['brake']
        self.control.steer = ctrl['steer']
        self._actor.apply_control(self.control)

        self._spectator_update()

        # ---------- 평가 로그 ----------
        now_ms = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0
        elapsed_sec = (now_ms - self.sim_start_ms) / 1000.0

        # Step 1 – 전방 차량 유무 확인
        object_count = len(rel_objs)
        ego_started = ego.velocity_x > 0.1

        # Step 2 – 목표 속도 도달 여부, 가속 출력 확인
        target_speed_reached = 21.22 <= ego.velocity_x <= 23.22
        throttle_cmd = round(self.control.throttle, 3)
        brake_cmd = round(self.control.brake, 3)

        # Step 3 – 속도 유지 및 제어 안정성 평가
        # → 여기선 manual_check로 기록 (후처리용)
        speed_stable_check = "manual_check"

        # ACC 타겟 기준 TTC 계산
        ttc = -1.0
        if acc_tgt and acc_tgt.velocity_x < ego.velocity_x:
            rel_speed = ego.velocity_x - acc_tgt.velocity_x
            if rel_speed > 0:
                ttc = acc_tgt.distance / rel_speed

        # ACC 타겟 vs GT 비교
        gt_x, gt_y = 0.0, 0.0
        pos_error = -1.0
        if self.target_gt and acc_tgt:
            gt_loc = self.target_gt.get_location()
            gt_x, gt_y = gt_loc.x, gt_loc.y
            dx = gt_x - acc_tgt.position_x
            dy = gt_y - acc_tgt.position_y
            pos_error = (dx ** 2 + dy ** 2) ** 0.5

        # 로그 저장
        self.log_rows.append({
            "time": round(elapsed_sec, 2),

            # Step 1
            "ACC_Mode": acc_mode.name,
            "Object_Count": object_count,
            "Ego_Started": round(ego.velocity_x, 2),

            # Step 2
            "Ego_Velocity_X": round(ego.velocity_x, 2),
            "Target_Speed_Reached": target_speed_reached,
            "Throttle_Command": throttle_cmd,
            "Brake_Command": brake_cmd,

            # Step 3
            "Speed_Stable_5sec": speed_stable_check,
            "ACC_Target_ID": acc_tgt.object_id if acc_tgt else -1,
            "ACC_Target_Distance": round(acc_tgt.distance, 2) if acc_tgt else -1.0,
            "ACC_Target_TTC": round(ttc, 2) if ttc > 0 else -1.0,

            # GT 기준 비교용
            "GT_Target_X": round(gt_x, 2),
            "GT_Target_Y": round(gt_y, 2),
            "ACC_Target_Position_X": round(acc_tgt.position_x, 2) if acc_tgt else 0.0,
            "ACC_Target_Position_Y": round(acc_tgt.position_y, 2) if acc_tgt else 0.0,
            "Target_Position_Error": round(pos_error, 2) if pos_error >= 0 else -1.0
        })

    # ---------- 종료 시 로그 저장 ----------
    def reset(self):
        if self.log_rows:
            path = Path("C:/Users/MSI-Book/Desktop/log/ACC_S003.csv")
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self.log_rows[0].keys())
                writer.writeheader()
                writer.writerows(self.log_rows)
            print(f"[INFO] 로그 저장 완료: {path}")
        if hasattr(self, 'imu_sensor'):
            self.imu_sensor.stop();  self.imu_sensor.destroy()
        if hasattr(self, 'gnss_sensor'):
            self.gnss_sensor.stop(); self.gnss_sensor.destroy()
