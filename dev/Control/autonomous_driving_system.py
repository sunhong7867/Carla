# autonomous_driving_system.py

#!/usr/bin/env python
import math
import csv
from pathlib import Path
import carla

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from decision import ego_vehicle_estimation as EGO
from decision import lane_selection as LANE
from decision import target_selection as TS
from decision import adaptive_cruise_control as ACC
from decision import autonomous_emergency_brake as AEB
from decision import lane_following_assist as LFA
from decision import arbitration as ARB
from controller import engine_control, brake_control, steer_control
from decision.shared_types import (
    LaneData, LaneType, LaneChangeStatus,
    TimeData, GPSData, IMUData,
    ObjectData, ObjectType, ObjectStatus
)

def convert_dict_to_objectdata(d):
    return ObjectData(
        object_id=d["object_id"],
        object_type=ObjectType(d["object_type"]),
        position_x=d["position_x"],
        position_y=d["position_y"],
        position_z=d.get("position_z", 0.0),
        velocity_x=d["velocity_x"],
        velocity_y=d["velocity_y"],
        accel_x=d.get("accel_x", 0.0),
        accel_y=d.get("accel_y", 0.0),
        heading=d["heading"],
        distance=d["distance"],
        status=ObjectStatus(d["status"]),
        cell_id=d.get("cell_id", 0)
    )

class AutonomousDrivingSystem(BasicControl):
    def __init__(self, actor, args=None):
        super().__init__(actor)
        print("======= ADAS Python System Activated =======")

        self._actor = actor
        self.world = CarlaDataProvider.get_world()
        self.map = self.world.get_map()
        self.spectator = self.world.get_spectator()
        self.ego_id = actor.id

        self.control = carla.VehicleControl()
        self.kf_state = EGO.init_ego_vehicle_kf_state()
        self.imu_data = IMUData(0.0, 0.0, 0.0)
        self.gps_data = GPSData(0.0, 0.0, 0.0)
        self.time_data = TimeData(0.0)

        self.sim_start_time = None
        self.gps_sensor_time = 0.0
        self.last_imu_time = 0.0
        self.last_gps_time = 0.0
        self.imu_intervals = []
        self.gps_intervals = []
        self.log_data = []

        self._set_imu_sensor()
        self._set_gps_sensor()

    def _init_control(self):
        self.control = carla.VehicleControl()
        self.control.steer = 0.0
        self.control.throttle = 0.0
        self.control.brake = 0.0

    def _set_imu_sensor(self):
        bp = self.world.get_blueprint_library().find('sensor.other.imu')
        tf = carla.Transform(carla.Location(), carla.Rotation())
        self.imu_sensor = self.world.spawn_actor(bp, tf, attach_to=self._actor)
        self.imu_sensor.listen(lambda data: self._on_imu_update(data))

    def _set_gps_sensor(self):
        bp = self.world.get_blueprint_library().find('sensor.other.gnss')
        bp.set_attribute('sensor_tick', '0.05')  # 20Hz 설정
        tf = carla.Transform(carla.Location(), carla.Rotation())
        self.gps_sensor = self.world.spawn_actor(bp, tf, attach_to=self._actor)
        self.gps_sensor.listen(lambda data: self._on_gps_update(data))

    def _on_imu_update(self, data):
        now = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0
        if self.last_imu_time > 0:
            self.imu_intervals.append(now - self.last_imu_time)
        self.last_imu_time = now
        self.imu_data = IMUData(data.accelerometer.x, data.accelerometer.y, math.degrees(data.gyroscope.z))

    def _on_gps_update(self, data):
        now = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0
        if self.last_gps_time > 0:
            self.gps_intervals.append(now - self.last_gps_time)
        self.last_gps_time = now
        self.gps_sensor_time = data.timestamp * 1000.0

    def _spectator_update(self):
        tf = self._actor.get_transform()
        tf.location.x -= 10
        tf.location.z += 5
        self.spectator.set_transform(tf)

    def _get_relative_objects(self):
        vehicles = self.world.get_actors().filter('vehicle.*')
        ego_tf = self._actor.get_transform()
        ego_loc = ego_tf.location
        yaw = math.radians(ego_tf.rotation.yaw)
        cy, sy = math.cos(-yaw), math.sin(-yaw)

        objs = []
        for v in vehicles:
            if v.id == self.ego_id:
                continue
            loc = v.get_location()
            vel = v.get_velocity()
            dx, dy = loc.x - ego_loc.x, loc.y - ego_loc.y
            rx = dx * cy - dy * sy
            ry = dx * sy + dy * cy
            rvx = vel.x * cy - vel.y * sy
            rvy = vel.x * sy + vel.y * cy
            heading = v.get_transform().rotation.yaw - ego_tf.rotation.yaw
            objs.append({
                'object_id': v.id,
                'position_x': rx,
                'position_y': ry,
                'velocity_x': rvx,
                'velocity_y': rvy,
                'accel_x': 0.0,
                'accel_y': 0.0,
                'distance': math.hypot(rx, ry),
                'heading': heading,
                'object_type': 0,
                'status': 0
            })
        return objs

    def run_step(self):
        timestamp = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0
        if self.sim_start_time is None:
            self.sim_start_time = timestamp
        self.time_data.current_time = timestamp
        elapsed_sec = (timestamp - self.sim_start_time) / 1000.0

        vel = self._actor.get_velocity()
        gps_data = GPSData(vel.x, vel.y, self.gps_sensor_time, self.gps_sensor_time)
        gps_dt = abs(timestamp - self.gps_sensor_time)
        gps_update_enabled = gps_dt <= 50.0

        ego = EGO.ego_vehicle_estimation(self.time_data, gps_data, self.imu_data, self.kf_state)

        # === Waypoint 기반 차선 중심 offset 계산 ===
        waypoint = self.map.get_waypoint(self._actor.get_location(), project_to_road=True,
                                         lane_type=carla.LaneType.Driving)
        lane_center_y = waypoint.transform.location.y
        ego_y = self._actor.get_location().y
        offset = ego_y - lane_center_y

        lane_data = LaneData(
            lane_type=LaneType.STRAIGHT,
            curvature=0.0,
            next_curvature=0.0,
            offset=offset,
            heading=waypoint.transform.rotation.yaw,
            width=waypoint.lane_width,
            change_status=LaneChangeStatus.KEEP
        )

        lane_output = LANE.lane_selection(lane_data, ego)

        obj_list = [convert_dict_to_objectdata(d) for d in self._get_relative_objects()]
        filtered = TS.select_target_from_object_list(obj_list, ego, lane_output)
        predicted = TS.predict_object_future_path(filtered, lane_output)
        acc_target, aeb_target = TS.select_targets_for_acc_aeb(ego, predicted, lane_output)

        acc_mode = ACC.acc_mode_selection(acc_target, ego, lane_output)
        acc_dist = ACC.calculate_accel_for_distance_pid(acc_mode, acc_target, ego, timestamp)
        acc_speed = ACC.calculate_accel_for_speed_pid(ego, lane_output, 0.05)
        acc_accel = ACC.acc_output_selection(acc_mode, acc_dist, acc_speed)

        ttc = AEB.calculate_ttc_for_aeb(aeb_target, ego)
        aeb_mode = AEB.aeb_mode_selection(aeb_target, ego, ttc)
        aeb_accel = AEB.calculate_decel_for_aeb(aeb_mode, ttc)

        lfa_mode = LFA.lfa_mode_selection(ego)
        steer_pid = LFA.calculate_steer_in_low_speed_pid(lane_output, 0.05)
        steer_sta = LFA.calculate_steer_in_high_speed_stanley(ego, lane_output)
        steer_cmd = LFA.lfa_output_selection(lfa_mode, steer_pid, steer_sta, lane_output, ego)

        ctrl = ARB.arbitration(acc_accel, aeb_accel, steer_cmd, aeb_mode)
        self.control.throttle = engine_control.calc_engine_control_command(ctrl['throttle'])
        self.control.brake = brake_control.calc_brake_command(ctrl['brake'])
        self.control.steer = steer_control.calc_steer_command(ctrl['steer'])
        self._actor.apply_control(self.control)

        self._spectator_update()

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

    def reset(self):
        if self.log_data:
            path = Path("C:/Users/MSI-Book/Desktop/log/LANE_S001.csv")
            with open(path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self.log_data[0].keys())
                writer.writeheader()
                writer.writerows(self.log_data)
            print(f"[INFO] Step 로그 저장 완료: {path.name}")
        if self.imu_sensor:
            self.imu_sensor.stop()
            self.imu_sensor.destroy()
        if self.gps_sensor:
            self.gps_sensor.stop()
            self.gps_sensor.destroy()
