import math
import carla
from datetime import datetime

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

# ========== dict → ObjectData 전환 ==========
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
        super(AutonomousDrivingSystem, self).__init__(actor)
        print("======= ADAS Python System Activated =======")

        self.start_time_ms = None
        self.world = CarlaDataProvider.get_world()
        self.map = self.world.get_map()
        self.spectator = self.world.get_spectator()
        self.ego_id = actor.id
        self._actor = actor

        self.control = carla.VehicleControl()
        self.current_time_ms = 0.0

        self.imu_data = IMUData(0.0, 0.0, 0.0)
        self.gps_data = GPSData(0.0, 0.0, 0.0)
        self.kf_state = EGO.init_ego_vehicle_kf_state()

        self._set_imu_sensor()
        self._set_gps_sensor()

    def _set_imu_sensor(self):
        blueprint = self.world.get_blueprint_library().find('sensor.other.imu')
        transform = carla.Transform(carla.Location(x=0, y=0, z=0))
        self._imu_sensor = self.world.spawn_actor(blueprint, transform, attach_to=self._actor)
        self._imu_sensor.listen(lambda data: self._on_imu_update(data))

    def _set_gps_sensor(self):
        blueprint = self.world.get_blueprint_library().find('sensor.other.gnss')
        transform = carla.Transform(carla.Location(x=0, y=0, z=0))
        self._gps_sensor = self.world.spawn_actor(blueprint, transform, attach_to=self._actor)
        self._gps_sensor.listen(lambda data: self._on_gps_update(data))

    def _on_imu_update(self, data):
        self.imu_data.accel_x = data.accelerometer.x
        self.imu_data.accel_y = data.accelerometer.y
        self.imu_data.yaw_rate = data.gyroscope.z

    def _on_gps_update(self, data):
        vel = self._actor.get_velocity()
        self.gps_data.velocity_x = vel.x
        self.gps_data.velocity_y = vel.y
        self.gps_data.timestamp = data.timestamp * 1000.0

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
        if self.start_time_ms is None:
            self.start_time_ms = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0

        self.current_time_ms = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0

        time_data = TimeData(self.current_time_ms)
        ego_data = EGO.ego_vehicle_estimation(time_data, self.gps_data, self.imu_data, self.kf_state)

        lane_data = LaneData(
            lane_type=LaneType.STRAIGHT,
            curvature=0.0,
            next_curvature=0.0,
            offset=0.0,
            heading=ego_data.heading,
            width=3.5,
            change_status=LaneChangeStatus.KEEP
        )
        lane_output = LANE.lane_selection(lane_data, ego_data)

        # 정확한 거리 계산을 위한 앞바퀴 기준 거리 측정
        vehicles = self.world.get_actors().filter("vehicle.*")
        ego_tf = self._actor.get_transform()
        ego_loc = ego_tf.location
        ego_forward = ego_tf.get_forward_vector()
        ego_bbox = self._actor.bounding_box
        ego_front = ego_loc + ego_forward * ego_bbox.extent.x

        min_distance = float('inf')
        closest_vehicle_id = -1

        for v in vehicles:
            if v.id == self.ego_id:
                continue
            tf = v.get_transform()
            loc = tf.location
            forward = tf.get_forward_vector()
            bbox = v.bounding_box
            target_front = loc + forward * bbox.extent.x
            dist = ego_front.distance(target_front)
            if dist < min_distance:
                min_distance = dist
                closest_vehicle_id = v.id

        obj_dict_list = self._get_relative_objects()
        obj_list = [convert_dict_to_objectdata(d) for d in obj_dict_list]

        filtered = TS.select_target_from_object_list(obj_list, ego_data, lane_output)
        predicted = TS.predict_object_future_path(filtered, lane_output)
        acc_target, aeb_target = TS.select_targets_for_acc_aeb(ego_data, predicted, lane_output)

        dt_sec = 0.05
        acc_mode = ACC.acc_mode_selection(acc_target, ego_data, lane_output)
        acc_dist = ACC.calculate_accel_for_distance_pid(acc_mode, acc_target, ego_data, self.current_time_ms)
        acc_speed = ACC.calculate_accel_for_speed_pid(ego_data, lane_output, dt_sec)
        acc_accel = ACC.acc_output_selection(acc_mode, acc_dist, acc_speed)

        ttc = AEB.calculate_ttc_for_aeb(aeb_target, ego_data)
        aeb_mode = AEB.aeb_mode_selection(aeb_target, ego_data, ttc)
        aeb_accel = AEB.calculate_decel_for_aeb(aeb_mode, ttc)

        lfa_mode = LFA.lfa_mode_selection(ego_data)
        steer_pid = LFA.calculate_steer_in_low_speed_pid(lane_output, dt_sec)
        steer_sta = LFA.calculate_steer_in_high_speed_stanley(ego_data, lane_output)
        steer_cmd = LFA.lfa_output_selection(lfa_mode, steer_pid, steer_sta, lane_output, ego_data)

        ctrl = ARB.arbitration(acc_accel, aeb_accel, steer_cmd, aeb_mode)
        self.control.throttle = engine_control.calc_engine_control_command(ctrl['throttle'])
        self.control.brake = brake_control.calc_brake_command(ctrl['brake'])
        self.control.steer = steer_control.calc_steer_command(ctrl['steer'])

        self._actor.apply_control(self.control)
        self._update_spectator()

        # === Debugging Print ===
        elapsed_sec = int((self.current_time_ms - self.start_time_ms) // 1000)
        gps_dt = abs(self.current_time_ms - self.gps_data.timestamp)
        gt_vel = self._actor.get_velocity().x
        vel_err = abs(gt_vel - ego_data.velocity_x)
        imu_valid = abs(self.imu_data.accel_x) < 9.8
        gps_valid = gps_dt <= 50.0

        print(f"[STEP1][{elapsed_sec} sec] 정속 주행 확인 → Velocity = {ego_data.velocity_x:.2f} m/s")

    def reset(self):
        if hasattr(self, "_imu_sensor"):
            self._imu_sensor.stop()
            self._imu_sensor.destroy()
        if hasattr(self, "_gps_sensor"):
            self._gps_sensor.stop()
            self._gps_sensor.destroy()
        self.kf_state = EGO.init_ego_vehicle_kf_state()
        self.current_time_ms = 0.0

    def _update_spectator(self):
        tf = self._actor.get_transform()
        tf.location.x -= 10
        tf.location.z += 3
        self.spectator.set_transform(tf)
