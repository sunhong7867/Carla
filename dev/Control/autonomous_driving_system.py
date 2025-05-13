import math
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

# ========== dict → ObjectData 변환 ==========
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
        self.gps_data.timestamp = self.current_time_ms

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
                'object_type': 0,  # CAR
                'status': 0        # MOVING
            })
        return objs

    def run_step(self):
        self.current_time_ms += 50.0
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
<<<<<<< HEAD
        print("======================================")
        print(f"[EGO] Velocity: {ego_data.velocity_x:.2f} m/s | Accel: {ego_data.accel_x:.2f} m/s² | Heading: {ego_data.heading:.2f}°")
        print(f"[ACC] Mode: {acc_mode.name} | ACC Accel: {acc_accel:.2f} m/s²")
        # print(f"[AEB] Mode: {aeb_mode.name} | AEB Accel: {aeb_accel:.2f} m/s²")
        # print(f"[LFA] Mode: {lfa_mode.name} | Steer_PID: {steer_pid:.2f}° | Steer_Stanley: {steer_sta:.2f}° | Final Steer: {steer_cmd:.2f}°")
        #print(f"[CTRL] Throttle: {self.control.throttle:.2f} | Brake: {self.control.brake:.2f} | Steer: {self.control.steer:.2f}")
        #print(f"[Filtered] {len(filtered)} objects")
        #if filtered:
        #    for i, obj in enumerate(filtered):
        #        print(
        #            f"  F{i}: ID={obj.object_id}, dist={obj.distance:.2f}, status={obj.status.name}, cell={obj.cell_id}")
        #print(f"[Predicted] {len(predicted)} objects")
        #print(f"[ACC Target] ID={acc_target.object_id}, Dist={acc_target.distance:.2f}, Mode={acc_mode.name}, Accel={acc_accel:.2f}")
=======
        # 기준 좌표 변환
        ego_tf = self._actor.get_transform()
        ego_loc = ego_tf.location
        front_offset_x = 1.5  # 차량 앞바퀴 중심 오프셋 (예시값)
        ref_x = ego_loc.x - front_offset_x
        ref_y = ego_loc.y
        print(f"[STEP2] ego.get_location() = ({ego_loc.x:.2f}, {ego_loc.y:.2f}) → 기준점 변환 후 ({ref_x:.2f}, {ref_y:.2f})")

>>>>>>> develop

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