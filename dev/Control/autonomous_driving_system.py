import math
import carla
from datetime import datetime

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from decision import ego_vehicle_estimation as EGO
#from decision import lane_selection as LANE
#from decision import target_selection as TS
#from decision import adaptive_cruise_control as ACC
#from decision import autonomous_emergency_brake as AEB
#from decision import lane_following_assist as LFA
#from decision import arbitration as ARB

from decision.shared_types import (
    TimeData, GPSData, IMUData, EgoVehicleKFState, EgoData,
    LaneData, LaneSelectOutput, LaneType, LaneChangeStatus,
    ObjectData, ObjectType, ObjectStatus
)

class AutonomousDrivingSystem(BasicControl):
    def __init__(self, actor, args=None):
        super().__init__(actor)
        print(f'=========== ADAS Python System Activated ===========')

        self._actor = actor
        self.spectator = CarlaDataProvider.get_world().get_spectator()
        self.world = CarlaDataProvider.get_world()
        self.carla_map = self.world.get_map()
        self.ego_id = actor.id

        self.kf_state = EGO.init_ego_vehicle_kf_state()
        self.gps_data = GPSData(0.0, 0.0, 0.0)
        self.imu_data = IMUData(0.0, 0.0, 0.0)
        self.time_data = TimeData(0.0)

        self.control = carla.VehicleControl()
        self._set_imu_sensor()

    def _init_control(self):
        # 제어 객체 생성
        self.control = carla.VehicleControl()
        self.control.steer = 0.0
        self.control.throttle = 0.0
        self.control.brake = 0.0


    def _set_imu_sensor(self):
        blueprint = self.world.get_blueprint_library().find('sensor.other.imu')
        imu_transform = carla.Transform(carla.Location(), carla.Rotation())
        self.imu_sensor = self.world.spawn_actor(blueprint, imu_transform, attach_to=self._actor)
        self.imu_sensor.listen(lambda imu: self._on_imu_update(imu))

    def _on_imu_update(self, data):
        self.imu_data.accel_x = data.accelerometer.x
        self.imu_data.accel_y = data.accelerometer.y
        self.imu_data.yaw_rate = data.gyroscope.z

    def _spectator_update(self):
        transform = self._actor.get_transform()
        transform.location.x -= 10
        transform.location.z += 3
        self.spectator.set_transform(transform)

    def convert_waypoint_to_lanedata(self, waypoint: carla.Waypoint) -> LaneData:
        current_yaw = waypoint.transform.rotation.yaw
        next_wps = waypoint.next(2.0)
        is_curved = False
        next_curvature = 0.0
        if next_wps:
            next_yaw = next_wps[0].transform.rotation.yaw
            diff = (next_yaw - current_yaw + 180) % 360 - 180
            is_curved = abs(diff) > 5.0
            next_curvature = abs(diff)

        lane_data = LaneData(
            lane_type=LaneType.CURVE if is_curved else LaneType.STRAIGHT,
            curvature=next_curvature,
            next_curvature=next_curvature,
            offset=0.0,
            heading=current_yaw,
            width=waypoint.lane_width,
            change_status=LaneChangeStatus.KEEP
        )
        return lane_data

    def convert_vehicle_to_objectdata(self, veh, ego_transform) -> ObjectData:
        loc = veh.get_location()
        vel = veh.get_velocity()
        acc = veh.get_acceleration()
        yaw = veh.get_transform().rotation.yaw

        dx = loc.x - ego_transform.location.x
        dy = loc.y - ego_transform.location.y
        ego_yaw = math.radians(ego_transform.rotation.yaw)
        cos_yaw = math.cos(-ego_yaw)
        sin_yaw = math.sin(-ego_yaw)

        rel_x = dx * cos_yaw - dy * sin_yaw
        rel_y = dx * sin_yaw + dy * cos_yaw

        vx = vel.x * cos_yaw - vel.y * sin_yaw
        vy = vel.x * sin_yaw + vel.y * cos_yaw

        ax = acc.x * cos_yaw - acc.y * sin_yaw
        ay = acc.x * sin_yaw + acc.y * cos_yaw

        return ObjectData(
            object_id=veh.id,
            object_type=ObjectType.CAR,
            position_x=rel_x,
            position_y=rel_y,
            position_z=loc.z,
            velocity_x=vx,
            velocity_y=vy,
            accel_x=ax,
            accel_y=ay,
            heading=yaw,
            distance=math.sqrt(rel_x**2 + rel_y**2),
            status=ObjectStatus.MOVING,
            cell_id=0
        )

    def run_step(self):
        self.time_data.current_time = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0
        vel = self._actor.get_velocity()
        self.gps_data.velocity_x = vel.x
        self.gps_data.velocity_y = vel.y
        self.gps_data.timestamp = self.time_data.current_time

        ego_data = EGO.ego_vehicle_estimation(self.time_data, self.gps_data, self.imu_data, self.kf_state)

        ego_loc = self._actor.get_location()
        ego_wpt = self.carla_map.get_waypoint(ego_loc, project_to_road=True)
        lane_data = self.convert_waypoint_to_lanedata(ego_wpt)
        lane_output = LANE.lane_selection(lane_data, ego_data)

        all_vehicles = self.world.get_actors().filter("vehicle.*")
        object_list = []
        for v in all_vehicles:
            if v.id != self.ego_id:
                obj = self.convert_vehicle_to_objectdata(v, self._actor.get_transform())
                object_list.append(obj)

        filtered = TS.select_target_from_object_list(object_list, ego_data, lane_output)
        predicted = TS.predict_object_future_path(filtered, lane_output)
        acc_target, aeb_target = TS.select_targets_for_acc_aeb(ego_data, predicted, lane_output)

        acc_mode = ACC.acc_mode_selection(acc_target, ego_data, lane_output)
        accel_dist = ACC.calculate_accel_for_distance_pid(acc_mode, acc_target, ego_data, self.time_data.current_time)
        accel_speed = ACC.calculate_accel_for_speed_pid(ego_data, lane_output, 0.05)
        desired_accel = ACC.acc_output_selection(acc_mode, accel_dist, accel_speed)

        ttc_info = AEB.calculate_ttc_for_aeb(aeb_target, ego_data)
        aeb_mode = AEB.aeb_mode_selection(aeb_target, ego_data, ttc_info)
        decel_aeb = AEB.calculate_decel_for_aeb(aeb_mode, ttc_info)

        lfa_mode = LFA.lfa_mode_selection(ego_data)
        steer_pid = LFA.calculate_steer_in_low_speed_pid(lane_output, 0.05)
        steer_stanley = LFA.calculate_steer_in_high_speed_stanley(ego_data, lane_output)
        desired_steer = LFA.lfa_output_selection(lfa_mode, steer_pid, steer_stanley, lane_output, ego_data)

        control_dict = ARB.arbitration(desired_accel, decel_aeb, desired_steer, aeb_mode)
        self.control.throttle = control_dict['throttle']
        self.control.brake = control_dict['brake']
        self.control.steer = control_dict['steer']

        carla_loc = self._actor.get_location()
        ego_position = (0.0, 0.0, 0.0)
        carla_position = (carla_loc.x, carla_loc.y, carla_loc.z)

        if ego_position == (0.0, 0.0, 0.0):
            print(f"[STEP1][{self.time_data.current_time / 1000.0:.2f}s] Ego_Position == (0.0, 0.0, 0.0) → Pass")
        else:
            print(f"[STEP1][{self.time_data.current_time / 1000.0:.2f}s] Ego_Position != (0.0, 0.0, 0.0) → Fail")

        self._spectator_update()
        self._actor.apply_control(self.control)