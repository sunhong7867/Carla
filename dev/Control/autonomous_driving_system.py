# autonomous_driving_system.py

import math
import numpy as np
import carla

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from decision import adaptive_cruise_control
from decision import autonomous_emergency_brake
from decision import ego_vehicle_estimation
from decision import lane_following_assist
from decision import target_selection
from decision import arbitration

from controller import brake_control
from controller import steer_control
from controller import engine_control


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
        self.imu_acceleration = None
        self.gps_velocity = None
        self.gps_location = None
        self.current_time_ms = 0.0

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
        self.imu_acceleration = data.accelerometer

    def _on_gps_update(self, data):
        self.gps_location = (data.latitude, data.longitude)
        self.gps_velocity = self._actor.get_velocity()

    def _get_lane_equations(self):
        ego_loc = self._actor.get_location()
        ego_wpt = self.map.get_waypoint(ego_loc, project_to_road=True)

        def sample_waypoints(wpt):
            samples = []
            for _ in range(20):
                loc = wpt.transform.location
                samples.append((loc.x, loc.y))
                nexts = wpt.next(5.0)
                if not nexts:
                    break
                wpt = nexts[0]
            return samples

        def fit(points):
            xs, ys = zip(*points)
            return np.polyfit(xs, ys, 1) if len(points) >= 2 else (1.0, 1.0)

        lane_info = []
        for side in ['get_left_lane', None, 'get_right_lane']:
            target = getattr(ego_wpt, side)() if side else ego_wpt
            fit_line = fit(sample_waypoints(target)) if target else (1.0, 1.0)
            lane_info.append(fit_line)

        return lane_info[0], lane_info[2]  # left, right

    def _get_relative_objects(self):
        vehicles = self.world.get_actors().filter('vehicle.*')
        ego_tf = self._actor.get_transform()
        ego_loc = ego_tf.location
        ego_yaw = math.radians(ego_tf.rotation.yaw)
        cos_yaw, sin_yaw = math.cos(-ego_yaw), math.sin(-ego_yaw)

        objects = {}
        for veh in vehicles:
            if veh.id == self.ego_id:
                continue
            loc = veh.get_location()
            vel = veh.get_velocity()
            dx, dy = loc.x - ego_loc.x, loc.y - ego_loc.y
            rel_x = dx * cos_yaw - dy * sin_yaw
            rel_y = dx * sin_yaw + dy * cos_yaw
            rel_vx = vel.x * cos_yaw - vel.y * sin_yaw
            rel_vy = vel.x * sin_yaw + vel.y * cos_yaw
            objects[veh.id] = (rel_x, rel_y, rel_vx, rel_vy)
        return objects

    def run_step(self):
        self.current_time_ms += 50  # 시뮬레이션용 시간 tick

        ego_velocity = ego_vehicle_estimation.ego_vehicle_estimation(self.imu_acceleration)
        lane_left, lane_right = self._get_lane_equations()
        lane_info = [lane_left, lane_right]
        object_list = self._get_relative_objects()

        # Target Selection
        target_prec = target_selection.target_selection_preceding(object_list, lane_info, ego_velocity)
        target_inter = target_selection.target_selection_intersection(object_list, ego_velocity)

        # ACC / AEB / LFA
        acc_accel = adaptive_cruise_control.adaptive_cruise_control(target_prec, ego_velocity)
        aeb_accel = autonomous_emergency_brake.autonomous_emergency_brake(target_prec, target_inter, ego_velocity)
        steer_cmd = lane_following_assist.lane_following_assist_pid(lane_info, ego_velocity)

        # Arbitration
        final_accel, final_steer = arbitration.arbitration(acc_accel, aeb_accel, steer_cmd)

        # Control Output
        self.control.throttle = engine_control.calc_engine_control_command(final_accel)
        self.control.brake = brake_control.calc_brake_command(final_accel)
        self.control.steer = steer_control.calc_steer_command(final_steer)
        self._actor.apply_control(self.control)

        self._update_spectator()

    def _update_spectator(self):
        tf = self._actor.get_transform()
        tf.location.x -= 10
        tf.location.z += 3
        self.spectator.set_transform(tf)
