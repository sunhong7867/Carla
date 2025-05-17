#!/usr/bin/env python

import math
import carla

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from decision import ego_vehicle_estimation as EGO
from decision.shared_types import TimeData, GPSData, IMUData, EgoVehicleKFState


class EgoEstimationSystem(BasicControl):
    def __init__(self, actor, args=None):
        super(EgoEstimationSystem, self).__init__(actor)
        print("========== Ego Estimation Scenario Start ==========")

        self._init_control()
        self.world = CarlaDataProvider.get_world()
        self.map = self.world.get_map()
        self.ego_id = actor.id
        self._actor = actor
        self.spectator = self.world.get_spectator()

        self.kf_state = EGO.init_ego_vehicle_kf_state()
        self.imu_data = IMUData(0.0, 0.0, 0.0)
        self.prev_timestamp = 0.0

        self.last_imu_time = 0.0
        self.imu_intervals = []

        self.last_gps_time = 0.0
        self.gps_intervals = []

        self._set_imu_sensor()
        self._set_gps_sensor()

        self.gps_sensor_time = 0.0
        self.gps_updated = False


    def _init_control(self):
        self.control = carla.VehicleControl()
        self.control.steer = 0.0
        self.control.throttle = 0.0
        self.control.brake = 0.0

    def _set_imu_sensor(self):
        bp = self.world.get_blueprint_library().find('sensor.other.imu')
        imu_transform = carla.Transform(carla.Location(), carla.Rotation())
        self._imu_sensor = self.world.spawn_actor(bp, imu_transform, attach_to=self._actor)
        self._imu_sensor.listen(lambda data: self._on_imu_update(data))

    def _set_gps_sensor(self):
        bp = self.world.get_blueprint_library().find('sensor.other.gnss')
        bp.set_attribute('sensor_tick', '0.05')
        transform = carla.Transform(carla.Location(), carla.Rotation())
        self._gps_sensor = self.world.spawn_actor(bp, transform, attach_to=self._actor)
        self._gps_sensor.listen(lambda data: self._on_gps_update(data))

    def _on_imu_update(self, data):
        now = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0  # ms
        if self.last_imu_time > 0:
            interval = now - self.last_imu_time
            self.imu_intervals.append(interval)
        self.last_imu_time = now

        self.imu_data = IMUData(
            accel_x=data.accelerometer.x,
            accel_y=data.accelerometer.y,
            yaw_rate=math.degrees(data.gyroscope.z)
        )

    def _on_gps_update(self, data):
        now = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0
        if self.last_gps_time > 0:
            interval = now - self.last_gps_time
            self.gps_intervals.append(interval)
        self.last_gps_time = now

        # GPS 센서에서 직접 제공하는 시각 사용 (센서 자체가 생성한 시간)
        self.gps_sensor_time = data.timestamp * 1000.0

    def _spectator_update(self):
        transform = self._actor.get_transform()
        transform.location.x -= 10
        transform.location.z += 5
        self.spectator.set_transform(transform)

    def run_step(self):
        timestamp = self.world.get_snapshot().timestamp.elapsed_seconds * 1000.0  # ms
        time_data = TimeData(current_time=timestamp)

        if self.prev_timestamp == 0.0:
            self.sim_start_time = timestamp

        elapsed_sec = (timestamp - self.sim_start_time) / 1000.0
        self.prev_timestamp = timestamp

        vel = self._actor.get_velocity()  # ← 반드시 먼저 정의!

        gps_data = GPSData(
            velocity_x=vel.x,
            velocity_y=vel.y,
            timestamp=self.gps_sensor_time,  # 반드시 센서 콜백 기반 시간
            last_received_time_ms=self.gps_sensor_time
        )

        # 이 부분은 sensor time 기준으로 gps_dt 계산
        gps_dt = abs(timestamp - self.gps_sensor_time)
        gps_update_enabled = gps_dt <= 50.0

        ego = EGO.ego_vehicle_estimation(
            time_data=time_data,
            gps_data=gps_data,
            imu_data=self.imu_data,
            kf=self.kf_state
        )

        # STEP 3: Kalman 추정 속도 vs Ground Truth 비교
        gt_vel = self._actor.get_velocity()
        gt_speed = math.sqrt(gt_vel.x ** 2 + gt_vel.y ** 2 + gt_vel.z ** 2)
        velocity_error = abs(ego.velocity_x - gt_speed)
        print(f"[STEP3][{elapsed_sec:.2f}s] Ego_Speed = {ego.velocity_x:.2f} m/s, "
              f"Carla_Speed = {gt_speed:.2f} m/s, Error = {velocity_error:.2f} m/s")




        # 차량은 계속 가속하여 속도 유지
        gt_speed = math.sqrt(gt_vel.x ** 2 + gt_vel.y ** 2 + gt_vel.z ** 2)

        if gt_speed < 15.0:
            self.control.throttle = 0.8  # 가속
            self.control.brake = 0.0
        else:
            self.control.throttle = 0.6  # 유지
            self.control.brake = 0.0

        self._actor.apply_control(self.control)
        self._spectator_update()


    def reset(self):
        if self._actor and self._actor.is_alive:
            self._actor = None
        if self._imu_sensor:
            self._imu_sensor.stop()
            self._imu_sensor.destroy()
