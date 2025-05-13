import numpy as np

from dataclasses import dataclass
from enum import Enum

# ===== ENUM 정의 =====
class LaneType(Enum):
    STRAIGHT = 0
    CURVE = 1

class LaneChangeStatus(Enum):
    KEEP = 0
    CHANGING = 1
    DONE = 2

class ObjectType(Enum):
    CAR = 0
    PEDESTRIAN = 1
    BICYCLE = 2
    MOTORCYCLE = 3

class ObjectStatus(Enum):
    MOVING = 0
    STOPPED = 1
    STATIONARY = 2
    ONCOMING = 3

class TargetSituation(Enum):
    NORMAL = 0
    CUTIN = 1
    CUTOUT = 2
    CURVE = 3

class ACCMode(Enum):
    SPEED = 0
    DISTANCE = 1
    STOP = 2

class AEBMode(Enum):
    NORMAL = 0
    ALERT = 1
    BRAKE = 2

class LFAMode(Enum):
    LOW_SPEED = 0
    HIGH_SPEED = 1

# ===== STRUCT 정의 =====
@dataclass
class TimeData:
    current_time: float

@dataclass
class GPSData:
    velocity_x: float
    velocity_y: float
    timestamp: float
    last_received_time_ms: float = 0.0  # ← 추가

@dataclass
class IMUData:
    accel_x: float
    accel_y: float
    yaw_rate: float

@dataclass
class EgoData:
    velocity_x: float
    velocity_y: float
    accel_x: float
    accel_y: float
    heading: float
    yaw_rate: float
    position_x: float
    position_y: float
    position_z: float

@dataclass
class EgoVehicleKFState:
    last_gps_velocity_x: float = 0.0
    last_gps_velocity_y: float = 0.0
    last_gps_timestamp: float = 0.0

    previous_update_time: float = 0.0
    prev_accel_x: float = 0.0
    prev_accel_y: float = 0.0
    prev_yaw_rate: float = 0.0
    prev_gps_vel_x: float = 0.0
    prev_gps_vel_y: float = 0.0

    gps_update_enabled: bool = False

    X: np.ndarray = np.zeros(5)
    P: np.ndarray = np.eye(5) * 100.0

@dataclass
class LaneData:
    lane_type: LaneType
    curvature: float
    next_curvature: float
    offset: float
    heading: float
    width: float
    change_status: LaneChangeStatus

@dataclass
class LaneSelectOutput:
    lane_type: LaneType
    is_curved_lane: bool
    curve_transition_flag: bool
    heading_error: float
    lane_offset: float
    lane_width: float
    is_within_lane: bool
    is_changing_lane: bool

@dataclass
class ObjectData:
    object_id: int
    object_type: ObjectType
    position_x: float
    position_y: float
    position_z: float
    velocity_x: float
    velocity_y: float
    accel_x: float
    accel_y: float
    heading: float
    distance: float
    status: ObjectStatus
    cell_id: int

@dataclass
class FilteredObject(ObjectData):
    pass

@dataclass
class PredictedObject(ObjectData):
    cutin: bool = False
    cutout: bool = False

@dataclass
class ACCTarget:
    object_id: int
    position_x: float
    position_y: float
    velocity_x: float
    velocity_y: float
    accel_x: float
    accel_y: float
    distance: float
    heading: float
    status: ObjectStatus
    situation: TargetSituation

@dataclass
class AEBTarget:
    object_id: int
    position_x: float
    position_y: float
    velocity_x: float
    velocity_y: float
    accel_x: float
    accel_y: float
    distance: float
    heading: float
    status: ObjectStatus
    situation: TargetSituation

@dataclass
class VehicleControl:
    throttle: float
    brake: float
    steer: float

# ===== 상수 정의 =====
LANE_CURVE_THRESHOLD = 800.0
LANE_CURVE_DIFF_THRESHOLD = 400.0
MAX_OBJECT_DISTANCE = 200.0
LATERAL_THRESHOLD = 4.0

MAX_ACCEL = 10.0
MIN_ACCEL = -10.0

AEB_MAX_BRAKE_DECEL = -10.0
AEB_MIN_BRAKE_DECEL = -2.0
AEB_ALERT_BUFFER_TIME = 1.2
AEB_DEFAULT_MAX_DECEL = 9.0

LFA_LOW_SPEED_THRESHOLD = 16.67
LFA_MAX_STEERING_ANGLE = 540.0
