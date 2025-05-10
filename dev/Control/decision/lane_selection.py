from dataclasses import dataclass
from enum import Enum
import math

# ===============================
# enum, dataclass 정의 (adas_shared 기반)
# ===============================
class LaneType(Enum):
    STRAIGHT = 0
    CURVED = 1
    INTERSECTION = 2
    CROSSWALK = 3

class LaneChangeStatus(Enum):
    KEEP = 0
    CHANGING = 1
    CHANGED = 2

@dataclass
class LaneData:
    lane_type: LaneType
    lane_offset: float
    lane_position: float
    lane_curvature: float
    next_lane_curvature: float
    lane_heading: float
    lane_width: float
    lane_change_status: LaneChangeStatus

@dataclass
class EgoData:
    ego_heading: float
    ego_position_x: float = 0.0
    ego_position_y: float = 0.0
    ego_position_z: float = 0.0

@dataclass
class LaneSelectOutput:
    ls_lane_type: LaneType = LaneType.STRAIGHT
    ls_is_curved_lane: bool = False
    ls_curve_transition_flag: bool = False
    ls_heading_error: float = 0.0
    ls_lane_offset: float = 0.0
    ls_lane_width: float = 0.0
    ls_is_within_lane: bool = False
    ls_is_changing_lane: bool = False

# ===============================
# 상수 정의 (설계서 기반)
# ===============================
LANE_CURVE_THRESHOLD = 800.0
LANE_CURVE_DIFF_THRESHOLD = 200.0

# ===============================
# Lane Selection 함수
# ===============================
def lane_selection(lane_data: LaneData, ego_data: EgoData) -> LaneSelectOutput:
    if lane_data is None or ego_data is None:
        raise ValueError("Invalid input data")

    out = LaneSelectOutput()

    # 1) 차선 유형 판단 및 곡선 여부
    out.ls_lane_type = lane_data.lane_type

    if 0.0 < lane_data.lane_curvature < LANE_CURVE_THRESHOLD:
        out.ls_is_curved_lane = True
    else:
        out.ls_is_curved_lane = False

    if lane_data.lane_curvature > 0.0 and lane_data.next_lane_curvature > 0.0:
        curvature_diff = abs(lane_data.next_lane_curvature - lane_data.lane_curvature)
        out.ls_curve_transition_flag = curvature_diff > LANE_CURVE_DIFF_THRESHOLD
    else:
        out.ls_curve_transition_flag = False

    # 2) Heading 오차 계산
    heading_diff = ego_data.ego_heading - lane_data.lane_heading
    while heading_diff > 180.0:
        heading_diff -= 360.0
    while heading_diff < -180.0:
        heading_diff += 360.0
    out.ls_heading_error = heading_diff

    # 3) 차선 중심 오차 및 차로 내부 여부
    out.ls_lane_offset = lane_data.lane_offset
    out.ls_lane_width = lane_data.lane_width
    threshold = lane_data.lane_width * 0.5
    out.ls_is_within_lane = abs(lane_data.lane_offset) < threshold

    # 4) 차로 변경 여부
    out.ls_is_changing_lane = lane_data.lane_change_status != LaneChangeStatus.KEEP

    return out
