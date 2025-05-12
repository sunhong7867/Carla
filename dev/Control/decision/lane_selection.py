# lane_selection.py
# Python version of Lane Selection
# - Input: LaneData, EgoData (from shared_types)
# - Output: LaneSelectOutput

from decision.shared_types import LaneData, EgoData, LaneSelectOutput, LaneChangeStatus

# ===== 상수 정의 =====
LANE_CURVE_THRESHOLD = 800.0
LANE_CURVE_DIFF_THRESHOLD = 200.0

# ===== Lane Selection 함수 =====
def lane_selection(lane_data: LaneData, ego_data: EgoData) -> LaneSelectOutput:
    if lane_data is None or ego_data is None:
        raise ValueError("Invalid input data")

    out = LaneSelectOutput(
        lane_type=lane_data.lane_type,
        is_curved_lane=False,
        curve_transition_flag=False,
        heading_error=0.0,
        lane_offset=0.0,
        lane_width=0.0,
        is_within_lane=False,
        is_changing_lane=False
    )

    # 1) 차선 유형 판단 및 곡선 여부
    out.lane_type = lane_data.lane_type
    out.is_curved_lane = (0.0 < lane_data.curvature < LANE_CURVE_THRESHOLD)

    if lane_data.curvature > 0.0 and lane_data.next_curvature > 0.0:
        curvature_diff = abs(lane_data.next_curvature - lane_data.curvature)
        out.curve_transition_flag = curvature_diff > LANE_CURVE_DIFF_THRESHOLD
    else:
        out.curve_transition_flag = False

    # 2) Heading 오차 계산
    heading_diff = ego_data.heading - lane_data.heading
    while heading_diff > 180.0:
        heading_diff -= 360.0
    while heading_diff < -180.0:
        heading_diff += 360.0
    out.heading_error = heading_diff

    # 3) 차선 중심 오차 및 차로 내부 여부
    out.lane_offset = lane_data.offset
    out.lane_width = lane_data.width
    threshold = lane_data.width * 0.5
    out.is_within_lane = abs(lane_data.offset) < threshold

    # 4) 차로 변경 여부
    out.is_changing_lane = lane_data.change_status != LaneChangeStatus.KEEP

    return out
