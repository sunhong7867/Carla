# target_selection.py
# Python version of target_selection module
# Implements:
# - select_target_from_object_list
# - predict_object_future_path
# - select_targets_for_acc_aeb

import math
from decision.shared_types import (
    ObjectData, ObjectStatus, ObjectType, TargetSituation,
    FilteredObject, PredictedObject, EgoData, LaneSelectOutput,
    ACCTarget, AEBTarget
)
from typing import List, Tuple

# ===== 유틸리티 =====
def normalize_heading(hdg):
    while hdg > 180.0:
        hdg -= 360.0
    while hdg < -180.0:
        hdg += 360.0
    return hdg

# ===== 함수 1: select_target_from_object_list =====
def select_target_from_object_list(obj_list: List[ObjectData],
                                   ego_data: EgoData,
                                   lane_info: LaneSelectOutput) -> List[FilteredObject]:
    filtered_list = []
    if not obj_list or not ego_data or not lane_info:
        return filtered_list

    heading_coeff = 0.05
    lateral_eps = 1e-3
    adjusted_threshold = lane_info.lane_width * 0.5
    if lane_info.is_curved_lane and abs(lane_info.heading_error) > 1.0:
        adjusted_threshold += abs(lane_info.heading_error) * heading_coeff

    for obj in obj_list:
        if obj.distance > 200:
            continue

        lateral_pos = obj.position_y - lane_info.lane_offset
        center_offset = abs(lateral_pos)

        quarter_w = lane_info.lane_width * 0.25
        three_qw = lane_info.lane_width * 0.75

        if center_offset > (adjusted_threshold + lateral_eps):
            continue
        if (center_offset > (lane_info.lane_width * 0.5 + lateral_eps)) and (center_offset < (three_qw - lateral_eps)):
            continue

        rel_vel = obj.velocity_x - ego_data.velocity_x
        heading_diff = abs(obj.heading - ego_data.heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff

        status = obj.status
        if heading_diff >= 150:
            status = ObjectStatus.ONCOMING
        elif abs(rel_vel) >= 0.5:
            status = ObjectStatus.MOVING
        else:
            status = ObjectStatus.STATIONARY

        adjusted_distance = obj.distance
        if lane_info.is_curved_lane:
            heading_rad = lane_info.heading_error * math.pi / 180.0
            cos_val = math.cos(heading_rad)
            if abs(cos_val) > 1.0e-3:
                adjusted_distance /= cos_val

        if adjusted_distance <= 60:
            base_cell = 1 + int(adjusted_distance / 10.0)
            base_cell = min(base_cell, 6)
        elif adjusted_distance < 120:
            base_cell = 7 + int((adjusted_distance - 60) / 10.0)
            base_cell = min(base_cell, 12)
        else:
            base_cell = 13 + int((adjusted_distance - 120) / 10.0)

        offset_adjustment = 0
        if center_offset <= quarter_w:
            offset_adjustment = -1
        elif center_offset >= three_qw:
            offset_adjustment = 1

        cell_id = base_cell + offset_adjustment
        cell_id = max(1, min(20, cell_id))

        filtered_list.append(FilteredObject(
            object_id=obj.object_id,
            object_type=obj.object_type,
            position_x=obj.position_x,
            position_y=obj.position_y,
            position_z=obj.position_z,
            velocity_x=obj.velocity_x,
            velocity_y=obj.velocity_y,
            accel_x=obj.accel_x,
            accel_y=obj.accel_y,
            heading=normalize_heading(obj.heading),
            distance=adjusted_distance,
            status=status,
            cell_id=cell_id
        ))

    return filtered_list

# ===== 함수 2: predict_object_future_path =====
def predict_object_future_path(filtered_list: List[FilteredObject],
                               lane_info: LaneSelectOutput) -> List[PredictedObject]:
    predicted_list = []
    t = 3.0  # seconds

    for fo in filtered_list:
        vx, vy, ax, ay = fo.velocity_x, fo.velocity_y, fo.accel_x, fo.accel_y
        x0, y0 = fo.position_x, fo.position_y

        if fo.status == ObjectStatus.MOVING:
            px = x0 + vx * t
            py = y0 + vy * t
        else:
            px = x0 + vx * t + 0.5 * ax * t * t
            py = y0 + vy * t + 0.5 * ay * t * t

        dist = math.sqrt(px ** 2 + py ** 2)
        lateral_pos = py - lane_info.lane_offset
        cutin = (vx >= 0.5 and abs(vy) >= 0.2 and abs(lateral_pos) <= 0.85)
        cutout = (abs(vy) >= 0.2 and abs(lateral_pos) > lane_info.lane_width * 0.5 + 0.85)

        predicted_list.append(PredictedObject(
            object_id=fo.object_id,
            object_type=fo.object_type,
            position_x=px,
            position_y=py,
            position_z=fo.position_z,
            velocity_x=vx,
            velocity_y=vy,
            accel_x=ax,
            accel_y=ay,
            heading=fo.heading,
            distance=dist,
            status=fo.status,
            cell_id=fo.cell_id,
            cutin=cutin,
            cutout=cutout
        ))

    return predicted_list

# ===== 함수 3: select_targets_for_acc_aeb =====
def select_targets_for_acc_aeb(ego_data: EgoData,
                               pred_list: List[PredictedObject],
                               lane_info: LaneSelectOutput) -> Tuple[ACCTarget, AEBTarget]:
    acc_target = ACCTarget(
        object_id=-1, position_x=0, position_y=0,
        velocity_x=0, velocity_y=0, accel_x=0, accel_y=0,
        distance=0, heading=0, status=ObjectStatus.STATIONARY,
        situation=TargetSituation.NORMAL
    )

    aeb_target = AEBTarget(
        object_id=-1, position_x=0, position_y=0,
        velocity_x=0, velocity_y=0, accel_x=0, accel_y=0,
        distance=0, heading=0, status=ObjectStatus.STATIONARY,
        situation=TargetSituation.NORMAL
    )

    best_acc_score = -1e9
    best_aeb_score = -1e9
    best_acc = None
    best_aeb = None
    brake_status = abs(ego_data.velocity_x) < 0.1

    for obj in pred_list:
        if obj.cutout or obj.position_x < 0:
            continue

        py = obj.position_y

        # ACC 후보
        if abs(py) <= 1.75 and obj.object_type == ObjectType.CAR and obj.status in [ObjectStatus.MOVING, ObjectStatus.STOPPED]:
            score = 200 - obj.distance
            if lane_info.is_curved_lane and obj.cell_id < 5:
                score += 10
            if score > best_acc_score:
                best_acc_score = score
                best_acc = obj

        # AEB 후보
        is_front = abs(py) <= 1.75
        is_side = 1.75 < abs(py) <= 3.5
        aeb_candidate = False

        if is_front:
            if obj.status in [ObjectStatus.MOVING, ObjectStatus.STOPPED]:
                aeb_candidate = True
            elif obj.status == ObjectStatus.STATIONARY and brake_status:
                aeb_candidate = True
        elif is_side and obj.cutin:
            aeb_candidate = True

        if aeb_candidate:
            rel_speed = ego_data.velocity_x - obj.velocity_x
            ttc = obj.distance / rel_speed if rel_speed > 0.1 else 1e9
            score = 200 - obj.distance
            if obj.cutin:
                score += 30
            if ttc < 3:
                score += 20
            if score > best_aeb_score:
                best_aeb_score = score
                best_aeb = obj

    if best_acc:
        acc_target = ACCTarget(
            object_id=best_acc.object_id,
            position_x=best_acc.position_x,
            position_y=best_acc.position_y,
            velocity_x=best_acc.velocity_x,
            velocity_y=best_acc.velocity_y,
            accel_x=best_acc.accel_x,
            accel_y=best_acc.accel_y,
            distance=best_acc.distance,
            heading=best_acc.heading,
            status=best_acc.status,
            situation=TargetSituation.CUTIN if best_acc.cutin else TargetSituation.CURVE if lane_info.is_curved_lane else TargetSituation.NORMAL
        )

    if best_aeb:
        aeb_target = AEBTarget(
            object_id=best_aeb.object_id,
            position_x=best_aeb.position_x,
            position_y=best_aeb.position_y,
            velocity_x=best_aeb.velocity_x,
            velocity_y=best_aeb.velocity_y,
            accel_x=best_aeb.accel_x,
            accel_y=best_aeb.accel_y,
            distance=best_aeb.distance,
            heading=best_aeb.heading,
            status=best_aeb.status,
            situation=TargetSituation.CUTIN if best_aeb.cutin else TargetSituation.CURVE if lane_info.is_curved_lane else TargetSituation.NORMAL
        )

    return acc_target, aeb_target
