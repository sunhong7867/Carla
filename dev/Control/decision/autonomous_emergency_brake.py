# aeb.py
# Python version of AEB module implementing:
# - calculate_ttc_for_aeb
# - aeb_mode_selection
# - calculate_decel_for_aeb

import math
from shared_types import EgoData, AEBTarget, AEBMode, TargetSituation

# ===== CONSTANTS =====
INF_TTC = 99999.0
MIN_DIST = 0.01
AEB_MAX_DECEL = -10.0
AEB_MIN_DECEL = -2.0
AEB_DEFAULT_MAX_DECEL = 9.0
AEB_ALERT_BUFFER_TIME = 1.2  # shared_types에도 동일 상수 존재

# ===== FUNCTION 1: calculate_ttc_for_aeb =====
def calculate_ttc_for_aeb(target: AEBTarget, ego: EgoData) -> dict:
    if target is None or ego is None:
        return _ttc_dict(INF_TTC, 0.0, 0.0, 0.0)

    if not math.isfinite(ego.velocity_x) or ego.velocity_x < 0.0:
        return _ttc_dict(INF_TTC, 0.0, 0.0, 0.0)

    if target.object_id < 0 or target.situation == TargetSituation.CUTOUT:
        return _ttc_dict(INF_TTC, 0.0, 0.0, 0.0)

    rel_speed = ego.velocity_x - target.velocity_x
    if not math.isfinite(rel_speed) or rel_speed <= 0.0:
        return _ttc_dict(INF_TTC, 0.0, 0.0, 0.0)

    rel_speed = round(rel_speed, 2)
    if rel_speed < 1e-6:
        return _ttc_dict(INF_TTC, 0.0, 0.0, 0.0)

    dist = target.distance
    if not math.isfinite(dist):
        return _ttc_dict(INF_TTC, 0.0, 0.0, rel_speed)
    if dist < MIN_DIST:
        dist = MIN_DIST

    ttc = dist / rel_speed
    ttc_brake = ego.velocity_x / AEB_DEFAULT_MAX_DECEL if ego.velocity_x > 0.1 else 0.0
    ttc_alert = ttc_brake + AEB_ALERT_BUFFER_TIME

    return _ttc_dict(ttc, ttc_brake, ttc_alert, rel_speed)

def _ttc_dict(ttc: float, ttc_brake: float, ttc_alert: float, rel_speed: float) -> dict:
    return {
        'ttc': ttc,
        'ttc_brake': ttc_brake,
        'ttc_alert': ttc_alert,
        'rel_speed': rel_speed
    }

# ===== FUNCTION 2: aeb_mode_selection =====
def aeb_mode_selection(target: AEBTarget, ego: EgoData, ttc_data: dict) -> AEBMode:
    if target is None or ego is None or not ttc_data:
        return AEBMode.NORMAL

    ttc = ttc_data['ttc']
    ttc_brake = ttc_data['ttc_brake']
    ttc_alert = ttc_data['ttc_alert']

    if target.object_id < 0 or ego.velocity_x < 0.5 or ttc <= 0.0 or ttc >= INF_TTC:
        return AEBMode.NORMAL

    if target.situation == TargetSituation.CUTOUT:
        return AEBMode.NORMAL

    if ttc > ttc_alert:
        return AEBMode.NORMAL
    elif ttc > ttc_brake:
        return AEBMode.ALERT
    elif ttc > 0.0:
        return AEBMode.BRAKE

    return AEBMode.NORMAL

# ===== FUNCTION 3: calculate_decel_for_aeb =====
def calculate_decel_for_aeb(mode: AEBMode, ttc_data: dict) -> float:
    if mode != AEBMode.BRAKE or not ttc_data:
        return 0.0

    ttc = ttc_data['ttc']
    ttc_brake = ttc_data['ttc_brake']

    if not math.isfinite(ttc) or not math.isfinite(ttc_brake) or ttc_brake <= 0.0:
        return 0.0

    if ttc < 0.0:
        if -0.005 <= ttc <= 0.0 or -0.20 <= ttc <= -0.05:
            ttc = 0.0
        else:
            return 0.0

    EPS = 1e-6
    if ttc > ttc_brake + EPS:
        return AEB_MIN_DECEL
    if abs(ttc - ttc_brake) <= EPS:
        return 0.0

    ratio = 1.0 - (ttc / ttc_brake)
    decel = AEB_MAX_DECEL * ratio
    return round(max(AEB_MAX_DECEL, min(AEB_MIN_DECEL, decel)) * 10.0) / 10.0
