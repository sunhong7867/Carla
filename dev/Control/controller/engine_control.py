
def calc_engine_control_command(desired_acceleration):

    if desired_acceleration >= 0:
        throttle_cmd = desired_acceleration
    else:
        throttle_cmd = 0

    return throttle_cmd