def calc_brake_command(desired_acceleration):

    if desired_acceleration >= 0:
        brake_cmd = 0
    else:
        brake_cmd = -desired_acceleration

    return brake_cmd
