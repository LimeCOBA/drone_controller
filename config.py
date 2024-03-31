class Settings:
    k_vertical_thrust = 68.5
    k_vertical_offset = 0.2

    # k_vertical_p = 3.0
    k_roll_p = 30.0
    k_pitch_p = 50.0

    kp_x = 10
    kd_x = 10

    kp_z = 10
    kd_z = 10

    kp_altitude = 3
    kd_altitude = 10

    kp_yawn = 1
    kd_yawn = 1

    key = 0

    #           roll_dist, pitch_dist, yaw_dist, target_altitude, target_x, target_z, target_yawn
    defaults = (0,         0,          0,        1,               0,        0,        -1)


settings = Settings
