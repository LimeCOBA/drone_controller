from controller import Robot
from config import *
from calculate import *


class Mavic (Robot):

    def __init__(self):
        super().__init__()

        self.timestep = int(self.getBasicTimeStep())

        self.camera = self.getCamera('camera')
        self.camera.enable(self.timestep)

        self.f_l_LED = self.getLED("front left led")
        self.f_r_LED = self.getLED("front right led")

        self.imu = self.getInertialUnit("inertial unit")
        self.imu.enable(self.timestep)

        self.gps = self.getGPS("gps")
        self.gps.enable(self.timestep)

        self.compass = self.getCompass("compass")
        self.compass.enable(self.timestep)

        self.gyro = self.getGyro("gyro")
        self.gyro.enable(self.timestep)

        self.c_roll_motor = self.getMotor("camera roll")
        self.c_pitch_motor = self.getMotor("camera pitch")

        self.propellers = [self.getMotor('front left propeller'),
                           self.getMotor('front right propeller'),
                           self.getMotor('rear left propeller'),
                           self.getMotor('rear right propeller')]

        self.drone_on = True
        self.old_check_offed_state = False

        for m in self.propellers:
            m.setPosition(float('inf'))
            m.setVelocity(1)

        self.keyboard.enable(1)

        print('drone starts')

    def run(self):
        roll_dist, pitch_dist, yaw_dist, target_altitude, target_x, target_z, target_yawn = settings.defaults

        last_error_x = 0.0

        last_error_z = 0.0

        last_error_altitude = 0.0

        last_error_yawn = 0.0

        while self.step(self.timestep) != -1:
            if self.getTime() > 1:
                break

        while self.step(self.timestep) != -1:
            time = self.getTime()
            time_to_wait = 0.0

            roll = self.imu.getRollPitchYaw()[0]
            pitch = self.imu.getRollPitchYaw()[1]
            yawn = self.compass.getValues()[0]

            position_x = self.gps.getValues()[0]
            altitude = self.gps.getValues()[1]
            position_z = self.gps.getValues()[2]

            roll_acceleration = self.gyro.getValues()[0]
            pitch_acceleration = self.gyro.getValues()[1]

            led_state = int(time) % 2
            self.f_l_LED.set(led_state)
            self.f_r_LED.set(not led_state)

            self.c_roll_motor.setPosition(-0.115 * roll_acceleration)
            self.c_pitch_motor.setPosition(-0.1 * pitch_acceleration)

            key = self.keyboard.getKey()

            if key > 0:
                if key == self.keyboard.UP:
                    pitch_dist = -0.5
                elif key == self.keyboard.DOWN:
                    pitch_dist = 0.5
                else:
                    pitch_dist = 0

                if key == self.keyboard.RIGHT:
                    roll_dist = -0.5
                elif key == self.keyboard.LEFT:
                    roll_dist = 0.5
                else:
                    roll_dist = 0

                if key == self.keyboard.SHIFT + self.keyboard.RIGHT:
                    target_x = -0.01
                elif key == self.keyboard.SHIFT + self.keyboard.LEFT:
                    target_x = 0.01
                else:
                    target_x = 0

                if key == self.keyboard.SHIFT + self.keyboard.UP:
                    target_x += 0.01
                elif key == self.keyboard.SHIFT + self.keyboard.DOWN:
                    target_x -= 0.01

                # on/off mavic
                if key == self.keyboard.HOME and self.getTime() > time_to_wait:
                    print('drone ' + ((not self.drone_on) and 'on' or 'off'))
                    time_to_wait = self.getTime() + 1
                    self.drone_on = not self.drone_on

                # exit
                if key == self.keyboard.END:
                    for m in self.propellers:
                        m.setPosition(float('inf'))
                        m.setVelocity(1)

                    self.f_r_LED.set(False)
                    self.f_l_LED.set(False)

                    break

            else:
                roll_dist, pitch_dist, yaw_dist = settings.defaults[0], settings.defaults[1], settings.defaults[2]

            if self.drone_on:
                # X
                error_x = clamp(target_x - position_x, -1, 1)
                d_error_x = error_x - last_error_x
                last_error_x = error_x
                pitch_input = (settings.k_pitch_p * clamp(pitch, -1, 1)) + pitch_acceleration + pitch_dist
                pitch_input = pitch_input + (settings.kp_x * error_x) + (settings.kd_x * d_error_x)

                # Z
                error_z = clamp(target_z - position_z, -1, 1)
                d_error_z = error_z - last_error_z
                last_error_z = error_z
                roll_input = (settings.k_roll_p * clamp(roll, -1, 1)) + roll_acceleration + roll_dist
                roll_input = roll_input + (settings.kp_z * error_z) + (settings.kd_z * d_error_z)

                # Y - Altitude
                error_altitude = clamp(target_altitude - altitude + settings.k_vertical_offset, -1, 1)
                d_error_altitude = error_altitude - last_error_altitude
                last_error_altitude = error_altitude
                vertical_input = (settings.kp_altitude * error_altitude) + (settings.kd_altitude * d_error_altitude)

                # Rotation
                error_yawn = target_yawn - yawn
                d_error_yawn = error_yawn - last_error_yawn
                yaw_input = settings.kp_yawn * clamp(error_yawn, -1, 1) + settings.kd_yawn * d_error_yawn + yaw_dist
                last_error_yawn = error_yawn

                # roll_input = settings.k_roll_p * clamp(roll, -1, 1) + roll_velocity + roll_dist
                # pitch_input = settings.k_pitch_p * clamp(pitch, -1, 1) + pitch_velocity + pitch_dist
                # yaw_input = yaw_dist
                # clamped_diff_alt = clamp(target_altitude - altitude + settings.k_vertical_offset, -1, 1)
                # vertical_input = settings.k_vertical_p * pow(clamped_diff_alt, 3)

                f_l_motor_input = settings.k_vertical_thrust + vertical_input - roll_input + pitch_input# - yaw_input
                f_r_motor_input = settings.k_vertical_thrust + vertical_input + roll_input + pitch_input# + yaw_input
                r_l_motor_input = settings.k_vertical_thrust + vertical_input - roll_input - pitch_input# + yaw_input
                r_r_motor_input = settings.k_vertical_thrust + vertical_input + roll_input - pitch_input# - yaw_input

                self.propellers[0].setVelocity(f_l_motor_input)
                self.propellers[1].setVelocity(-f_r_motor_input)
                self.propellers[2].setVelocity(-r_l_motor_input)
                self.propellers[3].setVelocity(r_r_motor_input)

                print(roll_dist, pitch_dist, round(target_altitude, 2), yaw_dist, position_x, altitude, position_z, '\t', f_l_motor_input, f_r_motor_input,
                      r_l_motor_input, r_r_motor_input, pitch_input, roll_input, vertical_input, yaw_input, pitch, roll)
            else:
                for m in self.propellers:
                    m.setPosition(float('inf'))
                    m.setVelocity(1)

                self.f_r_LED.set(False)
                self.f_l_LED.set(False)
