"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
import time

DRONE_MASS_KG = 2
GRAVITY = -9.81


class PDController(object):

    def __init__(
        self,
        move_speed=15,
        turn_speed=2,
        max_tilt=0.5,
        max_ascent_rate=3,
        max_descent_rate=1,
        Kp_hdot=10,
        Kp_yaw=6.5,
        Kp_r=20,
        Kp_roll=6.5,
        Kp_q=20,
        Kp_pitch=6.5,
        Kp_p=20,
        Kp_pos=0.10,
        Kp_vel=0.3,
        Kd_vel=0,
        Kp_alt=0.4,
        Ki_hdot=0.1
    ):
        self.move_speed = move_speed
        self.turn_speed = turn_speed
        self.max_tilt = max_tilt
        self.max_ascent_rate = max_ascent_rate
        self.max_descent_rate = max_descent_rate
        self.Kp_hdot = Kp_hdot
        self.Kp_yaw = Kp_yaw
        self.Kp_r = Kp_r
        self.Kp_roll = Kp_roll
        self.Kp_p = Kp_p
        self.Kp_pitch = Kp_pitch
        self.Kp_q = Kp_q
        self.Kp_pos = Kp_pos
        self.Kp_vel = Kp_vel
        self.Kd_vel = Kd_vel
        self.Kp_alt = Kp_alt
        self.Ki_hdot = Ki_hdot
        self.last_vel_error_body = np.float32([0, 0, 0])
        self.time_since_last_update = time.time()
        self.hdot_int = 0

    def update(self, local_position, target_position, euler_angles, local_velocity, angular_velocity):
        """
        local_position: 3-element numpy array (ENU frame), current position
        target_position: 3-element numpy array (ENU frame), target position
        euler_angles: 3-element numpy array (pitch, roll, yaw in radians) ENU frame
        body_velocity: 3-element numpy array (ENU frame)
        angular_velocity: 3-element numpy array (ENU frame)
        """
        now = time.time()
        self.dt = now - self.time_since_last_update
        self.time_since_last_update = now

        pitch = euler_angles[0]
        roll = euler_angles[1]
        yaw = euler_angles[2]

        position_err = target_position - local_position
        # print('local position', local_position)
        # print('target position', target_position)
        # print('position error', position_err)
        # print('euler angles', np.degrees(euler_angles))

        vel_cmd_local = np.float32([0, 0, 0])
        # deadband position error
        if np.linalg.norm([position_err[0], position_err[1]]) >= 1:
            vel_cmd_local[0] = self.Kp_pos * position_err[0]
            vel_cmd_local[1] = self.Kp_pos * position_err[1]
        vel_cmd_local[2] = self.Kp_alt * position_err[2]

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        vel_cmd_body = np.float32([0, 0, 0])
        vel_cmd_body[0] = cos_yaw * vel_cmd_local[0] - sin_yaw * vel_cmd_local[1]
        vel_cmd_body[1] = sin_yaw * vel_cmd_local[0] + cos_yaw * vel_cmd_local[1]
        vel_cmd_body[2] = vel_cmd_local[2]

        # TODO: add yaw error

        vel_error_body = np.float32([0, 0, 0])
        vel_error_body[0] = self.move_speed * vel_cmd_body[0] - local_velocity[0]
        vel_error_body[1] = self.move_speed * vel_cmd_body[1] - local_velocity[1]
        vel_error_bodyd = (vel_error_body - self.last_vel_error_body) / self.dt
        self.last_vel_error_body = vel_error_body

        thrust = vel_cmd_body[2]
        pitch_rate = self.Kp_vel * vel_error_body[1] + self.Kd_vel * vel_error_bodyd[1]
        roll_rate = -self.Kp_vel * vel_error_body[0] - self.Kd_vel * vel_error_bodyd[0]
        yaw_rate = 0

        angle_magnitude = np.linalg.norm([pitch_rate, roll_rate])
        if angle_magnitude > self.max_tilt:
            pitch_rate = self.max_tilt * pitch_rate / angle_magnitude
            roll_rate = self.max_tilt * roll_rate / angle_magnitude

        # return thrust, pitch_rate, yaw_rate, roll_rate
        # # TODO: implement 2nd half to this
        # # assume stabilization
        # # inner control loop

        thrust_nom = -DRONE_MASS_KG * GRAVITY
        hdot_error = 0
        if thrust > 0:
            hdot_error = self.max_ascent_rate * thrust - local_velocity[2]
        else:
            hdot_error = self.max_descent_rate * thrust - local_velocity[2]
        self.hdot_int += hdot_error * self.dt
        # self.hdot_int += 0

        # hdot to thrust
        thrust = (self.Kp_hdot * hdot_error + self.Ki_hdot * self.hdot_int + thrust_nom) / (np.cos(pitch) * np.cos(roll))

        # yaw rate to yaw moment
        yaw_moment = self.Kp_r * (self.turn_speed * yaw_rate - angular_velocity[2])

        # angle to angular rate command (for pitch and roll)
        pitch_error = pitch_rate - pitch
        roll_error = roll_rate - roll
        pitch_rate_error = self.Kp_pitch * pitch_error - angular_velocity[0]
        roll_rate_error = self.Kp_roll * roll_error - angular_velocity[1]

        # angular rate to moment (pitch and roll)
        pitch_moment = self.Kp_q * pitch_rate_error
        roll_moment = self.Kp_p * roll_rate_error

        # print('thrust, pitch rate, yaw rate, roll rate', thrust, pitch_rate, yaw_rate, roll_rate)
        # print('dt = ', self.dt)
        # print()
        return thrust, pitch_moment, yaw_moment, roll_moment
