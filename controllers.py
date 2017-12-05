"""
PD Controller
"""
import numpy as np
from frame_utils import ned_to_eun


class PDController(object):
    def __init__(self,
                 move_speed=10,
                 max_tilt=0.5,
                 max_ascent_rate=5,
                 max_descent_rate=2,
                 dt=0.02,
                 Kp_hdot=10,
                 Kp_yaw=6.5,
                 Kp_r=20,
                 Kp_roll=6.5,
                 Kp_p=10,
                 Kp_pitch=6.5,
                 Kp_q=10,
                 Kp_pos=0.1,
                 Kp_vel=0.3,
                 Kd_vel=0,
                 Kp_alt=1,
                 Ki_hdot=0.1):
        self.move_speed = move_speed
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
        self.dt = dt

    def update(self, local_pos, target_pos, euler_angles, local_velocity):
        # thrust = np.float32([0, 0, 0])
        # yaw_moment = np.float32([0, 0, 0])
        # pitch_moment = np.float32([0, 0, 0])
        # roll_moment = np.float32([0, 0, 0])
        angle_input = np.float32([0, 0, 0, 0])
        vel_cmd_body = np.float32([0, 0, 0])
        vel_cmd_local = np.float32([0, 0, 0])

        yaw = euler_angles[2]

        local_pos = ned_to_eun(local_pos)
        target_pos = ned_to_eun(target_pos)

        pos_error = target_pos - local_pos

        # deadband position error
        if np.linalg.norm([pos_error[0], pos_error[2]]) >= 1:
            vel_cmd_local[0] = self.Kp_pos * pos_error[0]
            vel_cmd_local[2] = self.Kp_pos * pos_error[2]

        vel_cmd_local[1] = self.Kp_alt * pos_error[1]

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        vel_cmd_body[
            0] = cos_yaw * vel_cmd_local[0] - sin_yaw * vel_cmd_local[2]
        vel_cmd_body[1] = vel_cmd_local[1]
        vel_cmd_body[
            2] = sin_yaw * vel_cmd_local[0] + cos_yaw * vel_cmd_local[2]

        # NOTE: Not bothering with yaw error since we don't
        # alter that at this point.

        local_vel = ned_to_eun(local_velocity)
        vel_error_body = np.float32([0, 0, 0])
        vel_error_bodyd = np.float32([0, 0, 0])

        vel_error_body[0] = self.move_speed * vel_cmd_body[0] - local_vel[0]
        vel_error_body[2] = self.move_speed * vel_cmd_body[2] - local_vel[2]
        vel_error_bodyd = vel_error_body - self.last_vel_error_body / self.dt
        self.last_vel_error_body = vel_error_body

        angle_input[0] = vel_cmd_body[1]
        angle_input[1] = 0
        angle_input[
            2] = self.Kp_vel * vel_error_body[2] + self.Kd_vel * vel_error_bodyd[2]
        angle_input[
            3] = -self.Kp_vel * vel_error_body[0] - self.Kd_vel * vel_error_bodyd[0]

        angle_magnitude = np.linalg.norm([angle_input[2], angle_input[3]])
        if angle_magnitude > self.max_tilt:
            angle_input[2] = self.max_tilt * angle_input[2] / angle_magnitude
            angle_input[3] = self.max_tilt * angle_input[3] / angle_magnitude

        # throttle, yaw_rate, pitch_rate, roll_rate
        return angle_input
