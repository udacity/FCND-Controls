"""
PD Controller
"""
import numpy as np
from frame_utils import ned_to_eun
import time


class PDController(object):
    def __init__(self,
                 move_speed=15,
                 max_tilt=0.5,
                 max_ascent_rate=1.5,
                 max_descent_rate=1,
                 dt=0.25,
                 Kp_hdot=10,
                 Kp_yaw=6.5,
                 Kp_r=20,
                 Kp_roll=6.5,
                 Kp_p=10,
                 Kp_pitch=6.5,
                 Kp_q=10,
                 Kp_pos=0.10,
                 Kp_vel=0.3,
                 Kd_vel=0,
                 Kp_alt=0.4,
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
        self.time_since_last_update = time.time()

    def update(self, local_pos, target_pos, euler_angles, local_vel):
        """
        local_pos - 3-element numpy array (NED frame), current position
        target_pos - 3-element numpy array (NED frame), target position
        euler_angles - 3-element numpy array (roll, pitch, yaw)
        local_vel - 3-element numpy array (NED frame), local body velocity
        """
        # thrust = np.float32([0, 0, 0])
        # yaw_moment = np.float32([0, 0, 0])
        # pitch_moment = np.float32([0, 0, 0])
        # roll_moment = np.float32([0, 0, 0])

        now = time.time()
        self.dt = now - self.time_since_last_update
        self.time_since_last_update = now

        vel_cmd_body = np.float32([0, 0, 0])
        vel_cmd_local = np.float32([0, 0, 0])

        yaw = euler_angles[2]

        local_pos = ned_to_eun(local_pos)
        local_pos[1] = np.abs(local_pos[1])
        target_pos = ned_to_eun(target_pos)
        target_pos[1] = np.abs(target_pos[1])

        pos_error = target_pos - local_pos
        print('local position', local_pos)
        print('target position', target_pos)
        print('position error', pos_error)
        print('euler angles', np.degrees(euler_angles))

        # deadband position error
        if np.linalg.norm([pos_error[0], pos_error[2]]) >= 1:
            vel_cmd_local[0] = self.Kp_pos * pos_error[0]
            vel_cmd_local[2] = self.Kp_pos * pos_error[2]
        vel_cmd_local[1] = self.Kp_alt * pos_error[1]

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        vel_cmd_body[0] = cos_yaw * vel_cmd_local[0] - sin_yaw * vel_cmd_local[2]
        vel_cmd_body[1] = vel_cmd_local[1]
        vel_cmd_body[2] = sin_yaw * vel_cmd_local[0] + cos_yaw * vel_cmd_local[2]

        # NOTE: Not bothering with yaw error since we don't
        # currently the drone doesn't rotate.

        local_vel = ned_to_eun(local_vel)
        vel_error_body = np.float32([0, 0, 0])
        vel_error_bodyd = np.float32([0, 0, 0])

        vel_error_body[0] = self.move_speed * vel_cmd_body[0] - local_vel[0]
        vel_error_body[2] = self.move_speed * vel_cmd_body[2] - local_vel[2]
        vel_error_bodyd = (vel_error_body - self.last_vel_error_body) / self.dt
        self.last_vel_error_body = vel_error_body

        thrust = vel_cmd_body[1]
        pitch_rate = self.Kp_vel * vel_error_body[2] + self.Kd_vel * vel_error_bodyd[2]
        yaw_rate = 0
        roll_rate = -self.Kp_vel * vel_error_body[0] - self.Kd_vel * vel_error_bodyd[0]

        angle_magnitude = np.linalg.norm([pitch_rate, roll_rate])
        if angle_magnitude > self.max_tilt:
            pitch_rate = self.max_tilt * pitch_rate / angle_magnitude
            roll_rate = self.max_tilt * roll_rate / angle_magnitude

        # TODO: implement 2nd half to this

        # throttle, pitch rate, yaw rate, roll rate
        print('throttle, pitch rate, yaw rate, roll rate', thrust, pitch_rate, yaw_rate, roll_rate)
        print('dt = ', self.dt)
        print()
        return thrust, pitch_rate, yaw_rate, roll_rate
