"""
PD Controller

misc notes:

for starters limit to just the filling the angle_input array
and send that back

need to replicate:
    - euler angles
    - angular acceleration body (acceleration of rotations?)
    - body velocity if this is not equal to standard velocity

errors to replicate backyard flyer:

local position error = commanded position  - current position
yaw angle error = commanded heading - current heading

TODO: why is vel_cmd_local velocity related?
vel_cmd_local = local position error * proportional coeff
vel_cmd_body.x =  cos(heading) * vel_cmd_local.x - sin(heading) * vel_cmd_local.z
vel_cmd_body.z =  sin(heading) * vel_cmd_local.x + cos(heading) * vel_cmd_local.z
vel_cmd_body.y = vel_cmd_local.y
"""
import numpy as np

def ned_to_unity(v):
    """
    Helper function to convert from NED frame to unity frame (EUN)
    """
    north = v[0]
    east = v[1]
    down = v[2]
    return np.float32([east, -down, north])

class PDController(object):
    def __init__(self,
                 drone,
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
        self.drone = drone
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


    def update(self):
        thrust = np.float32([0, 0, 0])
        yaw_moment = np.float32([0, 0, 0])
        pitch_moment = np.float32([0, 0, 0])
        roll_moment = np.float32([0, 0, 0])
        angle_input = np.float32([0, 0, 0, 0])
        vel_cmd_body = np.float32([0, 0, 0])
        vel_cmd_local = np.float32([0, 0, 0])

        # todo: what frame is this in?
        target_pos = self.drone.target_pos

        roll, pitch, yaw = self.drone.euler_angles
        # NED
        local_pos = self.drone.local_pos
        # EUN
        unity_pos = ned_to_unity(local_pos)