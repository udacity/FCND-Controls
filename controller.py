"""
Nonlinear Controller

"""
import numpy as np
from frame_utils import euler2RM

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1 #1.0


class NonlinearController(object):

    def __init__(
        self,
        Kp_pos=6,#12.0,
        Kp_vel=4,#8.0,
        Kp_alt=4.0,
        Kp_hdot=1.5, #2.0,
        
        Kp_roll=8,  #8,#6.5,
        Kp_pitch=8,  #8,#6.5,
        Kp_yaw=4.5,  #4.5, #4.5,
        
        Kp_p=20,#10,
        Kp_q=20,#10,
        Kp_r=5, #10,
        
        max_tilt=1.0,
        max_ascent_rate=5,
        max_descent_rate=2,
        max_speed=5.0
    ):
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
        self.Kp_alt = Kp_alt
        self.max_speed = max_speed
    
    
    def trajectory_control(self,position_trajectory,
                           yaw_trajectory,
                           time_trajectory,
                           current_time):
        """Generate a commanded position, velocity and yaw based on the
            trajectory
        
        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the
                            position and yaw commands
            current_time: float corresponding to the current time in seconds
            
        Returns: tuple (commanded position, commanded velocity, commanded yaw)
                
        """
        ind_min = np.argmin(np.abs(np.array(time_trajectory)-current_time))
        time_ref = time_trajectory[ind_min]
        
        
        if current_time < time_ref:
            position0 = position_trajectory[ind_min-1]
            position1 = position_trajectory[ind_min]
            
            time0 = time_trajectory[ind_min-1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min-1]
            
        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory)-1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]
                
                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min+1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min+1]
            
        position_cmd = (position1-position0)* \
                        (current_time-time0)/(time1-time0)+position0
        velocity_cmd = (position1-position0)/(time1-time0)
        
        
        return (position_cmd,velocity_cmd,yaw_cmd)
    
    def lateral_position_control(self, local_position_cmd,
                         local_velocity_cmd, local_position, local_velocity,
                         acceleration_ff = np.array([0.0,0.0])):
        """Generate horizontal acceleration commands for the vehicle in the
           local frame

        Args:
            local_position_cmd: desired 2D position in local frame 
                                [north, east]
            local_velocity_cmd: desired 2D velocity in local frame 
                                [north_velocity, east_velocity]
            local_position: vehicle position in the local frame 
                            [north, east]
            local_velocity: vehicle velocity in the local frame
                            [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command
            
        Returns: desired vehicle 2D acceleration in the local frame 
                [north, east]
        """
        velocity_cmd = self.Kp_pos*(local_position_cmd-local_position)
        
        #Limit speed
        velocity_norm = np.sqrt(velocity_cmd[0]*velocity_cmd[0] + \
                                velocity_cmd[1]*velocity_cmd[1])
        
        if velocity_norm > self.max_speed:
            velocity_cmd = velocity_cmd*self.max_speed/velocity_norm
            
        acceleration_cmd = acceleration_ff + \
                            self.Kp_pos*(local_position_cmd-local_position) +\
                            self.Kp_vel*(local_velocity_cmd-local_velocity)

        return acceleration_cmd
    
    def altitude_control(self, altitude_cmd,
                         vertical_velocity_cmd,
                         altitude,
                         vertical_velocity,
                         attitude,
                         acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            acceleration_ff: feedforward acceleration command (+up)
            
        Returns: thrust command for the vehicle (+up)
        """
        
        hdot_cmd = self.Kp_alt * (altitude_cmd - altitude) + vertical_velocity_cmd
        
        # Limit the ascent/descent rate
        hdot_cmd = np.clip(hdot_cmd, -self.max_descent_rate, self.max_ascent_rate)

        acceleration_cmd = acceleration_ff + self.Kp_hdot*(hdot_cmd - vertical_velocity)
        
        R33 = np.cos(attitude[0]) * np.cos(attitude[1])
        thrust = DRONE_MASS_KG * acceleration_cmd / R33
        
        if thrust > MAX_THRUST:
            thrust = MAX_THRUST
        elif thrust < 0.0:
            thrust = 0.0
        return thrust
    
    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array 
                        (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll,pitch,yaw) in radians
            thrust_cmd: vehicle thruts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and 
                pitchrate (q) commands in radians/s
        """
        #Calculate rotation matrix        
        R = euler2RM(attitude[0], attitude[1], attitude[2])
        c_d = thrust_cmd/DRONE_MASS_KG
        
        if thrust_cmd > 0.0:
            target_R13 = -np.clip(acceleration_cmd[0].item()/c_d, -self.max_tilt, self.max_tilt) #-min(max(acceleration_cmd[0].item()/c_d, -self.max_tilt), self.max_tilt)
            target_R23 = -np.clip(acceleration_cmd[1].item()/c_d, -self.max_tilt, self.max_tilt) #-min(max(acceleration_cmd[1].item()/c_d, -self.max_tilt), self.max_tilt)
            
            p_cmd = (1/R[2, 2]) * \
                    (-R[1, 0] * self.Kp_roll * (R[0, 2]-target_R13) + \
                     R[0, 0] * self.Kp_pitch * (R[1, 2]-target_R23))
            q_cmd = (1/R[2, 2]) * \
                    (-R[1, 1] * self.Kp_roll * (R[0, 2]-target_R13) + \
                     R[0, 1] * self.Kp_pitch * (R[1, 2]-target_R23))
        else:  # Otherwise command no rate
            print("negative thrust command")
            p_cmd = 0.0
            q_cmd = 0.0
            thrust_cmd = 0.0
        return np.array([p_cmd, q_cmd])
    
    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) 
                        in radians/second^2
            attitude: 3-element numpy array (p,q,r) in radians/second^2
            
        Returns: 3-element numpy array, desired roll moment, pitch moment, and
                yaw moment commands in Newtons*meters
        """
        Kp_rate = np.array([self.Kp_p, self.Kp_q, self.Kp_r])
        rate_error = body_rate_cmd - body_rate
        
        moment_cmd = MOI * np.multiply(Kp_rate, rate_error)
        if np.linalg.norm(moment_cmd) > MAX_TORQUE:
            moment_cmd = moment_cmd*MAX_TORQUE/np.linalg.norm(moment_cmd)
        return moment_cmd
    
    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate
        
        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/sec
        """
        
        # Ensure the target is within range of 0 to 2*pi
        yaw_cmd = np.mod(yaw_cmd, 2.0*np.pi)
        
        yaw_error = yaw_cmd - yaw
        if yaw_error > np.pi:
            yaw_error = yaw_error - 2.0*np.pi
        elif yaw_error < -np.pi:
            yaw_error = yaw_error + 2.0*np.pi
        
        yawrate_cmd = self.Kp_yaw*yaw_error
        return yawrate_cmd
    
