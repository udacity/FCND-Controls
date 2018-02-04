import traceback

import numpy as np

from udacidrone import Drone


class UnityDrone(Drone):
    """
    UnityDrone class adds additional low-level capabilities to control the
    Unity simulation version of the drone
    """
    
    def __init__(self, connection, tlog_name="TLog.txt"):
        
        super().__init__(connection, tlog_name)
        
        self._target_north = 0.0
        self._target_east = 0.0
        self._target_down = 0.0
        self._target_position_time = 0.0
        
        self._target_velocity_north = 0.0
        self._target_velocity_east = 0.0
        self._target_velocity_down = 0.0
        self._target_velocity_time = 0.0
        
        self._target_acceleration_north = 0.0
        self._target_acceleration_east = 0.0
        self._target_acceleration_down = 0.0
        self._target_acceleration_time = 0.0
        
        self._target_roll = 0.0
        self._target_pitch = 0.0
        self._target_yaw = 0.0
        self._target_attitude_time = 0.0
        
        self._target_roll_rate = 0.0
        self._target_pitch_rate = 0.0
        self._target_yaw_rate = 0.0
        self._target_body_rate_time = 0.0   


    def cmd_moment(self, roll_moment, pitch_moment, yaw_moment, thrust):
        """Command the drone moments.

        Args:
            roll_moment: in Newton*meter
            pitch_moment: in Newton*meter
            yaw_moment: in Newton*meter
            thrust: upward force in Newtons
        """
        try:
            self.connection.cmd_moment(roll_moment, pitch_moment, yaw_moment, thrust)
        except Exception as e:
            traceback.print_exc()
        
    @property
    def local_position_target(self):
        return np.array([self._target_north,self._target_east,self._target_down])
    
    @local_position_target.setter    
    def local_position_target(self, target):
        """Pass the local position target to the drone (not a command)"""
        self._target_north = target[0]
        self._target_east = target[1]
        self._target_down = target[2]
        t = 0 #TODO: pass along the target time
        try:
            self.connection.local_position_target(target[0], target[1], target[2], t)
        except:
            traceback.print_exec()
    
    @property
    def local_velocity_target(self):
        return np.array([self._target_velocity_north,self._target_velocity_east,self._target_velocity_down])
    
    @local_velocity_target.setter
    def local_velocity_target(self, target):
        """Pass the local velocity target to the drone (not a command)"""
        
        self._target_velocity_north = target[0]
        self._target_velocity_east = target[1]
        self._target_velocity_down = target[2]
        t = 0 #TODO: pass along the target time
        try:
            self.connection.local_velocity_target(target[0], target[1], target[2], t)
        except:
            traceback.print_exec()
            
    @property
    def local_acceleration_target(self):
        return np.array([self._target_acceleration_north,self._target_acceleration_east,self._target_acceleration_down])
    
    @local_acceleration_target.setter
    def local_acceleration_target(self,target):
        self._target_acceleration_north = target[0]
        self._target_acceleration_east = target[1]
        self._target_acceleration_down = target[2]
        t = 0 #TODO: pass along the target time
        try:
            self.connection.local_acceleration_target(target[0],target[1],target[2], t)
        except:
            traceback.print_exec()
    @property
    def attitude_target(self):
        return np.array([self._target_roll,self._target_pitch,self._target_yaw])
    
    @attitude_target.setter
    def attitude_target(self, target):
        """Pass the attitude target to the drone (not a command)"""
        self._target_roll = target[0]
        self._target_pitch = target[1]
        self._target_yaw = target[2]
        t = 0 #TODO: pass along the target time        
        try:
            self.connection.attitude_target(target[0], target[1], target[2], t)
        except:
            traceback.print_exec()
            
    @property
    def body_rate_target(self):
        return np.array([self._target_roll_rate,self._target_pitch_rate,self._target_yaw_rate])
    
    @body_rate_target.setter
    def body_rate_target(self, target):
        """Pass the local position target to the drone (not a command)"""
        self._target_roll_rate = target[0]
        self._target_pitch_rate = target[1]
        self._target_yaw_rate = target[2]
        t = 0 #TODO: pass along the target time
        try:
            self.connection.body_rate_target(target[0],target[1],target[2], t)
        except:
            traceback.print_exec()
            

