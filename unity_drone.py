import traceback

import numpy as np

from udacidrone import Drone
import time


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
        
        #Used for the autograder
        self._threshold_xtrack = 2.0
        self._threshold_time = 40.0
        self._average_xtrack = 0.0
        self._maximum_xtrack = 0.0
        self._mission_time = 0.0
        self._time0 = None
        self._mission_success = True


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
        
        #Check for current xtrack error
        if self._time0 is None:
            self._time0 = time.clock()
        
        self._xtrack_error = self.calculate_xtrack_error()
        self._mission_time = time.clock() - self._time0
        if self._mission_success:
            self.check_mission_success()
            
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
    
    @property
    def threshold_xtrack(self):
        """Maximum allowed xtrack error on the mission"""
        return self._threshold_xtrack
    
    @threshold_xtrack.setter
    def threshold_xtrack(self, threshold):
        if threshold > 0.0:
            self._threshold_xtrack = threshold
        else:
            print('Xtrack threshold must be greater than 0.0')
    
    @property
    def threshold_time(self):
        """Maximum mission time"""
        return self._threshold_time
    
    @threshold_time.setter
    def threshold_time(self,threshold):
        if threshold > 0.0:
            self._threshold_time = threshold
        else:
            print('Time threshold must be greater than 0.0')
            
            
    
    def load_test_trajectory(self,time_mult=1.0):
        """Loads the test_trajectory.txt
        
        Args:
            time_mult: a multiplier to decrease the total time of the trajectory
        
        """
        data  = np.loadtxt('test_trajectory.txt', delimiter=',', dtype='Float64')
        position_trajectory = []
        time_trajectory = []
        yaw_trajectory = []
        current_time = time.time()
        for i in range(len(data[:,0])):
            position_trajectory.append(data[i,1:4])
            time_trajectory.append(data[i,0]*time_mult+current_time)
        for i in range(0,len(position_trajectory)-1):
            yaw_trajectory.append(np.arctan2(position_trajectory[i+1][1]-position_trajectory[i][1],position_trajectory[i+1][0]-position_trajectory[i][0]))
        yaw_trajectory.append(yaw_trajectory[-1])
        return(position_trajectory,time_trajectory,yaw_trajectory)
    
    def calculate_xtrack_error(self):
        """Calcuate the error beteween the local position and target local position
        
        """
        target_position = np.array([self._target_north,self._target_east,self._target_down])
        return np.linalg.norm(target_position-self.local_position)
    
    def print_mission_score(self):
        """Prints the maximum xtrack error, total time, and mission success

        """
        print('Maximum XTrack Error: ', self._maximum_xtrack)
        print('Mission Time: ', self._mission_time)
        print('Mission Success: ', self._mission_success)
        
    def check_mission_success(self):
        """Check the mission success criterion (xtrack and time)
        
        """
        if self._xtrack_error > self._maximum_xtrack:
            self._maximum_xtrack = self._xtrack_error
            if self._maximum_xtrack > self._threshold_xtrack:
                self._mission_success = False
        if self._mission_time > self._threshold_time:
            self._mission_success = False
            

