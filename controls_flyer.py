import time
from enum import Enum

import numpy as np

from controller import NonlinearController
#from udacidrone import Drone
from unity_drone import UnityDrone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class ControlsFlyer(UnityDrone):

    def __init__(self, connection, tlog_name="TLog.txt"):
        self.prev_time = 0.0
        self.total_commands = 0.0
        
        self.prev_time2 = 0.0
        self.total_commands2 = 0.0
        
        
        super().__init__(connection)
        self.controller = NonlinearController()
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoint_number = int(0)
        self.prev_target_position = None
        self.position_trajectory = None
        self.time_trajectory = None
        self.yaw_trajectory = None
        self.speed = 1.0
        self.target_velocity = np.array([0.0,0.0,0.0])
        self.target_attitude = np.array([0.0, 0.0, 0.0, 0.0])
        self.thrust_cmd = 0.0
        self.body_rate_cmd = np.array([0.0,0.0,0.0])
        # self.global_home = np.array([0.0,0.0,0.0])  # can't set this here, no setter for this property
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        self.register_callback(MsgID.ATTITUDE, self.attitude_callback)
        self.register_callback(MsgID.RAW_GYROSCOPE, self.gyro_callback)
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)


    def position_controller(self):
        
        
        (position_cmd,velocity_cmd,yaw_cmd) = self.controller.trajectory_control(self.position_trajectory,self.yaw_trajectory,self.time_trajectory,time.time())
        self.local_position_target = position_cmd
        self.local_velocity_target = velocity_cmd
        acceleration_cmd = self.controller.position_control(position_cmd[0:2],velocity_cmd[0:2],self.local_position[0:2],self.local_velocity[0:2])
        self.target_attitude[0] = acceleration_cmd[0]
        self.target_attitude[1] = acceleration_cmd[1]
        self.target_attitude[2] = yaw_cmd
        self.local_acceleration_target = np.array([acceleration_cmd[0],acceleration_cmd[1],0.0])
        
    def attitude_controller(self):
        self.thrust_cmd = self.controller.altitude_control(-self.target_position[2],-self.local_velocity[2],-self.local_position[2],-self.local_velocity[2],self.attitude,9.81*2.0)
        roll_pitch_rate_cmd = self.controller.roll_pitch_controller(self.target_attitude[0:2],self.attitude,self.thrust_cmd,yaw_cmd=self.target_attitude[2])
        
        yawrate_cmd = self.controller.yaw_control(self.target_attitude[2],self.attitude[2])
        self.body_rate_cmd = np.array([roll_pitch_rate_cmd[0],roll_pitch_rate_cmd[1],yawrate_cmd])
        self.body_rate_target = self.body_rate_cmd
        
    def bodyrate_controller(self):        
        moment_cmd = self.controller.body_rate_control(self.body_rate_cmd,self.gyro_raw)
        self.cmd_moment(moment_cmd[0],moment_cmd[1],moment_cmd[2],self.thrust_cmd)
                
    def attitude_callback(self):
        if self.flight_state == States.WAYPOINT:
            self.attitude_controller()
    
    def gyro_callback(self):
        if self.flight_state == States.WAYPOINT:
            self.bodyrate_controller()

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                #self.all_waypoints = self.calculate_box()
                (self.position_trajectory,self.time_trajectory,self.yaw_trajectory) = self.load_test_trajectory(time_mult=1.0)#self.calculate_box_trajectory()
                self.all_waypoints = self.position_trajectory.copy()#[self.position_trajectory[-1]]
                self.waypoint_number = -1
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            #self.position_controller()
            #if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
            if time.time() > self.time_trajectory[self.waypoint_number]:
                if len(self.all_waypoints) > 0:
                    #self.print_error()
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        #self.print_error()
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()
        elif self.flight_state == States.WAYPOINT:
            self.position_controller()
            pass

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if not self.armed:
                    if not self.guided:
                        self.manual_transition()

    def calculate_box_trajectory(self):
        print("Calculating Box Trajectory")
        position_trajectory = [self.local_position,np.array([10.0, 0.0, -3.0]),np.array([0.0, 10.0, -3.0]),np.array([10.0, 0.0, -3.0])]
        current_time = time.time()
        time_trajectory = [current_time,current_time+3.0,current_time+8.0,current_time+11.0]
        yaw_trajectory = []
        for i in range(0,len(position_trajectory)-1):
            yaw_trajectory.append(np.arctan2(position_trajectory[i+1][1]-position_trajectory[i][1],position_trajectory[i+1][0]-position_trajectory[i][0]))
        yaw_trajectory.append(yaw_trajectory[-1])
        return(position_trajectory,time_trajectory,yaw_trajectory)
    
        
    def calculate_box(self):
        print("Calculatin Box Waypoints")
        local_waypoints = [[10.0, 0.0, -3.0], [10.0, 10.0, -3.0], [0.0, 10.0, -3.0], [0.0, 0.0, -3.0]]
        return local_waypoints
    
    def print_error(self):
        waypoint_error = np.linalg.norm(self.target_position-self.local_position)
        print("Waypoint Error: ", waypoint_error)

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0], self.global_position[1],
                               self.global_position[2])  # set the current location to be the home position

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        # self.global_home = np.copy(self.global_position)  # can't write to this variable!
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self.waypoint_number = self.waypoint_number+1
        if self.prev_target_position is None:
            self.prev_target_position = self.local_position
        else:
            self.prev_target_position = np.array(self.target_position)
                    
        self.target_position = self.all_waypoints.pop(0)
        print('target position', self.target_position)
        #self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):

        self.start_log("Logs", "NavLog.txt")
        # self.connect()

        print("starting connection")
        # self.connection.start()

        super().start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone = ControlsFlyer(conn)
    time.sleep(2)
    drone.start()
    drone.print_mission_score()
    
