import time
from enum import Enum

import numpy as np

from controller import PDController, NonlinearController
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        self.prev_time = 0.0
        self.total_commands = 0.0
        
        self.prev_time2 = 0.0
        self.total_commands2 = 0.0
        
        
        super().__init__(connection)
        self.controller = NonlinearController()
        self.target_position = np.array([0.0, 0.0, 0.0])
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
        start_time = time.clock()
        
        #position_error = self.target_position-self.local_position
        #if np.sqrt(position_error[0]*position_error[0]+position_error[1]*position_error[1])>1.0:
        #    self.controller.Kp_pos = 10.0
        #else:
        #    self.controller.Kp_pos2 = 2.0
        #self.target_velocity = self.controller.position_loop(self.target_position,self.local_position,Kp_pos,self.controller.Kp_alt)
        #self.target_attitude = self.controller.velocity_loop(self.target_velocity,self.local_velocity,self.attitude[2],self.controller.Kp_vel)
        #pos_cmd = self.controller.position_velocity_loop(self.target_position,np.array([0.0,0.0,0.0]),self.local_position,self.local_velocity,self.attitude,np.array([0.0,0.0,-9.81*2]))
      
        acceleration_cmd = self.controller.position_control(self.target_position[0:2],np.array([0.0,0.0]),self.local_position[0:2],self.local_velocity[0:2])
        #thrust_cmd = self.controller.altitude_control(-self.target_position[2],-self.local_velocity[2],-self.local_position[2],-self.local_velocity[2],self.attitude,9.81*2.0)
        self.target_attitude[0] = acceleration_cmd[0]
        self.target_attitude[1] = acceleration_cmd[1]
        #self.target_attitude[3] = thrust_cmd[2]
        
        #TEMP to measure frequency
        curr_time = time.clock()
        self.total_commands2 = self.total_commands2+1
        if(curr_time-self.prev_time2>2.0):
            #print("Position Freq: ",self.total_commands2/(curr_time-self.prev_time2))
            self.prev_time2 = curr_time
            self.total_commands2 = 0.0
            
            #print("Position Calc Time: ",curr_time-start_time)
            
            
    def attitude_controller(self):
        start_time = time.clock()        
        
        self.thrust_cmd = self.controller.altitude_control(-self.target_position[2],-self.local_velocity[2],-self.local_position[2],-self.local_velocity[2],self.attitude,9.81*2.0)
        
        roll_pitch_rate_cmd = self.controller.roll_pitch_controller(self.target_attitude[0:2],self.attitude,self.thrust_cmd)
        
        yawrate_cmd = self.controller.yaw_control(0.0,self.attitude[2])
        
        self.body_rate_cmd = np.array([roll_pitch_rate_cmd[0],roll_pitch_rate_cmd[1],yawrate_cmd])
        
        
        
        
        
        #rate_cmd = self.controller.roll_pitch_loop(self.target_attitude,self.attitude)
        #thrust_cmd = self.controller.vertical_velocity_control(-self.target_velocity[2],self.attitude,-self.local_velocity[2])

        #thrust_cmd = rate_cmd[2]
        #roll_cmd = self.controller.angular_rate_loop(rate_cmd[0],self.gyro_raw[0],self.controller.Kp_p)
        #pitch_cmd = self.controller.angular_rate_loop(rate_cmd[1],self.gyro_raw[1],self.controller.Kp_q)
        
        #yawrate_cmd = self.controller.angle_loop(self.target_attitude[2],self.attitude[2],self.controller.Kp_yaw)
        #yaw_cmd = self.controller.angular_rate_loop(yawrate_cmd,self.gyro_raw[2],1.0)
        
        #print("Thrust Cmd ", thrust_cmd)
        #print("Rate Cmd: ", rate_cmd)
        #print("Roll Cmd: ", roll_cmd)
        #print("Pitch Cmd: ", pitch_cmd)
        
        
        
        #TEMP to measure frequency
        curr_time = time.clock()
        self.total_commands = self.total_commands+1
        if(curr_time-self.prev_time>0.1):
            #print("Attitude Freq: ",self.total_commands/(curr_time-self.prev_time))
            self.prev_time = curr_time
            self.total_commands = 0.0
            #print("Attitude Calc Time: ",curr_time-start_time)
    
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
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            self.position_controller()
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()
        elif self.flight_state == States.WAYPOINT:
            self.position_controller()

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

    def calculate_box(self):
        print("Setting Home")
        local_waypoints = [[10.0, 0.0, -3.0], [10.0, 10.0, -3.0], [0.0, 10.0, -3.0], [0.0, 0.0, -3.0]]
        return local_waypoints

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
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
