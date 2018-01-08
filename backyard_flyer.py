import time
from enum import Enum

import numpy as np

from controller import PDController
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
        super().__init__(connection)
        self.controller = PDController()
        self.target_position = np.array([0.0, 0.0, 0.0])
        # self.global_home = np.array([0.0,0.0,0.0])  # can't set this here, no setter for this property
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        self.register_callback(MsgID.ATTITUDE, self.attitude_callback)
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def use_controller(self):
        
        position_error = self.target_position-self.local_position
        if np.sqrt(position_error[0]*position_error[0]+position_error[1]*position_error[1])>1.0:
            Kp_pos = self.controller.Kp_pos
        else:
            Kp_pos = self.controller.Kp_pos2
        velocity_cmd = self.controller.position_loop(self.target_position,self.local_position,Kp_pos,self.controller.Kp_alt)
        attitude_cmd = self.controller.velocity_loop(velocity_cmd,self.local_velocity,self.attitude[2],self.controller.Kp_vel)
        self.cmd_attitude(attitude_cmd[0],attitude_cmd[1],0.0,velocity_cmd[2])
                
        print("Velocity command", velocity_cmd)
        print("Attitude command", attitude_cmd)
        
        self.cmd_attitude(attitude_cmd[0], attitude_cmd[1], 0.0, -velocity_cmd[2])
        #self.cmd_attitude(0.0,0.0,0.0,-velocity_cmd[2])
    def attitude_callback(self):
        if self.flight_state == States.WAYPOINT:
            self.use_controller()

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            print("Local position",self.local_position[2])
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

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
