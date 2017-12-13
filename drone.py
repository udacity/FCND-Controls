# -*- coding: utf-8 -*-

import numpy as np
import logger
from connection import mavlink_connection as mc
from connection import message_types as mt
import time


class Drone:

    def __init__(self, **kwargs):
        if 'threaded' in kwargs.keys():
            thread = kwargs['threaded']
        else:
            thread = True

        if 'connection' in kwargs.keys():
            self.connection = kwargs['connection']
        else:
            self.connection = mc.MavlinkConnection("tcp:127.0.0.1:5760", threaded=thread)

        # Global position in degrees
        self._longitude = 0.0
        self._latitude = 0.0
        self._altitude = 0.0

        # Reference home position in degrees
        self._home_longitude = 0.0
        self._home_latitude = 0.0
        self._home_altitude = 0.0

        # Local positions in meters from the global home
        self._north = 0.0
        self._east = 0.0
        self._down = 0.0

        # Locally oriented velocity in m/s
        self._velocity_north = 0.0
        self._velocity_east = 0.0
        self._velocity_down = 0.0

        # Vehicle state information
        self._armed = False
        self._guided = False
        self._connected = False

        # Euler angles in radians
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

        # Body accelerations
        self._acceleration_x = 0.0
        self._acceleration_y = 0.0
        self._acceleration_z = 0.0

        # Gyro rates
        self._gyro_x = 0.0
        self._gyro_y = 0.0
        self._gyro_z = 0.0

        # Barometer
        self._baro_altitude = 0.0

        self._update_property = {
            mt.MSG_STATE: self._update_state,
            mt.MSG_GLOBAL_POSITION: self._update_global_position,
            mt.MSG_LOCAL_POSITION: self._update_local_position,
            mt.MSG_GLOBAL_HOME: self._update_global_home,
            mt.MSG_VELOCITY: self._update_local_velocity,
            mt.MSG_RAW_GYROSCOPE: self._update_gyro_raw,
            mt.MSG_RAW_ACCELEROMETER: self._update_acceleration_raw,
            mt.MSG_BAROMETER: self._update_barometer,
            mt.MSG_EULER_ANGLES: self._update_euler_angle
        }

        self._message_listeners = {}
        self.callbacks()
        self.tlog = logger.Logger("Logs", "TLog.txt")

    @property
    def global_position(self):
        """
        Longitude, latitude, altitude
        """
        return np.array([self._longitude, self._latitude, self._altitude])

    def _update_global_position(self, msg):
        self._longitude = msg.longitude
        self._latitude = msg.latitude
        self._altitude = msg.altitude

    @property
    def global_home(self):
        """
        Longitude, latitude, altitude
        """
        return np.array([self._home_longitude, self._home_latitude, self._home_altitude])

    def _update_global_home(self, msg):
        self._home_longitude = msg.longitude
        self._home_latitude = msg.latitude
        self._home_altitude = msg.altitude

    @property
    def local_position(self):
        """
        Return local position in NED frame
        """
        return np.array([self._north, self._east, self._down])

    def _update_local_position(self, msg):

        self._north = msg.north
        self._east = msg.east
        self._down = msg.down

    @property
    def local_velocity(self):
        """
        Returns local velocity in NED frame
        """
        return np.array([self._velocity_north, self._velocity_east, self._velocity_down])

    def _update_local_velocity(self, msg):
        self._velocity_north = msg.north
        self._velocity_east = msg.east
        self._velocity_down = msg.down

    @property
    def armed(self):
        return self._armed

    @property
    def guided(self):
        return self._guided

    @property
    def connected(self):
        return self._connected

    def _update_state(self, msg):
        self._armed = msg.armed
        self._guided = msg.guided
        self._connected = True

    @property
    def euler_angles(self):
        """
        roll, pitch, yaw (NED frame)
        """
        return np.array([self._roll, self._pitch, self._yaw])

    def _update_euler_angle(self, msg):
        self._roll = msg.roll
        self._pitch = msg.pitch
        self._yaw = msg.yaw

    @property
    def acceleration_raw(self):
        return np.array([self._acceleration_x, self._acceleration_z, self._acceleration_y])

    def _update_acceleration_raw(self, msg):
        self._acceleration_x = msg.x
        self._acceleration_y = msg.y
        self._acceleration_z = msg.z

    @property
    def gyro_raw(self):
        """Return angular velocity NED frame"""
        return np.array([self._gyro_x, self._gyro_y, self._gyro_z])

    def _update_gyro_raw(self, msg):
        self._gyro_x = msg.x
        self._gyro_y = msg.y
        self._gyro_z = msg.z

    @property
    def barometer(self):
        return np.array(self._baro_altitude)

    def _update_barometer(self, msg):
        self._baro_altitude = msg.altitude

    def callbacks(self):

        @self.connection.on_message('*')
        def on_message_receive(_, msg_name, msg):
            if msg_name == mt.MSG_CONNECTION_CLOSED:
                self.stop()
            """Sorts incoming messages, updates the drone state variables and runs callbacks"""
            if msg_name in self._update_property.keys():
                self._update_property[msg_name](msg)

            self.notify_message_listeners(msg_name, msg)

            self.log_telemetry(msg_name, msg)

    def log_telemetry(self, msg_name, msg):
        """Save the msg information to the telemetry log"""
        if self.tlog.open:
            data = [msg_name]
            data.append(msg.time)
            for k in msg.__dict__.keys():
                if k != '_time':
                    data.append(msg.__dict__[k])
            self.tlog.log_telemetry_data(data)

    @staticmethod
    def read_telemetry_data(filename):
        log_dict = {}
        tlog = open(filename, 'r')
        lines = tlog.readlines()

        for line in lines:
            line_split = line.rstrip('\n').split(',')
            if line_split[0] in log_dict.keys():
                entry = log_dict[line_split[0]]
                for i in range(1, len(line_split)):
                    if line_split[i] == 'True':
                        entry[i - 1] = np.append(entry[i - 1], True)
                    elif line_split[i] == 'False':
                        entry[i - 1] = np.append(entry[i - 1], False)
                    else:
                        entry[i - 1] = np.append(entry[i - 1], float(line_split[i]))
            else:
                entry = []
                for i in range(1, len(line_split)):
                    if line_split[i] == 'True':
                        entry.append(np.array(True))
                    elif line_split[i] == 'False':
                        entry.append(np.array(False))
                    else:
                        entry.append(np.array(float(line_split[i])))

            log_dict[line_split[0]] = entry
        return log_dict

    def msg_callback(self, name):
        """decorator for being able to add a listener for a specific message type        

        @self.msg_callback(message_types.MSG_GLOBAL_POSITION)
        def gps_listener(name, gps):
            # do whatever with the gps, which will be of type GlobalPosition

        or 

        @self.msg_callback('*')
        def all_msg_listener(name, msg):
            # this is a listener for all message types, so break out the msg as defined by the name
        
        These listeners need to be defined within the method self.callbacks() or directly within self.__init__() which calls self.callbacks
        
        Callbacks defined with decorators cannot be removed, use add_message_listener/remove_message_listener if the callback needs to be removed
        """

        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_message_listener(n, fn)
            else:
                self.add_message_listener(name, fn)

        return decorator

    def add_message_listener(self, name, fn):
        """Add the function, fn, as a callback for the message type, name
        
        For example:
            self.add_message_listener(message_types.MSG_GLOBAL_POSITION,global_msg_listener)
            
            OR
            self.add_message_listener('*',all_msg_listener)
            
        These can be added anywhere in the code and are identical to initializing a callback with the decorator
        """
        name = str(name)
        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)

    def remove_message_listener(self, name, fn):
        """Remove the function, fn, as a callback for the message type, name
        
        For example:
            self.remove_message_listener(message_types.MSG_GLOBAL_POSITION,global_msg_listener)
            
        """
        name = str(name)
        if name in self._message_listeners:
            if fn in self._message_listeners[name]:
                self._message_listeners[name].remove(fn)
                if len(self._message_listeners[name]) == 0:
                    del self._message_listeners[name]

    def notify_message_listeners(self, name, msg):
        """Passes the message to the appropriate listeners"""
        for fn in self._message_listeners.get(name, []):
            try:
                #fn(self, name, msg)
                fn(name, msg)
            except Exception as e:
                print('>>> Exception in message handler for %s' % name)
                print('>>> ' + str(e))

        for fn in self._message_listeners.get('*', []):
            try:
                #fn(self, name, msg)
                fn(name, msg)
            except Exception as e:
                print('>>> Exception in message handler for %s' % name)
                print('>>> ' + str(e))

    #Command method wrappers
    def connect(self):
        ''' Conect to the specified device'''
        self.connection.start()
        while not self.connected:
            pass
        print("Connected to the Drone!")

    def disconnect(self):
        self.connection.stop()
        self._connected = False

    def arm(self):
        """Send an arm command to the vehicle"""
        try:
            self.connection.arm()
        except:
            print("arm command not defined")

    def disarm(self):
        """Send a disarm command to the vehicle"""
        try:
            self.connection.disarm()
        except:
            print("disarm command not defined")

    def take_control(self):
        """Take control of the vehicle """
        print('Take Control Messsage')
        try:
            self.connection.take_control()
        except:
            print("take_control command not defined")

    def release_control(self):
        """Take control of the vehicle """
        try:
            self.connection.release_control()
        except:
            print("release_control command not defined")

    def cmd_position(self, north, east, down, heading):
        """ Command the local position and vehicle heading
            north: local north in meters
            east: local east in meters
            down: local down in meters (positive down)
            heading: vehicle yaw in degrees
        """
        try:
            self.connection.cmd_position(north, east, down, heading)
        except:
            print("cmd_position not defined")

    def takeoff(self, target_altitude):
        """Command the vehicle to takeoff to the target_alt (in meters)"""
        try:
            self.connection.takeoff(self.local_position[0], self.local_position[1], target_altitude)
        except:
            print("takeoff no defined")

    def land(self):
        """Command the vehicle to land at its current position"""
        try:
            self.connection.land(self.local_position[0], self.local_position[1])
        except:
            print("land not defined")

    def cmd_attitude_rate(self, roll_rate, pitch_rate, yaw_rate, collective_thrust):
        """
        Command the vehicle orientation rates.
            
        roll_rate, pitch_rate, yaw_rate: in radians/second
        collective_thrust: normalized thrust s.t. between [-1, 1] or [0, 1]
        """
        try:
            self.connection.cmd_attitude_rate(roll_rate, pitch_rate, yaw_rate, collective_thrust)
        except:
            print("cmd_attitude_rate not defined")

    def cmd_velocity(self, velocity_north, velocity_east, velocity_down, heading):
        """
        Command the vehicle velocity.

        north_velocity, east_velocity, down_velocity: in meters/second
        heading: in radians
        """
        try:
            self.connection.cmd_velocity(velocity_north, velocity_east, velocity_down, heading)
        except:
            print("cmd_velocity not defined")

    def cmd_motors(self, motor_rpm):
        """
        Command the rmp of the motors.
        """
        try:
            self.connection.cmd_motors(motor_rpm)
        except:
            print("cmd_motors not defined")

    def set_home_position(self, longitude, latitude, altitude):
        """Set the drone's home position to these coordinates"""
        try:
            self.connection.set_home_position(latitude, longitude, altitude)
        except:
            print("set_home_position not defined")

    def start_log(self, directory, name):
        self.log = logger.Logger(directory, name)

    def stop_log(self):
        self.log.close()

    def start(self):
        """Starts the connection to the drone"""
        self.connection.start()

    def stop(self):
        """Stops the connection to the drone and closes the log"""
        self.disconnect()
        self.tlog.close()

    def run(self):
        """Runs the connection in a while loop,
        
            same as "start" for a non-threaded connection
        """
        if self.connection.threaded:
            self.connect()
            while self.connected:
                pass
        else:
            self.start()


if __name__ == "__main__":
    drone = Drone(threaded=False)
    time.sleep(2)
    drone.start()
