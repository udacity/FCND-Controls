"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
import time

DRONE_MASS_KG = 2
GRAVITY = -9.81


def euler2RM(roll,pitch,yaw):
    R = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
    cr = np.cos(roll)
    sr = np.sin(roll)
    
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    
    R[0,0] = cp*cy
    R[1,0] = -cr*sy+sr*sp*cy
    R[2,0] = sr*sy+cr*sp*cy
    
    R[0,1] = cp*sy
    R[1,1] = cr*cy+sr*sp*sy
    R[2,1] = -sr*cy+cr*sp*sy
    
    R[0,2] = -sp
    R[1,2] = sr*cp
    R[2,2] = cr*cp
    
    return R

class PDController(object):

    def __init__(
        self,
        max_speed=10,
        turn_speed=2,
        max_tilt=0.5,
        max_ascent_rate=5,
        max_descent_rate=2,
        Kp_hdot=5.0,
        Kp_yaw=6.5,
        Kp_r=20,
        Kp_roll=6.5,
        Kp_q=5,
        Kp_pitch=6.5,
        Kp_p=5,
        Kp_pos=2.0,#2.0,#0.10,
        Kp_pos2=0.4,
        Kp_vel=2.0,
        Kd_vel=0,
        Kp_alt=10.0,
        Ki_hdot=0.05,#0.1
    ):
        self.max_speed = max_speed
        self.turn_speed = turn_speed
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
        self.Kp_pos2 = Kp_pos2
        self.Kp_vel = Kp_vel
        self.Kd_vel = Kd_vel
        self.Kp_alt = Kp_alt
        self.Ki_hdot = Ki_hdot
        self.last_vel_error_body = np.float32([0, 0, 0])
        self.time_since_last_update = time.time()
        self.hdot_int = 0


    def velocity_control(self, target_velocity, attitude, angular_velocity, local_velocity):
        """
        target_velocity: 3-element numpy array (NED frame), target local velocity
        attitude: 3-element numpy array (roll,pitch,yaw) in radians
        angular_velocity: 3-element array  (roll_rate, pitch_rate, yaw_rate) in radians/sec
        local_velocity: 3-element array (NED frame) in meters/sec, current velocity
        """
        
        target_speed = np.linalg.norm(target_velocity[0:2])
        if(target_speed>self.max_speed):
            target_velocity[0] = self.max_speed*target_velocity[0]/target_speed
            target_velocity[1] = self.max_speed*target_velocity[1]/target_speed
        
        cos_yaw = np.cos(attitude[2])
        sin_yaw = np.sin(attitude[2])

        vel_error = target_velocity-local_velocity
        vel_error_body = np.float32([0, 0, 0])
        vel_error_body[0] = cos_yaw * vel_error[0] + sin_yaw * vel_error[1]
        vel_error_body[1] = -sin_yaw * vel_error[0] + cos_yaw * vel_error[1]
        
        
        pitch_cmd = -self.Kp_vel * vel_error_body[0]
        roll_cmd = self.Kp_vel * vel_error_body[1]

        attitude_cmd = np.array([roll_cmd,pitch_cmd,0.0])
            
        roll_moment,pitch_moment = self.roll_pitch_control(attitude_cmd, attitude, angular_velocity)
        thrust = self.vertical_velocity_control(target_velocity[2],attitude,local_velocity[2])
            
        return thrust, roll_moment, pitch_moment


    def position_control(self, target_position, attitude, angular_velocity, local_velocity, local_position):
        """
        target_position: 3-element numpy array (NED frame), target position
        attitude: 3-element numpy array (roll,pitch,yaw) in radians
        angular_velocity: 3-element array  (roll_rate, pitch_rate, yaw_rate) in radians/sec
        local_velocity: 3-element array (NED frame) in meters/sec, current velocity
        local_position: 3-element numpy array (NED frame), in meters, current position relative to home_position
        """
        
        
        position_error = target_position - local_position
        # print('local position', local_position)
        # print('target position', target_position)
        # print('position error', position_err)
        # print('euler angles', np.degrees(euler_angles))

        #Commanded velocity in the local NED frame
        velocity_cmd = np.float32([0, 0, 0])
        
        # deadband position error
        if np.linalg.norm([position_error[0], position_error[1]]) >= 1:
            velocity_cmd[0] = self.Kp_pos * position_error[0]
            velocity_cmd[1] = self.Kp_pos * position_error[1]
            
        velocity_cmd[2] = self.Kp_alt*position_error[2]
            
        return self.velocity_control(velocity_cmd, attitude, angular_velocity, local_velocity)
        
    
    """Yaw Controllers
    yaw_control controls the vehicle heading in order to point the front of vehicle in a specific direction
    yawrate_control stabilizes the vehicles heading rate or to set a desired yawrate
    """
    def yaw_control(self,target_yaw,yaw,yawrate):
        """
        target_yaw: desired vehicle yaw in radians
        yaw: vehicle yaw in radians
        yawrate: vehicle yawrate in radians/sec
        """
        
        #Ensure the target is within range of 0 to 2*pi
        target_yaw = np.mod(target_yaw,2.0*np.pi)
        
        yaw_error = target_yaw-yaw
        if(yaw_error > np.pi):
            yaw_error = yaw_error-2.0*np.pi
        elif(yaw_error<-np.pi):
            yaw_error = yaw_error+2.0*np.pi
        
        yawrate_cmd = self.Kp_yaw*yaw_error
        return self.yawrate_control(yawrate_cmd,yawrate)
    
    
    
    
    """The following three controllers are the inner most controllers. 
    They ae essentially stabilizing controllers
    I think these should be fixed within the simunlator (Unity) with the ability to set PID gains"""
    def yawrate_control(self,target_yawrate,yawrate):
        """
        target_yawrate: desired vehile yawrate in radians/sec
        yawrate: vehicle yawrate in radians/sec
        """
        yaw_moment = self.Kp_r*(target_yawrate-yawrate)
        return yaw_moment
    
    """Vertical Velocity Controller"""
    def vertical_velocity_control(self,target_vertical_velocity, attitude, vertical_velocity):
        """
        target_vertical_velocity: desired vertical velocity in meters/sec
        attitude: 3-element numpy array (roll,pitch,yaw) in radians
        local_velocity: 3-element array (NED frame) in meters/sec, current velocity
        """
        
        now = time.time()
        self.dt = now - self.time_since_last_update
        self.time_since_last_update = now
        thrust_nom = -DRONE_MASS_KG * GRAVITY
        hdot_error = 0
        if target_vertical_velocity > self.max_ascent_rate:
            target_vertical_velocity = self.max_ascent_rate
        elif target_vertical_velocity < - self.max_descent_rate:
            target_vertical_velocity = -self.max_descent_rate
        
        hdot_error = target_vertical_velocity - vertical_velocity
        self.hdot_int += hdot_error * self.dt
        # self.hdot_int += 0

        # hdot to thrust
        thrust = (self.Kp_hdot * hdot_error + self.Ki_hdot * self.hdot_int + thrust_nom) / (np.cos(attitude[0]) * np.cos(attitude[1]))
        return thrust
    
    """Attitude Controllers """
    def roll_pitch_control(self, target_attitude, attitude, angular_rate):
        """
        target_attitude: 3-element numpy array (roll,pitch,yaw) in radians
        attitude: 3-element numpy array (roll,pitch,yaw) in radians
        angular_velocity: 3-element array  (roll_rate, pitch_rate, yaw_rate) in radians/sec
        """
        
        angle_magnitude = np.linalg.norm([target_attitude[0], target_attitude[1]])
        if angle_magnitude > self.max_tilt:
            target_attitude[1] = self.max_tilt * target_attitude[1] / angle_magnitude
            target_attitude[0] = self.max_tilt * target_attitude[0] / angle_magnitude
        
        #Roll controller
        roll = attitude[0]
        rollrate = angular_rate[0]
        target_roll = target_attitude[0]
        target_roll = np.mod(target_roll,2.0*np.pi)
        
        # angle to angular rate command (for pitch and roll)
        roll_error = target_roll - roll
        
        
        if(roll_error>np.pi):
            roll_error = roll_error-2.0*np.pi
        elif(roll_error<-np.pi):
            roll_error = roll_error+2.0*np.pi
        
        roll_rate_error = self.Kp_roll * roll_error - rollrate

        # angular rate to moment (pitch and roll)
        roll_moment = self.Kp_p * roll_rate_error
        
        
        pitch = attitude[1]
        pitchrate = angular_rate[1]
        target_pitch = target_attitude[1]
        target_pitch = np.mod(target_pitch,2.0*np.pi)
        
        # angle to angular rate command (for pitch and roll)
        pitch_error = target_pitch - pitch
        
        
        if(pitch_error>np.pi):
            pitch_error = pitch_error-2.0*np.pi
        elif(pitch_error<-np.pi):
            pitch_error = pitch_error+2.0*np.pi
        
        pitch_rate_error = self.Kp_pitch * pitch_error - pitchrate

        # angular rate to moment (pitch and roll)
        pitch_moment = self.Kp_q * pitch_rate_error        

        return roll_moment, pitch_moment
    
    def roll_pitch_loop(self,target_attitude,attitude):
        """
        target_attitude: 3-element numpy array (roll,pitch,yaw) in radians
        attitude: 3-element numpy array (roll,pitch,yaw) in radians
        angular_velocity: 3-element array  (roll_rate, pitch_rate, yaw_rate) in radians/sec
        """
        
        R = euler2RM(attitude[0],attitude[1],0.0)
        
        thrust_cmd = target_attitude[3]/R[2,2]

        if thrust_cmd > 0.0:
            target_R13 = min(max(target_attitude[0].item()/thrust_cmd.item(),-1.0),1.0)
            target_pitch = np.arcsin(-target_R13)
    
            target_R23 = min(max(target_attitude[1].item()/thrust_cmd.item(),-1.0),1.0)
            target_roll = np.arctan2(target_R23,R[2,2])
    
            tilt_norm = target_roll*target_roll + target_pitch*target_pitch >self.max_tilt
            if abs(tilt_norm) >self.max_tilt:
                target_pitch = target_pitch*self.max_tilt/tilt_norm
                target_roll = target_roll*self.max_tilt/tilt_norm
                target_R13 = -np.sin(target_pitch)
                target_R23 = np.sin(target_roll)*np.cos(target_pitch)
            
            
            p_cmd = (1/R[2,2])*(R[1,0]*self.Kp_roll*(R[0,2]-target_R13)-R[0,0]*self.Kp_pitch*(R[1,2]-target_R23))
            q_cmd = (1/R[2,2])*(R[1,1]*self.Kp_roll*(R[0,2]-target_R13)-R[0,1]*self.Kp_pitch*(R[1,2]-target_R23))
        else:
            p_cmd = 0.0
            q_cmd = 0.0
            thrust_cmd = 0.0
        return [p_cmd,q_cmd,thrust_cmd]
    

    
    def angle_loop(self,target_angle,angle,Kp_angle):
        angle_error = target_angle-angle
        
        if(angle_error>np.pi):
            angle_error = angle_error-2.0*np.pi
        elif(angle_error < -np.pi):
            angle_error = angle_error+2.0*np.pi
            
        return Kp_angle*angle_error
    
    def angular_rate_loop(self,target_rate,angular_rate,Kp_rate):
        moment = Kp_rate*(target_rate-angular_rate)
        return moment
    
    def position_loop(self,target_position,local_position,Kp_pos,Kp_alt):
        position_error = target_position - local_position
        # print('local position', local_position)
        # print('target position', target_position)
        # print('position error', position_err)
        # print('euler angles', np.degrees(euler_angles))

        #Commanded velocity in the local NED frame
        velocity_cmd = np.float32([0, 0, 0])
        
        # deadband position error
        #if np.linalg.norm([position_error[0], position_error[1]]) >= 1:
        velocity_cmd[0] = Kp_pos * position_error[0]
        velocity_cmd[1] = Kp_pos * position_error[1]
            
        velocity_cmd[2] = Kp_alt*position_error[2]
        return velocity_cmd
    
    def velocity_loop(self,target_velocity,local_velocity,yaw,Kp_velocity):
        target_speed = np.linalg.norm(target_velocity[0:2])
        if(target_speed>self.max_speed):
            target_velocity[0] = self.max_speed*target_velocity[0]/target_speed
            target_velocity[1] = self.max_speed*target_velocity[1]/target_speed
        
        
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        vel_error = target_velocity-local_velocity
        vel_error_body = np.float32([0, 0, 0])
        vel_error_body[0] = cos_yaw * vel_error[0] + sin_yaw * vel_error[1]
        vel_error_body[1] = -sin_yaw * vel_error[0] + cos_yaw * vel_error[1]
        
        
        pitch_cmd = -self.Kp_vel * vel_error_body[0]
        if(pitch_cmd>0.9*np.pi/2.0):
            pitch_cmd = 0.9*np.pi/2.0
        elif(pitch_cmd < -0.9*np.pi/2.0):
            pitch_cmd = -0.9*np.pi/2.0
            
        roll_cmd = self.Kp_vel * vel_error_body[1]
        if(roll_cmd>0.9*np.pi/2.0):
            roll_cmd = 0.9*np.pi/2.0
        elif(roll_cmd<-0.9*np.pi/2.0):
            roll_cmd = -0.9*np.pi/2.0
        attitude_cmd = np.array([roll_cmd,pitch_cmd,0.0])
        return attitude_cmd
    
    def position_velocity_loop(self, position_cmd, velocity_cmd, position, velocity, attitude,
                               ff_cmd = np.array([0.0,0.0,0.0])):
        accel_cmd = ff_cmd + self.Kp_pos*(position_cmd-position) + self.Kp_vel*(velocity_cmd-velocity)
        
        #Different gains for altitude
        hdot_cmd = -1.0*(self.Kp_alt*(position_cmd[2]-position[2])+velocity_cmd[2])
        
        if(hdot_cmd > self.max_ascent_rate):
            hdot_cmd = self.max_ascent_rate
        elif(hdot_cmd < -self.max_descent_rate):
            hdot_cmd = -self.max_descent_rate
        hdot = -velocity[2]
        accel_cmd[2] = (-ff_cmd[2] + self.Kp_hdot*(hdot_cmd - hdot))

        R_yaw = euler2RM(0.0,0.0,attitude[2])
        accel_cmd_body = np.matmul(R_yaw,accel_cmd)
        
        return accel_cmd_body

        
    

    
class NonlinearController(object):

    def __init__(
        self,
        max_speed=10,
        turn_speed=2,
        max_tilt=0.5,
        max_ascent_rate=5,
        max_descent_rate=2,
        Kp_hdot=5.0,
        Kp_yaw=6.5,
        Kp_r=20,
        Kp_roll=5.5,
        Kp_q=2,
        Kp_pitch=5.5,
        Kp_p=2,
        Kp_pos=2.0,#2.0,#0.10,
        Kp_pos2=0.4,
        Kp_vel=2.0,
        Kd_vel=0,
        Kp_alt=10.0,
        Ki_hdot=0.05,#0.1
    ):
        self.max_speed = max_speed
        self.turn_speed = turn_speed
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
        self.Kp_pos2 = Kp_pos2
        self.Kp_vel = Kp_vel
        self.Kd_vel = Kd_vel
        self.Kp_alt = Kp_alt
        self.Ki_hdot = Ki_hdot
        self.last_vel_error_body = np.float32([0, 0, 0])
        self.time_since_last_update = time.time()
        self.hdot_int = 0
    
    def position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0,0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command
            
        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        
        acceleration_cmd = acceleration_ff + self.Kp_pos*(local_position_cmd-local_position) + self.Kp_vel*(local_velocity_cmd-local_velocity)
        
        return acceleration_cmd
    
    def altitude_control(self,altitude_cmd,vertical_velocity_cmd,altitude,vertical_velocity,attitude,acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            acceleration_ff: feedforward acceleration command (+up)
            
        Returns: 2-element numpy array, desired vehicle 2D acceleration in the local frame [north, east]
        """
        
        hdot_cmd = self.Kp_alt*(altitude_cmd-altitude)+vertical_velocity_cmd
        
        #Limit the ascent/descent rate
        if(hdot_cmd > self.max_ascent_rate):
            hdot_cmd = self.max_ascent_rate
        elif(hdot_cmd < -self.max_descent_rate):
            hdot_cmd = -self.max_descent_rate
            
        acceleration_cmd = acceleration_ff + self.Kp_hdot*(hdot_cmd - vertical_velocity)
        
        R33 = np.cos(attitude[0])*np.cos(attitude[1])
        thrust = acceleration_cmd/R33
        return thrust
    
    def roll_pitch_controller(self,acceleration_cmd,attitude,thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll,pitch,yaw) in radians
            thrust_cmd: vehicle thruts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
        #Calculate rotation matrix        
        R = euler2RM(attitude[0],attitude[1],attitude[2])

        #Only command if positive thrust
        if thrust_cmd > 0.0:
            target_R13 = min(max(acceleration_cmd[0].item()/thrust_cmd.item(),-1.0),1.0)
            target_pitch = np.arcsin(-target_R13)
    
            target_R23 = min(max(acceleration_cmd[1].item()/thrust_cmd.item(),-1.0),1.0)
            target_roll = np.arctan2(target_R23,R[2,2])
    
            #Limit maximum tilt
            tilt_norm = target_roll*target_roll + target_pitch*target_pitch >self.max_tilt
            if abs(tilt_norm) >self.max_tilt:
                target_pitch = target_pitch*self.max_tilt/tilt_norm
                target_roll = target_roll*self.max_tilt/tilt_norm
                target_R13 = -np.sin(target_pitch)
                target_R23 = np.sin(target_roll)*np.cos(target_pitch)
            
            
            p_cmd = (1/R[2,2])*(R[1,0]*self.Kp_roll*(R[0,2]-target_R13)-R[0,0]*self.Kp_pitch*(R[1,2]-target_R23))
            q_cmd = (1/R[2,2])*(R[1,1]*self.Kp_roll*(R[0,2]-target_R13)-R[0,1]*self.Kp_pitch*(R[1,2]-target_R23))
        else: #Otherwise command no rate
            p_cmd = 0.0
            q_cmd = 0.0
            thrust_cmd = 0.0
        return np.array([p_cmd,q_cmd])
    
    def body_rate_control(self,body_rate_cmd,body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            attitude: 3-element numpy array (p,q,r) in radians/second^2
            
        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        Kp_rate = np.array([self.Kp_p,self.Kp_q,self.Kp_r])
        rate_error = body_rate_cmd-body_rate
        moment_cmd = np.multiply(Kp_rate,rate_error)
        
        return moment_cmd
    
    def yaw_control(self,yaw_cmd,yaw):
        """ Generate the target yawrate
        
        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/sec
        """
        
        #Ensure the target is within range of 0 to 2*pi
        yaw_cmd = np.mod(yaw_cmd,2.0*np.pi)
        
        yaw_error = yaw_cmd-yaw
        if(yaw_error > np.pi):
            yaw_error = yaw_error-2.0*np.pi
        elif(yaw_error<-np.pi):
            yaw_error = yaw_error+2.0*np.pi
        
        yawrate_cmd = self.Kp_yaw*yaw_error
        return yawrate_cmd
    
