# FCND Controls Project
For this project, you will write the low level flight controllers for the vehicle. In the previous projects, commanded positions were passed to the simulation. In this project, commands will be passed as three directional body moments and thrust. Using the position, velocity, attitude, and body rates sent from the simulation, you will first write several nested control loops to achieve wayoint following control of the first project. Next, you'll expand these capabilities to follow a timed trajectory. The goal of the trajectory following will be to arrive at the end within the specified time while maintaining position errors below a threshold. After the Python portion of the project, you will modify a C++ controller.

## Step 1: Download the Simulator
In this beta version the simulator is provided in the repository. For students, this step will read: If you haven't already, download the version of the simulator that's appropriate for your operating system [from this repository]() (link to be added).

## Step 2: Set up your Python Environment
If you haven't already, set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

## Step 3: Clone this Repository (repository is currently private, but students will have access)
```sh
git clone https://github.com/udacity/FCND-Controls
```
## Step 4: Test setup
Your starting point here will be the [solution code](https://github.com/udacity/FCND-Backyard-Flyer/blob/solution/backyard_flyer.py) for the Backyard Flyer project. Before you start modifying the code, make sure that your Backyard Flyer solution code works as expected and your drone can perform the square flight pathin the new simulator. To do this, start the simulator and run the [`backyard_flyer.py`](https://github.com/udacity/FCND-Backyard-Flyer/blob/solution/backyard_flyer.py) script.

```py
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python backyard_flyer.py
```
The quad should take off, fly a square pattern and land, just as in the previous project. If everything works then you are ready to start work on this project. 

## Step 5: Inspect the relevant files
For this project, you'll be writing the control system in 'controller.py'. The controller is separated into five parts:
* body rate control
* reduced attitude control
* altitude control
* heading control
* position control
* trajectory control

Each of these will be implemented as methods of the NonlinearController class and will fit together as shown below. The next step will guide you through the changes to 'backyard_flyer.py' to fit the controllers into the right structure.
![Image of ControlStructure](ControlStructure.png)
## Step 6: Modifications to 'backyard_flyer.py'
The following modifications need to be made to the solution 'backyard_flyer.py'. Feel free to use a copy of your own solution to the Backyard Flyer Project or the one in the link provided.

1. Import the UnityDrone class and modify the BackyardFlyer class to be a subclass of the UnityDrone class instead of the Drone class. UnityDrone is a subclass the Drone class, so it provides all the functionality of the Drone class along with additional Unity specific commands/functionality (see below)
~~~
from unity_drone import UnityDrone
...
class BackyardFlyer(UnityDrone):
~~~

2. Add a controller object during the __init__ method
~~~
def __init__(self, connection):
	...
    self.controller = NonlinearController()
~~~

3. Add the following three methods to your class.
~~~
    def position_controller(self):  
        acceleration_cmd = self.controller.position_control(self.local_position_target[0:2],0*self.local_velocity_target[0:2],self.local_position[0:2],self.local_velocity[0:2])
        self.local_acceleration_target = np.array([acceleration_cmd[0],acceleration_cmd[1],0.0])
        
    def attitude_controller(self):
        self.thrust_cmd = self.controller.altitude_control(-self.local_position_target[2],-self.local_velocity_target[2],-self.local_position[2],-self.local_velocity[2],self.attitude,9.81*2.0)
        roll_pitch_rate_cmd = self.controller.roll_pitch_controller(self.local_acceleration_target[0:2],self.attitude,self.thrust_cmd,yaw_cmd=self.attitude_target[2])
        yawrate_cmd = self.controller.yaw_control(self.attitude_target[2],self.attitude[2])
        self.body_rate_target = np.array([roll_pitch_rate_cmd[0],roll_pitch_rate_cmd[1],yawrate_cmd])
        
    def bodyrate_controller(self):        
        moment_cmd = self.controller.body_rate_control(self.body_rate_target,self.gyro_raw)
        self.cmd_moment(moment_cmd[0],moment_cmd[1],moment_cmd[2],self.thrust_cmd)
~~~

4. Register and add callbacks (if you haven't already) for the RAW_GYROSCOPE, ATTITUDE, and LOCAL_VELOCITY messages. call the appropriate level of control in each callback (i.e. bodyrate_controller is called in the gyro_callback)
~~~
def __init___(self,connection):
	...
    self.register_callback(MsgID.ATTITUDE, self.attitude_callback)
	self.register_callback(MsgID.RAW_GYROSCOPE, self.gyro_callback)
    self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
    
def attitude_callback(self):
    ...
	if self.flight_state == States.WAYPOINT:
        self.attitude_controller()
    
def gyro_callback(self):
	....
    if self.flight_state == States.WAYPOINT:
    	self.bodyrate_controller()
            
def velocity_callback(self):
	...
    if self.flight_state == States.WAYPOINT:
    	self.position_controller()
~~~

5. In the waypoint transition method, replace the self.cmd_position method (which is disabled by the UnityDrone class) with setting the target local position
~~~
#replace this
self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)

#with this
self.local_position_target = np.array((self.target_position[0],self.target_position[1],self.target_position[2]))
~~~

## Step 7: Design the Backyard Flyer Controller
For this step, you'll be writing the nested low-level controller to achieve the waypoint following needed to complete the backyard flyer project. To do this you'll be filling in methods in the 'controller.py' class. Using the linnear or non-linear dynamics and control from the lessons, you'll write the control code for each of the controller parts showing control diagram shown above. The minimum requirements for a passing submission to this step include:

* (body_rate_control) A proportional controller on body rates to commanded moments
* (altitude_control) A linear altitude controller that uses both the down position and the down velocity to command thrust.
* (yaw_control) A linear/proportional heading controller to yaw rate commands (non-linear transformation not required)
* (roll_pitch_control) A reduced attitude controller taking in local acceleration or attitude commands and outputs body rate command. For this step, a linear controller is acceptable.
* (position_control) A linear position controller using the local north/east position and local north/east velocity to generate a commanded local acceleration
* The final moment/thrust commands limit the input at given saturation limits

Note: for this step, a linear or non-linear cascaded controller is perfectly acceptable. Later steps will require accounting for some of the non-linearities.

Each of the methods in controller.py contain additional instructions. It's highly suggested that you code/tune/test your controller from the lowest level. For example, body_rate_control, altitude_control, and yaw_control can all be tested prior to designing roll_pitch_control. The altitude/yaw should be stabilized and the roll/pitch will slowly drift to unstable (instead of immediately flipping over). Next, the roll_pitch_control can be tested by passing in zero acceleration commands. This should stabilize the drone's orientation completely. Tuning a faster and smoother inner loop will make tuning the outer loop easier due to less decoupling of the modes.

## Step 8: Test the Controller
After you've written and tuned the waypoint following controller, test the controller to ensure the waypoint following meets the minimum criterion of the Backyard Flyer project. The minimum requirements for a passing submission for this step include:

* The drone meets the waypoint criterion from the Backyard Flyer
* A plot of the drone's 2D (north/east) position overplotting the desired box north/east positions

Feel free to tune the controller to see how much better your custom designed controller can do than the linear waypoint following controller implemented in the Unity simulator.

## Step 9: Additional Modifications to backyard_flyer.py
To implement the trajectory controller, the following changes need to be made to backyard_flyer.py

1. Add the following lines to the top of the position_controller method
~~~
def position_controller(self):  
        (self.local_position_target,self.local_velocity_target,yaw_cmd) = self.controller.trajectory_control(self.position_trajectory,self.yaw_trajectory,self.time_trajectory,time.time())
        self.attitude_target = np.array((0.0,0.0,yaw_cmd)
~~~

2. Load the test trajectory instead of calculating the box
~~~
#replace
self.all_waypoints = self.calculate_box()

#with
(self.position_trajectory,self.time_trajectory,self.yaw_trajectory) = self.load_test_trajectory(time_mult=0.5)
self.all_waypoints = self.position_trajectory.copy()
self.waypoint_number = -1
~~~

3. Change the transition criterion from proximity based to time based

~~~
#Replace this
if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
	...

#with this
if time.time() > self.time_trajectory[self.waypoint_number]:
	...
...

def waypoint_transition(self):
	...
    self.waypoint_number = self.waypoint_number+1
~~~
## Step 10: Trajectory Following Controller
The previous controller commanded the vehicle to go to and stop at a waypoint. You may notice that the vehicle slows down as it gets closer to the waypoint prior to transitioning to the next. Additionally, the vehicle does not necessarily fly the sides of the box and ends up "rounding the corners" (you'll notice this more with tighter turns). The next step of the project will increase the functionality of the drone by providing trajectory following capabilities. Instead of go-to waypoints, trajectory is defined as a position/heading over time. The desired position changes over time and implicitly has a corresponding velocity. The array of positions/time/heading are spaced much closer than the waypoints.

The trajectory following will require a redesign of the control system to account for non-linear transformation from the local frame to the body frame. It may also require retuning of existing control gains because the gains set for the waypoint following may be too high/low to effectively follow the trajectory. The ability to follow a trajectory will be tested with an test trajectory. The trajectory is made up several straight-line segments of varying speed and curved segments. Successful completion of this step include:

* A trajectory controller (trajectory_control) that takes in three lists representing the trajectory (position, time, and heading) and the current time. The controller calculates the target position, velocity, heading from the trajectory lists.
* A reduced attitude controller (roll_pitch_control) accounting for the non-linear transformation from local accelerations to body rates
* An altitude controller (altitude_control) including the non-linear effects from non-zero roll/pitch angles


## Step 11: Test the Controller!
You will next test the trajectory following against a defined trajectory defined in 'test_trajectory.txt'. The time, horizontal error, and vertical error are all checked against thresholds in the UnityDrone class. See below for more information on 'Testing the Test Trajectory'. The minimum requirements for this step include:
* The drone flies the test trajectory faster than the default threshold (20 seconds)
* The maximum horizontal error is less than the default threshold (2 meters)
* The maximum vertical error is less than the default threshold (1 meter)
* A plot of the vehicle 2d position overplotted with the designed 2D position
* Plots of the horizontal and vertical position errors versus time

## Step 12: Write it up!
When you're finished, complete a detailed writeup of your solution and how you addressed each step. The following is required for submission:

* Write-up of how you addressed each step
* The telemetry log of a successful trajectory following
* A copy of controller.py, with all the methods filled in
* 2D position plot of your controller flying the Backyard Flyer 10m box
* 2D position plot of your controller flying the test trajectory
* A plot of horizontal position error vs. time from the test trajectory
* A plot of vertical position error vs. time  from the test trajectory

## Step 13: Additional Challenges (Optional)
* Minimize the time to fly the test trajectory while still meeting the error thresholds
* Integrate with the planning project solution to generate custom trajectories
* Test your control's robustness to wind disturbances and modeling errors. (Simulation Changes Coming)


## Unity Drone Class
the the capabilities of the udacidrone object are expanded to low-level control inputs. These additional commands are implemented within the UnityDrone class, which is a subclass of the drone class. The additional functionality of this class includes:
* Moment control
* Sending target vehicle states to the Unity simulation for visualization
* Load and test against a test trajectory

To use the additional functionality, change your custom drone subclass into a subclass of UnityDrone:
~~~
    import UnityDrone from unity_drone
    
    class BackyardFlyer(UnityDrone):
    ...
    
~~~

### Moment Control

~~~
    def cmd_moment(self, roll_moment, pitch_moment, yaw_moment, thrust):
        """Command the drone moments.

        Args:
            roll_moment: in Newton*meter
            pitch_moment: in Newton*meter
            yaw_moment: in Newton*meter
            thrust: upward force in Newtons
        """
~~~

The commanded roll moment, pitch moment, yaw moment and thrust force commands are defined in the body axis and passed at the lowest level to the Unity control system.

### Target Vehicle States
The following class properties are provided for use within the code. Setting the value of one a property automatically sends the value to the Unity simulation for plotting within the visualization.

* local_position_target (3 element numpy vector)
* local_velocity_target (3 element numpy vector)
* local_acceleration_target (3 element numpy vector)
* attitude_target (3 element numpy vector)
* body_rate_target (3 element numpy vector) 

Note: Setting these values are only used for visualization within the Unity simulator and do not actually affect the Unity vehicle control system. 

## Testing on the Test Trajectory
A test trajectory is stored in test_trajectory.txt. The position, time, and yaw information can be loaded using:

~~~
(self.position_trajectory,self.time_trajectory,self.yaw_trajectory) = self.load_test_trajectory(time_mult=1.0)

~~~
The time_mult argument scales the time_trajectory by its value. To attempt to complete the trajectory at a faster pace, use values below 1.0.

The UnityDrone class automatically checks the horizontal and vertical position error and time when the local_position_target property is set. The mission success is a failure if the maximum position (horizontal or vertical) error is greater than a specified threshold or the total mission time is greater than a specified threshold. The position error and time thresholds can be set using the following properties:

* threshold_horizontal_error (Maximum allowed horizontal error on the mission, float > 0.0)
* threshold_vertical_error (Maximum allowed vertical error on the mission, float > 0.0)
* threshold_time (Maximum mission time, float > 0.0)

At the end of the mission, the success can be printed to the terminal using:
~~~
drone.print_mission_score()
~~~

The printout will look something like:
~~~
Maximum Horizontal Error:  1.40065025436
Maximum Vertical Error:  1.40065025436
Mission Time:  39.27512404109243
Mission Success:  True
~~~

Additionally, if you run visdom, plots of the vertical and horizontal errors along the path (the plots are generated after the run ends). Before starting the script, run in a different terminal:
~~~
python -m visdom.server
~~~
The plots are default displayed on 'http://localhost:8097/'. Open a web browser after the run is finished to see the displayed error plots.



