# FCND Controls Project
For this project, the capabilities of the udacidrone object are expanded to low-level control inputs. These additional commands are implemented within the UnityDrone class, which is a subclass of the drone class. The additional functionality of this class includes:
* Moment control
* Sending target vehicle states to the Unity simulation for visualization
* Load and test against a test trajectory

To use the additional functionality, change your custom drone subclass into a subclass of UnityDrone:
~~~
    import UnityDrone from unity_drone
    
    class BackyardFlyer(UnityDrone):
    ...
    
~~~

# Moment Control

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

# Target Vehicle States
The following class properties are provided for use within the code. Setting the value of one a property automatically sends the value to the Unity simulation for plotting within the visualization.

* local_position_target (3 element numpy vector)
* local_velocity_target (3 element numpy vector)
* local_acceleration_target (3 element numpy vector)
* attitude_target (3 element numpy vector)
* body_rate_target (3 element numpy vector) 

Note: Setting these values are only used for visualization within the Unity simulator and do not actually affect the Unity vehicle control system. 

# Testing on the Test Trajectory
A test trajectory is stored in test_trajectory.txt. The position, time, and yaw information can be loaded using:

~~~
(self.position_trajectory,self.time_trajectory,self.yaw_trajectory) = self.load_test_trajectory(time_mult=1.0)#self.calculate_box_trajectory()

~~~
The time_mult argument scales the time_trajectory by its value. To attempt to complete the trajectory at a faster pace, use values below 1.0.

The UnityDrone class automatically checks the position error and time when the local_position_target property is set. The mission success is a failure if the maximum position (xtrack) error is greater than a specified threshold or the total mission time is greater than a specified threshold. The position error and time thresholds can be set using the following properties:

* threshold_xtrack (Maximum allowed xtrack error on the mission, float > 0.0)
* threshold_time (Maximum mission time, float > 0.0)

At the end of the mission, the success can be printed to the terminal using:
~~~
drone.print_mission_score()
~~~

The printout will look something like:
~~~
Maximum XTrack Error:  1.40065025436
Mission Time:  39.27512404109243
Mission Success:  True
~~~



