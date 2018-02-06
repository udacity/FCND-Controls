# FCND Controls Project
For this project, the capabilities of the udacidrone object are expanded to low-level control inputs. These additional commands are implemented within the UnityDrone class, which is a subclass of the drone class. The additional functionality of this class includes:
* Moment control
* Sending target vehicle states to the Unity simulation for visualization

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