## Workflow

The simulator will send back data required to manipulate individual quadrotor motors and ultimately effectively control the quadrotor.

Data sent back:

* position (either in global or local frame)
* angular velocity
* euler angles?
* ...

#### Receiving data messages

TODO: which mavlink messages will we use to get the data? Possiblilites:

- http://mavlink.org/messages/common#HIL_STATE_QUATERNION
- http://mavlink.org/messages/common#SCALED_IMU
- http://mavlink.org/messages/common#HIGHRES_IMU

#### Sending data messages

- [SET_ACTUATOR_CONTROL_TARGET](http://mavlink.org/messages/common#SET_ACTUATOR_CONTROL_TARGET) will be used to command individual motors.

Most important field:

Actuator controls. Normed (normalized?) to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.

So we can send back an 8-element float array. I think we'll only need to use the first four indices: roll, pitch, yaw, throttle where roll, pitch and yaw are moments.