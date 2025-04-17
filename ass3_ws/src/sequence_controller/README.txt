# Sequence controller

This node implements the closed-loop sequence controller for CBL students. To interface with this node, the following input topic is used:

- `light_position` (`geometry_msgs::msg::Point`): used to get the position of the light in the camera image (in pixels, top left is 0,0) from the node that you created yourselves in the first assignment.
- `white_ratio` (`std_msgs/msg/Float64`): used to determine whether the RELBOT has to move towards or away from the green ball

And the following output topics are used:
- `Ros2Xeno` (`xrf2_msgs::msg::Ros2Xeno`): contains as data:
    - `left_motor_setpoint_vel` (`std_msgs::msg::Float64`): velocity for the left motor.
    - `right_motor_setpoint_vel` (`std_msgs::msg::Float64`): velocity for the right motor.

This node has two parameters which can be used for tuning the behaviour of the controller:

- `rotation_gain`: this is the gain of the P-controller of the rotation part. The default value is 0.0008.
- `drive_gain`: this is the gain of the P-controller of the drive part. The default value is 2.0.
- `width`: this is the width of the camera image. For the RELBOT, this was found to be 320 pixels. This is also the default value.
- `zoom_threshold`: threshold used to determine ideal distance of RELBOT away from the green ball . The default value is 0.15.


Note that you need to use `/output/moving_camera` (the output of the simulator) as the input topic for your light position node. If your light position node cannot find a light position, you can set the x-value of the position to -1; then the sequence controller will ignore this value. It will then use the previously known value.

Last but not least, this node does quite a bit of debug printing. If you want to disable this, you can use the following line in your launch file:

```python
arguments=["--ros-args", "--log-level", "WARN"]
```