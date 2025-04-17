Assignment 3.4
------------------------------------------------------------
In the Ros2Xeno.msg file in the XRF2 framework add the following two lines:
float64 left_motor_setpoint_vel
float64 right_motor_setpoint_vel
 
1. Open four SSH -X connections to the RELBOT in wsl terminals, in the first run:
    sudo ./build/FRTtestBench/FRTtestBench
2. In the second run:
    ros2 run ros_xeno_bridge RosXenoBridge
4. In the third run:   
    ros2 launch object_detection_pkg sequence_generator_launch.xml
3. Wait 20 seconds for the object detection to be active, then in the fourth run:
    ros2 topic pub --once /XenoCmd std_msgs/msg/Int32 "{data: 1}"

 