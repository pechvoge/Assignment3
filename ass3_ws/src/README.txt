Assignment 3.3
------------------------------------------------------------
In the Ros2Xeno.msg file in the XRF2 framework add the following two lines:
float64 left_motor_setpoint_vel
float64 right_motor_setpoint_vel

1. Open five SSH -X connections to the RELBOT in wsl terminals, in the first run:
    sudo ./build/FRTtestBench/FRTtestBench
2. In the second run:
    ros2 run ros_xeno_bridge RosXenoBridge
3. In the third run:
    ros2 topic pub --once /XenoCmd std_msgs/msg/Int32 "{data: 1}"
4. In the fourth run:   
    ros2 run rosTestBench_pkg rosTestBench_node

5. In the last change the velocity sequence as follows(on the dots either "constant_velocity", "sinusoidal_velocity", "sequence_velocity" or "custom_velocity"):
    ros2 param set /rosTestBench_node velocity_test "..."

