Package rosTestBench_pkg
-------------------------------------------------
Description: This package is a ROS test node that can send different velocity sequences to XFR2 framework

Inputs:
-

Outputs:
/Ros2Xeno/left_motor_setpoint_vel
    Type: float64

/Ros2Xeno/right_motor_setpoint_vel
    Type: float64

Run 
1. Open a SSH -X connection to the RELBOT in a wsl terminal and run:
    ros2 run rosTestBench_pkg rosTestBench_node
2. To change the velocity sequence run the following in a new terminal(on the dots either "constant_velocity", "sinusoidal_velocity" or "sequence_velocity"):
    ros2 param set /rosTestBench_node velocity_test "..."

Parameters
    size_t depth_: Publisher and subscriber queue size. Default = 1
    float pub_freq_: Publisher frequency. Default = 33.0
    std::string velocity_test_: Velocity test sequences. Default = "constant_velocity", other options: "sinusoidal_velocity", "sequence_velocity"

Core Components(Modified)
    publisherCallback(): Checks whether the velocity_test parameter has changed and runs velocity function accordingly
    constant_velocity(): Publishes a constant velocity for both wheels
    sinusoidal_velocity(): Publishes a sinusoidal velocity to both wheels
    sequence_velocity(): Publishes a velocity sequence: steer right, steer left, drive forward, drive backward
