Package FRTtestBench
-------------------------------------------------
Description: This package is a modified version of the Template-20Sim from the XRF2 framework
 
Inputs:
sample_data.channel
    Type: int
ros_msg
    Type: xrf2_msgs/msg/Ros2Xeno
    Contains: float64 left_motor_setpoint_vel
              float64 right_motor_setpoint_vel
 
Outputs:
actuate_data.pwm
    Type: int
 
Run 
1. Open three SSH -X connections to the RELBOT in wsl terminals, in the first run:
    sudo ./build/FRTtestBench/FRTtestBench
2. In the second run:
    ros2 run ros_xeno_bridge RosXenoBridge
3. In the third run to go run state of FSM:
    ros2 topic pub --once /XenoCmd std_msgs/msg/Int32 "{data: 1}"
 
Parameters
    const int encoder_max: Maximum encoder value. Default = 16383, taken from manual
    d_wheel: Diameter of the wheels = 0.1
    count_p_turn: Number of encoder counts per full turn = 1024
    gear_ratio: Gear ratio of the motor shaft and wheels = 15.58:1
    quad_counter_ratio: Number of AB encoder edges = 4 (2 leading edges and 2 trailing edges)
 
Core Components(Modified)
    run(): calculates and sends actuator inputs using loop controller, which uses wheel positions from unwrapped encoder values and setpoint velocities