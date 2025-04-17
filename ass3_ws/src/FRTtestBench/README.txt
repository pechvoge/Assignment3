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

Core Components(Modified)
    run(): unwraps encoder values and sends setpoints velocities sent from ROS
