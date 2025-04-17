#include "rosTestBench_pkg/rosTestBench_node.hpp"

RosTestBench_node::RosTestBench_node(const rclcpp::NodeOptions &options)
: Node("rosTestBench_node", options)
{
    parse_parameters();
    initialize();
}
void RosTestBench_node::initialize()
{
    auto qos = rclcpp::QoS(depth_);
    // This publisher publishes the motor setpoint velocities to the topic "Ros2Xeno"
    motor_pub_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", qos);

    // Initialize the motor message with default values
    motor_msg.left_motor_setpoint_vel = 0.0;
    motor_msg.right_motor_setpoint_vel = 0.0;

    // Allows the user to set the publisher frequency through the ROS parameter "pub_freq"
    pub_freq_ = this->get_parameter("pub_freq").as_double();

    // Initialize the time to the current time
    init_time = get_clock()->now();

    // This timer calls the publisherCallback function every 1/pub_freq_ seconds
    pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / pub_freq_)),
      std::bind(&RosTestBench_node::publisherCallback, this));
}

// This function is called every time the timer is called and it allows to change the velocity test type through setting the ROS parameter "velocity_test"
void RosTestBench_node::publisherCallback()
{
    velocity_test_ = this->get_parameter("velocity_test").as_string();
    
    if (velocity_test_ == "constant_velocity")
    {
        RCLCPP_INFO(this->get_logger(), "Hello from constant_velocity test");
        constant_velocity();
    }
    else if (velocity_test_ == "sinusoidal_velocity")
    {
        RCLCPP_INFO(this->get_logger(), "Hello from sinusoidal_velocity test");
        sinusoidal_velocity();
    }
    else if (velocity_test_ == "sequence_velocity")
    {
        RCLCPP_INFO(this->get_logger(), "Hello from sequence_velocity test");
        sequence_velocity();
    }
    else if (velocity_test_ == "custom_velocity")
    {
        RCLCPP_INFO(this->get_logger(), "Hello from custom_velocity test");
        custom_velocity();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid velocity test type");
        return;
    }


}

// This functions publishes a constant velocity for both wheels
void RosTestBench_node::constant_velocity()
{
    motor_msg.left_motor_setpoint_vel = 0.5;
    motor_msg.right_motor_setpoint_vel = 0.5;

    motor_pub_->publish(motor_msg);
}

// This function publishes a sinusoidal velocity for both wheels
void RosTestBench_node::sinusoidal_velocity()
{
    auto time = get_clock()->now();
 
    motor_msg.left_motor_setpoint_vel = 0.5 * sin(0.1 * time.seconds());
    motor_msg.right_motor_setpoint_vel = 0.5 * sin(0.1 * time.seconds());
 
    motor_pub_->publish(motor_msg);
}
 
// This function publishes a sequence of velocities for both wheels
void RosTestBench_node::sequence_velocity()
{
    auto current_time = get_clock()->now();
    float time_diff = (current_time - init_time).seconds();
    if (time_diff < 5)
    {
        motor_msg.left_motor_setpoint_vel = 0.5;
        motor_msg.right_motor_setpoint_vel = 0.0;
        RCLCPP_INFO(this->get_logger(), "Steering right");
    } else if (time_diff < 10)
    {
        motor_msg.left_motor_setpoint_vel = 0.0;
        motor_msg.right_motor_setpoint_vel = 0.5;
        RCLCPP_INFO(this->get_logger(), "Steering left");
    } else if (time_diff < 15)
    {
        motor_msg.left_motor_setpoint_vel = 0.5;
        motor_msg.right_motor_setpoint_vel = 0.5;
        RCLCPP_INFO(this->get_logger(), "Driving forward");
    } else if (time_diff < 20)
    {
        motor_msg.left_motor_setpoint_vel = -0.5;
        motor_msg.right_motor_setpoint_vel = -0.5;
        RCLCPP_INFO(this->get_logger(), "Driving backward");
    } else
    {
        motor_msg.left_motor_setpoint_vel = 0.0;
        motor_msg.right_motor_setpoint_vel = 0.0;
        init_time = get_clock()->now();
    }
 
    motor_pub_->publish(motor_msg);
}

// This function publishes the sequence with a 90 degree turn
void RosTestBench_node::custom_velocity()
{
    const float driving_time = 2.0; // seconds
    const float steering_time = 2.0; // seconds
    auto time = get_clock()->now();
    float time_diff = (time - init_time).seconds();
    if (time_diff < driving_time){// drive forward for 5 seconds
        motor_msg.left_motor_setpoint_vel = 0.25;
        motor_msg.right_motor_setpoint_vel = 0.25;
        RCLCPP_INFO(this->get_logger(), "Driving forward");
    } else if (time_diff < driving_time + steering_time){// steer for 5 seconds
        motor_msg.left_motor_setpoint_vel = 0.5*pi*d_relbot/steering_time;
        motor_msg.right_motor_setpoint_vel = 0.0;
        RCLCPP_INFO(this->get_logger(), "Steering right");
    } else {
        motor_msg.left_motor_setpoint_vel = 0.0;
        motor_msg.right_motor_setpoint_vel = 0.0;
        init_time = get_clock()->now(); // reset the timer
    
        RCLCPP_INFO(this->get_logger(), "Stopping");
    }

    motor_pub_->publish(motor_msg);
}


void RosTestBench_node::parse_parameters()
{
    depth_ = this->declare_parameter("depth", 1);
    pub_freq_ = this->declare_parameter("pub_freq", 33.0);
    velocity_test_ = this->declare_parameter("velocity_test", "custom_velocity");
}

