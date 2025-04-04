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
    left_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("left_motor_setpoint_vel", qos);
    right_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("right_motor_setpoint_vel", qos);
    left_motor_setpoint_vel = 0.0;
    right_motor_setpoint_vel = 0.0;
    pub_freq_ = this->get_parameter("pub_freq").as_double();
    init_time = get_clock()->now();
    pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / pub_freq_)),
      std::bind(&RosTestBench_node::publisherCallback, this));
}

void RosTestBench_node::publisherCallback()
{
    velocity_test_ = this->get_parameter("velocity_test").as_string();
    
    if (velocity_test_ == "constant_velocity")
    {
        constant_velocity();
    }
    else if (velocity_test_ == "sinusoidal_velocity")
    {
        sinusoidal_velocity();
    }
    else if (velocity_test_ == "sequence_velocity")
    {
        sequence_velocity();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid velocity test type");
        return;
    }


}

void RosTestBench_node::constant_velocity()
{
    left_motor_setpoint_vel = 0.5;
    right_motor_setpoint_vel = 0.5;

    left_msg.data = left_motor_setpoint_vel;
    right_msg.data = right_motor_setpoint_vel;
 
    left_motor_pub_->publish(left_msg);
    right_motor_pub_->publish(right_msg);
}
 
void RosTestBench_node::sinusoidal_velocity()
{
    auto time = get_clock()->now();
 
    left_motor_setpoint_vel = 0.5 * sin(0.1 * time.seconds());
    right_motor_setpoint_vel = 0.5 * sin(0.1 * time.seconds());

    left_msg.data = left_motor_setpoint_vel;
    right_msg.data = right_motor_setpoint_vel;
 
    left_motor_pub_->publish(left_msg);
    right_motor_pub_->publish(right_msg);
}
 
void RosTestBench_node::sequence_velocity()
{
    auto current_time = get_clock()->now();
    float time_diff = (current_time - init_time).seconds();
    if (time_diff < 5)
    {
        left_motor_setpoint_vel = 0.5;
        right_motor_setpoint_vel = 0.0;
        RCLCPP_INFO(this->get_logger(), "Steering right");
    } else if (time_diff < 10)
    {
        left_motor_setpoint_vel = 0.0;
        right_motor_setpoint_vel = 0.5;
        RCLCPP_INFO(this->get_logger(), "Steering left");
    } else if (time_diff < 15)
    {
        left_motor_setpoint_vel = 0.5;
        right_motor_setpoint_vel = 0.5;
        RCLCPP_INFO(this->get_logger(), "Driving forward");
    } else if (time_diff < 20)
    {
        left_motor_setpoint_vel = -0.5;
        right_motor_setpoint_vel = -0.5;
        RCLCPP_INFO(this->get_logger(), "Driving backward");
    } else
    {
        left_motor_setpoint_vel = 0.0;
        right_motor_setpoint_vel = 0.0;
        init_time = get_clock()->now();
    }
    left_msg.data = left_motor_setpoint_vel;
    right_msg.data = right_motor_setpoint_vel;
 
    left_motor_pub_->publish(left_msg);
    right_motor_pub_->publish(right_msg);
}



void RosTestBench_node::parse_parameters()
{
    depth_ = this->declare_parameter("depth", 1);
    pub_freq_ = this->declare_parameter("pub_freq", 33.0);
    velocity_test_ = this->declare_parameter("velocity_test", "constant_velocity");
}

