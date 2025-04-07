#include <iostream>
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>


using std::placeholders::_1;

using namespace std::chrono_literals;

class SequenceController : public rclcpp::Node {
  public:
    SequenceController() : Node("sequence_controller"){
        left_motor_setpoint_vel = 0.0;
        right_motor_setpoint_vel = 0.0;

        publisher_left_ = this->create_publisher<std_msgs::msg::Float64>(
            "left_motor_setpoint_vel", 10);//std_msgs::msg::Float64

        publisher_right_ = this->create_publisher<std_msgs::msg::Float64>(
            "right_motor_setpoint_vel", 10);//std_msgs::msg::Float64

        timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            std::chrono::duration<double>(sample_time_s_),
            std::bind(&SequenceController::sequence_controller, this));

        init_time = get_clock()->now();
        this->declare_parameter("velocity_test", "constant_velocity");
    }

  private:
    void sequence_controller() {

        velocity_test_ = this->get_parameter("velocity_test").as_string();

        if (velocity_test_ == "constant_velocity")
        {
            constant_velocity();
        } else if (velocity_test_ == "sinusoidal_velocity")
        {
            sinusoidal_velocity();
        } else if (velocity_test_ == "sequence_velocity")
        {
            sequence_velocity();
        } else {
            break
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_left_; // std_msgs::msg::Float64
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_right_; //std_msgs::msg::Float64

    rclcpp::TimerBase::SharedPtr timer_;
    std::string velocity_test_;
    int left_motor_setpoint_vel;
    int right_motor_setpoint_vel;
};


int constant_velocity()
{
    left_motor_setpoint_vel = 0.5;
    right_motor_setpoint_vel = 0.5;

    left_pub_->publish(left_motor_setpoint_vel);
    right_pub_->publish(right_motor_setpoint_vel);
}

int sinusoidal_velocity()
{
    auto time = get_clock()->now();

    left_motor_setpoint_vel = 0.5 * sin(0.1 * time);
    right_motor_setpoint_vel = 0.5 * sin(0.1 * time);

    left_pub_->publish(left_motor_setpoint_vel);
    right_pub_->publish(right_motor_setpoint_vel);
}

int sequence_velocity()
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
        left_motor_setpoint_vel = 0.5;
        right_motor_setpoint_vel = 0.0;
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
        init_time += rclcpp::Duration::from_seconds(20);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();

    return 0;
}

