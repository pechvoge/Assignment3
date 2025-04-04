#ifndef ROSTESTBENCH_NODE_HPP
#define ROSTESTBENCH_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "std_msgs/msg/float64.hpp"
#include <string>

class RosTestBench_node : public rclcpp::Node
{
    public:
        explicit RosTestBench_node(const rclcpp::NodeOptions &options);

    private:
        void initialize();
        void parse_parameters();
        void publisherCallback();
        void constant_velocity();
        void sinusoidal_velocity();
        void sequence_velocity();

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_motor_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_motor_pub_;
        rclcpp::TimerBase::SharedPtr pub_timer_;

        rclcpp::Time init_time;
        float pub_freq_;
        size_t depth_;
        std::string velocity_test_;
        float left_motor_setpoint_vel;
        float right_motor_setpoint_vel;
        std_msgs::msg::Float64 left_msg;
        std_msgs::msg::Float64 right_msg;
};

#endif // ROSTESTBENCH_NODE_HPP

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RosTestBench_node)