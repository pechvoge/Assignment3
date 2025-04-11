#ifndef ROSTESTBENCH_NODE_HPP
#define ROSTESTBENCH_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "xrf2_msgs/msg/ros2_xeno.hpp"
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
        void custom_velocity();

        rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr motor_pub_;
        rclcpp::TimerBase::SharedPtr pub_timer_;

        rclcpp::Time init_time;
        float pub_freq_;
        size_t depth_;
        std::string velocity_test_;
        xrf2_msgs::msg::Ros2Xeno motor_msg;
        const float pi = 3.14159265358979323846;
        const float d_relbot = 0.209;// distance between wheels in meters
};

#endif // ROSTESTBENCH_NODE_HPP

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RosTestBench_node)