#include <chrono>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "example_interfaces/msg/float64.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>


using std::placeholders::_1;

using namespace std::chrono_literals;

class SequenceController : public rclcpp::Node {
  public:
    SequenceController() : Node("sequence_controller"), count_(0) {
        sample_time_s_ = 0.03;

        subscription_light_pos_ =
            this->create_subscription<geometry_msgs::msg::Point>(
                "light_position", 10,
                std::bind(&SequenceController::update_light_pos, this, _1));

        subscription_white_ratio_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "white_ratio", 10,
                std::bind(&SequenceController::update_zoom, this, _1));

        motor_pub_ =
            this->create_publisher<xrf2_msgs::msg::Ros2Xeno>(
                "Ros2Xeno", 10);

        timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            std::chrono::duration<double>(sample_time_s_),
            std::bind(&SequenceController::sequence_controller, this));

        this->declare_parameter("rotation_gain", 0.2);
        this->declare_parameter("drive_gain", 0.1);
        this->declare_parameter("width", 320);
        this->declare_parameter("zoom_threshold", 0.3);
    }

  private:
    void sequence_controller() {
        auto rotation_gain = this->get_parameter("rotation_gain").as_double();
        auto drive_gain = this->get_parameter("drive_gain").as_double();
        auto width = this->get_parameter("width").as_int();
        auto zoom_threshold = this->get_parameter("zoom_threshold").as_double();

        double rotate = rotation_gain * (light_pos_.x - (width / 2));

        RCLCPP_INFO(this->get_logger(), "light_pos.x: %f, rotate: %f", light_pos_.x,rotate);

        double drive = - drive_gain * (white_ratio_.data - zoom_threshold); // if ball is too close, then drive slower/backwards

        RCLCPP_INFO(this->get_logger(), "white_ratio: %f, drive: %f", white_ratio_.data,drive);

        motor_msg.left_motor_setpoint_vel = rotate + drive; 
        motor_msg.right_motor_setpoint_vel = -rotate + drive;
        motor_pub_->publish(motor_msg);
    }

    void update_light_pos(const geometry_msgs::msg::Point &msg) {
        if (msg.x == -1)
            return;

        light_pos_.x = msg.x;
        light_pos_.y = msg.y;
    }

    void update_zoom(const std_msgs::msg::Float64 &msg) {
        if (msg.data == -1)
            return;

        white_ratio_.data = msg.data;
    }

    size_t count_;
    double sample_time_s_;

    geometry_msgs::msg::Point light_pos_;
    std_msgs::msg::Float64 white_ratio_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
        subscription_light_pos_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        subscription_white_ratio_;

    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr motor_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_left_; // std_msgs::msg::Float64
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_right_; //std_msgs::msg::Float64

    rclcpp::TimerBase::SharedPtr timer_;

    xrf2_msgs::msg::Ros2Xeno motor_msg;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();

    return 0;
}
