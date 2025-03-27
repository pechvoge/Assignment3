#include <string>

#include <cv_bridge/cv_bridge.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

class PositionNode : public rclcpp::Node {
  public:
    PositionNode() : Node("position_node"), count_(0) {
        threshold_ = 250.0;

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/output/moving_camera", 10,
            std::bind(&PositionNode::image_callback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
            "light_position", 10);
    }

  private:
    void image_callback(const sensor_msgs::msg::Image &msg) {
        auto cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);
        cv::threshold(cv_ptr->image, cv_ptr->image, threshold_, 255,
                      cv::THRESH_BINARY);
        cv::Mat idx;

        auto pos = geometry_msgs::msg::Point();

        pos.x = -1.0;
        pos.y = -1.0;

        cv::findNonZero(cv_ptr->image, idx);

        for (uint i = 0; i < idx.total(); i++) {
            pos.x = (i * pos.x + idx.at<cv::Point>(i).x) / (i + 1);
            pos.y = (i * pos.y + idx.at<cv::Point>(i).y) / (i + 1);
        }

        pos.x = pos.x + msg.width;
        pos.y = pos.y + msg.height;

        publisher_->publish(pos);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

    size_t count_;
    double threshold_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionNode>());
    rclcpp::shutdown();

    return 0;
}
