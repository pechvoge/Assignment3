#include "object_detection_pkg/object_detection_node.hpp"

Object_detection_node::Object_detection_node(const rclcpp::NodeOptions &options)
    : Node("object_detection_node", options)
{
    parse_parameters();
    initialize();
}

void Object_detection_node::initialize(){
    auto qos = rclcpp::QoS(depth_);
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/output/moving_camera", qos, std::bind(&Object_detection_node::CoG_determiner, this, std::placeholders::_1));

    CoG_pub_ = this->create_publisher<geometry_msgs::msg::Point>("light_position", qos);  

    low_H = 35, low_S = 100, low_V = 100;
    high_H = 85, high_S = 255, high_V = 255;

    // cv::namedWindow(window_detection_name);
    // // Trackbars to set thresholds for HSV values
    // cv::createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    // cv::createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    // cv::createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    // cv::createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    // cv::createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    // cv::createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);  
}

// static void on_low_H_thresh_trackbar(int, void *)
// {
//     low_H = std::min(high_H-1, low_H);
//     cv::setTrackbarPos("Low H", window_detection_name, low_H);
// }
// static void on_high_H_thresh_trackbar(int, void *)
// {
//     high_H = std::max(high_H, low_H+1);
//     cv::setTrackbarPos("High H", window_detection_name, high_H);
// }
// static void on_low_S_thresh_trackbar(int, void *)
// {
//     low_S = std::min(high_S-1, low_S);
//     cv::setTrackbarPos("Low S", window_detection_name, low_S);
// }
// static void on_high_S_thresh_trackbar(int, void *)
// {
//     high_S = std::max(high_S, low_S+1);
//     cv::setTrackbarPos("High S", window_detection_name, high_S);
// }
// static void on_low_V_thresh_trackbar(int, void *)
// {
//     low_V = std::min(high_V-1, low_V);
//     cv::setTrackbarPos("Low V", window_detection_name, low_V);
// }
// static void on_high_V_thresh_trackbar(int, void *)
// {
//     high_V = std::max(high_V, low_V+1);
//     cv::setTrackbarPos("High V", window_detection_name, high_V);
// }

void Object_detection_node::CoG_determiner(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cvimage_ptr;
    cvimage_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cvimage_ptr->image;
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_RGB2HSV);
    //gray_threshold_ = this->get_parameter("gray_threshold").as_int();
    cv::Mat thresholded_image;
    //cv::threshold(gray_image, thresholded_image, gray_threshold_*0.99, gray_threshold_*1.01, cv::THRESH_BINARY);
    cv::inRange(hsv_image,cv::Scalar(low_H,low_S,low_V),cv::Scalar(high_H,high_S,high_V),thresholded_image); 
    cv::Moments mom = cv::moments(thresholded_image, true);
    geometry_msgs::msg::Point CoG;
    CoG.x = mom.m10 / mom.m00;
    CoG.y = mom.m01 / mom.m00;
    CoG.z = 0;
    CoG_pub_->publish(CoG);
    //RCLCPP_INFO(get_logger(), "CoG is at (%f, %f)", CoG.x, CoG.y);
    cv::imshow("object", thresholded_image);
    cv::waitKey(1);
}

void Object_detection_node::parse_parameters()
{   
    gray_threshold_ = this->declare_parameter("gray_threshold", 150);
    depth_ = this->declare_parameter("depth", 10);
}