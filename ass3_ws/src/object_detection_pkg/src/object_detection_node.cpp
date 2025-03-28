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
        "/image", qos, std::bind(&Object_detection_node::CoG_determiner, this, std::placeholders::_1));

    CoG_pub_ = this->create_publisher<geometry_msgs::msg::Point>("light_position", qos);  

    low_H = 35, low_S = 100, low_V = 100;
    high_H = 85, high_S = 255, high_V = 255; 
}

void Object_detection_node::CoG_determiner(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cvimage_ptr;
    cvimage_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cvimage_ptr->image;
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_RGB2HSV);
    cv::Mat thresholded_image;
    cv::inRange(hsv_image,cv::Scalar(low_H,low_S,low_V),cv::Scalar(high_H,high_S,high_V),thresholded_image); 
    cv::Moments mom = cv::moments(thresholded_image, true);
    geometry_msgs::msg::Point CoG;
    CoG.x = mom.m10 / mom.m00 + 90; 
    CoG.y = mom.m01 / mom.m00 + 90;
    CoG.z = 0;

    if(CoG.x != CoG.x || CoG.y != CoG.y){
        CoG.x = -1;
        CoG.y = -1;
    }
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