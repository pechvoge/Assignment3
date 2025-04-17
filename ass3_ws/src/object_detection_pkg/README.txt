Package object_detection_pkg
---------------------------------------
Description: The node inside this package determines the center of gravity of a bright object inside an image.

Inputs: 
\image
    Type: sensor_msgs/msg/Image

Outputs:
\light_position
    Type: geometry_msgs/msg/Point
\white_ratio
    Type: std_msgs/msg/Float64

Run:
    ros2 run object_detection_pkg object_detection_node

Parameters:
    depth: Sets the que size of the publisher/subscriber, Default = 10
    gray_threshold: Determines the threshold for a pixel to be considered bright, Default = 150

Core components:
    CoG_determiner(const sensor_msgs::msg::Image::SharedPtr msg): Determines the CoG of a ros image and 
    also determines the white ratio of the image
