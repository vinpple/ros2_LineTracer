#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <memory>
#include <chrono>

std::string src = "/home/jetson/ros2_ws/src/camera/src/simulation/5_lt_cw_100rpm_out.mp4";

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("campub");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto mypub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile );
    
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
    rclcpp::WallRate loop_rate(40.0);

    cv::VideoCapture cap(src);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open video!");
        rclcpp::shutdown();
        return -1;
    }
    cv::Mat frame;

    while(rclcpp::ok())
    {
        cap >> frame;
        if (frame.empty()) { RCLCPP_ERROR(node->get_logger(), "frame empty"); break;}
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
        mypub->publish(*msg);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
