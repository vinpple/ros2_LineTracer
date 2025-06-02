#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <memory>
#include <csignal>
#include <iostream>
#include <cmath>

// 종료 플래그
bool stop_signal = false;

// C++14용 clamp 함수
template<typename T>
T clamp(T val, T low, T high) {
    return std::min(std::max(val, low), high);
}

// Ctrl+C 핸들러
void signal_handler(int)
{
    stop_signal = true;
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"), "SIGINT (Ctrl+C) 감지, 종료 중...");
}

// 콜백 함수
void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("sub_wsl"), "수신된 프레임이 비어 있음.");
        return;
    }

    // 원본 영상 출력
    cv::imshow("Original", frame);

    // 영상 처리
    cv::Mat gray, binary;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);
    cv::Rect roi(0, frame.rows - 90, frame.cols, 90);
    cv::Mat roi_bin = binary(roi).clone();
    cv::Mat roi_color;
    cv::cvtColor(roi_bin, roi_color, cv::COLOR_GRAY2BGR);

    // Connected components
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(roi_bin, labels, stats, centroids);
    int roi_center_x = roi.width / 2;
    double best_score = DBL_MAX;
    cv::Point best_center(roi_center_x, 45);
    bool found = false;

    for (int i = 1; i < n; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < 50 || area > 4000) continue;

        int cx = (int)centroids.at<double>(i, 0);
        int cy = (int)centroids.at<double>(i, 1);
        if (cy < 45) continue;

        double dist = std::abs(cx - roi_center_x);
        double score = dist * 0.5 + (1.0 / area) * 0.5;

        if (score < best_score) {
            best_score = score;
            best_center = cv::Point(cx, cy);
            found = true;
        }

        cv::rectangle(roi_color,
                      cv::Rect(stats.at<int>(i, cv::CC_STAT_LEFT),
                               stats.at<int>(i, cv::CC_STAT_TOP),
                               stats.at<int>(i, cv::CC_STAT_WIDTH),
                               stats.at<int>(i, cv::CC_STAT_HEIGHT)),
                      (cv::Point(cx, cy) == best_center ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0)), 2);
        cv::circle(roi_color, cv::Point(cx, cy), 4, cv::Scalar(0, 0, 255), -1);  // 빨간 점
    }

    if (found) {
        int error = best_center.x - roi_center_x;
        int vel1 = 100 - 0.13 * error;
        int vel2 = -(100 + 0.13 * error);
        vel1 = clamp(vel1, -200, 200);
        vel2 = clamp(vel2, -200, 200);
        std::cout << "Error: " << error << ", Vel1: " << vel1 << ", Vel2: " << vel2 << std::endl;
    } else {
        std::cout << "No valid object detected." << std::endl;
    }

    // ROI 영상 출력
    cv::imshow("ROI Binary", roi_color);
    cv::waitKey(1);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sub_wsl");
    signal(SIGINT, signal_handler);

    // 영상 출력창
    cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("ROI Binary", cv::WINDOW_AUTOSIZE);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto sub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos, mysub_callback);

    rclcpp::Rate rate(30);
    while (rclcpp::ok() && !stop_signal) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
