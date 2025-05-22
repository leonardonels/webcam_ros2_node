#ifndef WEBCAM_PUBLISHER_HPP
#define WEBCAM_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class WebcamPublisher : public rclcpp::Node {
public:
    WebcamPublisher();
    ~WebcamPublisher();

private:
    void timer_callback();
    rcl_interfaces::msg::SetParametersResult on_parameter_changed(const std::vector<rclcpp::Parameter> &params);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    cv::VideoCapture cap_;
    int camera_index_;
    double publish_rate_;
    std::string topic_name_;
};

#endif  // WEBCAM_PUBLISHER_HPP
