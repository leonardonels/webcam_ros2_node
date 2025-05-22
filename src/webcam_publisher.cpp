#include "webcam_publisher/webcam_publisher.hpp"
#include <cv_bridge/cv_bridge.h>

WebcamPublisher::WebcamPublisher() : Node("webcam_publisher_cpp") {
    this->declare_parameter("camera_index", 0);
    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("topci_name", "webcam/image_raw");

    camera_index_ = this->get_parameter("camera_index").as_int();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    topic_name_ = this->get_parameter("topci_name").as_string();

    cap_.open(camera_index_);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Impossibile aprire la webcam %d", camera_index_);
        rclcpp::shutdown();
        return;
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_),
        std::bind(&WebcamPublisher::timer_callback, this));

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&WebcamPublisher::on_parameter_changed, this, std::placeholders::_1));
}

WebcamPublisher::~WebcamPublisher() {
    cap_.release();
}

void WebcamPublisher::timer_callback() {
    cv::Mat frame;
    if (cap_.read(frame)) {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        publisher_->publish(*msg);
    } else {
        RCLCPP_WARN(this->get_logger(), "Impossibile leggere il frame dalla webcam");
    }
}

rcl_interfaces::msg::SetParametersResult WebcamPublisher::on_parameter_changed(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : params) {
        if (param.get_name() == "publish_rate") {
            publish_rate_ = param.as_double();
            timer_->cancel();
            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / publish_rate_),
                std::bind(&WebcamPublisher::timer_callback, this));
        } else if (param.get_name() == "camera_index") {
            int new_index = param.as_int();
            if (new_index != camera_index_) {
                cap_.release();
                cap_.open(new_index);
                if (!cap_.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Errore apertura webcam %d", new_index);
                    result.successful = false;
                } else {
                    camera_index_ = new_index;
                }
            }
        }
    }
    return result;
}
