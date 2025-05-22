#include "rclcpp/rclcpp.hpp"
#include "webcam_publisher/webcam_publisher.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebcamPublisher>());
    rclcpp::shutdown();
    return 0;
}
