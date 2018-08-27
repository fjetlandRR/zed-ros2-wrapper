#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    //    auto node = std::make_shared<realsense_ros2_camera::RealSenseCameraNode>();
    //    node->onInit();
    //    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
