#include "zed_component.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exe;

    // namespace: zed - node_name: zed_node - intra-process communication: false
    auto lc_node = std::make_shared<stereolabs::ZedCameraComponent>("zed_node", "zed", false);
    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
