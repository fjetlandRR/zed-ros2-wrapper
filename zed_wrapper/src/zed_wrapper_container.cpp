#include "zed_component.hpp"
#include "zed_tf2_broadcaster.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exe;

    auto lc_node = std::make_shared<stereolabs::ZedCameraComponent>("zed_node", "", true);
    exe.add_node(lc_node->get_node_base_interface());

    auto tf_node = std::make_shared<stereolabs::ZedTF2Broadcaster>("zed_tf2_broadcaster", "", true);
    exe.add_node(tf_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
