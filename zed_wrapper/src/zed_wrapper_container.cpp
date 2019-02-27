// /////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// /////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>

#include "zed_component.hpp"
#include "zed_it_broadcaster.hpp"
#include "zed_tf_broadcaster.hpp"

#include <iostream>

int main(int argc, char* argv[]) {

    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    // namespace: zed - node_name: zed_node - intra-process communication: true
    std::string defNamespace = "zed";
    std::string defNodeName = "zed_node";
    bool intraProcComm = false;

    // ZED main component
    auto lc_node = std::make_shared<stereolabs::ZedCameraComponent>(defNodeName, defNamespace, intraProcComm);
    exe.add_node(lc_node->get_node_base_interface());

    // Note: image topics published by the main component do not support the ROS standard for `camera_info`
    //       topics to be compatible with the `camera view` plugin of `RVIZ2`.
    //       See
    //          * https://answers.ros.org/question/312930/ros2-image_transport-and-rviz2-camera-something-wrong/
    //          * https://github.com/ros2/rviz/issues/207

    // ZED Image Transport broadcaster
    // Note: this is required since `image_transport` stack in ROS Crystal Clemmys does not support
    //       Lifecycle nodes. The component subscribes to image and depth topics from the main component
    //       and re-publish them using `image_transport`
    auto it_node = std::make_shared<stereolabs::ZedItBroadcaster>(defNodeName, defNamespace, intraProcComm);
    exe.add_node(it_node->get_node_base_interface());

    // ZED TF broadcaster
    // Note: this is required since `tf2_ros::TransformBroadcaster` in ROS Crystal Clemmys does not support
    //       Lifecycle nodes. The component subscribes to ODOM and POSE topics from the main component
    //       and re-publish them using `TransformBroadcaster`
    auto tf_node = std::make_shared<stereolabs::ZedTfBroadcaster>(defNodeName, defNamespace, intraProcComm);
    exe.add_node(tf_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
