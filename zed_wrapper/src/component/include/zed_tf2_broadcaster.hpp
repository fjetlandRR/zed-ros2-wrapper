#ifndef ZED_TF2_BROADCASTER_HPP
#define ZED_TF2_BROADCASTER_HPP

#include "visibility_control.h"

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"

namespace stereolabs {

    // >>>>> Typedefs to simplify declarations
    typedef std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TransformStamped>> transformSub;
    typedef std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    // <<<<< Typedefs to simplify declarations

    class ZedTF2Broadcaster : public rclcpp::Node {
      public:
        ZED_PUBLIC
        explicit ZedTF2Broadcaster(const std::string& node_name = "zed_tf2_broadcaster",
                                   const std::string& ros_namespace = "zed",
                                   bool intra_process_comms = false);

      protected:
        void onPoseTf(const geometry_msgs::msg::TransformStamped::UniquePtr msg);
        void onOdomTf(const geometry_msgs::msg::TransformStamped::UniquePtr msg);
        void onImuTf(const geometry_msgs::msg::TransformStamped::UniquePtr msg);

      private:
        transformSub mSubPoseTransf;
        transformSub mSubOdomTransf;
        transformSub mSubImuTransf;

        std::string mPoseTfTopic;
        std::string mOdomTfTopic;
        std::string mImuTfTopic;

        tfBroadcaster mTransformPoseBroadcaster = nullptr;
        tfBroadcaster mTransformOdomBroadcaster = nullptr;
        tfBroadcaster mTransformImuBroadcaster = nullptr;
    };
}

#endif //#define ZED_TF2_BROADCASTER_HPP
