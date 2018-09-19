#include "zed_tf2_broadcaster.hpp"

namespace stereolabs {

    ZedTF2Broadcaster::ZedTF2Broadcaster(const std::string& node_name,
                                         const std::string& ros_namespace,
                                         bool intra_process_comms)
        : Node(node_name, ros_namespace, intra_process_comms) {

#ifndef NDEBUG
        rcutils_ret_t res = rcutils_logging_set_logger_level(get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting DEBUG logger");
        }
#endif

        RCLCPP_INFO(get_logger(), "ZED TF2 Broadcaster Component created");

        RCLCPP_DEBUG(get_logger(), "ZED TF2 Broadcaster node: %s", get_name());
        RCLCPP_DEBUG(get_logger(), "ZED TF2 Broadcaster namespace: %s", get_namespace());

        std::string topicPrefix = "/zed_node/"; // TODO add parameter

        mPoseTfTopic = topicPrefix + "transform/pose";
        mOdomTfTopic = topicPrefix + "transform/odom";
        mImuTfTopic  = topicPrefix + "transform/imu";

        rmw_qos_profile_t tracking_qos_profile = rmw_qos_profile_default; //rmw_qos_profile_sensor_data; // Default QOS profile

        tracking_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        tracking_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        tracking_qos_profile.depth = 2;
        tracking_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;


        mSubPoseTransf = create_subscription<geometry_msgs::msg::TransformStamped>(mPoseTfTopic,
                         std::bind(&ZedTF2Broadcaster::onPoseTf, this, std::placeholders::_1), tracking_qos_profile);
        RCLCPP_INFO(get_logger(), "Subscribing to topic: %s", mSubPoseTransf->get_topic_name());


        mSubOdomTransf = create_subscription<geometry_msgs::msg::TransformStamped>(mOdomTfTopic,
                         std::bind(&ZedTF2Broadcaster::onOdomTf, this, std::placeholders::_1), tracking_qos_profile);
        RCLCPP_INFO(get_logger(), "Subscribing to topic: %s", mSubOdomTransf->get_topic_name());

        mSubImuTransf = create_subscription<geometry_msgs::msg::TransformStamped>(mImuTfTopic,
                        std::bind(&ZedTF2Broadcaster::onImuTf, this, std::placeholders::_1), tracking_qos_profile);
        RCLCPP_INFO(get_logger(), "Subscribing to topic: %s", mSubImuTransf->get_topic_name());

    }

    void ZedTF2Broadcaster::onPoseTf(const geometry_msgs::msg::TransformStamped::UniquePtr msg) {
        RCLCPP_INFO_ONCE(get_logger(), "Broadcasting Pose TF2");

        if (mTransformPoseBroadcaster == nullptr) {
            mTransformPoseBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
        }

        //msg->header.stamp = this->now();

        mTransformPoseBroadcaster->sendTransform(*(msg.get()));
        //RCLCPP_DEBUG(get_logger(), "Broadcasted POSE FRAME");
    }

    void ZedTF2Broadcaster::onOdomTf(const geometry_msgs::msg::TransformStamped::UniquePtr msg) {
        RCLCPP_INFO_ONCE(get_logger(), "Broadcasting Odometry TF2");

        if (mTransformOdomBroadcaster == nullptr) {
            mTransformOdomBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
        }

        //msg->header.stamp = this->now();

        mTransformOdomBroadcaster->sendTransform(*(msg.get()));
        //RCLCPP_DEBUG(get_logger(), "Broadcasted ODOM FRAME");
    }

    void ZedTF2Broadcaster::onImuTf(const geometry_msgs::msg::TransformStamped::UniquePtr msg) {
        RCLCPP_INFO_ONCE(get_logger(), "Broadcasting IMU TF2");

        if (mTransformImuBroadcaster == nullptr) {
            mTransformImuBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
        }

        //msg->header.stamp = this->now();

        mTransformImuBroadcaster->sendTransform(*msg);
    }
}

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(stereolabs::ZedTF2Broadcaster, rclcpp::Node)

