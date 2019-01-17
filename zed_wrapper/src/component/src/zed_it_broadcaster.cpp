#include "zed_it_broadcaster.hpp"

using namespace std::placeholders;

namespace stereolabs {

    ZedItBroadcaster::ZedItBroadcaster(const std::string& node_name /*= "zed_it_broadcaster"*/,
                                       const std::string& ros_namespace /*= "zed"*/,
                                       bool intra_process_comms /*= true*/)
        : Node(node_name, ros_namespace, intra_process_comms) {

#ifndef NDEBUG
        rcutils_ret_t res = rcutils_logging_set_logger_level(get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting DEBUG logger");
        }

#endif

        RCLCPP_INFO(get_logger(), "ZED Image Transport Broadcaster Component created");

        RCLCPP_DEBUG(get_logger(), "ZED Image Transport Broadcaster node: %s", get_name());
        RCLCPP_DEBUG(get_logger(), "ZED Image Transport Broadcaster namespace: %s", get_namespace());

        // Topics
        std::string topicPrefix = get_namespace();

        if (topicPrefix.length() > 1) {
            topicPrefix += "/";
        }

        topicPrefix += "zed_node"; // get_name();
        topicPrefix += "/";

        // QOS Profile
        mCamQosProfile = rmw_qos_profile_sensor_data; // Default QOS profile
        mCamQosProfile.depth = 2;

        // Video topics
        std::string img_topic = "/image_rect_color";
        std::string img_raw_topic = "/image_raw_color";
        std::string cam_info_topic = "/camera_info";
        std::string raw_suffix = "_raw";

        std::string leftTopicRoot = "left";
        std::string rightTopicRoot = "right";
        std::string rgbTopicRoot = "rgb";

        // Set the default topic names
        std::string leftTopic = topicPrefix + leftTopicRoot + img_topic;
        std::string leftCamInfoTopic = leftTopic + cam_info_topic;
        std::string leftRawTopic = topicPrefix + leftTopicRoot + raw_suffix + img_raw_topic;
        std::string leftCamInfoRawTopic = leftRawTopic + cam_info_topic;

        std::string rightTopic = topicPrefix + rightTopicRoot + img_topic;
        std::string rightCamInfoTopic = rightTopic + cam_info_topic;
        std::string rightRawTopic = topicPrefix + rightTopicRoot + raw_suffix + img_raw_topic;
        std::string rightCamInfoRawTopic = rightRawTopic + cam_info_topic;

        std::string rgbTopic = topicPrefix + rgbTopicRoot + img_topic;
        std::string rgbCamInfoTopic = rgbTopic + cam_info_topic;
        std::string rgbRawTopic = topicPrefix + rgbTopicRoot + raw_suffix + img_raw_topic;
        std::string rgbCamInfoRawTopic = rgbRawTopic + cam_info_topic;

        // Image Transport topic names
        mRgbTopic = topicPrefix + "it_" + rgbTopicRoot + img_topic;
        mRightTopic = topicPrefix + "it_" + rightTopicRoot + img_topic;
        mLeftTopic = topicPrefix + "it_" + leftTopicRoot + img_topic;
        mRawRgbTopic = topicPrefix + "it_" + rgbTopicRoot + raw_suffix + img_raw_topic;
        mRawRightTopic = topicPrefix + "it_" + rightTopicRoot + raw_suffix + img_raw_topic;
        mRawLeftTopic = topicPrefix + "it_" + leftTopicRoot + raw_suffix + img_raw_topic;

        // Subscribers
        mRgbSub = create_subscription<sensor_msgs::msg::Image>(
                      rgbTopic,
                      std::bind(&ZedItBroadcaster::rgbCallback, this, _1),
                      mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRgbSub->get_topic_name());

        mRgbInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                          rgbCamInfoTopic,
                          std::bind(&ZedItBroadcaster::rgbInfoCallback, this, _1),
                          mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRgbInfoSub->get_topic_name());

        mRightSub = create_subscription<sensor_msgs::msg::Image>(
                        rightTopic,
                        std::bind(&ZedItBroadcaster::rightCallback, this, _1),
                        mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRightSub->get_topic_name());

        mRightInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                            rightCamInfoTopic,
                            std::bind(&ZedItBroadcaster::rightInfoCallback, this, _1),
                            mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRightInfoSub->get_topic_name());

        mLeftSub = create_subscription<sensor_msgs::msg::Image>(
                       leftTopic,
                       std::bind(&ZedItBroadcaster::leftCallback, this, _1),
                       mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mLeftSub->get_topic_name());


        mLeftInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                           leftCamInfoTopic,
                           std::bind(&ZedItBroadcaster::leftInfoCallback, this, _1),
                           mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mLeftInfoSub->get_topic_name());

        mRawRgbSub = create_subscription<sensor_msgs::msg::Image>(
                         rgbRawTopic,
                         std::bind(&ZedItBroadcaster::rgbRawCallback, this, _1),
                         mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRgbSub->get_topic_name());

        mRawRgbInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                             rgbCamInfoRawTopic,
                             std::bind(&ZedItBroadcaster::rgbInfoRawCallback, this, _1),
                             mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRgbInfoSub->get_topic_name());

        mRawRightSub = create_subscription<sensor_msgs::msg::Image>(
                           rightRawTopic,
                           std::bind(&ZedItBroadcaster::rightRawCallback, this, _1),
                           mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRightSub->get_topic_name());

        mRawRightInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                               rightCamInfoRawTopic,
                               std::bind(&ZedItBroadcaster::rightInfoRawCallback, this, _1),
                               mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRightInfoSub->get_topic_name());

        mRawLeftSub = create_subscription<sensor_msgs::msg::Image>(
                          leftRawTopic,
                          std::bind(&ZedItBroadcaster::leftRawCallback, this, _1),
                          mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawLeftSub->get_topic_name());


        mRawLeftInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                              leftCamInfoRawTopic,
                              std::bind(&ZedItBroadcaster::leftInfoRawCallback, this, _1),
                              mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawLeftInfoSub->get_topic_name());

    }

    void ZedItBroadcaster::initPub() {
        if (mPubInitialized) {
            return;
        }

        RCLCPP_INFO(get_logger(), "IMAGE TRANSPORT TOPICS:");

        mRgbPub = image_transport::create_camera_publisher(this, mRgbTopic, mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRgbPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mRgbPub.getInfoTopic().c_str());

        mRightPub = image_transport::create_camera_publisher(this, mRightTopic, mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRightPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mRightPub.getInfoTopic().c_str());

        mLeftPub = image_transport::create_camera_publisher(this, mLeftTopic, mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * '%s'", mLeftPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mLeftPub.getInfoTopic().c_str());

        mRawRgbPub = image_transport::create_camera_publisher(this, mRawRgbTopic, mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRawRgbPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mRawRgbPub.getInfoTopic().c_str());

        mRawRightPub = image_transport::create_camera_publisher(this, mRawRightTopic, mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRawRightPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mRawRightPub.getInfoTopic().c_str());

        mRawLeftPub = image_transport::create_camera_publisher(this, mRawLeftTopic, mCamQosProfile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRawLeftPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mRawLeftPub.getInfoTopic().c_str());

        mPubInitialized = true;
    }

    void ZedItBroadcaster::rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        //        if (mRgbPub.getNumSubscribers() > 0) {
        mRgbPub.publish(*msg, mRgbInfoMsg);
        //        }
    }

    void ZedItBroadcaster::rgbInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPub();

        mRgbInfoMsg = *msg;
    }

    void ZedItBroadcaster::rightCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        //        if (mRightPub.getNumSubscribers() > 0) {
        mRightPub.publish(*msg, mRightInfoMsg);
        //        }
    }

    void ZedItBroadcaster::rightInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPub();

        mRightInfoMsg = *msg;
    }

    void ZedItBroadcaster::leftCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        //        if (mLeftPub.getNumSubscribers() > 0) {
        mLeftPub.publish(*msg, mLeftInfoMsg);
        //        }
    }

    void ZedItBroadcaster::leftInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPub();

        mLeftInfoMsg = *msg;
    }

    void ZedItBroadcaster::rgbRawCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        //        if (mRawRgbPub.getNumSubscribers() > 0) {
        mRawRgbPub.publish(*msg, mRawRgbInfoMsg);
        //        }
    }

    void ZedItBroadcaster::rgbInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPub();

        mRawRgbInfoMsg = *msg;
    }

    void ZedItBroadcaster::rightRawCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        //        if (mRawRightPub.getNumSubscribers() > 0) {
        mRawRightPub.publish(*msg, mRawRightInfoMsg);
        //        }
    }

    void ZedItBroadcaster::rightInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPub();

        mRawRightInfoMsg = *msg;
    }

    void ZedItBroadcaster::leftRawCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        //        if (mRawLeftPub.getNumSubscribers() > 0) {
        mRawLeftPub.publish(*msg, mRawLeftInfoMsg);
        //        }
    }

    void ZedItBroadcaster::leftInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPub();

        mRawLeftInfoMsg = *msg;
    }

}  // namespace stereolabs
