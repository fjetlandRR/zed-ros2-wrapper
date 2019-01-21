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

        RCLCPP_INFO(get_logger(), "ZED Image Transport Broadcaster node: %s", get_name());
        RCLCPP_INFO(get_logger(), "ZED Image Transport Broadcaster namespace: %s", get_namespace());

        // Topics
        std::string topicPrefix = get_namespace();

        if (topicPrefix.length() > 1) {
            topicPrefix += "/";
        }

        topicPrefix += get_name();
        topicPrefix += "/";

        // Parameters
        initParameters();

        // Video topics
        std::string img_topic = "/image_rect_color";
        std::string img_raw_topic = "/image_raw_color";
        std::string cam_info_topic = "/camera_info";
        std::string raw_suffix = "_raw";
        std::string it_prefix = "it_";
        std::string depth_topic = "/depth_registered";
        std::string depth_openni_topic = "/depth_raw_registered";

        // Set the default input topic names
        std::string leftTopic = topicPrefix + mLeftTopicRoot + img_topic;
        std::string leftCamInfoTopic = leftTopic + cam_info_topic;
        std::string leftRawTopic = topicPrefix + mLeftTopicRoot + raw_suffix + img_raw_topic;
        std::string leftCamInfoRawTopic = leftRawTopic + cam_info_topic;

        std::string rightTopic = topicPrefix + mRightTopicRoot + img_topic;
        std::string rightCamInfoTopic = rightTopic + cam_info_topic;
        std::string rightRawTopic = topicPrefix + mRightTopicRoot + raw_suffix + img_raw_topic;
        std::string rightCamInfoRawTopic = rightRawTopic + cam_info_topic;

        std::string rgbTopic = topicPrefix + mRgbTopicRoot + img_topic;
        std::string rgbCamInfoTopic = rgbTopic + cam_info_topic;
        std::string rgbRawTopic = topicPrefix + mRgbTopicRoot + raw_suffix + img_raw_topic;
        std::string rgbCamInfoRawTopic = rgbRawTopic + cam_info_topic;

        std::string depthTopic = topicPrefix + mDepthTopicRoot + (mOpenniDepthMode ? depth_openni_topic :
                                 depth_topic);
        std::string depthCamInfoTopic = depthTopic + cam_info_topic;

        // Image Transport output topic names
        mRgbTopic = topicPrefix + it_prefix + mRgbTopicRoot + img_topic;
        mRightTopic = topicPrefix + it_prefix + mRightTopicRoot + img_topic;
        mLeftTopic = topicPrefix + it_prefix + mLeftTopicRoot + img_topic;
        mRawRgbTopic = topicPrefix + it_prefix + mRgbTopicRoot + raw_suffix + img_raw_topic;
        mRawRightTopic = topicPrefix + it_prefix + mRightTopicRoot + raw_suffix + img_raw_topic;
        mRawLeftTopic = topicPrefix + it_prefix + mLeftTopicRoot + raw_suffix + img_raw_topic;

        mDepthTopic = topicPrefix + it_prefix + mDepthTopicRoot + (mOpenniDepthMode ? depth_openni_topic :
                      depth_topic);

        // Subscribers QoS
        // Note: settings compatible with all the possible publisher QoS settings
        rmw_qos_profile_t subQos;
        subQos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        subQos.depth = 1;
        subQos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        subQos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

        // Video Subscribers
        mRgbSub = create_subscription<sensor_msgs::msg::Image>(
                      rgbTopic,
                      std::bind(&ZedItBroadcaster::rgbCallback, this, _1),
                      subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRgbSub->get_topic_name());

        mRgbInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                          rgbCamInfoTopic,
                          std::bind(&ZedItBroadcaster::rgbInfoCallback, this, _1),
                          subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRgbInfoSub->get_topic_name());

        mRightSub = create_subscription<sensor_msgs::msg::Image>(
                        rightTopic,
                        std::bind(&ZedItBroadcaster::rightCallback, this, _1),
                        subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRightSub->get_topic_name());

        mRightInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                            rightCamInfoTopic,
                            std::bind(&ZedItBroadcaster::rightInfoCallback, this, _1),
                            subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRightInfoSub->get_topic_name());

        mLeftSub = create_subscription<sensor_msgs::msg::Image>(
                       leftTopic,
                       std::bind(&ZedItBroadcaster::leftCallback, this, _1),
                       subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mLeftSub->get_topic_name());


        mLeftInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                           leftCamInfoTopic,
                           std::bind(&ZedItBroadcaster::leftInfoCallback, this, _1),
                           subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mLeftInfoSub->get_topic_name());

        mRawRgbSub = create_subscription<sensor_msgs::msg::Image>(
                         rgbRawTopic,
                         std::bind(&ZedItBroadcaster::rgbRawCallback, this, _1),
                         subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRgbSub->get_topic_name());

        mRawRgbInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                             rgbCamInfoRawTopic,
                             std::bind(&ZedItBroadcaster::rgbInfoRawCallback, this, _1),
                             subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRgbInfoSub->get_topic_name());

        mRawRightSub = create_subscription<sensor_msgs::msg::Image>(
                           rightRawTopic,
                           std::bind(&ZedItBroadcaster::rightRawCallback, this, _1),
                           subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRightSub->get_topic_name());

        mRawRightInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                               rightCamInfoRawTopic,
                               std::bind(&ZedItBroadcaster::rightInfoRawCallback, this, _1),
                               subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRightInfoSub->get_topic_name());

        mRawLeftSub = create_subscription<sensor_msgs::msg::Image>(
                          leftRawTopic,
                          std::bind(&ZedItBroadcaster::leftRawCallback, this, _1),
                          subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawLeftSub->get_topic_name());


        mRawLeftInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                              leftCamInfoRawTopic,
                              std::bind(&ZedItBroadcaster::leftInfoRawCallback, this, _1),
                              subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawLeftInfoSub->get_topic_name());

        // Depth Subsubscribers
        mDepthSub = create_subscription<sensor_msgs::msg::Image>(
                        depthTopic,
                        std::bind(&ZedItBroadcaster::depthCallback, this, _1),
                        subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mDepthSub->get_topic_name());

        mDepthInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                            depthCamInfoTopic,
                            std::bind(&ZedItBroadcaster::depthInfoCallback, this, _1),
                            subQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mDepthInfoSub->get_topic_name());
    }

    void ZedItBroadcaster::initParameters() {
        rclcpp::Parameter paramVal;
        std::string paramName;

        paramName = "video.rgb_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mRgbTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.left_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mLeftTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.right_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mRightTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.openni_depth_mode";

        if (get_parameter(paramName, paramVal)) {
            mOpenniDepthMode = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.depth_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mDepthTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.qos_history";

        if (get_parameter(paramName, paramVal)) {
            mVideoQos.history = paramVal.as_int() == 0 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.qos_depth";

        if (get_parameter(paramName, paramVal)) {
            mVideoQos.depth = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.qos_reliability";

        if (get_parameter(paramName, paramVal)) {
            mVideoQos.reliability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT :
                                    RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.qos_durability";

        if (get_parameter(paramName, paramVal)) {
            mVideoQos.durability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL :
                                   RMW_QOS_POLICY_DURABILITY_VOLATILE;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.qos_history";

        if (get_parameter(paramName, paramVal)) {
            mDepthQos.history = paramVal.as_int() == 0 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.qos_depth";

        if (get_parameter(paramName, paramVal)) {
            mDepthQos.depth = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.qos_reliability";

        if (get_parameter(paramName, paramVal)) {
            mDepthQos.reliability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT :
                                    RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.qos_durability";

        if (get_parameter(paramName, paramVal)) {
            mDepthQos.durability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL :
                                   RMW_QOS_POLICY_DURABILITY_VOLATILE;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
    }

    void ZedItBroadcaster::initSubscribers() {

    }

    void ZedItBroadcaster::initPublishers() {
        if (mPubInitialized) {
            return;
        }

        RCLCPP_INFO(get_logger(), "IMAGE TRANSPORT TOPICS:");

        mRgbPub = image_transport::create_camera_publisher(this, mRgbTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * '%s'", mRgbPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mRgbPub.getInfoTopic().c_str());

        mRightPub = image_transport::create_camera_publisher(this, mRightTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * '%s'", mRightPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mRightPub.getInfoTopic().c_str());

        mLeftPub = image_transport::create_camera_publisher(this, mLeftTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * '%s'", mLeftPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mLeftPub.getInfoTopic().c_str());

        mRawRgbPub = image_transport::create_camera_publisher(this, mRawRgbTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * '%s'", mRawRgbPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mRawRgbPub.getInfoTopic().c_str());

        mRawRightPub = image_transport::create_camera_publisher(this, mRawRightTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * '%s'", mRawRightPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mRawRightPub.getInfoTopic().c_str());

        mRawLeftPub = image_transport::create_camera_publisher(this, mRawLeftTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * '%s'", mRawLeftPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mRawLeftPub.getInfoTopic().c_str());

        mDepthPub = image_transport::create_camera_publisher(this, mDepthTopic, mDepthQos);
        RCLCPP_INFO(get_logger(), " * '%s'", mDepthPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * '%s'", mDepthPub.getInfoTopic().c_str());

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
        initPublishers();

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
        initPublishers();

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
        initPublishers();

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
        initPublishers();

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
        initPublishers();

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
        initPublishers();

        mRawLeftInfoMsg = *msg;
    }

    void ZedItBroadcaster::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        //        if (mDepthPub.getNumSubscribers() > 0) {
        mDepthPub.publish(*msg, mDepthInfoMsg);
        //        }
    }

    void ZedItBroadcaster::depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPublishers();

        mDepthInfoMsg = *msg;
    }

}  // namespace stereolabs
