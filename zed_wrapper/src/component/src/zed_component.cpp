
#include "zed_component.hpp"
#include "sl_tools.h"
#include <string>

#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>

using namespace std::chrono_literals;

#ifndef TIMER_ELAPSED
#define TIMER_ELAPSED double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
#endif

namespace stereolabs {

    ZedCameraComponent::ZedCameraComponent(const std::string& node_name, const std::string& ros_namespace,
                                           bool intra_process_comms)
        : rclcpp_lifecycle::LifecycleNode(node_name, ros_namespace, intra_process_comms) {

#ifndef NDEBUG
        std::string logger = ros_namespace.empty() ? "" : ros_namespace + ".";
        logger += node_name;
        rcutils_ret_t res = rcutils_logging_set_logger_level(logger.c_str(), RCUTILS_LOG_SEVERITY_DEBUG);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting DEBUG logger");
        }
#endif
        RCLCPP_DEBUG(get_logger(), "[ROS2] Using RMW_IMPLEMENTATION = %s", rmw_get_implementation_identifier());

        RCLCPP_INFO(get_logger(), "ZED Camera Component created");

        RCLCPP_DEBUG(get_logger(), "ZED node: %s", get_name());
        RCLCPP_DEBUG(get_logger(), "ZED namespace: %s", get_namespace());

        RCLCPP_INFO(get_logger(), "Waiting for `CONFIGURE` request...");
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_shutdown(const rclcpp_lifecycle::State& previous_state) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        RCLCPP_DEBUG(get_logger(), "on_shutdown() is called.");
        RCLCPP_DEBUG(get_logger(), "Current state: %s", this->get_current_state().label().c_str());
        RCLCPP_DEBUG(get_logger(), "Previous state: %s (%d)", previous_state.label().c_str(), previous_state.id());

        // >>>>> Verify that all the threads are not active
        try {
            if (!mThreadStop) {
                mThreadStop = true;
                if (mGrabThread.joinable()) {
                    mGrabThread.join();
                }
                if (mPcThread.joinable()) {
                    mPcThread.join();
                }
            }
        } catch (std::system_error& e) {
            RCLCPP_WARN(get_logger(), "Thread joining exception: %s", e.what());
        }
        // <<<<< Verify that the grab thread is not active

        // >>>>> Verify that ZED is not opened
        if (mZed.isOpened()) {
            mZed.close();
            RCLCPP_INFO(get_logger(), "ZED Closed");
        }
        // <<<<< Verify that ZED is not opened

        RCLCPP_INFO(get_logger(), "shutdown complete");

        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    ZedCameraComponent::~ZedCameraComponent() {
        try {
            if (!mThreadStop) {
                mThreadStop = true;
                if (mGrabThread.joinable()) {
                    mGrabThread.join();
                }
                if (mPcThread.joinable()) {
                    mPcThread.join();
                }
            }
        } catch (std::system_error& e) {
            RCLCPP_WARN(get_logger(), "Thread joining exception: %s", e.what());
        }
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_error(const rclcpp_lifecycle::State& previous_state) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        RCLCPP_DEBUG(get_logger(), "on_error() is called.");
        RCLCPP_DEBUG(get_logger(), "Current state: %s", this->get_current_state().label().c_str());
        RCLCPP_DEBUG(get_logger(), "Previous state: %s (%d)", previous_state.label().c_str(), previous_state.id());

        if (mPrevTransition != 255) {
            switch (mPrevTransition) {
            case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE: { // Error during configuration
                RCLCPP_INFO(get_logger(), "Node entering 'FINALIZED' state. Kill and restart the node.");
                return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
            }
            break;

            case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE: { // Error during activation
                if (mSvoMode) {
                    RCLCPP_INFO(get_logger(), "Please verify the SVO path and reboot the node: %s", mSvoFilepath.c_str());
                    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
                } else {

                    if (mZedUserCamModel == 0) {
                        RCLCPP_INFO(get_logger(), "Please verify the USB connection");
                    } else {
                        RCLCPP_INFO(get_logger(), "Please verify the USB connection. Try to flip the USB TypeC cable on ZED mini");
                    }

                    RCLCPP_INFO(get_logger(), "Node entering 'UNCONFIGURED' state");

                    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
                }
            }
            break;

            case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
            case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
            default:
                RCLCPP_INFO(get_logger(), "Transition error not handled: %d", mPrevTransition);
                RCLCPP_INFO(get_logger(), "Node entering 'FINALIZED' state. Kill and restart the node.");
            }
        }

        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
    }

    void ZedCameraComponent::initParameters() {
        rclcpp::Parameter paramVal;
        std::string paramName;

        // >>>>> GENERAL parameters
        RCLCPP_INFO(get_logger(), "*** GENERAL parameters ***");

        paramName = "general.svo_file";
        if (get_parameter(paramName, paramVal)) {
            mSvoFilepath = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * SVO: `%s`", mSvoFilepath.c_str());

        paramName = "general.camera_model";
        if (get_parameter(paramName, paramVal)) {
            mZedUserCamModel = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Camera model: %d (%s)",
                    mZedUserCamModel, sl::toString(static_cast<sl::MODEL>(mZedUserCamModel)).c_str());

        paramName = "general.camera_flip";
        if (get_parameter(paramName, paramVal)) {
            mCameraFlip = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Camera flip: %s", mCameraFlip ? "TRUE" : "FALSE");

        paramName = "general.zed_id";
        if (get_parameter(paramName, paramVal)) {
            mZedId = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * ZED ID: %d", mZedId);

        paramName = "general.serial_number";
        if (get_parameter(paramName, paramVal)) {
            mZedSerialNumber = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * ZED serial number: %d", mZedSerialNumber);

        paramName = "general.resolution";
        if (get_parameter(paramName, paramVal)) {
            mZedResol = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * ZED resolution: %d (%s)", mZedResol,
                    sl::toString(static_cast<sl::RESOLUTION>(mZedResol)).c_str());

        paramName = "general.verbose";
        if (get_parameter(paramName, paramVal)) {
            mVerbose = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Verbose: %s", mVerbose ? "TRUE" : "FALSE");

        paramName = "general.mat_resize_factor";
        if (get_parameter(paramName, paramVal)) {
            mZedMatResizeFactor = paramVal.as_double();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Data resize factor: %g [DYNAMIC]", mZedMatResizeFactor);

        paramName = "general.frame_rate";
        if (get_parameter(paramName, paramVal)) {
            mZedFrameRate = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * ZED framerate: %d", mZedFrameRate);

        paramName = "general.gpu_id";
        if (get_parameter(paramName, paramVal)) {
            mGpuId = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * GPU ID: %d", mGpuId);
        // <<<<< GENERAL parameters

        // >>>>> VIDEO parameters
        RCLCPP_INFO(get_logger(), "*** VIDEO parameters ***");

        paramName = "video.auto_exposure";
        if (get_parameter(paramName, paramVal)) {
            mZedAutoExposure = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Auto exposure: %s [DYNAMIC]", mZedAutoExposure ? "TRUE" : "FALSE");

        paramName = "video.exposure";
        if (get_parameter(paramName, paramVal)) {
            mZedExposure = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Exposure: %d [DYNAMIC]", mZedExposure);

        paramName = "video.gain";
        if (get_parameter(paramName, paramVal)) {
            mZedGain = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Gain: %d [DYNAMIC]", mZedGain);

        paramName = "video.rgb_topic";
        if (get_parameter(paramName, paramVal)) {
            mRgbTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * RGB topic: '%s'", mRgbTopic.c_str());

        paramName = "video.rgb_raw_topic";
        if (get_parameter(paramName, paramVal)) {
            mRgbRawTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * RAW RGB topic: '%s'", mRgbRawTopic.c_str());

        paramName = "video.left_topic";
        if (get_parameter(paramName, paramVal)) {
            mLeftTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Left topic: '%s'", mLeftTopic.c_str());

        paramName = "video.left_raw_topic";
        if (get_parameter(paramName, paramVal)) {
            mLeftRawTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * RAW Left topic: '%s'", mLeftRawTopic.c_str());

        paramName = "video.right_topic";
        if (get_parameter(paramName, paramVal)) {
            mRightTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Right topic: '%s'", mRightTopic.c_str());

        paramName = "video.right_raw_topic";
        if (get_parameter(paramName, paramVal)) {
            mRightRawTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * RAW Right topic: '%s'", mRightRawTopic.c_str());
        // <<<<< VIDEO parameters

        // >>>>>> DEPTH parameters
        RCLCPP_INFO(get_logger(), "*** DEPTH parameters ***");

        paramName = "depth.quality";
        if (get_parameter(paramName, paramVal)) {
            mZedQuality = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Quality: %d (%s)", mZedQuality,
                    sl::toString(static_cast<sl::DEPTH_MODE>(mZedQuality)).c_str());

        paramName = "depth.sensing_mode";
        if (get_parameter(paramName, paramVal)) {
            mZedSensingMode = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Sensing mode: %d (%s)", mZedSensingMode,
                    sl::toString(static_cast<sl::SENSING_MODE>(mZedSensingMode)).c_str());

        paramName = "depth.confidence";
        if (get_parameter(paramName, paramVal)) {
            mZedConfidence = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Confidence: %d", mZedConfidence);

        paramName = "depth.depth_stabilization";
        if (get_parameter(paramName, paramVal)) {
            mDepthStabilization = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Depth stabilization: %s", mDepthStabilization ? "ENABLED" : "DISABLED");

        paramName = "depth.openni_depth_mode";
        if (get_parameter(paramName, paramVal)) {
            mOpenniDepthMode = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * OpenNI mode: %s", mOpenniDepthMode == 0 ? "DISABLED" : "ENABLED");

        paramName = "depth.depth_topic";
        if (get_parameter(paramName, paramVal)) {
            mDepthTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Depth topic: '%s'", mDepthTopic.c_str());

        paramName = "depth.point_cloud_topic";
        if (get_parameter(paramName, paramVal)) {
            mPointcloudTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Pointcloud topic: '%s'", mPointcloudTopic.c_str());

        paramName = "depth.disparity_topic";
        if (get_parameter(paramName, paramVal)) {
            mDispTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Disparity topic: '%s'", mDispTopic.c_str());

        paramName = "depth.confidence_map_topic";
        if (get_parameter(paramName, paramVal)) {
            mConfMapTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Confidence map topic: '%s'", mConfMapTopic.c_str());

        paramName = "depth.confidence_img_topic";
        if (get_parameter(paramName, paramVal)) {
            mConfImgTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
        RCLCPP_INFO(get_logger(), " * Confidence image topic: '%s'", mConfImgTopic.c_str());
        // <<<<<< DEPTH parameters

        // >>>>>> TRACKING parameters
        //RCLCPP_INFO(get_logger(), "*** TRACKING parameters ***" );
        // TODO parse tracking parameters when TRACKING is available
        // <<<<<< TRACKING parameters

        // >>>>>> IMU parameters
        if (mZedUserCamModel == 1) {
            RCLCPP_INFO(get_logger(), "*** IMU parameters ***");
            // TODO parse IMU parameters from zedm.yaml
        }
        // <<<<<< IMU parameters

    }

    void ZedCameraComponent::initPublishers() {
        RCLCPP_INFO(get_logger(), "*** PUBLISHED TOPICS ***");

        std::string topicPrefix = get_namespace();
        if (topicPrefix.length() > 1) {
            topicPrefix += "/";
        }
        topicPrefix += get_name();
        topicPrefix += "/";

        // >>>>> Image publishers QOS
        // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
        rmw_qos_profile_t camera_qos_profile = rmw_qos_profile_sensor_data; // Default QOS profile

        //        camera_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE; // to be sure to publish images
        //        camera_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;// KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth" parameter
        //        camera_qos_profile.depth = 1; // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
        //        camera_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;

        // >>>>> Video topics
        std::string img_topic = "image_rect_color";
        std::string img_raw_topic = "image_raw_color";
        std::string cam_info_topic = "/camera_info";
        // Set the default topic names
        mLeftTopic = topicPrefix + "left/" + img_topic;
        mLeftCamInfoTopic = mLeftTopic + cam_info_topic;
        mLeftRawTopic = topicPrefix + "left/" + img_raw_topic;
        mLeftCamInfoRawTopic = mLeftRawTopic + cam_info_topic;

        mRightTopic = topicPrefix + "right/" + img_topic;
        mRightCamInfoTopic = mRightTopic + cam_info_topic;
        mRightRawTopic = topicPrefix + "right/" + img_raw_topic;
        mRightCamInfoRawTopic = mRightRawTopic + cam_info_topic;

        mRgbTopic = topicPrefix + "rgb/" + img_topic;
        mRgbCamInfoTopic = mRgbTopic + cam_info_topic;
        mRgbRawTopic = topicPrefix + "rgb/" + img_raw_topic;
        mRgbCamInfoRawTopic = mRgbRawTopic + cam_info_topic;
        // <<<<< Video topics

        // >>>>> Depth Topics
        mDepthTopic = topicPrefix + "depth/";
        if (mOpenniDepthMode) {
            RCLCPP_INFO(get_logger(), "Openni depth mode activated");
            mDepthTopic += "depth_raw_registered";
        } else {
            mDepthTopic += "depth_registered";
        }
        std::string mDepthCamInfoTopic = mDepthTopic + cam_info_topic;

        mConfImgTopic = topicPrefix + "confidence/image";
        mConfidenceCamInfoTopic = mConfImgTopic + cam_info_topic;
        mConfMapTopic = topicPrefix + "confidence/map";

        mDispTopic = topicPrefix + "disparity/disparity_image";

        mPointcloudTopic = topicPrefix + "point_cloud/cloud_registered";
        // <<<<< Depth Topics

        // >>>>> Create Video publishers
        mPubRgb = create_publisher<sensor_msgs::msg::Image>(mRgbTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRgbTopic.c_str());
        mPubRawRgb = create_publisher<sensor_msgs::msg::Image>(mRgbRawTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRgbRawTopic.c_str());
        mPubLeft = create_publisher<sensor_msgs::msg::Image>(mLeftTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mLeftTopic.c_str());
        mPubRawLeft = create_publisher<sensor_msgs::msg::Image>(mLeftRawTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mLeftRawTopic.c_str());
        mPubRight = create_publisher<sensor_msgs::msg::Image>(mRightTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRightTopic.c_str());
        mPubRawRight = create_publisher<sensor_msgs::msg::Image>(mRightRawTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRightRawTopic.c_str());
        mPubDepth = create_publisher<sensor_msgs::msg::Image>(mDepthTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mDepthTopic.c_str());
        mPubConfImg = create_publisher<sensor_msgs::msg::Image>(mConfImgTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mConfImgTopic.c_str());
        mPubConfMap = create_publisher<sensor_msgs::msg::Image>(mConfMapTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mConfMapTopic.c_str());
        // <<<<< Create Video publishers

        // >>>>> Create Camera Info publishers
        mPubRgbCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(mRgbCamInfoTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRgbCamInfoTopic.c_str());
        mPubRgbCamInfoRaw = create_publisher<sensor_msgs::msg::CameraInfo>(mRgbCamInfoRawTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRgbCamInfoRawTopic.c_str());
        mPubLeftCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(mLeftCamInfoTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mLeftCamInfoTopic.c_str());
        mPubLeftCamInfoRaw = create_publisher<sensor_msgs::msg::CameraInfo>(mLeftCamInfoRawTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mLeftCamInfoRawTopic.c_str());
        mPubRightCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(mRightCamInfoTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRightCamInfoTopic.c_str());
        mPubRightCamInfoRaw = create_publisher<sensor_msgs::msg::CameraInfo>(mRightCamInfoRawTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mRightCamInfoRawTopic.c_str());
        mPubDepthCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(mDepthCamInfoTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mDepthCamInfoTopic.c_str());
        mPubConfidenceCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(mConfidenceCamInfoTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mConfidenceCamInfoTopic.c_str());
        // <<<<< Create Camera Info publishers


        // >>>>> Create Depth Publishers
        // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
        rmw_qos_profile_t depth_qos_profile = rmw_qos_profile_sensor_data; // Default QOS profile
        depth_qos_profile.depth = 2;

        mPubPointcloud = create_publisher<sensor_msgs::msg::PointCloud2>(mPointcloudTopic, depth_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mPointcloudTopic.c_str());

        mPubDisparity = create_publisher<stereo_msgs::msg::DisparityImage>(mDispTopic, depth_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mDispTopic.c_str());
        // <<<<< Create Depth Publishers
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_configure(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

        RCLCPP_DEBUG(get_logger(), "on_configure() is called.");

        // >>>>> Check SDK version
#if (ZED_SDK_MAJOR_VERSION<2 || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION<5))
        RCLCPP_ERROR(get_logger(), "ROS2 ZED node requires ZED SDK > v2.5.0");

        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR;
#endif
        // <<<<< Check SDK version

        // >>>>> Load params from param server
        initParameters();
        // <<<<< Load params from param server

        // >>>>> Frame IDs
        mRightCamFrameId = "zed_right_camera_frame";
        mRightCamOptFrameId = "zed_right_camera_optical_frame";
        mLeftCamFrameId = "zed_left_camera_frame";
        mLeftCamOptFrameId = "zed_left_camera_optical_frame";

        mDepthFrameId = mLeftCamFrameId;
        mDepthOptFrameId = mLeftCamOptFrameId;

        mBaseFrameId = "base_link";
        mCameraFrameId = "zed_camera_center";
        mImuFrameId = "zed_imu_link";
        // <<<<< Frame IDs

        // >>>>> Create camera info
        mRgbCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mLeftCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mRightCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mRgbCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mLeftCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mRightCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mDepthCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        // <<<<< Create camera info

        // Create pointcloud message
        mPointcloudMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // Initialize Message Publishers
        initPublishers();

        // >>>>> ZED configuration
        if (!mSvoFilepath.empty()) {
            RCLCPP_INFO(get_logger(), "*** SVO OPENING ***");

            mZedParams.svo_input_filename = mSvoFilepath.c_str();
            mZedParams.svo_real_time_mode = true;
            mSvoMode = true;
        } else {
            RCLCPP_INFO(get_logger(), "*** CAMERA OPENING ***");

            mZedParams.camera_fps = mZedFrameRate;
            mZedParams.camera_resolution = static_cast<sl::RESOLUTION>(mZedResol);

            if (mZedSerialNumber == 0) {
                mZedParams.camera_linux_id = mZedId;
            }
        }
        mZedParams.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD;
        mZedParams.coordinate_units = sl::UNIT_METER;
        mZedParams.depth_mode = static_cast<sl::DEPTH_MODE>(mZedQuality);
        mZedParams.sdk_verbose = mVerbose;
        mZedParams.sdk_gpu_id = mGpuId;
        mZedParams.depth_stabilization = mDepthStabilization;
        mZedParams.camera_image_flip = mCameraFlip;
        // <<<<< ZED configuration

        // >>>>> Try to open ZED camera or to load SVO
        INIT_TIMER
        START_TIMER

        if (mSvoFilepath != "") {
            mSvoMode = true;
        }

        mThreadStop = false;

        if (mZedSerialNumber == 0) {
            mZedParams.camera_linux_id = mZedId;
        } else {
            bool waiting_for_camera = true;

            while (waiting_for_camera) {
                // Ctrl+C check
                if (!rclcpp::ok()) {

                    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
                }

                sl::DeviceProperties prop = sl_tools::getZEDFromSN(mZedSerialNumber);

                if (prop.id < -1 ||
                    prop.camera_state == sl::CAMERA_STATE::CAMERA_STATE_NOT_AVAILABLE) {
                    std::string msg = "ZED SN " + std::to_string(mZedSerialNumber) +
                                      " not detected ! Please connect this ZED";
                    RCLCPP_INFO(get_logger(), msg.c_str());
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                } else {
                    waiting_for_camera = false;
                    mZedParams.camera_linux_id = prop.id;
                }

                TIMER_ELAPSED

                if (elapsed > mZedTimeoutMsec) {
                    RCUTILS_LOG_WARN_NAMED(get_name(), "Camera detection timeout");

                    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR;
                }
            }
        }

        START_TIMER

        while (1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            sl::ERROR_CODE err = mZed.open(mZedParams);
            if (err == sl::SUCCESS) {
                RCLCPP_INFO(get_logger(), "ZED Opened");
                break;
            }

            RCUTILS_LOG_WARN_NAMED(get_name(), toString(err));

            if (mSvoMode) {
                return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR;
            }

            if (!rclcpp::ok() || mThreadStop) {
                RCLCPP_INFO(get_logger(), "ZED activation interrupted");

                return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
            }

            TIMER_ELAPSED

            if (elapsed > mZedTimeoutMsec) {
                RCUTILS_LOG_WARN_NAMED(get_name(), "Camera detection timeout");

                return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR;
            }
        }
        // <<<<< Try to open ZED camera or to load SVO

        sl::CameraInformation camInfo = mZed.getCameraInformation();
        mZedRealCamModel = camInfo.camera_model;

        if (mZedRealCamModel == sl::MODEL_ZED) {

            if (mZedUserCamModel != 0) {
                RCUTILS_LOG_WARN_NAMED(get_name(), "Camera model does not match user parameter. Please modify "
                                       "the value of the parameter 'camera_model' to 0");
            }
        } else if (mZedRealCamModel == sl::MODEL_ZED_M) {

            if (mZedUserCamModel != 1) {
                RCUTILS_LOG_WARN_NAMED(get_name(), "Camera model does not match user parameter. Please modify "
                                       "the value of the parameter 'camera_model' to 1");
            }
        }

        // >>>>> Camera Parameters user feedback
        RCLCPP_INFO(get_logger(), "CAMERA MODEL: %s", sl::toString(mZedRealCamModel).c_str());
        mZedSerialNumber = mZed.getCameraInformation().serial_number;
        RCLCPP_INFO(get_logger(), "SERIAL NUMBER: %s", std::to_string(mZedSerialNumber).c_str());
        RCLCPP_INFO(get_logger(), "FW VERSION: %s", std::to_string(camInfo.firmware_version).c_str());

        RCLCPP_INFO(get_logger(), "RESOLUTION: %s", sl::toString(mZedParams.camera_resolution).c_str());
        RCLCPP_INFO(get_logger(), "FRAMERATE: %s FPS", std::to_string(mZedParams.camera_fps).c_str());
        RCLCPP_INFO(get_logger(), "CAMERA FLIPPED: %s", std::to_string(mZedParams.camera_image_flip).c_str());
        RCLCPP_INFO(get_logger(), "COORDINATE SYSTEM: %s", sl::toString(mZedParams.coordinate_system).c_str());
        RCLCPP_INFO(get_logger(), "COORDINATE UNITS: %s", sl::toString(mZedParams.coordinate_units).c_str());
        RCLCPP_INFO(get_logger(), "DEPTH MODE: %s", sl::toString(mZedParams.depth_mode).c_str());
        float minDist = mZedParams.depth_minimum_distance;
        minDist = minDist == -1.0f ? (mZedRealCamModel == sl::MODEL_ZED_M ? 0.2f : 0.7f) : minDist;
        RCLCPP_INFO(get_logger(), "DEPTH MINIMUM DISTANCE: %s m", std::to_string(minDist).c_str());
        RCLCPP_INFO(get_logger(), "DEPTH STABILIZATION: %s", std::to_string(mZedParams.depth_stabilization).c_str());
        RCLCPP_INFO(get_logger(), "GPU ID: %s", std::to_string(mZedParams.sdk_gpu_id).c_str());
        // <<<<< Camera Parameters user feedback

        // >>>>> Images info
        // Get the parameters of the ZED images
        mCamWidth = mZed.getResolution().width;
        mCamHeight = mZed.getResolution().height;
        RCLCPP_INFO(get_logger(), "Camera Frame size: %d x %d", mCamWidth, mCamHeight);
        mMatWidth = static_cast<int>(mCamWidth * mZedMatResizeFactor);
        mMatHeight = static_cast<int>(mCamHeight * mZedMatResizeFactor);
        RCLCPP_INFO(get_logger(), "Data size: %d x %d (Resize factor: %g)", mMatWidth, mMatHeight, mZedMatResizeFactor);

        // Create and fill the camera information messages
        fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId, mRightCamOptFrameId);
        fillCamInfo(mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId, mRightCamOptFrameId, true);
        mRgbCamInfoMsg = mLeftCamInfoMsg;
        mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
        mDepthCamInfoMsg = mLeftCamInfoMsg;
        mConfidenceCamInfoMsg = mLeftCamInfoMsg;
        // <<<<< Images info

        RCLCPP_INFO(get_logger(), "ZED configured");
        RCLCPP_INFO(get_logger(), "Waiting for `ACTIVATE` request...");

        // We return a success and hence invoke the transition to the next
        // step: "inactive".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "unconfigured" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_activate(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

        RCLCPP_DEBUG(get_logger(), "on_activate() is called.");

        if (!mZed.isOpened()) {
            RCLCPP_WARN(get_logger(), "ZED Camera not opened");
            return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
        }

        // >>>>> Publishers activation
        mPubRgb->on_activate();
        mPubRawRgb->on_activate();
        mPubLeft->on_activate();
        mPubRawLeft->on_activate();
        mPubRight->on_activate();
        mPubRawRight->on_activate();
        mPubDepth->on_activate();
        mPubConfImg->on_activate();
        mPubConfMap->on_activate();

        mPubRgbCamInfo->on_activate();
        mPubRgbCamInfoRaw->on_activate();
        mPubLeftCamInfo->on_activate();
        mPubLeftCamInfoRaw->on_activate();
        mPubRightCamInfo->on_activate();
        mPubRightCamInfoRaw->on_activate();
        mPubDepthCamInfo->on_activate();
        mPubConfidenceCamInfo->on_activate();

        mPubDisparity->on_activate();

        mPubPointcloud->on_activate();
        // <<<<< Publishers activation

        // >>>>> Start Pointcloud thread
        mPcDataReady = false;
        RCLCPP_DEBUG(get_logger(), "on_activate -> mPcDataReady FALSE")
        mPcThread = std::thread(&ZedCameraComponent::pointcloudThreadFunc, this);
        // <<<<< Start Pointcloud thread

        // >>>>> Start ZED thread
        mThreadStop = false;
        mGrabThread = std::thread(&ZedCameraComponent::zedGrabThreadFunc, this);
        // <<<<< Start ZED thread

        // We return a success and hence invoke the transition to the next
        // step: "active".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "inactive" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_deactivate(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;

        RCLCPP_DEBUG(get_logger(), "on_deactivate() is called.");

        // >>>>> Verify that all the threads are not active
        try {
            if (!mThreadStop) {
                mThreadStop = true;
                if (mGrabThread.joinable()) {
                    mGrabThread.join();
                }
                if (mPcThread.joinable()) {
                    mPcThread.join();
                }
            }
        } catch (std::system_error& e) {
            RCLCPP_WARN(get_logger(), "Thread joining exception: %s", e.what());
        }
        // <<<<< Verify that the grab thread is not active

        // >>>>> Publishers deactivation
        mPubRgb->on_deactivate();
        mPubRawRgb->on_deactivate();
        mPubLeft->on_deactivate();
        mPubRawLeft->on_deactivate();
        mPubRight->on_deactivate();
        mPubRawRight->on_deactivate();
        mPubDepth->on_deactivate();
        mPubConfImg->on_deactivate();
        mPubConfMap->on_deactivate();

        mPubRgbCamInfo->on_deactivate();
        mPubRgbCamInfoRaw->on_deactivate();
        mPubLeftCamInfo->on_deactivate();
        mPubLeftCamInfoRaw->on_deactivate();
        mPubRightCamInfo->on_deactivate();
        mPubRightCamInfoRaw->on_deactivate();
        mPubDepthCamInfo->on_deactivate();
        mPubConfidenceCamInfo->on_deactivate();

        mPubDisparity->on_deactivate();

        mPubPointcloud->on_deactivate();
        // <<<<< Publishers deactivation

        // We return a success and hence invoke the transition to the next
        // step: "inactive".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "active" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_cleanup(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

        RCLCPP_DEBUG(get_logger(), "on_cleanup() is called.");

        // >>>>> Close ZED if opened
        if (mZed.isOpened()) {
            mZed.close();
            RCLCPP_INFO(get_logger(), "ZED closed");
        }
        // <<<<< Close ZED if opened

        // TODO clean data structures

        // We return a success and hence invoke the transition to the next
        // step: "unconfigured".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "inactive" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    void ZedCameraComponent::zedGrabThreadFunc() {
        RCLCPP_INFO(get_logger(), "ZED thread started");

        mPrevTransition = 255;
        sl::ERROR_CODE grab_status;

        // >>>>> Last frame time initialization
        rclcpp::Time startTime;
        if (mSvoMode) {
            startTime = now();
        } else {
            startTime = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_CURRENT));
        }
        mLastGrabTimestamp = startTime;
        // <<<<< Last frame time initialization

        // >>>>> Grab parameters
        sl::RuntimeParameters runParams;
        runParams.sensing_mode = static_cast<sl::SENSING_MODE>(mZedSensingMode);
        runParams.enable_depth = false;
        runParams.enable_point_cloud = false;
        // <<<<< Grab parameters

        rclcpp::Rate loop_rate(mZedFrameRate);

        INIT_TIMER;

        while (1) {
            //>>>>> Interruption check
            if (!rclcpp::ok()) {
                RCLCPP_DEBUG(get_logger(), "Ctrl+C received");
                break;
            }

            if (mThreadStop) {
                RCLCPP_DEBUG(get_logger(), "Grab thread stopped");
                break;
            }
            //<<<<< Interruption check

            //>>>>> Subscribers check
            size_t rgbSub = count_subscribers(mRgbTopic);           // mPubRgb subscribers
            size_t rgbRawSub = count_subscribers(mRgbRawTopic);     // mPubRawRgb subscribers
            size_t leftSub = count_subscribers(mLeftTopic);         // mPubLeft subscribers
            size_t leftRawSub = count_subscribers(mLeftRawTopic);   // mPubRawLeft subscribers
            size_t rightSub = count_subscribers(mRightTopic);       // mPubRight subscribers
            size_t rightRawSub = count_subscribers(mRightRawTopic); // mPubRawRight subscribers
            size_t depthSub = count_subscribers(mDepthTopic);       // mPubDepth subscribers
            size_t confImgSub = count_subscribers(mConfImgTopic);   // mPubConfImg subscribers
            size_t confMapSub = count_subscribers(mConfMapTopic);   // mPubConfMap subscribers
            size_t dispSub = count_subscribers(mDispTopic);         // mPubDisparity subscribers
            size_t cloudSub = count_subscribers(mPointcloudTopic);  // mPubPointcloud subscribers

            bool pubImages = ((rgbSub + rgbRawSub + leftSub + leftRawSub + rightSub + rightRawSub) > 0);
            bool pubDepthData = ((depthSub + confImgSub + confMapSub + dispSub + cloudSub) > 0);

            bool runLoop = pubImages | pubDepthData;
            //<<<<< Subscribers check

            if (runLoop) {
                if (pubDepthData) {
                    int actual_confidence = mZed.getConfidenceThreshold();

                    if (actual_confidence != mZedConfidence) {
                        mZed.setConfidenceThreshold(mZedConfidence);
                    }

                    double actual_max_depth = static_cast<double>(mZed.getDepthMaxRangeValue());

                    if (actual_max_depth != mZedMaxDepth) {
                        mZed.setDepthMaxRangeValue(static_cast<double>(mZedMaxDepth));
                    }

                    runParams.enable_depth = true; // Ask to compute the depth
                } else {
                    runParams.enable_depth = false;
                }

                // ZED grab
                grab_status = mZed.grab(runParams);

                // >>>>> Timestamp
                rclcpp::Time grabTimestamp;
                if (mSvoMode) {
                    grabTimestamp = now();
                } else {
                    grabTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_IMAGE));
                }
                // <<<<< Timestamp

                if (grab_status != sl::ERROR_CODE::SUCCESS) {
                    // Detect if a error occurred (for example:
                    // the zed have been disconnected) and
                    // re-initialize the ZED
                    if (grab_status != sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                        RCLCPP_WARN(get_logger(), sl::toString(grab_status));
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(500));

                    rclcpp::Time now = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_CURRENT));

                    rcl_time_point_value_t elapsed = (now - mLastGrabTimestamp).nanoseconds();
                    rcl_time_point_value_t timeout = rclcpp::Duration(5, 0).nanoseconds();

                    if (elapsed > timeout && !mSvoMode) {
                        // TODO Better handle the error: throw exception?
                        //throw std::runtime_error("ZED Camera timeout");
                        RCLCPP_WARN(get_logger(), "Camera Timeout");
                    }

                    continue;
                }

                // Last frame timestamp
                mLastGrabTimestamp = grabTimestamp;

                mCamDataMutex.lock();

                if (pubImages) {
                    publishImages(grabTimestamp);
                }

                if (pubDepthData) {
                    publishDepthData(grabTimestamp);
                }

                mCamDataMutex.unlock();

                // >>>>> Thread sleep
                TIMER_ELAPSED;
                START_TIMER;

                static int rateWarnCount = 0;
                if (!loop_rate.sleep()) {
                    rateWarnCount++;

                    if (rateWarnCount == mZedFrameRate) {
                        RCLCPP_WARN(get_logger(),  "Expected cycle time: %g sec  - Real cycle time: %g sec ",
                                    1.0 / mZedFrameRate, elapsed / 1000.0);
                        RCLCPP_INFO(get_logger(),  "Elaboration takes longer than requested "
                                    "by the FPS rate. Please consider to "
                                    "lower the 'frame_rate' setting.");

                        rateWarnCount = 0;
                        loop_rate.reset();
                    }
                } else {
                    rateWarnCount = 0;
                }

                //RCLCPP_DEBUG(get_logger(), "Thread freq: %g Hz", 1000.0 / elapsed);
                // <<<<< Thread sleep
            } else {
                static int noSubInfoCount = 0;

                if (noSubInfoCount % 500 == 0) {
                    RCLCPP_INFO(get_logger(), "No subscribers");
                }

                noSubInfoCount++;

                std::this_thread::sleep_for(std::chrono::milliseconds(10));  // No subscribers, we just wait
                loop_rate.reset();
            }
        }

        RCLCPP_INFO(get_logger(), "ZED thread finished");
    }

    void ZedCameraComponent::publishImages(rclcpp::Time timeStamp) {
        size_t rgbSubnumber = count_subscribers(mRgbTopic);  // mPubRgb subscribers
        size_t rgbRawSubnumber = count_subscribers(mRgbRawTopic);  //mPubRawRgb subscribers
        size_t leftSubnumber = count_subscribers(mLeftTopic);  //mPubLeft subscribers
        size_t leftRawSubnumber = count_subscribers(mLeftRawTopic);  //mPubRawLeft subscribers
        size_t rightSubnumber = count_subscribers(mRightTopic);  //mPubRight subscribers
        size_t rightRawSubnumber = count_subscribers(mRightRawTopic);  //mPubRawRight subscribers

        sl::Mat leftZEDMat, rightZEDMat;

        // >>>>> Publish the left == rgb image if someone has subscribed to
        if (leftSubnumber > 0 || rgbSubnumber > 0) {
            // Retrieve RGBA Left image
            mZed.retrieveImage(leftZEDMat, sl::VIEW_LEFT, sl::MEM_CPU, mMatWidth, mMatHeight);

            if (leftSubnumber > 0) {
                publishCamInfo(mLeftCamInfoMsg, mPubLeftCamInfo, timeStamp);
                publishImage(leftZEDMat, mPubLeft, mLeftCamOptFrameId, timeStamp);
            }

            if (rgbSubnumber > 0) {
                publishCamInfo(mRgbCamInfoMsg, mPubRgbCamInfo, timeStamp);
                publishImage(leftZEDMat, mPubRgb, mDepthOptFrameId, timeStamp); // rgb is the left image
            }
        }
        // <<<<< Publish the left == rgb image if someone has subscribed to

        // >>>>> Publish the left_raw == rgb_raw image if someone has subscribed to
        if (leftRawSubnumber > 0 || rgbRawSubnumber > 0) {
            // Retrieve RGBA Left image
            mZed.retrieveImage(leftZEDMat, sl::VIEW_LEFT_UNRECTIFIED, sl::MEM_CPU, mMatWidth, mMatHeight);

            if (leftRawSubnumber > 0) {
                publishCamInfo(mLeftCamInfoRawMsg, mPubLeftCamInfoRaw, timeStamp);
                publishImage(leftZEDMat, mPubRawLeft, mLeftCamOptFrameId, timeStamp);
            }

            if (rgbRawSubnumber > 0) {
                publishCamInfo(mRgbCamInfoRawMsg, mPubRgbCamInfoRaw, timeStamp);
                publishImage(leftZEDMat, mPubRawRgb, mDepthOptFrameId, timeStamp);
            }
        }
        // <<<<< Publish the left_raw == rgb_raw image if someone has subscribed to

        // >>>>> Publish the right image if someone has subscribed to
        if (rightSubnumber > 0) {
            // Retrieve RGBA Right image
            mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT, sl::MEM_CPU, mMatWidth, mMatHeight);

            publishCamInfo(mRightCamInfoMsg, mPubRightCamInfo, timeStamp);
            publishImage(rightZEDMat, mPubRight, mRightCamOptFrameId, timeStamp);
        }
        // <<<<< Publish the right image if someone has subscribed to

        // >>>>> Publish the right image if someone has subscribed to
        if (rightRawSubnumber > 0) {
            // Retrieve RGBA Right image
            mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT_UNRECTIFIED, sl::MEM_CPU, mMatWidth, mMatHeight);

            publishCamInfo(mRightCamInfoRawMsg, mPubRightCamInfoRaw, timeStamp);
            publishImage(rightZEDMat, mPubRawRight, mRightCamOptFrameId, timeStamp);
        }
        // <<<<< Publish the right image if someone has subscribed to
    }

    void ZedCameraComponent::publishDepthData(rclcpp::Time timeStamp) {
        size_t depthSub = count_subscribers(mDepthTopic);       // mPubDepth subscribers
        size_t confImgSub = count_subscribers(mConfImgTopic);   // mConfImg subscribers
        size_t confMapSub = count_subscribers(mConfMapTopic);   // mConfMap subscribers
        size_t dispSub = count_subscribers(mDispTopic);         // mDisparity subscribers
        size_t cloudSub = count_subscribers(mPointcloudTopic);  //mPubPointcloud subscribers

        sl::Mat depthZEDMat, confImgZedMat, confMapZedMat, disparityZEDMat;

        // >>>>>  Publish the depth image if someone has subscribed to
        if (depthSub > 0 /*|| dispImgSub > 0*/) {
            mZed.retrieveMeasure(depthZEDMat, sl::MEASURE_DEPTH, sl::MEM_CPU, mMatWidth, mMatHeight);
            publishCamInfo(mDepthCamInfoMsg, mPubDepthCamInfo, timeStamp);
            publishDepth(depthZEDMat, timeStamp);
        }
        // <<<<<  Publish the depth image if someone has subscribed to

        // >>>>>  Publish the confidence image and map if someone has subscribed to
        if (confImgSub > 0 || confMapSub > 0) {
            publishCamInfo(mConfidenceCamInfoMsg, mPubConfidenceCamInfo, timeStamp);

            if (confImgSub > 0) {
                mZed.retrieveImage(confImgZedMat, sl::VIEW_CONFIDENCE, sl::MEM_CPU, mMatWidth, mMatHeight);

                publishImage(confImgZedMat, mPubConfImg, mDepthOptFrameId, timeStamp);
            }

            if (confMapSub > 0) {
                mZed.retrieveMeasure(confMapZedMat, sl::MEASURE_CONFIDENCE, sl::MEM_CPU, mMatWidth, mMatHeight);

                mPubConfMap->publish(sl_tools::imageToROSmsg(confMapZedMat, mDepthOptFrameId, timeStamp));
            }
        }
        // <<<<<  Publish the confidence image and map if someone has subscribed to

        // >>>>> Publish the disparity image if someone has subscribed to
        if (dispSub > 0) {
            mZed.retrieveMeasure(disparityZEDMat, sl::MEASURE_DISPARITY, sl::MEM_CPU, mMatWidth, mMatHeight);

            publishDisparity(disparityZEDMat, timeStamp);
        }
        // <<<<< Publish the disparity image if someone has subscribed to

        // >>>>> Publish the point cloud if someone has subscribed to
        if (cloudSub > 0) {
            // Run the point cloud conversion asynchronously to avoid slowing down
            // all the program
            // Retrieve raw pointCloud data if latest Pointcloud is ready
            std::unique_lock<std::mutex> lock(mPcMutex, std::defer_lock);
            if (lock.try_lock()) {
                mZed.retrieveMeasure(mCloud, sl::MEASURE_XYZBGRA, sl::MEM_CPU, mMatWidth, mMatHeight);

                mPointCloudTime = timeStamp;

                // Signal Pointcloud thread that a new pointcloud is ready
                mPcDataReady = true;
                RCLCPP_DEBUG(get_logger(), "publishDepthData -> mPcDataReady TRUE")

                mPcDataReadyCondVar.notify_one();
            }
        }
        // <<<<< Publish the point cloud if someone has subscribed to
    }

    void ZedCameraComponent::fillCamInfo(sl::Camera& zed, std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
                                         std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg,
                                         std::string leftFrameId, std::string rightFrameId,
                                         bool rawParam /*= false*/) {
        sl::CalibrationParameters zedParam;

        if (rawParam) {
            zedParam = zed.getCameraInformation(sl::Resolution(mMatWidth, mMatHeight))
                       .calibration_parameters_raw;
        } else {
            zedParam = zed.getCameraInformation(sl::Resolution(mMatWidth, mMatHeight))
                       .calibration_parameters;
        }

        float baseline = zedParam.T[0];
        leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        leftCamInfoMsg->d.resize(5);
        rightCamInfoMsg->d.resize(5);
        leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];   // k1
        leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];   // k2
        leftCamInfoMsg->d[2] = zedParam.left_cam.disto[4];   // k3
        leftCamInfoMsg->d[3] = zedParam.left_cam.disto[2];   // p1
        leftCamInfoMsg->d[4] = zedParam.left_cam.disto[3];   // p2
        rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0]; // k1
        rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1]; // k2
        rightCamInfoMsg->d[2] = zedParam.right_cam.disto[4]; // k3
        rightCamInfoMsg->d[3] = zedParam.right_cam.disto[2]; // p1
        rightCamInfoMsg->d[4] = zedParam.right_cam.disto[3]; // p2
        leftCamInfoMsg->k.fill(0.0);
        rightCamInfoMsg->k.fill(0.0);
        leftCamInfoMsg->k[0] = static_cast<double>(zedParam.left_cam.fx);
        leftCamInfoMsg->k[2] = static_cast<double>(zedParam.left_cam.cx);
        leftCamInfoMsg->k[4] = static_cast<double>(zedParam.left_cam.fy);
        leftCamInfoMsg->k[5] = static_cast<double>(zedParam.left_cam.cy);
        leftCamInfoMsg->k[8] = 1.0;
        rightCamInfoMsg->k[0] = static_cast<double>(zedParam.right_cam.fx);
        rightCamInfoMsg->k[2] = static_cast<double>(zedParam.right_cam.cx);
        rightCamInfoMsg->k[4] = static_cast<double>(zedParam.right_cam.fy);
        rightCamInfoMsg->k[5] = static_cast<double>(zedParam.right_cam.cy);
        rightCamInfoMsg->k[8] = 1.0;
        leftCamInfoMsg->r.fill(0.0);
        rightCamInfoMsg->r.fill(0.0);

        for (size_t i = 0; i < 3; i++) {
            // identity
            rightCamInfoMsg->r[i + i * 3] = 1;
            leftCamInfoMsg->r[i + i * 3] = 1;
        }

        if (rawParam) {
            std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
            float* p = R_.data();
            for (int i = 0; i < 9; i++) {
                rightCamInfoMsg->r[i] = p[i];
            }
        }

        leftCamInfoMsg->p.fill(0.0);
        rightCamInfoMsg->p.fill(0.0);
        leftCamInfoMsg->p[0] = static_cast<double>(zedParam.left_cam.fx);
        leftCamInfoMsg->p[2] = static_cast<double>(zedParam.left_cam.cx);
        leftCamInfoMsg->p[5] = static_cast<double>(zedParam.left_cam.fy);
        leftCamInfoMsg->p[6] = static_cast<double>(zedParam.left_cam.cy);
        leftCamInfoMsg->p[10] = 1.0;
        // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
        rightCamInfoMsg->p[3] = static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
        rightCamInfoMsg->p[0] = static_cast<double>(zedParam.right_cam.fx);
        rightCamInfoMsg->p[2] = static_cast<double>(zedParam.right_cam.cx);
        rightCamInfoMsg->p[5] = static_cast<double>(zedParam.right_cam.fy);
        rightCamInfoMsg->p[6] = static_cast<double>(zedParam.right_cam.cy);
        rightCamInfoMsg->p[10] = 1.0;
        leftCamInfoMsg->width = rightCamInfoMsg->width = static_cast<uint32_t>(mMatWidth);
        leftCamInfoMsg->height = rightCamInfoMsg->height = static_cast<uint32_t>(mMatHeight);
        leftCamInfoMsg->header.frame_id = leftFrameId;
        rightCamInfoMsg->header.frame_id = rightFrameId;
    }

    void ZedCameraComponent::publishCamInfo(camInfoMsgPtr camInfoMsg, camInfoPub pubCamInfo, rclcpp::Time timeStamp) {
        camInfoMsg->header.stamp = timeStamp;
        pubCamInfo->publish(camInfoMsg);
    }

    void ZedCameraComponent::publishImage(sl::Mat img,
                                          imagePub pubImg,
                                          std::string imgFrameId, rclcpp::Time timeStamp) {
        pubImg->publish(sl_tools::imageToROSmsg(img, imgFrameId, timeStamp)) ;
    }

    void ZedCameraComponent::publishDepth(sl::Mat depth, rclcpp::Time t) {

        if (!mOpenniDepthMode) {
            mPubDepth->publish(sl_tools::imageToROSmsg(depth, mDepthOptFrameId, t));
            return;
        }

        // OPENNI CONVERSION (meter -> millimeters - float32 -> uint16)
        std::shared_ptr<sensor_msgs::msg::Image> depthMessage = std::make_shared<sensor_msgs::msg::Image>();

        depthMessage->header.stamp = t;
        depthMessage->header.frame_id = mDepthOptFrameId;
        depthMessage->height = depth.getHeight();
        depthMessage->width = depth.getWidth();

        int num = 1; // for endianness detection
        depthMessage->is_bigendian = !(*(char*)&num == 1);

        depthMessage->step = depthMessage->width * sizeof(uint16_t);
        depthMessage->encoding = sensor_msgs::image_encodings::MONO16;

        size_t size = depthMessage->step * depthMessage->height;
        depthMessage->data.resize(size);

        uint16_t* data = (uint16_t*)(&depthMessage->data[0]);

        int dataSize = depthMessage->width * depthMessage->height;
        sl::float1* depthDataPtr = depth.getPtr<sl::float1>();

        for (int i = 0; i < dataSize; i++) {
            *(data++) = static_cast<uint16_t>(std::round(*(depthDataPtr++) * 1000));    // in mm, rounded
        }

        mPubDepth->publish(depthMessage);
    }


    void ZedCameraComponent::publishDisparity(sl::Mat disparity, rclcpp::Time timestamp) {
        sl::CameraInformation zedParam = mZed.getCameraInformation(sl::Resolution(mMatWidth, mMatHeight));

        std::shared_ptr<sensor_msgs::msg::Image> disparity_image =
            sl_tools::imageToROSmsg(disparity, mDepthOptFrameId, timestamp);

        stereo_msgs::msg::DisparityImage msg;
        msg.image = *disparity_image;
        msg.header = msg.image.header;
        msg.f = zedParam.calibration_parameters.left_cam.fx;
        msg.t = zedParam.calibration_parameters.T.x;
        msg.min_disparity = msg.f * msg.t / mZed.getDepthMaxRangeValue();
        msg.max_disparity = msg.f * msg.t / mZed.getDepthMinRangeValue();

        mPubDisparity->publish(msg);
    }

    void ZedCameraComponent::publishPointCloud() {
        // Initialize Point Cloud message
        // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h

        int ptsCount = mMatWidth * mMatHeight;
        mPointcloudMsg->header.stamp = mPointCloudTime;
        if (mPointcloudMsg->width != mMatWidth || mPointcloudMsg->height != mMatHeight) {
            mPointcloudMsg->header.frame_id = mDepthFrameId; // Set the header values of the ROS message

            mPointcloudMsg->is_bigendian = false;
            mPointcloudMsg->is_dense = false;

            mPointcloudMsg->width = mMatWidth;
            mPointcloudMsg->height = mMatHeight;

            sensor_msgs::PointCloud2Modifier modifier(*mPointcloudMsg);
            modifier.setPointCloud2Fields(4,
                                          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        }

        sl::Vector4<float>* cpu_cloud = mCloud.getPtr<sl::float4>();

        // Data copy
        float* ptCloudPtr = (float*)(&mPointcloudMsg->data[0]);

        #pragma omp parallel for
        for (size_t i = 0; i < ptsCount; ++i) {
            ptCloudPtr[i * 4 + 0] = cpu_cloud[i][0];
            ptCloudPtr[i * 4 + 1] = cpu_cloud[i][1];
            ptCloudPtr[i * 4 + 2] = cpu_cloud[i][2];
            ptCloudPtr[i * 4 + 3] = cpu_cloud[i][3];
        }

        // Pointcloud publishing
        mPubPointcloud->publish(mPointcloudMsg);
    }

    void ZedCameraComponent::pointcloudThreadFunc() {
        std::unique_lock<std::mutex> lock(mPcMutex);
        while (!mThreadStop) {

            RCLCPP_DEBUG(get_logger(), "pointcloudThreadFunc -> mPcDataReady value: %s", mPcDataReady ? "TRUE" : "FALSE");

            while (!mPcDataReady) { // loop to avoid spurious wakeups
                if (mPcDataReadyCondVar.wait_for(lock, std::chrono::milliseconds(500)) == std::cv_status::timeout) {
                    // Check thread stopping
                    if (mThreadStop) {
                        break;
                    } else {
                        RCLCPP_DEBUG(get_logger(), "pointcloudThreadFunc -> WAIT FOR CLOUD DATA");
                        continue;
                    }
                }
            }

            publishPointCloud();
            mPcDataReady = false;
            RCLCPP_DEBUG(get_logger(), "pointcloudThreadFunc -> mPcDataReady FALSE")

        }

        RCLCPP_DEBUG(get_logger(), "Pointcloud thread finished");
    }
}

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(stereolabs::ZedCameraComponent, rclcpp_lifecycle::LifecycleNode)
