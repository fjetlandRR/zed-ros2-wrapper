
#include "zed_component.hpp"
#include "sl_tools.h"
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

#ifndef TIMER_ELAPSED
#define TIMER_ELAPSED double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
#endif

namespace stereolabs {

    ZedCameraComponent::ZedCameraComponent(const std::string& node_name, const std::string& ros_namespace, bool intra_process_comms)
        : rclcpp_lifecycle::LifecycleNode(node_name, ros_namespace, intra_process_comms) {

#ifndef NDEBUG
        rcutils_ret_t res = rcutils_logging_set_logger_level(get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting DEBUG logger");
        }
#endif

        RCLCPP_INFO(get_logger(), "ZED Camera Component created");
        RCLCPP_INFO(get_logger(), "Waiting for `CONFIGURE` request...");

        RCLCPP_DEBUG(get_logger(), "ZED node: %s", get_name());
        RCLCPP_DEBUG(get_logger(), "ZED namespace: %s", get_namespace());
        RCLCPP_DEBUG(get_logger(), "LifecycleNode node: %s", rclcpp_lifecycle::LifecycleNode::get_name());
        RCLCPP_DEBUG(get_logger(), "LifecycleNode namespace: %s", rclcpp_lifecycle::LifecycleNode::get_namespace());
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

    void ZedCameraComponent::initPublishers() {

        std::string topicPrefix = "~/";

        // >>>>> Image publishers QOS
        // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
        rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_default; // Default QOS profile

        custom_camera_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE; // to be sure to publish images
        custom_camera_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;// KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth" parameter
        custom_camera_qos_profile.depth = 1; // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
        custom_camera_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;

        // >>>>> Image topics
        std::string img_topic = "image_rect_color";
        std::string img_raw_topic = "image_raw_color";
        std::string cam_info_topic = "camera_info";
        std::string cam_info_raw_topic = "camera_info_raw";
        // Set the default topic names
        mLeftTopic = topicPrefix + "left/" + img_topic;
        mLeftRawTopic = topicPrefix + "left/" + img_raw_topic;
        mLeftCamInfoTopic = topicPrefix + "left/" + cam_info_topic;
        mLeftCamInfoRawTopic = topicPrefix + "left/" + cam_info_raw_topic;
        mRightTopic = topicPrefix + "right/" + img_topic;
        mRightRawTopic = topicPrefix + "right/" + img_raw_topic;
        mRightCamInfoTopic = topicPrefix + "right/" + cam_info_topic;;
        mRightCamInfoRawTopic = topicPrefix + "right/" + cam_info_raw_topic;
        mRgbTopic = topicPrefix + "rgb/" + img_topic;
        mRgbRawTopic = topicPrefix + "rgb/" + img_raw_topic;
        mRgbCamInfoTopic = topicPrefix + "rgb/" + cam_info_topic;
        mRgbCamInfoRawTopic = topicPrefix + "rgb/" + cam_info_raw_topic;

        mDepthTopic = topicPrefix + "depth/";
        if (mOpenniDepthMode) {
            RCLCPP_INFO(get_logger(), "Openni depth mode activated");
            mDepthTopic += "depth_raw_registered";
        } else {
            mDepthTopic += "depth_registered";
        }
        std::string mDepthCamInfoTopic = topicPrefix + "depth/camera_info";
        // <<<<< Image topics

        // >>>>> Create Image publishers
        mPubRgb = create_publisher<sensor_msgs::msg::Image>(mRgbTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mRgbTopic.c_str());
        mPubRawRgb = create_publisher<sensor_msgs::msg::Image>(mRgbRawTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mRgbRawTopic.c_str());
        mPubLeft = create_publisher<sensor_msgs::msg::Image>(mLeftTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mLeftTopic.c_str());
        mPubRawLeft = create_publisher<sensor_msgs::msg::Image>(mLeftRawTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mLeftRawTopic.c_str());
        mPubRight = create_publisher<sensor_msgs::msg::Image>(mRightTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mRightTopic.c_str());
        mPubRawRight = create_publisher<sensor_msgs::msg::Image>(mRightRawTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mRightRawTopic.c_str());
        mPubDepth = create_publisher<sensor_msgs::msg::Image>(mDepthTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mDepthTopic.c_str());
        // <<<<< Create Image publishers

        // >>>>> Create Camera Info publishers
        mPubRgbCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(mRgbCamInfoTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mRgbCamInfoTopic.c_str());
        mPubRgbCamInfoRaw = create_publisher<sensor_msgs::msg::CameraInfo>(mRgbCamInfoRawTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mRgbCamInfoRawTopic.c_str());
        mPubLeftCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(mLeftCamInfoTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mLeftCamInfoTopic.c_str());
        mPubLeftCamInfoRaw = create_publisher<sensor_msgs::msg::CameraInfo>(mLeftCamInfoRawTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mLeftCamInfoRawTopic.c_str());
        mPubRightCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(mRightCamInfoTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mRightCamInfoTopic.c_str());
        mPubRightCamInfoRaw = create_publisher<sensor_msgs::msg::CameraInfo>(mRightCamInfoRawTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mRightCamInfoRawTopic.c_str());
        mPubDepthCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(mDepthCamInfoTopic, custom_camera_qos_profile);
        RCLCPP_INFO(get_logger(), "Publishing data on topic '%s'", mDepthCamInfoTopic.c_str());
        // <<<<< Create Camera Info publishers
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
        // TODO load params from param server
        // <<<<< Load params from param server

        // >>>>> Frame IDs
        mRightCamFrameId = "right_camera_frame";
        mRightCamOptFrameId = "right_camera_optical_frame";
        mLeftCamFrameId = "left_camera_frame";
        mLeftCamOptFrameId = "left_camera_optical_frame";

        mDepthFrameId = mLeftCamFrameId;
        mDepthOptFrameId = mLeftCamOptFrameId;
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

        // Initialize Message Publishers
        initPublishers();

        // >>>>> ZED configuration
        if (!mSvoFilepath.empty()) {
            mZedParams.svo_input_filename = mSvoFilepath.c_str();
            mZedParams.svo_real_time_mode = true;
            mSvoMode = true;
        } else {
            mZedParams.camera_fps = mCamFrameRate;
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

                if (elapsed > mCamTimeoutMsec) {
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

            if (elapsed > mCamTimeoutMsec) {
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

        cv::Size cvSize(mMatWidth, mMatWidth);
        mCvLeftImRGB = cv::Mat(cvSize, CV_8UC3);
        mCvRightImRGB = cv::Mat(cvSize, CV_8UC3);
        // Create and fill the camera information messages
        fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId, mRightCamOptFrameId);
        fillCamInfo(mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId, mRightCamOptFrameId, true);
        mRgbCamInfoMsg = mLeftCamInfoMsg;
        mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
        mDepthCamInfoMsg = mLeftCamInfoMsg;
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

        mPubRgbCamInfo->on_activate();
        mPubRgbCamInfoRaw->on_activate();
        mPubLeftCamInfo->on_activate();
        mPubLeftCamInfoRaw->on_activate();
        mPubRightCamInfo->on_activate();
        mPubRightCamInfoRaw->on_activate();
        mPubDepthCamInfo->on_activate();

        //mPubConfImg->on_activate();
        //mPubConfImgCamInfo->on_activate();
        // <<<<< Publishers activation

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
        mPubDepth->on_activate();

        mPubRgbCamInfo->on_deactivate();
        mPubRgbCamInfoRaw->on_deactivate();
        mPubLeftCamInfo->on_deactivate();
        mPubLeftCamInfoRaw->on_deactivate();
        mPubRightCamInfo->on_deactivate();
        mPubRightCamInfoRaw->on_deactivate();
        mPubDepthCamInfo->on_deactivate();
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
        mLastFrameTime = startTime;
        // <<<<< Last frame time initialization

        // >>>>> Grab parameters
        sl::RuntimeParameters runParams;
        runParams.sensing_mode = static_cast<sl::SENSING_MODE>(mCamSensingMode);
        runParams.enable_depth = false;
        runParams.enable_point_cloud = false;
        // <<<<< Grab parameters

        rclcpp::Rate loop_rate(mCamFrameRate);

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
            size_t rightRawSub = count_subscribers(mRightRawTopic); //mPubRawRight subscribers
            size_t depthSub = count_subscribers(mDepthTopic);       // mPubDepth subscribers

            bool pubImages = ((rgbSub + rgbRawSub + leftSub + leftRawSub + rightSub + rightRawSub) > 0);
            bool pubDepthData = (depthSub > 0);

            bool runLoop = pubImages | pubDepthData;
            //<<<<< Subscribers check

            if (runLoop) {
                // Timestamp
                rclcpp::Time frameTime;
                if (mSvoMode) {
                    frameTime = now();
                } else {
                    frameTime = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_IMAGE));
                }

                if (pubDepthData) {
                    int actual_confidence = mZed.getConfidenceThreshold();

                    if (actual_confidence != mCamConfidence) {
                        mZed.setConfidenceThreshold(mCamConfidence);
                    }

                    double actual_max_depth = static_cast<double>(mZed.getDepthMaxRangeValue());

                    if (actual_max_depth != mCamMaxDepth) {
                        mZed.setDepthMaxRangeValue(static_cast<double>(mCamMaxDepth));
                    }

                    runParams.enable_depth = true; // Ask to compute the depth
                } else {
                    runParams.enable_depth = false;
                }

                // ZED grab
                grab_status = mZed.grab(runParams);

                if (grab_status != sl::ERROR_CODE::SUCCESS) {
                    // Detect if a error occurred (for example:
                    // the zed have been disconnected) and
                    // re-initialize the ZED
                    if (grab_status != sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                        RCLCPP_WARN(get_logger(), sl::toString(grab_status));
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(500));

                    rclcpp::Time now = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_CURRENT));

                    rcl_time_point_value_t elapsed = (now - mLastFrameTime).nanoseconds();
                    rcl_time_point_value_t timeout = rclcpp::Duration(5, 0).nanoseconds();

                    if (elapsed > timeout && !mSvoMode) {
                        // TODO Better handle the error: throw exception?
                        throw std::runtime_error("ZED Camera timeout");
                    }

                    continue;
                }

                mLastFrameTime = frameTime;

                mCamDataMutex.lock();

                if (pubImages) {
                    publishImages(frameTime);
                }

                if (pubDepthData) {
                    publishDepthData(frameTime);
                }

                mCamDataMutex.unlock();

                // >>>>> Thread sleep
                TIMER_ELAPSED;
                START_TIMER;

                static int rateWarnCount = 0;
                if (!loop_rate.sleep()) {
                    rateWarnCount++;

                    if (rateWarnCount == mCamFrameRate) {
                        RCLCPP_WARN(get_logger(),  "Expected cycle time: %g sec  - Real cycle time: %g sec ",
                                    1.0 / mCamFrameRate, elapsed / 1000.0);
                        RCLCPP_INFO(get_logger(),  "Elaboration takes longer than requested "
                                    "by the FPS rate. Please consider to "
                                    "lower the 'frame_rate' setting.");

                        rateWarnCount = 0;
                        loop_rate.reset();
                    }
                } else {
                    rateWarnCount = 0;
                }

                RCLCPP_DEBUG(get_logger(), "Thread freq: %g Hz", 1000.0 / elapsed);
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
            cv::cvtColor(sl_tools::toCVMat(leftZEDMat), mCvLeftImRGB, CV_RGBA2RGB);

            if (leftSubnumber > 0) {
                publishCamInfo(mLeftCamInfoMsg, mPubLeftCamInfo, timeStamp);
                publishImage(mCvLeftImRGB, mPubLeft, mLeftCamOptFrameId, timeStamp);
            }

            if (rgbSubnumber > 0) {
                publishCamInfo(mRgbCamInfoMsg, mPubRgbCamInfo, timeStamp);
                publishImage(mCvLeftImRGB, mPubRgb, mDepthOptFrameId, timeStamp); // rgb is the left image
            }
        }
        // <<<<< Publish the left == rgb image if someone has subscribed to

        // >>>>> Publish the left_raw == rgb_raw image if someone has subscribed to
        if (leftRawSubnumber > 0 || rgbRawSubnumber > 0) {
            // Retrieve RGBA Left image
            mZed.retrieveImage(leftZEDMat, sl::VIEW_LEFT_UNRECTIFIED, sl::MEM_CPU, mMatWidth, mMatHeight);
            cv::cvtColor(sl_tools::toCVMat(leftZEDMat), mCvLeftImRGB, CV_RGBA2RGB);

            if (leftRawSubnumber > 0) {
                publishCamInfo(mLeftCamInfoRawMsg, mPubLeftCamInfoRaw, timeStamp);
                publishImage(mCvLeftImRGB, mPubRawLeft, mLeftCamOptFrameId, timeStamp);
            }

            if (rgbRawSubnumber > 0) {
                publishCamInfo(mRgbCamInfoRawMsg, mPubRgbCamInfoRaw, timeStamp);
                publishImage(mCvLeftImRGB, mPubRawRgb, mDepthOptFrameId, timeStamp);
            }
        }
        // <<<<< Publish the left_raw == rgb_raw image if someone has subscribed to

        // >>>>> Publish the right image if someone has subscribed to
        if (rightSubnumber > 0) {
            // Retrieve RGBA Right image
            mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT, sl::MEM_CPU, mMatWidth, mMatHeight);
            cv::cvtColor(sl_tools::toCVMat(rightZEDMat), mCvRightImRGB, CV_RGBA2RGB);
            publishCamInfo(mRightCamInfoMsg, mPubRightCamInfo, timeStamp);
            publishImage(mCvRightImRGB, mPubRight, mRightCamOptFrameId, timeStamp);
        }
        // <<<<< Publish the right image if someone has subscribed to

        // >>>>> Publish the right image if someone has subscribed to
        if (rightRawSubnumber > 0) {
            // Retrieve RGBA Right image
            mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT_UNRECTIFIED, sl::MEM_CPU, mMatWidth, mMatHeight);
            cv::cvtColor(sl_tools::toCVMat(rightZEDMat), mCvRightImRGB, CV_RGBA2RGB);
            publishCamInfo(mRightCamInfoRawMsg, mPubRightCamInfoRaw, timeStamp);
            publishImage(mCvRightImRGB, mPubRawRight, mRightCamOptFrameId, timeStamp);
        }
        // <<<<< Publish the right image if someone has subscribed to
    }

    void ZedCameraComponent::publishDepthData(rclcpp::Time timeStamp) {
        size_t depthSub = count_subscribers(mDepthTopic);       // mPubDepth subscribers

        sl::Mat depthZEDMat;

        // >>>>>  Publish the depth image if someone has subscribed to
        if (depthSub > 0 /*|| disparitySubnumber*/ > 0) {
            mZed.retrieveMeasure(depthZEDMat, sl::MEASURE_DEPTH, sl::MEM_CPU, mMatWidth, mMatHeight);
            publishCamInfo(mDepthCamInfoMsg, mPubDepthCamInfo, timeStamp);
            publishDepth(sl_tools::toCVMat(depthZEDMat), timeStamp); // in meters
        }
        // <<<<<  Publish the depth image if someone has subscribed to
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
            cv::Mat R_ = sl_tools::convertRodrigues(zedParam.R);
            float* p = (float*)(R_.data);

            for (size_t i = 0; i < 9; i++) {
                rightCamInfoMsg->r[i] = static_cast<double>(p[i]);
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
        static int seq = 0;
        camInfoMsg->header.stamp = timeStamp;
        pubCamInfo->publish(camInfoMsg);
        seq++;
    }

    void ZedCameraComponent::publishImage(cv::Mat img,
                                          imagePub pubImg,
                                          std::string imgFrameId, rclcpp::Time timeStamp) {
        pubImg->publish(sl_tools::imageToROSmsg(img, sensor_msgs::image_encodings::BGR8, imgFrameId, timeStamp)) ;
    }

    void ZedCameraComponent::publishDepth(cv::Mat depth, rclcpp::Time timeStamp) {
        std::string encoding;

        if (mOpenniDepthMode) {
            depth *= 1000.0f;
            depth.convertTo(depth, CV_16UC1); // in mm, rounded
            encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        } else {
            encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        }

        mPubDepth->publish(sl_tools::imageToROSmsg(depth, encoding, mDepthOptFrameId, timeStamp));

        //        cv::imshow("Depth Image", depth);
        //        cv::waitKey(1);
    }
}

#include "class_loader/class_loader_register_macro.h"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(stereolabs::ZedCameraComponent, rclcpp_lifecycle::LifecycleNode)
