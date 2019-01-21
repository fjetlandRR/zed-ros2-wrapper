// /////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
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

#include "zed_component.hpp"
#include "sl_tools.h"
#include <string>

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>

using namespace std::chrono_literals;

#ifndef TIMER_ELAPSED
#define TIMER_ELAPSED double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
#endif

namespace stereolabs {

#ifndef DEG2RAD
#define DEG2RAD 0.017453293
#endif

    ZedCameraComponent::ZedCameraComponent(const std::string& node_name, const std::string& ros_namespace,
                                           bool intra_process_comms)
        : rclcpp_lifecycle::LifecycleNode(node_name, ros_namespace, intra_process_comms) {

#ifndef NDEBUG
        std::string logger = ros_namespace.empty() ? "" : ros_namespace + ".";
        logger += node_name;
        rcutils_ret_t res = rcutils_logging_set_logger_level(logger.c_str(), RCUTILS_LOG_SEVERITY_DEBUG);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting DEBUG logger");
        } else {
            RCLCPP_INFO(get_logger(), "Debug Mode enabled");
        }

#endif

        RCLCPP_INFO(get_logger(), "ZED Camera Component created");

        RCLCPP_INFO(get_logger(), "ZED namespace: '%s'", get_namespace());
        RCLCPP_INFO(get_logger(), "ZED node: '%s'", get_name());

        RCLCPP_DEBUG(get_logger(), "[ROS2] Using RMW_IMPLEMENTATION = %s", rmw_get_implementation_identifier());

        RCLCPP_INFO(get_logger(), "Waiting for `CONFIGURE` request...");
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ZedCameraComponent::on_shutdown(
        const rclcpp_lifecycle::State& previous_state) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        RCLCPP_DEBUG(get_logger(), "on_shutdown() is called.");
        RCLCPP_DEBUG(get_logger(), "Current state: %s", this->get_current_state().label().c_str());
        RCLCPP_DEBUG(get_logger(), "Previous state: %s (%d)", previous_state.label().c_str(), previous_state.id());

        // >>>>> Verify that all the threads are not active
        if (!mThreadStop) {
            mThreadStop = true;

            try {
                if (mGrabThread.joinable()) {
                    mGrabThread.join();
                }
            } catch (std::system_error& e) {
                RCLCPP_WARN(get_logger(), "Grab thread joining exception: %s", e.what());
            }

            try {
                if (mPcThread.joinable()) {
                    mPcThread.join();
                }
            } catch (std::system_error& e) {
                RCLCPP_WARN(get_logger(), "Pointcloud thread joining exception: %s", e.what());
            }
        }

        // <<<<< Verify that the grab thread is not active

        // >>>>> Verify that ZED is not opened
        if (mZed.isOpened()) {
            std::lock_guard<std::mutex> lock(mImuMutex);
            mZed.close();
            RCLCPP_INFO(get_logger(), "ZED Closed");
        }

        // <<<<< Verify that ZED is not opened

        RCLCPP_INFO(get_logger(), "Shutdown complete");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    ZedCameraComponent::~ZedCameraComponent() {
        // >>>>> Verify that all the threads are not active
        if (!mThreadStop) {
            mThreadStop = true;

            try {
                if (mGrabThread.joinable()) {
                    mGrabThread.join();
                }
            } catch (std::system_error& e) {
                RCLCPP_WARN(get_logger(), "Grab thread joining exception: %s", e.what());
            }

            try {
                if (mPcThread.joinable()) {
                    mPcThread.join();
                }
            } catch (std::system_error& e) {
                RCLCPP_WARN(get_logger(), "Pointcloud thread joining exception: %s", e.what());
            }
        }

        // <<<<< Verify that the grab thread is not active
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ZedCameraComponent::on_error(
        const rclcpp_lifecycle::State& previous_state) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        RCLCPP_DEBUG(get_logger(), "on_error() is called.");
        RCLCPP_DEBUG(get_logger(), "Current state: %s", this->get_current_state().label().c_str());
        RCLCPP_DEBUG(get_logger(), "Previous state: %s (%d)", previous_state.label().c_str(), previous_state.id());

        if (mPrevTransition != 255) {
            switch (mPrevTransition) {
            case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE: { // Error during configuration
                RCLCPP_INFO(get_logger(), "Node entering 'FINALIZED' state. Kill and restart the node.");

                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            break;

            case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE: { // Error during activation
                if (mSvoMode) {
                    RCLCPP_INFO(get_logger(), "Please verify the SVO path and reboot the node: %s", mSvoFilepath.c_str());

                    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
                } else {

                    if (mZedUserCamModel == 0) {
                        RCLCPP_INFO(get_logger(), "Please verify the USB connection");
                    } else {
                        RCLCPP_INFO(get_logger(), "Please verify the USB connection. Try to flip the USB TypeC cable on ZED mini");
                    }

                    RCLCPP_INFO(get_logger(), "Node entering 'UNCONFIGURED' state");

                    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    void ZedCameraComponent::initParameters() {
        rclcpp::Parameter paramVal;
        std::string paramName;

        // >>>>> GENERAL parameters
        RCLCPP_INFO(get_logger(), "*** GENERAL parameters ***");

        paramName = "general.svo_file";

        if (get_parameter(paramName, paramVal)) {
            mSvoFilepath = paramVal.as_string();
            RCLCPP_INFO(get_logger(), " * SVO: `%s`", mSvoFilepath.c_str());
        }

        paramName = "general.camera_model";

        if (get_parameter(paramName, paramVal)) {
            mZedUserCamModel = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * Camera model: %d (%s)",
                    mZedUserCamModel, sl::toString(static_cast<sl::MODEL>(mZedUserCamModel)).c_str());

        paramName = "general.camera_timeout_sec";

        if (get_parameter(paramName, paramVal)) {
            mCamTimeoutSec = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * Camera timeout: %d sec", mCamTimeoutSec);

        paramName = "general.camera_reactivate";

        if (get_parameter(paramName, paramVal)) {
            mZedReactivate = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * Camera reconnection if disconnected: %s", mZedReactivate ? "ENABLED" : "DISABLED");

        paramName = "general.camera_max_reconnect";

        if (get_parameter(paramName, paramVal)) {
            mMaxReconnectTemp = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * Camera reconnection temptatives: %d", mMaxReconnectTemp);

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
            if (paramVal.get_type() == rclcpp::PARAMETER_DOUBLE) {
                mZedMatResizeFactor = paramVal.as_double();

                if (mZedMatResizeFactor < 0.1) {
                    mZedMatResizeFactor = 0.1;
                    RCLCPP_WARN(get_logger(), "The minimum value allowed for '%s' is 0.1", paramName.c_str());
                } else if (mZedMatResizeFactor > 1.0) {
                    mZedMatResizeFactor = 1.0;
                    RCLCPP_WARN(get_logger(), "The maximum value allowed for '%s' is 1.0", paramName.c_str());
                }
            } else {
                RCLCPP_WARN(get_logger(), "The parameter '%s' must be a DOUBLE, using the default value", paramName.c_str());
            }
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

        paramName = "general.base_frame";

        if (get_parameter(paramName, paramVal)) {
            mBaseFrameId = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * BASE frame: '%s'", mBaseFrameId.c_str());

        paramName = "general.camera_frame";

        if (get_parameter(paramName, paramVal)) {
            mCameraFrameId = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * CAMERA CENTER frame: '%s'", mCameraFrameId.c_str());

        paramName = "general.left_camera_frame";

        if (get_parameter(paramName, paramVal)) {
            mLeftCamFrameId = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * LEFT CAMERA frame: '%s'", mLeftCamFrameId.c_str());

        paramName = "general.left_camera_optical_frame";

        if (get_parameter(paramName, paramVal)) {
            mLeftCamOptFrameId = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * LEFT CAMERA OPTICAL frame: '%s'", mLeftCamOptFrameId.c_str());

        paramName = "general.right_camera_frame";

        if (get_parameter(paramName, paramVal)) {
            mRightCamFrameId = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * RIGHT CAMERA frame: '%s'", mRightCamFrameId.c_str());

        paramName = "general.right_camera_optical_frame";

        if (get_parameter(paramName, paramVal)) {
            mRightCamOptFrameId = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * RIGHT CAMERA OPTICAL frame: '%s'", mRightCamOptFrameId.c_str());
        // <<<<< GENERAL parameters

        // >>>>> VIDEO parameters
        RCLCPP_INFO(get_logger(), "*** VIDEO parameters ***");

        paramName = "video.auto_exposure";

        if (get_parameter(paramName, paramVal)) {
            mZedAutoExposure = paramVal.as_bool();

            if (mZedAutoExposure) {
                mTriggerAutoExposure = true;
            }
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

        paramName = "video.rgb_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mRgbTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * RGB topic root: '%s'", mRgbTopicRoot.c_str());


        paramName = "video.left_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mLeftTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * Left topic root: '%s'", mLeftTopicRoot.c_str());

        paramName = "video.right_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mRightTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * Right topic root: '%s'", mRightTopicRoot.c_str());
        // <<<<< VIDEO parameters

        // >>>>>> DEPTH parameters
        RCLCPP_INFO(get_logger(), "*** DEPTH parameters ***");

        paramName = "depth.min_depth";

        if (get_parameter(paramName, paramVal)) {
            if (paramVal.get_type() == rclcpp::PARAMETER_DOUBLE) {
                mZedMinDepth = paramVal.as_double();

                if (mZedMinDepth < 0.1) {
                    RCLCPP_WARN(get_logger(), "The parameter '%s' must be a greater or equal to 0.1", paramName.c_str());
                    mZedMinDepth = 0.1;
                }

                if (mZedMinDepth > 3.0) {
                    RCLCPP_WARN(get_logger(), "The parameter '%s' must be a smaller or equal to 3.0", paramName.c_str());
                    mZedMinDepth = 3.0;
                }
            } else {
                RCLCPP_WARN(get_logger(), "The parameter '%s' must be a DOUBLE, using the default value", paramName.c_str());
            }
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * Min depth: %g m", mZedMinDepth);

        paramName = "depth.max_depth";

        if (get_parameter(paramName, paramVal)) {
            if (paramVal.get_type() == rclcpp::PARAMETER_DOUBLE) {
                mZedMaxDepth = paramVal.as_double();
            } else {
                RCLCPP_WARN(get_logger(), "The parameter '%s' must be a DOUBLE, using the default value", paramName.c_str());
            }
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * Max depth: %g m [DYNAMIC]", mZedMaxDepth);

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

        paramName = "depth.depth_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mDepthTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        if (!mDepthTopicRoot.back() != '/') {
            mDepthTopicRoot.push_back('/');
        }

        RCLCPP_INFO(get_logger(), " * Depth topic root: '%s'", mDepthTopicRoot.c_str());

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

        paramName = "depth.confidence_root";

        if (get_parameter(paramName, paramVal)) {
            mConfTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        if (!mConfTopicRoot.back() != '/') {
            mConfTopicRoot.push_back('/');
        }

        RCLCPP_INFO(get_logger(), " * Confidence topics root: '%s'", mConfTopicRoot.c_str());

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

            paramName = "imu.imu_frame";

            if (get_parameter(paramName, paramVal)) {
                mImuFrameId = paramVal.as_string();
            } else {
                RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
            }

            RCLCPP_INFO(get_logger(), " * IMU frame: '%s'", mImuFrameId.c_str());

            paramName = "imu.imu_topic_root";

            if (get_parameter(paramName, paramVal)) {
                mImuTopicRoot = paramVal.as_string();
            } else {
                RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
            }

            if (!mImuTopicRoot.back() != '/') {
                mImuTopicRoot.push_back('/');
            }

            RCLCPP_INFO(get_logger(), " * IMU topic root: '%s'", mImuTopicRoot.c_str());

            paramName = "imu.imu_topic";

            if (get_parameter(paramName, paramVal)) {
                mImuTopic = paramVal.as_string();
            } else {
                RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
            }

            RCLCPP_INFO(get_logger(), " * IMU topic: '%s'", mImuTopic.c_str());

            paramName = "imu.imu_raw_topic";

            if (get_parameter(paramName, paramVal)) {
                mImuRawTopic = paramVal.as_string();
            } else {
                RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
            }

            RCLCPP_INFO(get_logger(), " * IMU RAW topic: '%s'", mImuRawTopic.c_str());

            paramName = "imu.imu_pub_rate";

            if (get_parameter(paramName, paramVal)) {
                if (paramVal.get_type() == rclcpp::PARAMETER_DOUBLE) {
                    mImuPubRate = paramVal.as_double();
                } else {
                    RCLCPP_WARN(get_logger(), "The parameter '%s' must be a DOUBLE, using the default value", paramName.c_str());
                }
            } else {
                RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
            }

            RCLCPP_INFO(get_logger(), " * IMU rate: %g Hz", mImuPubRate);

            paramName = "imu.imu_sync_frame";

            if (get_parameter(paramName, paramVal)) {
                mImuTimestampSync = paramVal.as_bool();
            } else {
                RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
            }

            RCLCPP_INFO(get_logger(), " * IMU timestamp sync with last frame: %s", mImuTimestampSync ? "ENABLED" : "DISABLED");
        }

        // <<<<<< IMU parameters

        using namespace std::placeholders;

        register_param_change_callback(std::bind(&ZedCameraComponent::paramChangeCallback, this, _1));
    }

    rcl_interfaces::msg::SetParametersResult ZedCameraComponent::paramChangeCallback(std::vector<rclcpp::Parameter>
            parameters) {

        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = false;

        for (size_t i = 0; i < parameters.size(); i++) {
            rclcpp::Parameter param = parameters[i];

            if (param.get_name() == "general.mat_resize_factor") {
                if (param.get_type() == rclcpp::PARAMETER_DOUBLE) {

                    double new_val = param.as_double();

                    if (new_val > 0.01 && new_val <= 1.0) {
                        mZedMatResizeFactor = new_val;
                        RCLCPP_INFO(get_logger(), "The param '%s' has changed to %g", param.get_name().c_str(), mZedMatResizeFactor);
                        result.successful = true;

                        // >>>>> Modify data sizes
                        mCamDataMutex.lock();
                        mMatWidth = static_cast<size_t>(mCamWidth * mZedMatResizeFactor);
                        mMatHeight = static_cast<size_t>(mCamHeight * mZedMatResizeFactor);
                        RCLCPP_INFO(get_logger(), "Data Mat size : %d x %d", mMatWidth, mMatHeight);

                        // Update Camera Info
                        fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId, mRightCamOptFrameId);
                        fillCamInfo(mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId, mRightCamOptFrameId, true);
                        mRgbCamInfoMsg = mDepthCamInfoMsg = mLeftCamInfoMsg;
                        mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
                        mCamDataMutex.unlock();
                        // <<<<< Modify data sizes
                    } else {
                        RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT value in the range ]0.0,1.0]",
                                    param.get_name().c_str());
                        result.successful = false;
                        return result;
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT positive value!", param.get_name().c_str());
                    result.successful = false;
                    return result;
                }
            } else if (param.get_name() == "video.auto_exposure") {
                if (param.get_type() == rclcpp::PARAMETER_BOOL) {

                    mZedAutoExposure = param.as_bool();

                    if (mZedAutoExposure) {
                        mTriggerAutoExposure = true;
                    }

                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %s", param.get_name().c_str(),
                                mZedAutoExposure ? "ENABLED" : "DISABLED");
                    result.successful = true;
                } else {
                    RCLCPP_WARN(get_logger(), "The param '%s' requires a BOOL value!", param.get_name().c_str());
                    result.successful = false;
                    return result;
                }
            } else if (param.get_name() == "video.exposure") {
                if (param.get_type() == rclcpp::PARAMETER_INTEGER) {

                    int new_val = param.as_int();

                    if (new_val > 0 && new_val <= 100) {
                        mZedExposure = new_val;
                        RCLCPP_INFO(get_logger(), "The param '%s' has changed to %d", param.get_name().c_str(), mZedExposure);
                        result.successful = true;
                    } else {
                        RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value in the range ]0,100]", param.get_name().c_str());
                        result.successful = false;
                        return result;
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value!", param.get_name().c_str());
                    result.successful = false;
                    return result;
                }
            } else if (param.get_name() == "video.gain") {
                if (param.get_type() == rclcpp::PARAMETER_INTEGER) {

                    int new_val = param.as_int();

                    if (new_val > 0 && new_val <= 100) {
                        mZedGain = new_val;
                        RCLCPP_INFO(get_logger(), "The param '%s' has changed to %d", param.get_name().c_str(), mZedGain);
                        result.successful = true;
                    } else {
                        RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value in the range ]0,100]", param.get_name().c_str());
                        result.successful = false;
                        return result;
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value!", param.get_name().c_str());
                    result.successful = false;
                    return result;
                }
            } else if (param.get_name() == "depth.confidence") {
                if (param.get_type() == rclcpp::PARAMETER_INTEGER) {
                    int new_val = param.as_int();

                    if (new_val > 0 && new_val <= 100) {
                        mZedConfidence = new_val;
                        RCLCPP_INFO(get_logger(), "The param '%s' has changed to %d", param.get_name().c_str(), mZedConfidence);
                        result.successful = true;
                    } else {
                        RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value in the range ]0,100]", param.get_name().c_str());
                        result.successful = false;
                        return result;
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value!", param.get_name().c_str());
                    result.successful = false;
                    return result;
                }
            } else if (param.get_name() == "depth.max_depth") {
                if (param.get_type() == rclcpp::PARAMETER_DOUBLE) {

                    double new_val = param.as_double();

                    if (new_val > 0) {
                        mZedMaxDepth = new_val;
                        RCLCPP_INFO(get_logger(), "The param '%s' has changed to %g", param.get_name().c_str(), mZedMaxDepth);
                        result.successful = true;
                    } else {
                        RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT positive value", param.get_name().c_str());
                        result.successful = false;
                        return result;
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT positive value!", param.get_name().c_str());
                    result.successful = false;
                    return result;
                }
            } else {
                RCLCPP_WARN(get_logger(), "The param '%s' cannot be dinamically changed!", param.get_name().c_str());
                result.successful = false;
                return result;
            }
        }

        return result;
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
        camera_qos_profile.depth = 2;

        //        camera_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE; // to be sure to publish images
        //        camera_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;// KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth" parameter
        //        camera_qos_profile.depth = 1; // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
        //        camera_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;

        // >>>>> Video topics
        std::string img_topic = "/image_rect_color";
        std::string img_raw_topic = "/image_raw_color";
        std::string cam_info_topic = "/camera_info";
        std::string raw_suffix = "_raw";

        // Set the default topic names
        mLeftTopic = topicPrefix + mLeftTopicRoot + img_topic;
        mLeftCamInfoTopic = mLeftTopic + cam_info_topic;
        mLeftRawTopic = topicPrefix + mLeftTopicRoot + raw_suffix + img_raw_topic;
        mLeftCamInfoRawTopic = mLeftRawTopic + cam_info_topic;

        mRightTopic = topicPrefix + mRightTopicRoot + img_topic;
        mRightCamInfoTopic = mRightTopic + cam_info_topic;
        mRightRawTopic = topicPrefix + mRightTopicRoot + raw_suffix + img_raw_topic;
        mRightCamInfoRawTopic = mRightRawTopic + cam_info_topic;

        mRgbTopic = topicPrefix + mRgbTopicRoot + img_topic;
        mRgbCamInfoTopic = mRgbTopic + cam_info_topic;
        mRgbRawTopic = topicPrefix + mRgbTopicRoot + raw_suffix + img_raw_topic;
        mRgbCamInfoRawTopic = mRgbRawTopic + cam_info_topic;
        // <<<<< Video topics

        // >>>>> Depth Topics
        mDepthTopic = topicPrefix + mDepthTopicRoot;

        if (mOpenniDepthMode) {
            RCLCPP_INFO(get_logger(), "Openni depth mode activated");
            mDepthTopic += "depth_raw_registered";
        } else {
            mDepthTopic += "depth_registered";
        }

        std::string mDepthCamInfoTopic = mDepthTopic + cam_info_topic;

        mConfImgTopic = topicPrefix + mConfTopicRoot + mConfImgTopic;
        mConfCamInfoTopic = mConfImgTopic + cam_info_topic;
        mConfMapTopic = topicPrefix + mConfTopicRoot + mConfMapTopic;

        mDispTopic = topicPrefix + mDispTopic;

        mPointcloudTopic = topicPrefix + mPointcloudTopic;
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
        mPubConfidenceCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(mConfCamInfoTopic, camera_qos_profile);
        RCLCPP_INFO(get_logger(), " * '%s'", mConfCamInfoTopic.c_str());
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

        // >>>>> Create IMU Publishers
        if (mImuPubRate > 0 && mZedUserCamModel == 1) {
            // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
            rmw_qos_profile_t imu_qos_profile = rmw_qos_profile_sensor_data; // Default QOS profile
            imu_qos_profile.depth = 2;

            mImuTopic = topicPrefix + mImuTopicRoot + mImuTopic;
            mImuRawTopic = topicPrefix + mImuTopicRoot + mImuRawTopic;

            mPubImu = create_publisher<sensor_msgs::msg::Imu>(mImuTopic, imu_qos_profile);
            RCLCPP_INFO(get_logger(), " * '%s'", mImuTopic.c_str());

            mPubImuRaw = create_publisher<sensor_msgs::msg::Imu>(mImuRawTopic, imu_qos_profile);
            RCLCPP_INFO(get_logger(), " * '%s'", mImuRawTopic.c_str());
        }

        // <<<<< Create IMU Publishers
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ZedCameraComponent::on_configure(
        const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

        RCLCPP_DEBUG(get_logger(), "on_configure() is called.");

        // >>>>> Check SDK version
        RCLCPP_INFO(get_logger(), "SDK Version: %d.%d.%d - Build %d", ZED_SDK_MAJOR_VERSION, ZED_SDK_MINOR_VERSION,
                    ZED_SDK_PATCH_VERSION, ZED_SDK_BUILD_ID);
#if (ZED_SDK_MAJOR_VERSION<2 || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION<6))
        RCLCPP_ERROR(get_logger(), "ROS2 ZED node requires at least ZED SDK v2.6.0");

        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR;
#endif
        // <<<<< Check SDK version

        // >>>>> Load params from param server
        initParameters();
        // <<<<< Load params from param server

        // >>>>> Frame IDs
        mDepthFrameId = mLeftCamFrameId;
        mDepthOptFrameId = mLeftCamOptFrameId;
        // <<<<< Frame IDs

        // >>>>> Create camera info
        mRgbCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mRgbCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mLeftCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mLeftCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mRightCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mRightCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        mDepthCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        // <<<<< Create camera info

        // Create pointcloud message
        mPointcloudMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // >>>>> Create IMU messages
        mImuMsg = std::make_shared<sensor_msgs::msg::Imu>();
        mImuMsgRaw = std::make_shared<sensor_msgs::msg::Imu>();
        // <<<<< Create IMU messages

        // Initialize Message Publishers
        initPublishers();

        // >>>>> ZED configuration
        if (!mSvoFilepath.empty()) {
            RCLCPP_INFO(get_logger(), "*** SVO OPENING ***");

            mZedParams.svo_input_filename = mSvoFilepath.c_str();
            //mZedParams.svo_real_time_mode = true;
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
        mZedParams.depth_minimum_distance = mZedMinDepth;
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
                    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
                }

                sl::DeviceProperties prop = sl_tools::getZEDFromSN(mZedSerialNumber);

                if (prop.id < -1 ||
                    prop.camera_state == sl::CAMERA_STATE::CAMERA_STATE_NOT_AVAILABLE) {
                    std::string msg = "Camera with SN " + std::to_string(mZedSerialNumber) +
                                      " not detected! Please verify the connection.";
                    RCLCPP_INFO(get_logger(), msg.c_str());
                } else {
                    waiting_for_camera = false;
                    mZedParams.camera_linux_id = prop.id;
                }

                TIMER_ELAPSED; // Initialize a variable named "elapsed" with the msec elapsed from the latest "START_TIMER" call

                if (elapsed >= mMaxReconnectTemp * mCamTimeoutSec * 1000) {
                    RCLCPP_ERROR(get_logger(), "Camera detection timeout");

                    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
                }

                std::this_thread::sleep_for(std::chrono::seconds(mCamTimeoutSec));
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

            if (mSvoMode) {
                RCLCPP_WARN(get_logger(), "Error opening SVO: %s", sl::toString(err).c_str());

                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }

            RCLCPP_WARN(get_logger(), "Error opening camera: %s", sl::toString(err).c_str());

            if (err == sl::ERROR_CODE_CAMERA_DETECTION_ISSUE && mZedUserCamModel == 1) {
                RCLCPP_INFO(get_logger(), "Try to flip the USB3 Type-C connector");
            } else {
                RCLCPP_INFO(get_logger(), "Please verify the USB3 connection");
            }

            if (!rclcpp::ok() || mThreadStop) {
                RCLCPP_INFO(get_logger(), "ZED activation interrupted");

                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }

            TIMER_ELAPSED

            if (elapsed > mMaxReconnectTemp * mCamTimeoutSec * 1000) {
                RCLCPP_ERROR(get_logger(), "Camera detection timeout");

                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }

            std::this_thread::sleep_for(std::chrono::seconds(mCamTimeoutSec));
        }

        // <<<<< Try to open ZED camera or to load SVO

        sl::CameraInformation camInfo = mZed.getCameraInformation();
        mZedRealCamModel = camInfo.camera_model;

        if (mZedRealCamModel == sl::MODEL_ZED) {

            if (mZedUserCamModel != 0) {
                RCLCPP_WARN(get_logger(), "Camera model does not match user parameter. Please modify "
                            "the value of the parameter 'camera_model' to '0'");
            }
        } else if (mZedRealCamModel == sl::MODEL_ZED_M) {

            if (mZedUserCamModel != 1) {
                RCLCPP_WARN(get_logger(), "Camera model does not match user parameter. Please modify "
                            "the value of the parameter 'camera_model' to '1'");
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

        // >>>>> IMU sensor
        sl::Transform imuTransf = camInfo.camera_imu_transform;

        sl::Translation imuPos = imuTransf.getTranslation();
        sl::Orientation imuOrient = imuTransf.getOrientation();

        RCLCPP_INFO(get_logger(), "IMU POSE RELATED TO LEFT CAMERA:");
        RCLCPP_INFO(get_logger(), " * POSITION [x,y,z]: %g,%g,%g", imuPos.x, imuPos.y, imuPos.z);
        RCLCPP_INFO(get_logger(), " * ORIENTATION [qx, qy, qz, qw]: %g,%g,%g,%g", imuOrient.ox, imuOrient.oy, imuOrient.oz,
                    imuOrient.ow);

        RCLCPP_INFO(get_logger(), "*** %s configured ***", sl::toString(mZedRealCamModel).c_str());
        RCLCPP_INFO(get_logger(), "Waiting for `ACTIVATE` request...");
        // >>>>> IMU sensor

        // We return a success and hence invoke the transition to the next
        // step: "inactive".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "unconfigured" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ZedCameraComponent::on_activate(
        const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

        RCLCPP_DEBUG(get_logger(), "on_activate() is called.");

        if (!mZed.isOpened()) {
            RCLCPP_WARN(get_logger(), "ZED Camera not opened");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
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

        if (mImuPubRate > 0 && mZedRealCamModel == sl::MODEL_ZED_M) {
            mPubImu->on_activate();
            mPubImuRaw->on_activate();
        }

        // <<<<< Publishers activation

        // >>>>> Start Pointcloud thread
        mPcDataReady = false;
        //RCLCPP_DEBUG(get_logger(), "on_activate -> mPcDataReady FALSE")
        mPcThread = std::thread(&ZedCameraComponent::pointcloudThreadFunc, this);
        // <<<<< Start Pointcloud thread

        // >>>>> Start ZED thread
        mThreadStop = false;
        mGrabThread = std::thread(&ZedCameraComponent::zedGrabThreadFunc, this);
        // <<<<< Start ZED thread

        // >>>>> Start IMU timer
        if (mImuPubRate > 0 && mZedRealCamModel == sl::MODEL_ZED_M) {
            std::chrono::milliseconds imuPeriodMsec(static_cast<int>(1000.0 / mImuPubRate));

            mImuTimer = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(imuPeriodMsec),
                                          std::bind(&ZedCameraComponent::imuPubCallback, this));
        }

        // <<<<< Start IMU timer

        // We return a success and hence invoke the transition to the next
        // step: "active".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "inactive" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ZedCameraComponent::on_deactivate(
        const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;

        RCLCPP_DEBUG(get_logger(), "on_deactivate() is called.");

        // >>>>> Stop IMU timer
        if (mImuTimer) {
            mImuTimer->cancel();
        }

        // <<<<< Stop IMU timer

        // >>>>> Verify that all the threads are not active
        if (!mThreadStop) {
            mThreadStop = true;

            try {
                if (mGrabThread.joinable()) {
                    mGrabThread.join();
                }
            } catch (std::system_error& e) {
                RCLCPP_WARN(get_logger(), "Grab thread joining exception: %s", e.what());
            }

            try {
                if (mPcThread.joinable()) {
                    mPcThread.join();
                }
            } catch (std::system_error& e) {
                RCLCPP_WARN(get_logger(), "Pointcloud thread joining exception: %s", e.what());
            }
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

        if (mImuPubRate > 0 && mZedRealCamModel == sl::MODEL_ZED_M) {
            mPubImu->on_deactivate();
            mPubImuRaw->on_deactivate();
        }

        // <<<<< Publishers deactivation

        // We return a success and hence invoke the transition to the next
        // step: "inactive".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "active" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ZedCameraComponent::on_cleanup(
        const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

        RCLCPP_DEBUG(get_logger(), "on_cleanup() is called.");

        // >>>>> Close ZED if opened
        if (mZed.isOpened()) {
            std::lock_guard<std::mutex> lock(mImuMutex);
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
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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
        // <<<<< Grab parameters

        rclcpp::WallRate loop_rate(mZedFrameRate);

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

            mRunGrabLoop = pubImages | pubDepthData;
            //<<<<< Subscribers check

            if (mRunGrabLoop) {
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
                    grabTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_CURRENT));
                } else {
                    grabTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_IMAGE));
                }

                // <<<<< Timestamp

                if (grab_status != sl::ERROR_CODE::SUCCESS) {
                    // Detect if a error occurred (for example:
                    // the zed have been disconnected) and
                    // re-initialize the ZED
                    if (grab_status != sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                        RCLCPP_WARN(get_logger(), "%s", sl::toString(grab_status).c_str());
                    } else {
                        continue;
                    }

                    rclcpp::Time now = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_CURRENT));

                    rcl_time_point_value_t elapsed = (now - mLastGrabTimestamp).nanoseconds();
                    rcl_time_point_value_t timeout = rclcpp::Duration(mCamTimeoutSec, 0).nanoseconds();

                    if (elapsed > timeout) {
                        if (!mSvoMode) {
                            //                            mThreadStop = true;
                            //                            RCLCPP_ERROR(get_logger(), "Camera timeout. Please verify the connection and restart the node.");
                            //                            this->shutdown();
                            //                            break;
                            std::lock_guard<std::mutex> lock(mReconnectMutex);
                            mReconnectThread = std::thread(&ZedCameraComponent::zedReconnectThreadFunc, this);
                            mReconnectThread.detach();
                            return;
                        } else {
                            RCLCPP_WARN(get_logger(), "The SVO reached the end");
                        }
                    }

                    std::this_thread::sleep_for(std::chrono::seconds(mCamTimeoutSec));
                    continue;
                }

                // Last frame timestamp
                mLastGrabTimestamp = grabTimestamp;

                // >>>>> Apply video dynamic parameters
                if (mZedAutoExposure) {
                    if (mTriggerAutoExposure) {
                        mZed.setCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE, 0, true);
                        mTriggerAutoExposure = false;
                    }
                } else {
                    int actual_exposure =
                        mZed.getCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE);

                    if (actual_exposure != mZedExposure) {
                        mZed.setCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE, mZedExposure);
                    }

                    int actual_gain = mZed.getCameraSettings(sl::CAMERA_SETTINGS_GAIN);

                    if (actual_gain != mZedGain) {
                        mZed.setCameraSettings(sl::CAMERA_SETTINGS_GAIN, mZedGain);
                    }
                }

                // <<<<< Apply video dynamic parameters

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
                //RCLCPP_DEBUG(get_logger(), "publishDepthData -> mPcDataReady TRUE")

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

        memcpy(ptCloudPtr, (float*)cpu_cloud, ptsCount * 4 * sizeof(float));

        // Pointcloud publishing
        mPubPointcloud->publish(mPointcloudMsg);
    }

    void ZedCameraComponent::zedReconnectThreadFunc() {
        std::lock_guard<std::mutex> lock(mReconnectMutex);

        rclcpp_lifecycle::State retState;
        LifecycleNodeInterface::CallbackReturn cbRet;

        retState = deactivate(cbRet);

        if (cbRet == LifecycleNodeInterface::CallbackReturn::SUCCESS) {
            retState = cleanup(cbRet);

            if (cbRet == LifecycleNodeInterface::CallbackReturn::SUCCESS) {
                retState = configure(cbRet);

                if (cbRet == LifecycleNodeInterface::CallbackReturn::SUCCESS) {
                    if (mZedReactivate) {
                        retState = activate(cbRet);

                        if (cbRet == LifecycleNodeInterface::CallbackReturn::SUCCESS) {
                            RCLCPP_INFO(get_logger(), "%s correctly restarted", sl::toString(mZedRealCamModel));
                        } else {
                            shutdown();
                        }
                    }
                } else {
                    shutdown();
                }
            } else {
                shutdown();
            }
        }
    }

    void ZedCameraComponent::pointcloudThreadFunc() {
        std::unique_lock<std::mutex> lock(mPcMutex);

        while (!mThreadStop) {

            //RCLCPP_DEBUG(get_logger(), "pointcloudThreadFunc -> mPcDataReady value: %s", mPcDataReady ? "TRUE" : "FALSE");

            while (!mPcDataReady) { // loop to avoid spurious wakeups
                if (mPcDataReadyCondVar.wait_for(lock, std::chrono::milliseconds(500)) == std::cv_status::timeout) {
                    // Check thread stopping
                    if (mThreadStop) {
                        return;
                    } else {
                        //RCLCPP_DEBUG(get_logger(), "pointcloudThreadFunc -> WAIT FOR CLOUD DATA");
                        continue;
                    }
                }
            }

            publishPointCloud();
            mPcDataReady = false;
            //RCLCPP_DEBUG(get_logger(), "pointcloudThreadFunc -> mPcDataReady FALSE")
        }

        //RCLCPP_DEBUG(get_logger(), "Pointcloud thread finished");
    }

    void ZedCameraComponent::imuPubCallback() {

        std::lock_guard<std::mutex> lock(mImuMutex);

        if (!mZed.isOpened()) {
            return;
        }

        uint32_t imu_SubNumber = count_subscribers(mImuTopic);
        uint32_t imu_RawSubNumber = count_subscribers(mImuRawTopic);

        if (imu_SubNumber < 1 && imu_RawSubNumber < 1) {
            return;
        }

        rclcpp::Time t;

        if (mSvoMode || !mRunGrabLoop || !mImuTimestampSync) {
            t = now();
        } else {
            t = mLastGrabTimestamp;
        }

        sl::IMUData imu_data;
        mZed.getIMUData(imu_data, sl::TIME_REFERENCE_CURRENT);

        if (imu_SubNumber > 0) {
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = t;
            imu_msg.header.frame_id = mImuFrameId;
            imu_msg.orientation.x = imu_data.getOrientation()[0];
            imu_msg.orientation.y = imu_data.getOrientation()[1];
            imu_msg.orientation.z = imu_data.getOrientation()[2];
            imu_msg.orientation.w = imu_data.getOrientation()[3];
            imu_msg.angular_velocity.x = imu_data.angular_velocity[0] * DEG2RAD;
            imu_msg.angular_velocity.y = imu_data.angular_velocity[1] * DEG2RAD;
            imu_msg.angular_velocity.z = imu_data.angular_velocity[2] * DEG2RAD;
            imu_msg.linear_acceleration.x = imu_data.linear_acceleration[0];
            imu_msg.linear_acceleration.y = imu_data.linear_acceleration[1];
            imu_msg.linear_acceleration.z = imu_data.linear_acceleration[2];

            for (int i = 0; i < 9; i++) {
                imu_msg.orientation_covariance[i] = imu_data.orientation_covariance.r[i];
                imu_msg.linear_acceleration_covariance[i] = imu_data.linear_acceleration_convariance.r[i];
                imu_msg.angular_velocity_covariance[i] = imu_data.angular_velocity_convariance.r[i];
            }

            mPubImu->publish(imu_msg);
        }

        if (imu_RawSubNumber > 0) {
            sensor_msgs::msg::Imu imu_raw_msg;
            imu_raw_msg.header.stamp = t;
            imu_raw_msg.header.frame_id = mImuFrameId;
            imu_raw_msg.angular_velocity.x = imu_data.angular_velocity[0] * DEG2RAD;
            imu_raw_msg.angular_velocity.y = imu_data.angular_velocity[1] * DEG2RAD;
            imu_raw_msg.angular_velocity.z = imu_data.angular_velocity[2] * DEG2RAD;
            imu_raw_msg.linear_acceleration.x = imu_data.linear_acceleration[0];
            imu_raw_msg.linear_acceleration.y = imu_data.linear_acceleration[1];
            imu_raw_msg.linear_acceleration.z = imu_data.linear_acceleration[2];

            for (int i = 0; i < 9; i++) {
                imu_raw_msg.linear_acceleration_covariance[i] = imu_data.linear_acceleration_convariance.r[i];
                imu_raw_msg.angular_velocity_covariance[i] = imu_data.angular_velocity_convariance.r[i];
            }

            // Orientation data is not available in "data_raw" -> See ROS REP145
            // http://www.ros.org/reps/rep-0145.html#topics
            imu_raw_msg.orientation_covariance[0] = -1;

            mPubImuRaw->publish(imu_raw_msg);
        }

        // Publish IMU tf only if enabled
        //        if (mPublishTf) {
        //            // Camera to pose transform from TF buffer
        //            tf2::Transform cam_to_pose;

        //            std::string poseFrame;
        //            // Look up the transformation from base frame to map link
        //            try {
        //                poseFrame = mPublishMapTf ? mMapFrameId : mOdometryFrameId;

        //                // Save the transformation from base to frame
        //                geometry_msgs::TransformStamped c2p =
        //                    mTfBuffer->lookupTransform(poseFrame, mCameraFrameId, ros::Time(0));
        //                // Get the TF2 transformation
        //                tf2::fromMsg(c2p.transform, cam_to_pose);
        //            } catch (tf2::TransformException& ex) {
        //                NODELET_WARN_THROTTLE(
        //                    10.0, "The tf from '%s' to '%s' does not seem to be available. "
        //                    "IMU TF not published!",
        //                    mCameraFrameId.c_str(), mMapFrameId.c_str());
        //                NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
        //                return;
        //            }

        //            // IMU Quaternion in Map frame
        //            tf2::Quaternion imu_q;
        //            imu_q.setX(mSignX * imu_data.getOrientation()[mIdxX]);
        //            imu_q.setY(mSignY * imu_data.getOrientation()[mIdxY]);
        //            imu_q.setZ(mSignZ * imu_data.getOrientation()[mIdxZ]);
        //            imu_q.setW(imu_data.getOrientation()[3]);
        //            // Pose Quaternion from ZED Camera
        //            tf2::Quaternion map_q = cam_to_pose.getRotation();
        //            // Difference between IMU and ZED Quaternion
        //            tf2::Quaternion delta_q = imu_q * map_q.inverse();
        //            tf2::Transform imu_pose;
        //            imu_pose.setIdentity();
        //            imu_pose.setRotation(delta_q);
        //            // Note, the frame is published, but its values will only change if someone
        //            // has subscribed to IMU
        //            publishImuFrame(imu_pose, mFrameTimestamp); // publish the imu Frame
        //        }
    }
}

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(stereolabs::ZedCameraComponent, rclcpp_lifecycle::LifecycleNode)
