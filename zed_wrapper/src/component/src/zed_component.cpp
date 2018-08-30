
#include "zed_component.hpp"
#include "sl_tools.h"
#include <string>

using namespace std::chrono_literals;

#ifndef TIMER_ELAPSED
#define TIMER_ELAPSED double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
#endif

namespace stereolabs {

    ZedCameraComponent::ZedCameraComponent(const std::string& node_name, const std::string& ros_namespace, bool intra_process_comms)
        : rclcpp_lifecycle::LifecycleNode(node_name, ros_namespace, intra_process_comms) {


#ifndef NDEBUG
        rcutils_logging_set_logger_level(get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
#endif

        RCLCPP_INFO(get_logger(), "ZED Camera Component created");
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_shutdown(const rclcpp_lifecycle::State& previous_state) {
        RCLCPP_INFO(get_logger(), "*** State transition: %s ***", this->get_current_state().label().c_str());

        RCLCPP_DEBUG(get_logger(), "on_shutdown() is called.");

        // >>>>> Verify that all the threads are not active
        mThreadStop = true;
        if (mGrabThread.joinable()) {
            mGrabThread.join();
        }
        RCLCPP_DEBUG(get_logger(), "Grab thread joined");
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
        mThreadStop = true;
        if (mGrabThread.joinable()) {
            mGrabThread.join();
        }
        RCLCPP_DEBUG(get_logger(), "Grab thread joined");
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

        // >>>>> Start ZED thread
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
        mThreadStop = true;
        if (mGrabThread.joinable()) {
            mGrabThread.join();
        }
        // <<<<< Verify that the grab thread is not active

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

        // >>>>> Grab parameters
        sl::RuntimeParameters runParams;
        runParams.sensing_mode = static_cast<sl::SENSING_MODE>(mCamSensingMode);
        runParams.enable_depth = false;
        runParams.enable_point_cloud = false;
        // <<<<< Grab parameters

        rclcpp::Rate loop_rate(mCamFrameRate);

        INIT_TIMER;

        while (1) {
            // >>>>> Interruption check
            if (!rclcpp::ok()) {
                RCLCPP_DEBUG(get_logger(), "Ctrl+C received");
                break;
            }

            if (mThreadStop) {
                RCLCPP_DEBUG(get_logger(), "Grab thread stopped");
                break;
            }
            // <<<<< Interruption check

            bool runLoop = true;

            // TODO check subscribers!

            if (runLoop) {

                grab_status = mZed.grab(runParams);

                if (grab_status != sl::ERROR_CODE::SUCCESS) {
                    // Detect if a error occurred (for example:
                    // the zed have been disconnected) and
                    // re-initialize the ZED
                    if (grab_status != sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                        RCLCPP_WARN(get_logger(), sl::toString(grab_status));
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(2));

                    // TODO Check errors like in ROS1

                    continue;
                }

            }

            // >>>>> Thread sleep
            TIMER_ELAPSED;
            START_TIMER;

            static int rateWarnCount = 0;
            if (!loop_rate.sleep()) {
                rateWarnCount++;

                if (rateWarnCount == mCamFrameRate) {
                    RCLCPP_WARN(get_logger(),  "Expected cycle time: %g sec  - Real cycle time: %g sec ",
                                0.001 / mCamFrameRate, elapsed / 1000.0);
                    RCLCPP_INFO(get_logger(),  "Elaboration takes longer than requested "
                                "by the FPS rate. Please consider to "
                                "lower the 'frame_rate' setting.");

                    rateWarnCount = 0;
                }
            } else {
                rateWarnCount = 0;
            }

            RCLCPP_DEBUG(get_logger(), "Thread freq: %g Hz", 1000.0 / elapsed);
            // <<<<< Thread sleep
        }

        RCLCPP_INFO(get_logger(), "ZED thread finished");
    }
}

#include "class_loader/class_loader_register_macro.h"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(stereolabs::ZedCameraComponent, rclcpp_lifecycle::LifecycleNode)
