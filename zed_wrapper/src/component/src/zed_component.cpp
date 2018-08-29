
#include "zed_component.hpp"

using namespace std::chrono_literals;

namespace stereolabs {

    ZedCameraComponent::ZedCameraComponent(const std::string& node_name, const std::string& ros_namespace, bool intra_process_comms)
        : rclcpp_lifecycle::LifecycleNode(node_name, ros_namespace, intra_process_comms) {


#ifndef NDEBUG
        rcutils_logging_set_logger_level(get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
#endif

        RCLCPP_INFO(get_logger(), "ZED Camera Component created");
    }

    void ZedCameraComponent::publish() {
        static size_t count = 0;
        msg_->data = "ZED HelloWorld #" + std::to_string(++count) + " - Current state: " + this->get_current_state().label();

        // Print the current state for demo purposes
        if (!pub_->is_activated()) {
            RCLCPP_INFO(get_logger(), "ZED node is currently inactive. Messages are not published.")
        } else {
            RCLCPP_INFO(get_logger(), "ZED node is active. Publishing: [%s]", msg_->data.c_str())
        }

        // We independently from the current state call publish on the lifecycle
        // publisher.
        // Only if the publisher is in an active state, the message transfer is
        // enabled and the message actually published.
        pub_->publish(msg_);
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_shutdown(const rclcpp_lifecycle::State& previous_state) {
        RCLCPP_INFO(get_logger(), "on_shutdown() is called.");

        RCLCPP_INFO(get_logger(), "ZED node is shutting down");

        // >>>>> Verify that all the threads are not active
        mThreadStop = true;
        if (mGrabThread.joinable()) {
            mGrabThread.join();
        }
        // <<<<< Verify that the grab thread is not active

        // >>>>> Verify that ZED is not opened
        if (mZed.isOpened()) {
            mZed.close();
        }
        // <<<<< Verify that ZED is not opened

        RCLCPP_INFO(get_logger(), "shutdown complete");

        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_error(const rclcpp_lifecycle::State& previous_state) {
        RCLCPP_INFO(get_logger(), "on_error() is called.");

        switch (previous_state.id()) {
        case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE: { // Error in CONFIGURE STATE
            RCLCPP_INFO(get_logger(), "Finalizing the node");

            return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
        }
        break;

        default:
            RCLCPP_INFO(get_logger(), "Transition error not handled: %d", previous_state.id())
        }

        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_configure(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(get_logger(), "on_configure() is called.");

        // >>>>> Check SDK version
#if (ZED_SDK_MAJOR_VERSION<2 || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION<6))
        RCLCPP_ERROR(get_logger(), "ROS2 ZED node requires ZED SDK > v2.6.0");

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
            mZedParams.camera_resolution = static_cast<sl::RESOLUTION>(mCamResol);

            if (mZedSerialNumber == 0) {
                mZedParams.camera_linux_id = mZedId;
            }
        }
        // <<<<< ZED configuration

        // We return a success and hence invoke the transition to the next
        // step: "inactive".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "unconfigured" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_activate(const rclcpp_lifecycle::State&) {
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

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
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

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
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");

        // TODO close ZED and clean all the data structures to go back to "unconfigured" state
        if (mZed.isOpened()) {
            mZed.close();
        }

        // We return a success and hence invoke the transition to the next
        // step: "unconfigured".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "inactive" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    void ZedCameraComponent::zedGrabThreadFunc() {
        RCUTILS_LOG_INFO_NAMED(get_name(), "ZED thread started");

        while (1) {
            if (!rclcpp::ok() || mThreadStop) {
                RCUTILS_LOG_INFO_NAMED(get_name(), "ZED thread stop requested");

                if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    RCUTILS_LOG_INFO_NAMED(get_name(), "Forcing node shutdown")
                    shutdown();
                } else {
                    break;
                }
            }
        }

        RCUTILS_LOG_INFO_NAMED(get_name(), "ZED thread finished");
    }
}

#include "class_loader/class_loader_register_macro.h"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(stereolabs::ZedCameraComponent, rclcpp_lifecycle::LifecycleNode)
