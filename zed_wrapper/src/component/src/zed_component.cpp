
#include "zed_component.hpp"

using namespace std::chrono_literals;

namespace stereolabs {

    ZedCameraComponent::ZedCameraComponent(const std::string& node_name, const std::string& ros_namespace, bool intra_process_comms)
        : rclcpp_lifecycle::LifecycleNode(node_name, ros_namespace, intra_process_comms) {
        RCUTILS_LOG_INFO("ZED Camera Component created");
    }

    void ZedCameraComponent::publish() {
        static size_t count = 0;
        msg_->data = "ZED HelloWorld #" + std::to_string(++count);

        // Print the current state for demo purposes
        if (!pub_->is_activated()) {
            RCLCPP_INFO(
                get_logger(), "ZED node is currently inactive. Messages are not published.")
        } else {
            RCLCPP_INFO(
                get_logger(), "ZED node is active. Publishing: [%s]", msg_->data.c_str())
        }

        // We independently from the current state call publish on the lifecycle
        // publisher.
        // Only if the publisher is in an active state, the message transfer is
        // enabled and the message actually published.
        pub_->publish(msg_);
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_configure(const rclcpp_lifecycle::State&) {
        // This callback is supposed to be used for initialization and
        // configuring purposes.
        // We thus initialize and configure our messages, publishers and timers.
        // The lifecycle node API does return lifecycle components such as
        // lifecycle publishers. These entities obey the lifecycle and
        // can comply to the current state of the node.
        // As of the beta version, there is only a lifecycle publisher
        // available.
        msg_ = std::make_shared<std_msgs::msg::String>();
        pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter");
        timer_ = this->create_wall_timer(1s, std::bind(&ZedCameraComponent::publish, this));

        RCLCPP_INFO(get_logger(), "on_configure() is called.")

        // We return a success and hence invoke the transition to the next
        // step: "inactive".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "unconfigured" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_activate(const rclcpp_lifecycle::State&) {
        // We explicitly activate the lifecycle publisher.
        // Starting from this point, all messages are no longer
        // ignored but sent into the network.
        pub_->on_activate();

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.")

        // Let's sleep for 2 seconds.
        // We emulate we are doing important
        // work in the activating phase.
        std::this_thread::sleep_for(2s);

        // We return a success and hence invoke the transition to the next
        // step: "active".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "inactive" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_deactivate(const rclcpp_lifecycle::State&) {
        // We explicitly deactivate the lifecycle publisher.
        // Starting from this point, all messages are no longer
        // sent into the network.
        pub_->on_deactivate();

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.")

        // We return a success and hence invoke the transition to the next
        // step: "inactive".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "active" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }

    rcl_lifecycle_transition_key_t ZedCameraComponent::on_cleanup(const rclcpp_lifecycle::State&) {
        // In our cleanup phase, we release the shared pointers to the
        // timer and publisher. These entities are no longer available
        // and our node is "clean".
        timer_.reset();
        pub_.reset();

        RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.")

        // We return a success and hence invoke the transition to the next
        // step: "unconfigured".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "inactive" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
    }
}

#include "class_loader/class_loader_register_macro.h"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(stereolabs::ZedCameraComponent, rclcpp_lifecycle::LifecycleNode)
