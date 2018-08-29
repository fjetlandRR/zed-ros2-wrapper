#ifndef ZED_COMPONENT_HPP
#define ZED_COMPONENT_HPP

#include "visibility_control.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

#include "sl/Camera.hpp"

namespace stereolabs {

    /// ZedCameraComponent inheriting from rclcpp_lifecycle::LifecycleNode
    /**
    * The ZedCameraComponent does not like the regular "talker" node
    * inherit from node, but rather from lifecyclenode. This brings
    * in a set of callbacks which are getting invoked depending on
    * the current state of the node.
    * Every lifecycle node has a set of services attached to it
    * which make it controllable from the outside and invoke state
    * changes.
    * Available Services as for Beta1:
    * - <node_name>__get_state
    * - <node_name>__change_state
    * - <node_name>__get_available_states
    * - <node_name>__get_available_transitions
    * Additionally, a publisher for state change notifications is
    * created:
    * - <node_name>__transition_event
    */
    class ZedCameraComponent : public rclcpp_lifecycle::LifecycleNode {
      public:
        ZED_PUBLIC
        /// ZedCameraComponent constructor
        /**
        * The ZedCameraComponent/lifecyclenode constructor has the same
        * arguments a regular node.
        */
        explicit ZedCameraComponent(const std::string& node_name = "zed_node",
                                    const std::string& ros_namespace = "zed",
                                    bool intra_process_comms = false);

        /// Callback for walltimer in order to publish the message.
        /**
        * Callback for walltimer. This function gets invoked by the timer
        * and executes the publishing.
        * For this demo, we ask the node for its current state. If the
        * lifecycle publisher is not activate, we still invoke publish, but
        * the communication is blocked so that no messages is actually transferred.
        */
        void publish();

        /// Transition callback for state error
        /**
        * on_error callback is being called when the lifecycle node
        * enters the "error" state.
        */
        rcl_lifecycle_transition_key_t on_error(const rclcpp_lifecycle::State& previous_state);

        /// Transition callback for state shutting down
        /**
        * on_shutdown callback is being called when the lifecycle node
        * enters the "shutting down" state.
        */
        rcl_lifecycle_transition_key_t on_shutdown(const rclcpp_lifecycle::State& previous_state);

        /// Transition callback for state configuring
        /**
        * on_configure callback is being called when the lifecycle node
        * enters the "configuring" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "inactive" state or stays
        * in "unconfigured".
        * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
        * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rcl_lifecycle_transition_key_t on_configure(const rclcpp_lifecycle::State&);

        /// Transition callback for state activating
        /**
        * on_activate callback is being called when the lifecycle node
        * enters the "activating" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "active" state or stays
        * in "inactive".
        * TRANSITION_CALLBACK_SUCCESS transitions to "active"
        * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rcl_lifecycle_transition_key_t on_activate(const rclcpp_lifecycle::State&);

        /// Transition callback for state deactivating
        /**
        * on_deactivate callback is being called when the lifecycle node
        * enters the "deactivating" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "inactive" state or stays
        * in "active".
        * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
        * TRANSITION_CALLBACK_FAILURE transitions to "active"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rcl_lifecycle_transition_key_t on_deactivate(const rclcpp_lifecycle::State&);

        /// Transition callback for state cleaningup
        /**
        * on_cleanup callback is being called when the lifecycle node
        * enters the "cleaningup" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "unconfigured" state or stays
        * in "inactive".
        * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
        * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rcl_lifecycle_transition_key_t on_cleanup(const rclcpp_lifecycle::State&);

      protected:
        void zedGrabThreadFunc();

      private:
        std::shared_ptr<std_msgs::msg::String> msg_;

        // We hold an instance of a lifecycle publisher. This lifecycle publisher
        // can be activated or deactivated regarding on which state the lifecycle node
        // is in.
        // By default, a lifecycle publisher is inactive by creation and has to be
        // activated to publish messages into the ROS world.
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

        // We hold an instance of a timer which periodically triggers the publish function.
        // As for the beta version, this is a regular timer. In a future version, a
        // lifecycle timer will be created which obeys the same lifecycle management as the
        // lifecycle publisher.
        std::shared_ptr<rclcpp::TimerBase> timer_;

        // Grab thread
        std::thread mGrabThread;
        bool mThreadStop = false;

        // ZED SDK
        sl::Camera mZed;
        sl::InitParameters mZedParams;

        // ZED params
        int mZedId = 0;
        unsigned int mZedSerialNumber;
        int mCamFrameRate = 30;
        std::string mSvoFilepath = "";
        bool mSvoMode = false;
        int mCamResol = 2; // Default resolution: RESOLUTION_HD720


    };
}

#endif
