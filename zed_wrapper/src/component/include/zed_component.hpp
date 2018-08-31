#ifndef ZED_COMPONENT_HPP
#define ZED_COMPONENT_HPP

#include "visibility_control.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/core/core.hpp>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "sl/Camera.hpp"

namespace stereolabs {

    // >>>>> Typedefs to simplify declarations
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> imagePub;
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>> camInfoPub;

    typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;
    // <<<<< Typedefs to simplify declarations

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

        virtual ~ZedCameraComponent();

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

        void initPublishers();

        void publishImages(rclcpp::Time timeStamp);

        /* \brief Get the information of the ZED cameras and store them in an
         * information message
         * \param zed : the sl::zed::Camera* pointer to an instance
         * \param left_cam_info_msg : the information message to fill with the left
         * camera informations
         * \param right_cam_info_msg : the information message to fill with the right
         * camera informations
         * \param left_frame_id : the id of the reference frame of the left camera
         * \param right_frame_id : the id of the reference frame of the right camera
         */
        void fillCamInfo(sl::Camera& zed, std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
                         std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg,
                         std::string leftFrameId, std::string rightFrameId,
                         bool rawParam = false);

        /* \brief Publish the informations of a camera with a ros Publisher
         * \param cam_info_msg : the information message to publish
         * \param pub_cam_info : the publisher object to use
         * \param t : the ros::Time to stamp the message
         */
        void publishCamInfo(camInfoMsgPtr camInfoMsg, camInfoPub pubCamInfo, rclcpp::Time t);

        /* \brief Publish a cv::Mat image with a ros Publisher
         * \param img : the image to publish
         * \param pub_img : the publisher object to use (different image publishers
         * exist)
         * \param img_frame_id : the id of the reference frame of the image (different
         * image frames exist)
         * \param t : the ros::Time to stamp the image
         */
        void publishImage(cv::Mat img, imagePub pubImg, std::string imgFrameId, rclcpp::Time t);


      private:
        // Status variables
        rcl_lifecycle_transition_key_t mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_CREATE;

        // Grab thread
        std::thread mGrabThread;
        bool mThreadStop = false;
        int mCamTimeoutMsec = 5000; // Error generated if camera is not available after timeout

        // ZED SDK
        sl::Camera mZed;

        // ZED params
        sl::InitParameters mZedParams;
        int mZedId = 0;
        unsigned int mZedSerialNumber = 0;
        int mZedUserCamModel = 1;   // Camera model set by ROS Param
        sl::MODEL mZedRealCamModel; // Camera model requested to SDK
        int mCamFrameRate = 30;
        std::string mSvoFilepath = "";
        bool mSvoMode = false;
        bool mVerbose = true;
        int mGpuId = -1;
        int mZedResol = 2; // Default resolution: RESOLUTION_HD720
        int mZedQuality = 1; // Default quality: DEPTH_MODE_PERFORMANCE
        int mDepthStabilization = 1;
        bool mCameraFlip = false;
        double mZedMatResizeFactor = 0.5;
        int mCamSensingMode = 0; // Default Sensing mode: SENSING_MODE_STANDARD

        // Publishers
        imagePub mPubRgb;
        imagePub mPubRawRgb;
        imagePub mPubLeft;
        imagePub mPubRawLeft;
        imagePub mPubRight;
        imagePub mPubRawRight;

        camInfoPub mPubRgbCamInfo;
        camInfoPub mPubRgbCamInfoRaw;
        camInfoPub mPubLeftCamInfo;
        camInfoPub mPubLeftCamInfoRaw;
        camInfoPub mPubRightCamInfo;
        camInfoPub mPubRightCamInfoRaw;

        // Topics
        std::string mLeftTopic;
        std::string mLeftRawTopic;
        std::string mLeftCamInfoTopic;
        std::string mLeftCamInfoRawTopic;
        std::string mRightTopic;
        std::string mRightRawTopic;
        std::string mRightCamInfoTopic;
        std::string mRightCamInfoRawTopic;
        std::string mRgbTopic;
        std::string mRgbRawTopic;
        std::string mRgbCamInfoTopic;
        std::string mRgbCamInfoRawTopic;

        // Messages
        // Camera info
        camInfoMsgPtr mRgbCamInfoMsg;
        camInfoMsgPtr mLeftCamInfoMsg;
        camInfoMsgPtr mRightCamInfoMsg;
        camInfoMsgPtr mRgbCamInfoRawMsg;
        camInfoMsgPtr mLeftCamInfoRawMsg;
        camInfoMsgPtr mRightCamInfoRawMsg;

        // Frame IDs
        std::string mRightCamFrameId;
        std::string mRightCamOptFrameId;
        std::string mLeftCamFrameId;
        std::string mLeftCamOptFrameId;
        std::string mDepthFrameId;
        std::string mDepthOptFrameId;

        // OpenCV Mats
        int mCamWidth;
        int mCamHeight;
        int mMatWidth;
        int mMatHeight;
        cv::Mat mCvLeftImRGB;
        cv::Mat mCvRightImRGB;
        cv::Mat mCvConfImRGB;
        cv::Mat mCvConfMapFloat;

    };
}

#endif
