#ifndef ZED_COMPONENT_HPP
#define ZED_COMPONENT_HPP

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
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"


#include "sl/Camera.hpp"

namespace stereolabs {

    // >>>>> Typedefs to simplify declarations
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> imagePub;
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>> camInfoPub;
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<stereo_msgs::msg::DisparityImage>> disparityPub;

    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pointcloudPub;

    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imuPub;

    typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;
    typedef std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloudMsgPtr;
    typedef std::shared_ptr<sensor_msgs::msg::Imu> imuMsgPtr;
    // <<<<< Typedefs to simplify declarations

    /// ZedCameraComponent inheriting from rclcpp_lifecycle::LifecycleNode
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
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State&
                previous_state);

        /// Transition callback for state shutting down
        /**
        * on_shutdown callback is being called when the lifecycle node
        * enters the "shutting down" state.
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& previous_state);

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
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&);

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
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&);

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
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&);

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
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

      protected:
        void zedGrabThreadFunc();
        void pointcloudThreadFunc();
        void zedReconnectThreadFunc();

        void initPublishers();
        void initParameters();

        void publishImages(rclcpp::Time timeStamp);
        void publishDepthData(rclcpp::Time timeStamp);

        /** \brief Get the information of the ZED cameras and store them in an
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

        /** \brief Publish the informations of a camera with a ros Publisher
         * \param cam_info_msg : the information message to publish
         * \param pub_cam_info : the publisher object to use
         * \param timeStamp : the ros::Time to stamp the message
         */
        void publishCamInfo(camInfoMsgPtr camInfoMsg, camInfoPub pubCamInfo, rclcpp::Time timeStamp);

        /** \brief Publish a cv::Mat image with a ros Publisher
         * \param img : the image to publish
         * \param pub_img : the publisher object to use (different image publishers
         * exist)
         * \param img_frame_id : the id of the reference frame of the image (different
         * image frames exist)
         * \param timeStamp : the ros::Time to stamp the image
         */
        void publishImage(sl::Mat img, imagePub pubImg, std::string imgFrameId, rclcpp::Time timeStamp);

        /** \brief Publish a cv::Mat depth image with a ros Publisher
         * \param depth : the depth image to publish
         * \param timeStamp : the ros::Time to stamp the depth image
         */
        void publishDepth(sl::Mat depth, rclcpp::Time timeStamp);

        /** \brief Publish a cv::Mat disparity image with a ros Publisher
         * \param disparity : the disparity image to publish
         * \param timestamp : the ros::Time to stamp the depth image
         */
        void publishDisparity(sl::Mat disparity, rclcpp::Time timestamp);

        /** \brief Publish a pointCloud with a ros Publisher
         */
        void publishPointCloud();

        /* \brief Callback to publish IMU raw data with a ROS publisher.
         * \param e : the ros::TimerEvent binded to the callback
         */
        void imuPubCallback();

        /* \brief Callback to handle parameters changing
         * \param e : the ros::TimerEvent binded to the callback
         */
        rcl_interfaces::msg::SetParametersResult paramChangeCallback(std::vector<rclcpp::Parameter> parameters);

      private:
        // Status variables
        uint8_t mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_CREATE;

        // Timestamps
        rclcpp::Time mLastGrabTimestamp;
        rclcpp::Time mPointCloudTime;

        // Grab thread
        std::thread mGrabThread;
        bool mThreadStop = false;
        bool mRunGrabLoop = false;

        // Pointcloud thread
        std::thread mPcThread; // Point Cloud thread

        // Reconnect thread
        std::thread mReconnectThread;
        std::mutex mReconnectMutex;

        // IMU Timer
        rclcpp::TimerBase::SharedPtr mImuTimer = nullptr;

        // ZED SDK
        sl::Camera mZed;

        // Params
        sl::InitParameters mZedParams;
        int mZedId = 0;
        int mZedSerialNumber = 0;
        int mZedUserCamModel = 1;   // Camera model set by ROS Param
        sl::MODEL mZedRealCamModel; // Camera model requested to SDK
        int mZedFrameRate = 30;
        std::string mSvoFilepath = "";
        bool mSvoMode = false;
        bool mVerbose = true;
        int mGpuId = -1;
        int mZedResol = 2; // Default resolution: RESOLUTION_HD720
        int mZedQuality = 1; // Default quality: DEPTH_MODE_PERFORMANCE
        int mDepthStabilization = 1;
        int mCamTimeoutSec = 5;
        int mMaxReconnectTemp = 5;
        bool mZedReactivate =
            false; // Reactivate the camera after a "clean up"+"configure" due to a disconnection. Set to false if there is an external lifecycle node manager
        bool mCameraFlip = false;
        int mZedSensingMode = 0; // Default Sensing mode: SENSING_MODE_STANDARD
        bool mOpenniDepthMode = false; // 16 bit UC data in mm else 32F in m,
        // for more info -> http://www.ros.org/reps/rep-0118.html

        double mZedMinDepth = 0.2;

        double mImuPubRate = 500.0;
        bool mImuTimestampSync = true;

        // ZED dynamic params
        double mZedMatResizeFactor = 1.0;   // Dynamic...
        int mZedConfidence = 80;            // Dynamic...
        double mZedMaxDepth = 10.0;         // Dynamic...
        bool mZedAutoExposure;              // Dynamic...
        int mZedGain = 80;                  // Dynamic...
        int mZedExposure = 80;              // Dynamic...

        // Publishers
        imagePub mPubRgb;
        imagePub mPubRawRgb;
        imagePub mPubLeft;
        imagePub mPubRawLeft;
        imagePub mPubRight;
        imagePub mPubRawRight;
        imagePub mPubDepth;
        imagePub mPubConfImg;
        imagePub mPubConfMap;

        camInfoPub mPubRgbCamInfo;
        camInfoPub mPubRgbCamInfoRaw;
        camInfoPub mPubLeftCamInfo;
        camInfoPub mPubLeftCamInfoRaw;
        camInfoPub mPubRightCamInfo;
        camInfoPub mPubRightCamInfoRaw;
        camInfoPub mPubDepthCamInfo;
        camInfoPub mPubConfidenceCamInfo;

        disparityPub mPubDisparity;

        pointcloudPub mPubPointcloud;

        imuPub mPubImu;
        imuPub mPubImuRaw;

        // Topics
        std::string mLeftTopicRoot  = "left";
        std::string mRightTopicRoot = "right";
        std::string mRgbTopicRoot   = "rgb";
        std::string mDepthTopicRoot = "depth";
        std::string mConfTopicRoot = "confidence";
        std::string mImuTopicRoot = "imu";

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

        std::string mDepthTopic;
        std::string mDepthCamInfoTopic;
        std::string mConfImgTopic;
        std::string mConfCamInfoTopic;
        std::string mConfMapTopic;
        std::string mDispTopic;
        std::string mPointcloudTopic;
        std::string mImuTopic;
        std::string mImuRawTopic;

        // Messages
        // Camera info
        camInfoMsgPtr mRgbCamInfoMsg;
        camInfoMsgPtr mLeftCamInfoMsg;
        camInfoMsgPtr mRightCamInfoMsg;
        camInfoMsgPtr mRgbCamInfoRawMsg;
        camInfoMsgPtr mLeftCamInfoRawMsg;
        camInfoMsgPtr mRightCamInfoRawMsg;
        camInfoMsgPtr mDepthCamInfoMsg;
        camInfoMsgPtr mConfidenceCamInfoMsg;
        // Pointcloud
        pointcloudMsgPtr mPointcloudMsg;
        // IMU
        imuMsgPtr mImuMsg;
        imuMsgPtr mImuMsgRaw;

        // Frame IDs
        std::string mRightCamFrameId = "zed_right_camera_frame";
        std::string mRightCamOptFrameId = "zed_right_camera_optical_frame";
        std::string mLeftCamFrameId = "zed_left_camera_frame";
        std::string mLeftCamOptFrameId = "zed_left_camera_optical_frame";
        std::string mDepthFrameId;
        std::string mDepthOptFrameId;

        std::string mBaseFrameId = "base_link";
        std::string mCameraFrameId = "zed_camera_center";

        std::string mImuFrameId = "zed_imu_link";

        // SL Pointcloud
        sl::Mat mCloud;

        // Mats
        int mCamWidth;  // Camera frame width
        int mCamHeight; // Camera frame height
        int mMatWidth;  // Data width (mCamWidth*mZedMatResizeFactor)
        int mMatHeight; // Data height (mCamHeight*mZedMatResizeFactor)

        // Thread Sync
        std::mutex mImuMutex;
        std::mutex mCamDataMutex;
        std::mutex mPcMutex;
        std::condition_variable mPcDataReadyCondVar;
        bool mPcDataReady = false;
        bool mTriggerAutoExposure = false;
    };
}

#endif
