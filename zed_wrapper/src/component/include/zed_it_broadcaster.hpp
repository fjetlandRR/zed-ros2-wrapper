#ifndef ZED_IT_BROADCASTER_HPP
#define ZED_IT_BROADCASTER_HPP

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

#include <rclcpp/node.hpp>
#include <image_transport/image_transport.h>

namespace stereolabs {

    typedef rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub;
    typedef rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoSub;

    class ZedItBroadcaster : public rclcpp::Node {
      public:
        ZED_PUBLIC
        explicit ZedItBroadcaster(const std::string& node_name = "zed_it_broadcaster",
                                  const std::string& ros_namespace = "zed",
                                  bool intra_process_comms = true);

      protected:
        void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rgbInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void rightCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rightInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void leftCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void leftInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        void rgbRawCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rgbInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void rightRawCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rightInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void leftRawCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void leftInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        void initParameters();
        void initPublishers();

      private:
        rmw_qos_profile_t mCamQosProfile;

        // Params
        bool mOpenniDepthMode = false;
        std::string mLeftTopicRoot = "left";
        std::string mRightTopicRoot = "right";
        std::string mRgbTopicRoot = "rgb";
        std::string mDepthTopicRoot = "depth";

        // Subscribers
        imgSub mRgbSub;
        camInfoSub mRgbInfoSub;
        imgSub mRightSub;
        camInfoSub mRightInfoSub;
        imgSub mLeftSub;
        camInfoSub mLeftInfoSub;
        imgSub mRawRgbSub;
        camInfoSub mRawRgbInfoSub;
        imgSub mRawRightSub;
        camInfoSub mRawRightInfoSub;
        imgSub mRawLeftSub;
        camInfoSub mRawLeftInfoSub;
        imgSub mDepthSub;
        camInfoSub mDepthInfoSub;

        // Subscribers topic names


        // Publisher topics
        std::string mRgbTopic;
        std::string mRightTopic;
        std::string mLeftTopic;
        std::string mRawRgbTopic;
        std::string mRawRightTopic;
        std::string mRawLeftTopic;
        std::string mDepthTopic;

        // Camera Info Messages
        sensor_msgs::msg::CameraInfo mRgbInfoMsg;
        sensor_msgs::msg::CameraInfo mRightInfoMsg;
        sensor_msgs::msg::CameraInfo mLeftInfoMsg;
        sensor_msgs::msg::CameraInfo mRawRgbInfoMsg;
        sensor_msgs::msg::CameraInfo mRawRightInfoMsg;
        sensor_msgs::msg::CameraInfo mRawLeftInfoMsg;
        sensor_msgs::msg::CameraInfo mDepthInfoMsg;

        // Camera Publishers
        bool mPubInitialized = false;
        image_transport::CameraPublisher mRgbPub;
        image_transport::CameraPublisher mRightPub;
        image_transport::CameraPublisher mLeftPub;
        image_transport::CameraPublisher mRawRgbPub;
        image_transport::CameraPublisher mRawRightPub;
        image_transport::CameraPublisher mRawLeftPub;
        image_transport::CameraPublisher mDepthPub;
    };

}  // namespace stereolabs

#endif // #define ZED_IT_BROADCASTER_HPP
