#ifndef ZED_COMPONENT_HPP
#define ZED_COMPONENT_HPP

#include <rclcpp/rclcpp.hpp>
#include "visibility_control.h"

namespace zed_wrapper {

    inline void signalHandler(int signum) {
        RCUTILS_LOG_INFO("%s Signal is received! Terminate ZED Node...", strsignal(signum));
        rclcpp::shutdown();
        exit(signum);
    }

    class ZedCameraComponent : public rclcpp::Node {
      public:
        ZED_PUBLIC
        ZedCameraComponent();

      protected:
      private:

    };

}

#endif
