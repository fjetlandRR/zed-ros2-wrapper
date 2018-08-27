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
        COMPOSITION_PUBLIC
        ZedCameraComponent();

      protected:
      private:

    };

}

#include "class_loader/class_loader_register_macro.h"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(zed_wrapper::ZedCameraComponent, rclcpp::Node)

#endif
