
#include "zed_component.hpp"

namespace zed_wrapper {

    ZedCameraComponent::ZedCameraComponent()
        : rclcpp::Node("zed_wrapper") {

    }

}

#include "class_loader/class_loader_register_macro.h"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(zed_wrapper::ZedCameraComponent, rclcpp::Node)
