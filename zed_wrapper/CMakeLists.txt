cmake_minimum_required(VERSION 3.5)

project(stereolabs_zed)

# ROS2 Flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

function(checkPackage package customMessage)
    set(varName "${package}_FOUND")
    if (NOT "${${varName}}")
        string(REPLACE "_" "-" aptPackage ${package})
        if("${customMessage}" STREQUAL "")
            message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
        else()
            message(FATAL_ERROR "\n\n ${customMessage} \n\n")
        endif()
    endif()
endfunction(checkPackage)

find_package(ZED 2.3)
checkPackage("ZED" "ZED SDK not found, install it from:\n https://www.stereolabs.com/developers/")

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" ) # Jetson TX
    SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

find_package(OpenCV 3 COMPONENTS core highgui imgproc)
checkPackage("OPENCV_CORE" "OpenCV core not found, install it from the tutorial at:\n https://www.stereolabs.com/documentation/overview/getting-started/getting-started.html")
checkPackage("OPENCV_HIGHGUI" "OpenCV highgui not found, install it from the tutorial at:\n https://www.stereolabs.com/documentation/overview/getting-started/getting-started.html")
checkPackage("OPENCV_IMGPROC" "OpenCV imgproc not found, install it from the tutorial at:\n https://www.stereolabs.com/documentation/overview/getting-started/getting-started.html")

find_package(CUDA)
checkPackage("CUDA" "CUDA not found, install it from:\n https://developer.nvidia.com/cuda-downloads")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

###############################################################################
# INCLUDES and LIBS
include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include
    ${rclcpp_lifecycle_INCLUDE_DIRS}
    ${lifecycle_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${class_loader_INCLUDE_DIRS}
    ${rmw_implementation_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${stereo_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

if(NOT DEFINED CUDA_NPP_LIBRARIES_ZED)
    #To deal with cuda 9 nppi libs and previous versions of ZED SDK
    set(CUDA_NPP_LIBRARIES_ZED ${CUDA_npp_LIBRARY} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY})
endif()

set(LINK_LIBRARIES
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES} ${CUDA_NPP_LIBRARIES_ZED}
  ${OpenCV_LIBS}
  )

###############################################################################
# SOURCES
set(NODE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_wrapper_container.cpp
)

set(COMPONENT_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/src/zed_component.cpp
)

###############################################################################
# ZED COMPONENT and ZED EXECUTABLE

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(zed_component SHARED ${COMPONENT_SRC})
target_compile_definitions(zed_component PRIVATE "ZED_BUILDING_DLL")
target_link_libraries(zed_component
    ${rclcpp_lifecycle_LIBRARIES}
    ${LINK_LIBRARIES}
)
ament_target_dependencies(zed_component
  lifecycle_msgs
  rclcpp_lifecycle
  rclcpp
  std_msgs
  sensor_msgs
  stereo_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  nav_msgs
)
rclcpp_register_node_plugins(zed_component "stereolabs::ZedCameraComponent")
set(node_plugins "${node_plugins}stereolabs::ZedCameraComponent;$<TARGET_FILE:zed_component>\n")

add_executable(zed_wrapper_node ${NODE_SRC})
target_link_libraries(zed_wrapper_node
    zed_component
    ${LINK_LIBRARIES}
)
ament_target_dependencies(zed_wrapper_node
  rclcpp
  class_loader
  zed_component)

# Install executable
install(TARGETS zed_wrapper_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugin library
install(TARGETS zed_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install header files
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include/
  DESTINATION include/${PROJECT_NAME}/
)

ament_package()
