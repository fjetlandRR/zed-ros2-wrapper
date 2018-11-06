![](./images/Picto+STEREOLABS_Black.png)

# ZED ROS2 wrapper

The [ZED ROS2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper) lets you use the ZED stereo cameras with the second version of ROS. It provides access to the following data:

  - Left and right rectified/unrectified images
  - Depth data
  - Colored 3D point cloud
  - IMU data

![](./images/PointCloud_Depth_ROS.jpg)

## Prerequisites

- Ubuntu 16.04 or Ubuntu 18.04 (*support for Win10 will be provided soon*)
- [ZED SDK](https://www.stereolabs.com/developers/release/latest/) v2.6 or gratest
- [CUDA](https://developer.nvidia.com/cuda-downloads) dependency
- ROS2 Bouncy: Ubuntu 16.04 [[source](https://index.ros.org/doc/ros2/Linux-Development-Setup)] 
  - Ubuntu 18.04 [[binaries](https://index.ros.org/doc/ros2/Linux-Install-Debians) 
  - [source](https://index.ros.org/doc/ros2/Linux-Development-Setup)]
