![](./images/Picto+STEREOLABS_Black.png)

The [ZED ROS2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper) lets you use the ZED stereo cameras with the second version of ROS. It provides access to the following data:

  - Left and right rectified/unrectified images
  - Depth data
  - Colored 3D point cloud
  - IMU data

The [detailed guide](https://www.stereolabs.com/docs/ros2/) is available on [Stereolabs website](https://www.stereolabs.com/docs/ros2/)

![](./images/PointCloud_Depth_ROS.jpg)

## Installation

### Prerequisites

- Ubuntu 16.04 or Ubuntu 18.04 (*support for Windows 10 will be provided soon*)
- [ZED SDK](https://www.stereolabs.com/developers/release/latest/) v2.6 or later
- [CUDA](https://developer.nvidia.com/cuda-downloads) dependency
- ROS2 Bouncy: 
  - Ubuntu 16.04 [[source](https://index.ros.org/doc/ros2/Linux-Development-Setup)] 
  - Ubuntu 18.04 [[binaries](https://index.ros.org/doc/ros2/Linux-Install-Debians) - [source](https://index.ros.org/doc/ros2/Linux-Development-Setup)]

### Build the package
The **zed_ros2_wrapper** is a [colcon](http://design.ros2.org/articles/build_tool.html) package. It depends on the following ROS2 packages:

  - ament_cmake
  - ament_index_cpp
  - class_loader
  - lifecycle_msgs
  - rclcpp_lifecycle
  - sensor_msgs
  - tf2
  - tf2_ros
  - tf2_geometry_msgs
  - nav_msgs
  - stereo_msgs
  - urdf
  - robot_state_publisher
  - message_runtime

To install the **zed_ros2_wrapper**, open a bash terminal, clone the package from Github, and build it:

```bash
$ cd ~/ros2_ws/src/ #use your current catkin folder
$ git clone https://github.com/stereolabs/zed-ros2-wrapper.git
$ cd ..
$ colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
$ echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
$ source ~/.bashrc
```

## Starting the ZED node
The ZED is available in ROS2 as a [lifecycle managed node](https://index.ros.org/doc/ros2/Managed-Nodes/) that publishes its data to topics. You can get the full list of the available topics [here](/integrations/ros2/zed_node/). 

To start the ZED node, open a terminal and use the [CLI](https://index.ros.org/doc/ros2/Introspection-with-command-line-tools/) command `ros2 launch`:

```bash
$ ros2 launch stereolabs_zed zed.launch.py
```

The `zed.launch.py` is a Python launch script that automatically manages the lifecycle state transitions of the ZED ROS2 node. You can run the `zed_unmanaged.launch.py` launch script if you want to manually control the state of the node. For a full guide about manually managing the lifecycle states of the ZED ROS2 node, please follow the [lifecycle tutorial](/integrations/ros2/lifecycle/)

## Displaying ZED data

### Using RVIZ2
RVIZ2 is a useful visualization tool in ROS2. Using RVIZ2, you can visualize the ZED left and right images, the depth image and the 3D colored point cloud.

Launch the ZED wrapper along with RVIZ using the following command:

```bash
$ ros2 launch zed_rviz display_zed.launch.py
```
If you are using a ZED Mini camera:

```bash
$ roslaunch zed_rviz display_zedm.launch.py
```

### Displaying Images
The ZED node publishes both original and stereo rectified (aligned) left and right images. In RVIZ, select a topic and use the `image` preview mode. 

Here is the list of the available image topics:

  - **zed/zed_node/rgb/image_rect_color**: Color rectified image (left image by default)
  - **zed/zed_node/rgb/image_raw_color**: Color unrectified image (left image by default)
  - **zed/zed_node/right/image_rect_color**: Color rectified right image
  - **zed/zed_node/right/image_raw_color**: Color unrectified right image
  - **zed/zed_node/confidence/confidence_image**: Confidence map as image

**Note:** The Confidence Map is also available as a 32bit floating point image subscribing to the **/zed/zed_node/confidence/confidence_map** topic.

![](https://cdn.stereolabs.com/docs/integrations/ros/images/rgb.jpg)

### Displaying Depth
The depth map can be displayed in RVIZ with the following topic:

  - **zed/zed_node/depth/depth_registered**: 32-bit depth values in meters. RVIZ will normalize the depth map on 8-bit and display it as a grayscale depth image.

**Note:** An OpenNI compatibility mode is available in the `config/common.yaml` file. Set `depth.openni_depth_mode` to `1` to get depth in millimeters with 16-bit precision, then restart the ZED node.

![](https://cdn.stereolabs.com/docs/integrations/ros/images/depth.jpg)

### Displaying the Point cloud
A 3D colored point cloud can be displayed in RVIZ2 with the **zed/zed_node/point_cloud/cloud_registered** topic. 

Add it in RVIZ2 with `point_cloud` -> `cloud` -> `PointCloud2`. Note that displaying point clouds slows down RVIZ2, so open a new instance if you want to display other topics.

![](https://cdn.stereolabs.com/docs/integrations/ros/images/point_cloud.jpg)

## Tutorials

A few tutorials are provided to understand how to use the ZED node in the ROS environment :

- Lifecycle Node : See [stereolabs_zed_tutorial_lifecycle](./tutorials/zed_lifecycle_tutorial)
- Video subscribing : See [stereolabs_zed_tutorial_video](./tutorials/zed_video_tutorial)
- Depth subscribing : See [stereolabs_zed_tutorial_depth](./tutorials/zed_depth_tutorial)

