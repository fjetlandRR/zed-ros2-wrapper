# ZED Multi Camera

This is an example that illustrates how to configure and run a multi camera rig composed by a ZED-M camera lying on a ZED.

## Configuration
The folder `config` contains the modified YAML files used to configure the ROS2 nodes.

## Launch
The folder `launch` contains the Python script that configure two ROS2 wrapper nodes and launch them.

Command:
`$ ros2 launch stereolabs_example_multi_camera zed_multi_camera.launch.py`
