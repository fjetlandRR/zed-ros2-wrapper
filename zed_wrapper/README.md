# ZED ROS2 COMPONENT and CONTAINER

To start the container executable:

``` 
$ ros2 run stereolabs_zed zed_wrapper_node
```

To start the ZED component in an external container:

Console 1:
```
$ ros2 run composition api_composition
```

Console 2:

```
$ ros2 run composition api_composition_cli stereolabs_zed stereolabs::ZedCameraComponent
```
