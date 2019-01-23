![](../images/Picto+STEREOLABS_Black.jpg)
# ZED ROS2 COMPONENT and CONTAINER

## START THE NODE

To start the node in its own container executable:

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

### MANAGE NODE STATE USING SERVICE CALLS

Command line:
```
$ ros2 service call /zed_node/change_state lifecycle_msgs/ChangeState "{node_name: zed_node, transition: {id: X}}"
```

Replace `X` with one of the following ID
* ID: 1 -> CONFIGURE  - **UNCONFIGURED -> INACTIVE**
* ID: 2 -> CLEANUP    - **INACTIVE -> UNCONFIGURED**
* ID: 3 -> ACTIVATE   - **INACTIVE -> ACTIVE**
* ID: 4 -> DEACTIVATE - **ACTIVE -> INACTIVE**
* ID: 5 -> SHUTDOWN   - **everyState -> FINALIZED**

To receive a message for each valid state transition:
```
$ ros2 topic echo /zed_node/transition_event
```
