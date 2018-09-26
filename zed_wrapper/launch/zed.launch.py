# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a lifecycle talker and a lifecycle listener."""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events

from ament_index_python.packages import get_package_share_directory
from launch_ros import get_default_launch_description
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def main(argv=sys.argv[1:]):
    """Main."""    

    # use: 'zed' for "ZED" camera - 'zedm' for "ZED mini" camera
    camera_model = 'zedm' 

    # URDF file to be loaded by Robot State Publisher
    urdf = os.path.join(get_package_share_directory('stereolabs_zed'),
                        'urdf', camera_model + '.urdf')
    
    # ZED Configurations to be loaded by ZED Node
    config_common = os.path.join(get_package_share_directory('stereolabs_zed'),
                        'config', 'common.yaml')

    config_camera = os.path.join(get_package_share_directory('stereolabs_zed'),
                        'config', camera_model + '.yaml')

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'

    # Prepare the ZED node
    zed_node = launch_ros.actions.LifecycleNode( package='stereolabs_zed', 
                                                 node_executable='zed_wrapper_node',
                                                 output='screen', 
                                                 arguments=['__params:='+config_common, # Common parameters
                                                            '__params:='+config_camera  # Camera related parameters
                                                           ]
                                               )
    # Launch Description
    ld = launch.LaunchDescription()

    # Make the ZED node take the 'configure' transition.
    emit_event_to_request_that_zed_does_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.process.matches_action(zed_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # When the ZED node reaches the 'inactive' state, start the Robot State Publisher and make it take the 'activate' transition.
    register_event_handler_for_zed_reaches_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=zed_node, goal_state='inactive',
            entities=[
                       # Log
                       launch.actions.LogInfo(msg="'ZED' reached the 'inactive' state, start the 'Robot State Publisher' node and 'activating'."),
                       # Robot State Publisher
                       launch_ros.actions.Node( package='robot_state_publisher', 
                                                node_executable='robot_state_publisher',
                                                output='screen', 
                                                arguments=[urdf]
                                              ),
                       # Change State event
                       launch.actions.EmitEvent( event=launch_ros.events.lifecycle.ChangeState(
                         lifecycle_node_matcher=launch.events.process.matches_action(zed_node),
                         transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                        )),
             ],
        )
    )

    # When the ZED node reaches the 'active' state, log a message.
    register_event_handler_for_talker_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node, goal_state='active',
            entities=[ launch.actions.LogInfo( msg="'ZED' reached the 'active' state" ),],
        )
    )    

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action(register_event_handler_for_zed_reaches_inactive_state)
    ld.add_action(register_event_handler_for_zed_reaches_active_state)
    ld.add_action(zed_node)
    ld.add_action(emit_event_to_request_that_zed_does_configure_transition)

    print('Starting introspection of launch description...')
    print('')

    print(launch.LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    # ls = launch.LaunchService(argv=argv, debug=True)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(get_default_launch_description(prefix_output_with_name=False))
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
