"""Launch the ZED ROS2 node"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events

from launch_ros import get_default_launch_description
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def main(argv=sys.argv[1:]):
    """Main."""
    ld = launch.LaunchDescription()

    # Prepare the talker node.
    zed_node = launch_ros.actions.LifecycleNode(
        node_name='zed_node',
        package='stereolabs_zed', node_executable='zed_wrapper_node', output='screen')

    # When the talker reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_talker_reaches_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=zed_node, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'zed_node' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.process.matches_action(zed_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # When the zed node reaches the 'active' state, log a message
    register_event_handler_for_talker_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=zed_node, goal_state='active',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'talker' reached the 'active' state, launching 'listener'."),
            ],
        )
    )

    # Make the talker node take the 'configure' transition.
    emit_event_to_request_that_talker_does_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.process.matches_action(zed_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action(register_event_handler_for_talker_reaches_inactive_state)
    ld.add_action(register_event_handler_for_talker_reaches_active_state)
    ld.add_action(zed_node)
    ld.add_action(emit_event_to_request_that_talker_does_configure_transition)

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
