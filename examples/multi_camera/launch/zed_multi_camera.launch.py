from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    """Launch the example.launch.py launch file."""
    return LaunchDescription([
        LogInfo(msg=[
            'Including launch file located at: ', ThisLaunchFileDir(), '/example.launch.py'
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/example.launch.py']),
            launch_arguments={'node_prefix': 'FOO'}.items(),
        ),
    ])
