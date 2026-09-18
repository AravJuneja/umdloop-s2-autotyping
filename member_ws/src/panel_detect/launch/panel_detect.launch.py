"""Start panel_detect and the robot_state node it reads its frames from.

The detector subscribes to robot_state's ~/updates/image, so the two are
only useful together; starting them from one file is what keeps that pairing
from being something everybody has to remember.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description for the detector and its frame source."""
    robot_state = PythonLaunchDescriptionSource([
        FindPackageShare('robot_state'), '/launch/robot_state.launch.py'])
    return LaunchDescription([
        IncludeLaunchDescription(robot_state),
        Node(package='panel_detect', executable='panel_detect',
             name='panel_detect', output='screen'),
    ])
