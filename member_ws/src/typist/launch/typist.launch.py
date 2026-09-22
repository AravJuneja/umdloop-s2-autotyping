"""Start the full typing stack: state, detection, projection, control, typist."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_state = PythonLaunchDescriptionSource(
        [FindPackageShare('robot_state'), '/launch/robot_state.launch.py']
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(robot_state),
            Node(
                package='panel_detect',
                executable='panel_detect',
                name='panel_detect',
                output='screen',
            ),
            Node(
                package='panel_detect',
                executable='key_projector',
                name='key_projector',
                output='screen',
            ),
            Node(
                package='arm_control',
                executable='arm_control',
                name='arm_control',
                output='screen',
            ),
            Node(package='typist', executable='typist', name='typist', output='screen'),
        ]
    )
