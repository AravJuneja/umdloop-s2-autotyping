"""Start the robot_state node.

Thin on purpose: the node takes no parameters, and everything it reads is
fixed by the simulator contract in robot_state/config.py. This exists so the
node can be included from a larger launch file without hard-coding the
package and executable names at every call site.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for a single robot_state node."""
    return LaunchDescription([
        Node(package='robot_state', executable='robot_state',
             name='robot_state', output='screen'),
    ])
