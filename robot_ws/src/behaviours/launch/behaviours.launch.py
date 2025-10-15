from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='behaviours',
            executable='node',
            name='behaviour_node',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'warn'],
        )
    ])
