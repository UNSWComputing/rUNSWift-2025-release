from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hri',
            executable='chest_leds_control_game.py',
            name='chest_leds_control_game',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
        )
    ])
