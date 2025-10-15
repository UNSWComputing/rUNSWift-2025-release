from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='comms',
            executable='game_controller_spl.py',
            name='game_controller_spl',
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
        Node(
            package='comms',
            executable='player_number.py',
            name='player_number',
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
        Node(
            package='comms',
            executable='game_controller_publisher.py',
            name='game_controller_publisher',
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
        Node(
            package='comms',
            executable='whistle_detector.py',
            name='whistle_detector',
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
        Node(
            package='comms',
            executable='robot_communication_publisher.py',
            name='robot_communication_publisher',
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
    ])
