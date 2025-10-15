from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stateestimation',
            executable='SelfEstimator',
            name='self',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'warn'],
        ),
        Node(
            package='stateestimation',
            executable='BallEstimator',
            name='ball',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
        Node(
            package='vision',
            executable='Unwrap',
            name='Unwrap',
            output='screen',
        ),
        Node(
            package='vision',
            executable='CircleOnlyDetector',
            name='CircleOnlyDetector',
            output='screen',
        )
    ])
