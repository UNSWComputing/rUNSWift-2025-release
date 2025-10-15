from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    runswift_stateestimation_pkg_share = FindPackageShare('stateestimation').find('stateestimation')
    stateestimation_launch_file = os.path.join(runswift_stateestimation_pkg_share, 'launch', 'stateestimation.launch.py')

    stateestimation_with_cpu_pinning = ExecuteProcess(
        cmd=['taskset', '-c', '3', 'ros2', 'launch', stateestimation_launch_file],
        output='screen'
    )

    return LaunchDescription([

        stateestimation_with_cpu_pinning,
    ])

if __name__ == '__main__':
    import sys
    from launch import LaunchService

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())

    sys.exit(ls.run())