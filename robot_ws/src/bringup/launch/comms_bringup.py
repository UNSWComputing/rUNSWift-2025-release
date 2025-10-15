from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    runswift_comms_pkg_share = FindPackageShare('comms').find('comms')
    comms_launch_file = os.path.join(runswift_comms_pkg_share, 'launch', 'comms.launch.py')


    comms_with_cpu_pinning = ExecuteProcess(
        cmd=['taskset', '-c', '1', 'ros2', 'launch', comms_launch_file],
        output='screen'
    )

    return LaunchDescription([
        comms_with_cpu_pinning,
    ])

if __name__ == '__main__':
    import sys
    from launch import LaunchService

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())

    sys.exit(ls.run())
