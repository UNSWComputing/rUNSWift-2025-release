from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    runswift_vision_pkg_share = FindPackageShare('vision').find('vision')
    vision_launch_file = os.path.join(runswift_vision_pkg_share, 'launch', 'vision.launch.py')

    vision_with_cpu_pinning = ExecuteProcess(
        cmd=['taskset', '-c', '2', 'ros2', 'launch', vision_launch_file],
        output='screen'
    )

    return LaunchDescription([
        vision_with_cpu_pinning,
    ])

if __name__ == '__main__':
    import sys
    from launch import LaunchService

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())

    sys.exit(ls.run())
