from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    motion_pkg_share = FindPackageShare('motion_port').find('motion_port')
    motion_launch_file = os.path.join(motion_pkg_share, 'launch', 'motion_launch.py')

    library_path = os.getenv('LD_LIBRARY_PATH')

    motion_with_cpu_pinning = ExecuteProcess(
        cmd=['chrt', '-f', '99', 'env', f'LD_LIBRARY_PATH={library_path}', 'ros2', 'launch', motion_launch_file],
        output='screen'
    )
    # return LaunchDescription([
    #     IncludeLaunchDescription(
    #         AnyLaunchDescriptionSource(motion_launch_file)
    #     )
    # ])

    return LaunchDescription([
        motion_with_cpu_pinning,
    ])


if __name__ == '__main__':
    import sys
    from launch import LaunchService

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())

    sys.exit(ls.run())
