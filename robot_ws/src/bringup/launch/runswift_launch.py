from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    runswift_bringup_pkg_share = FindPackageShare('bringup').find('bringup')


    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(runswift_bringup_pkg_share, 'launch', 'motion_bringup.py'))
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(runswift_bringup_pkg_share, 'launch', 'vision_bringup.py'))
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(runswift_bringup_pkg_share, 'launch', 'stateestimation_bringup.py'))
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(runswift_bringup_pkg_share, 'launch', 'comms_bringup.py'))
        ),
        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(os.path.join(runswift_bringup_pkg_share, 'launch', 'behaviours_bringup.py'))
        # ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(runswift_bringup_pkg_share, 'launch', 'hri_bringup.py'))
        )
    ])

if __name__ == '__main__':
    import sys
    from launch import LaunchService

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())

    sys.exit(ls.run())
