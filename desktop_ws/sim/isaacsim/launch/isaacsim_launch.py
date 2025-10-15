from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    isaacsim_bridge = Node(
        package="isaacsim",
        executable="isaacsim_bridge",
        name="isaacsim_bridge",
        output="screen",
    )

    motion_launch = Node(
        package="motion_port",
        executable="motion_adapter",
        name="motion_node",
        output="screen",
        parameters=[
            {
                "simulation": True,
            },
        ],
    )

    foxglove_arg = DeclareLaunchArgument(
        "foxglove", default_value="True", description="Whether to launch foxglove bridge"
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove",
        output="screen",
        condition=IfCondition(LaunchConfiguration("foxglove")),
    )

    return LaunchDescription(
        [
            foxglove_arg,
            foxglove,
            isaacsim_bridge,
            motion_launch,
        ]
    )
