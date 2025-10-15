from configparser import ConfigParser
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def is_nao_robot():
    return os.path.exists("/etc/naoqi") or os.path.exists("/home/nao")


def get_robot_name():
    """Reads the robot's hostname from /etc/hostname.

    Returns:
        str: The robot's hostname.

    NOTE: Defaults to 'robotnotfound' if not on a NAO robot.
    """
    try:
        with open("/etc/hostname", "r") as f:
            robot_name = f.read().strip()
    except FileNotFoundError:
        raise FileNotFoundError("The /etc/hostname file is missing. Unable to determine robot name.")
    except Exception as e:
        raise RuntimeError(f"Error reading /etc/hostname: {e}")

    if not is_nao_robot():
        robot_name = "robotnotfound"

    return robot_name


def get_robot_body():
    """Reads the robot's body from /robots/robots.cfg.

    Returns:
        str: The robot's body.

    NOTE: Defaults to 'robotnotfound' if not on a NAO robot.
    """

    def get_body_id():
        try:
            with open("/var/volatile/log/firmware/hardware-id", "r") as f:
                robot_body_id = f.readlines()[-1].strip()
        except FileNotFoundError:
            raise FileNotFoundError(
                "The /var/volatile/log/firmware/hardware-id file is missing. Unable to determine robot body ID."
            )
        except Exception as e:
            raise RuntimeError(f"Error reading /var/volatile/log/firmware/hardware-id: {e}")

        return robot_body_id

    if not is_nao_robot():
        return "robotnotfound"

    robot_body_id = get_body_id()
    cfg_path = f"/home/nao/data/robots.cfg"
    if os.path.exists(cfg_path):
        with open(cfg_path, "r") as file:
            for line in file:
                if f"body_id={robot_body_id}" in line:
                    # Extract the name between 'name =' and the first semicolon
                    name_part = line.split(";")[0]
                    name = name_part.split("=")[1].strip()
                    return name
    else:
        print(f"Path not found for robots.cfg")
        return "robotnotfound"


def get_config_path(path, custom_dev_path=None):
    """gets config path based on /home/nao/data for robot or in workspace

    Args:
        path (string): the path to the config file from /data/

    Returns:
        string: the full config path or None if not found
    """
    robot_path = f"/home/nao/data/{path}"
    if custom_dev_path is not None:
        dev_path = f"{custom_dev_path}/{path}"
    else:
        dev_path = f"image/home/nao/data/{path}"
    if os.path.exists(robot_path):
        return robot_path
    elif os.path.exists(dev_path):
        return dev_path
    else:
        print(f"/home/nao/data/{path} not found")
        return None


def parse_config(config_path):
    if config_path is None:
        return None
    config = ConfigParser()
    config.read(config_path)

    return config


def generate_launch_description():
    robot_name = get_robot_name()
    robot_body = get_robot_body()
    os.system(f'espeak -a 100 -vf5 -p75 -g20 "I am {robot_name} on {robot_body}"')
    individual_config = parse_config(get_config_path(f"configs/{robot_name}.cfg"))
    individual_body_config = parse_config(get_config_path(f"configs/body/{robot_body}.cfg"))
    runswift_config = parse_config(get_config_path("runswift.cfg"))
    param_list = {}

    # kinematics parameters : read from robot (head) config except for body pitch
    for axis in ["Roll", "Pitch", "Yaw"]:
        for direction in ["Left", "Straight", "Right"]:
            value = individual_config.getfloat(
                "kinematics",
                f"camera{axis}TopWhenLooking{direction}",
                fallback=runswift_config.getfloat(
                    "kinematics",
                    f"camera{axis}TopWhenLooking{direction}",
                    fallback=None
                )
            )

            if value is not None:
                param_list[f"camera_{axis.lower()}_top_when_looking_{direction.lower()}"] = value

    camera_pitch_bottom = individual_config.getfloat(
        "kinematics",
        "cameraPitchBottom",
        fallback=runswift_config.getfloat("kinematics", "cameraPitchBottom", fallback=None),
    )
    if camera_pitch_bottom is not None:
        param_list["camera_pitch_bottom"] = camera_pitch_bottom

    camera_yaw_bottom = individual_config.getfloat(
        "kinematics",
        "cameraYawBottom",
        fallback=runswift_config.getfloat("kinematics", "cameraYawBottom", fallback=None),
    )
    if camera_yaw_bottom is not None:
        param_list["camera_yaw_bottom"] = camera_yaw_bottom

    camera_roll_bottom = individual_config.getfloat(
        "kinematics",
        "cameraRollBottom",
        fallback=runswift_config.getfloat("kinematics", "cameraRollBottom", fallback=None),
    )

    if camera_roll_bottom is not None:
        param_list["camera_roll_bottom"] = camera_roll_bottom

    # body pitch : read from body config
    body_pitch = individual_body_config.getfloat(
        "kinematics", "bodyPitch", fallback=runswift_config.getfloat("kinematics", "bodyPitch", fallback=None)
    )

    if body_pitch is not None:
        param_list["body_pitch"] = body_pitch

    # motion parameters : read from body config -> robot config -> runswift config
    getup_speed = individual_body_config.get(
        "motion", "getupSpeed", fallback=runswift_config.get("motion", "getupSpeed", fallback=None)
    )
    if getup_speed is not None:
        param_list["getup_speed"] = getup_speed

    # kick parameters : read from body config -> robot config -> runswift config
    kick_lean_offset_l = individual_body_config.getfloat("kick", "leanOffsetL", fallback=None)
    if kick_lean_offset_l is not None:
        param_list["kick_lean_offset_l"] = kick_lean_offset_l
    kick_lean_offset_r = individual_body_config.getfloat("kick", "leanOffsetR", fallback=None)
    if kick_lean_offset_r is not None:
        param_list["kick_lean_offset_r"] = kick_lean_offset_r
    kick_lean = individual_body_config.getfloat(
        "kick",
        "kickLean",
        fallback=individual_config.getfloat(
            "kick", "kickLean", fallback=runswift_config.getfloat("kick", "kickLean", fallback=None)
        ),
    )
    if kick_lean is not None:
        param_list["kick_lean"] = kick_lean

    walk_speed_cap = individual_body_config.getfloat(
        "motion", "walk_speed_cap", fallback=runswift_config.getfloat("motion", "walk_speed_cap", fallback=None)
    )
    if walk_speed_cap is not None:
        param_list["walk_speed_cap"] = walk_speed_cap
    # Load the motion launch file

    kick_calibration_arg = DeclareLaunchArgument(
        "kick_calibration", default_value="False", description="Whether to launch the kick calibration node"
    )
    kick_calibration = Node(
        package="motion_port",
        executable="kick_calibration",
        name="kick_calibration",
        output="screen",
        parameters=[
            {
                "individual_body_config": get_config_path(f"configs/body/{robot_body}.cfg"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("kick_calibration")),  # Only launch if kick_calibration is True
    )

    nao_lola_client = Node(
        package="nao_lola_client",
        executable="nao_lola_client",
        name="nao_lola_client",
        output="screen",
    )

    madgwick_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        output="screen",
        parameters=[
            {"use_mag": False},
            {"publish_tf": False},
        ],
        remappings=[
            ("/imu/data_raw", "/imu"),
            ("/imu/data", "/imu_madgwick"),
        ],
    )

    motion_launch = Node(
        package="motion_port",
        executable="motion_adapter",  # component not working yet
        name="motion_node",
        output="screen",
        parameters=[
            param_list,
        ],
    )

    return LaunchDescription(
        [
            kick_calibration_arg,
            nao_lola_client,
            madgwick_filter,
            # If there are other launch files that motion depends on, do them first
            motion_launch,
            kick_calibration,
        ]
    )
