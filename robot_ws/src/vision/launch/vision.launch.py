from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import configparser
import yaml
import subprocess
import re
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

def get_camera_devices():
    result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True)
    output = result.stdout

    cameras = {'top': None, 'bottom': None}

    devices = output.split('\n\n')
    for device in devices:
        if not device.strip():
            continue

        # Look for firmware revision
        if 'FW rev:022' in device:
            # Extract video device number
            match = re.search(r'/dev/video(\d+)', device)
            if match:
                cameras['top'] = int(match.group(1))
        elif 'FW rev:010' in device:
            match = re.search(r'/dev/video(\d+)', device)
            if match:
                cameras['bottom'] = int(match.group(1))

    return cameras


def generate_launch_description():
    package_path = get_package_share_directory('vision')  # starts at install/vision/share/vision
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_path))))  # go up to robot_ws
    cfg_path = os.path.join(
        os.path.dirname(workspace_root),  # go up one more from robot_ws
        # 'image',
        # 'home',
        # 'nao',
        'data',
        'runswift.cfg'
    )
    # Read the config file
    config = configparser.ConfigParser()
    config.read(cfg_path)
    # read under the [camera] section
    camera_config = config['camera']
    top_config = {}
    top_config['video_device'] = '/dev/video-top'
    bot_config = {}
    bot_config['video_device'] = '/dev/video-bottom'

    for key, value in camera_config.items():
        cam, attri = key.split('.')
        if attri == 'framerate':
            value = float(value)
        elif value.isdigit():
            value = int(value)
        elif value.lower() == 'true':
            value = True
        elif value.lower() == 'false':
            value = False
        if cam == 'top':
            top_config[attri] = value
        elif cam == 'bot':
            bot_config[attri] = value

    top_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='top',
        namespace='camera',
        parameters=[
            top_config
        ],
        remappings=[
            ('image_raw', '/camera/top/raw_image'),
        ],
        output='screen'
    )
    bot_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace='camera',
        name='bot',
        parameters=[
            bot_config
        ],
        remappings=[
            ('image_raw', '/camera/bot/raw_image'),
        ],
        output='screen'
    )
    flip_camera_node = Node(
        package='vision',
        executable='CameraFlip',
        name='CameraFlip',
        output='screen',
    )
    config_setter_node = Node(
        package='vision',
        executable='camera_configs_setter.py',
        name='camera_configs_setter',
        output='screen'
    )

    vision_pipeline_switch_node = Node(
        package='vision',
        executable='vision_pipeline_switch_node.py',
        name='vision_pipeline_switch_node',
        output='screen',
    )
    
    return LaunchDescription([
         RegisterEventHandler(
            OnProcessExit(
                target_action=flip_camera_node,
                on_exit=[top_cam_node, bot_cam_node, vision_pipeline_switch_node]
            )
        ),
        flip_camera_node,
        Node(
            package='vision',
            executable='Preprocessing',
            name='Preprocessing',
            output='screen',
        ),
        Node(
            package='vision',
            executable='ball_detector_model.py',
            name='ball_detector_model',
            output='screen',
        ),
        Node(
            package='vision',
            executable='BallDetector',
            name='BallDetector',
            output='screen',
        ),
        Node(
            package='vision',
            executable='BotSegmentation',
            name='BotSegmentation',
            output='screen',
            emulate_tty=True,
            parameters=[
                # Set the parameter here. Change to True to enable debugging.
                {'enable_debug': False} 
            ]
        ),
        Node(
            package='vision',
            executable='eye_leds_control.py',
            name='EyeLedControl',
            output='screen'
        ),
    ])
