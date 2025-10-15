import math
import time

def head_scan_ball_goalie(blackboard):
    global head_hit_left
    blackboard.motion_command.head_command.is_relative = False

    HEAD_TURN_RATE = 0.02
    MAX_HEAD_YAW = math.radians(60)
    HEAD_SCAN_PITCH = 0.3

    yaw = blackboard.motion_command.head_command.yaw
    if yaw > MAX_HEAD_YAW:
        blackboard.head_yaw_increment = -HEAD_TURN_RATE
    elif yaw < -MAX_HEAD_YAW:
        blackboard.head_yaw_increment = HEAD_TURN_RATE
    blackboard.motion_command.head_command.pitch = HEAD_SCAN_PITCH
    blackboard.motion_command.head_command.yaw += blackboard.head_yaw_increment
    blackboard.motion_command.head_command.yaw_speed = 0.05

    return


