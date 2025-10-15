import math
from src.utils.Ball import getAbsoluteBall

def se_head_track_ball(blackboard):
    abs_ball = getAbsoluteBall(blackboard)
    if abs_ball is None:
        return None

    rw = blackboard.robot_world
    dx = abs_ball.pos_x - rw.point.x
    dy = abs_ball.pos_y - rw.point.y

    # rotate into robot-local frame (x = forward, y = left)
    cos_h = math.cos(rw.heading)
    sin_h = math.sin(rw.heading)
    rel_x = dx * cos_h + dy * sin_h
    rel_y = -dx * sin_h + dy * cos_h

    # desired head yaw: angle of (rel_x, rel_y) from forward
    distance = math.hypot(rel_x, rel_y)
    head_yaw = math.atan2(rel_y, rel_x)
    head_pitch = 0.2 if distance > 3000 else 0.4

    blackboard.motion_command.head_command.is_relative = False
    blackboard.motion_command.head_command.yaw = head_yaw
    blackboard.motion_command.head_command.yaw_speed = 0.2
    blackboard.motion_command.head_command.pitch = head_pitch
    blackboard.motion_command.head_command.pitch_speed = 0.1

