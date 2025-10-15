def head_look_straight(blackboard, pitch=0.0):
    blackboard.motion_command.head_command.is_relative = False
    blackboard.motion_command.head_command.yaw = 0.0
    blackboard.motion_command.head_command.pitch = pitch

