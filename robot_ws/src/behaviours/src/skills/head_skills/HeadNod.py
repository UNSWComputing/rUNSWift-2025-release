def head_nod(blackboard):
    blackboard.motion_command.head_command.is_relative = False
    MAX_HEAD_PITCH_TOP = 0.1
    MAX_HEAD_PITCH_BOTTOM = 0.5
    HEAD_PITCH_RATE = 0.01
    pitch = blackboard.motion_command.head_command.pitch
    if pitch < MAX_HEAD_PITCH_TOP:
        blackboard.head_pitch_increment = HEAD_PITCH_RATE
    elif pitch > MAX_HEAD_PITCH_BOTTOM:
        blackboard.head_pitch_increment = -HEAD_PITCH_RATE
    blackboard.motion_command.head_command.yaw = 0.0
    blackboard.motion_command.head_command.pitch += blackboard.head_pitch_increment
