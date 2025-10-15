from construct import Struct, Int8ul, Int32ul, Float32l, Flag

# Define the BehavioursRobotInfo struct
BehavioursRobotInfo = Struct(
    "sec" / Int32ul,             # ROS timestamp seconds
    "player_number" / Int32ul,   # int32
    "robot_pos_x" / Float32l,    # float32
    "robot_pos_y" / Float32l,    # float32
    "heading" / Float32l,        # float32
    "ball_pos_x" / Float32l,     # float32
    "ball_pos_y" / Float32l,     # float32
    "confidence" / Float32l,     # float32
    "kick_success" / Flag,       # bool
    "player_role" / Int8ul,      # int8
    "my_ball" / Flag,             # bool
    "kicking_team" / Flag,       # bool 
    "ref_detected" / Int8ul,        # int8
)

# Define the main struct
RoboCupGameControlRobotReturnData = Struct(
    # 31 bytes
    "behaviours_info" / BehavioursRobotInfo  # Nested struct
)
