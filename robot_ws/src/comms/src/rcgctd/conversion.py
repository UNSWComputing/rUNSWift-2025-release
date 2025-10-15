from rcgctd.robocup_game_control_team_data import RoboCupGameControlTeamData
from runswift_interfaces.msg import CommsRCGCRRD
from builtin_interfaces.msg import Time

def rcgctd_data_to_msg(data: bytes) -> CommsRCGCRRD:
    """Convert binary data to RCGCRRD ROS msg."""
    
    parsed = RoboCupGameControlTeamData.parse(data)
    msg = CommsRCGCRRD()
    
    # Timestamp
    stamp = Time()
    stamp.sec = parsed.behaviours_info.sec
    stamp.nanosec = 0
    msg.behaviours_info.timestamp = stamp
    
    # Populate behaviours_info from parsed behaviours_info
    msg.behaviours_info.player_number = parsed.behaviours_info.player_number
    msg.behaviours_info.robot_pos_x = parsed.behaviours_info.robot_pos_x
    msg.behaviours_info.robot_pos_y = parsed.behaviours_info.robot_pos_y
    msg.behaviours_info.heading = parsed.behaviours_info.heading
    msg.behaviours_info.ball_pos_x = parsed.behaviours_info.ball_pos_x
    msg.behaviours_info.ball_pos_y = parsed.behaviours_info.ball_pos_y
    msg.behaviours_info.confidence = parsed.behaviours_info.confidence
    msg.behaviours_info.kick_success = parsed.behaviours_info.kick_success
    msg.behaviours_info.player_role = parsed.behaviours_info.player_role
    msg.behaviours_info.my_ball = parsed.behaviours_info.my_ball
    msg.behaviours_info.kicking_team = parsed.behaviours_info.kicking_team
    msg.behaviours_info.ref_detected = parsed.behaviours_info.ref_detected

    return msg

