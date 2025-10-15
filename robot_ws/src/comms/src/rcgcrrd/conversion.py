from construct import Container

from rcgcrrd.robocup_game_control_robot_return_data import RoboCupGameControlRobotReturnData
from runswift_interfaces.msg import CommsRCGCRRD

def rcgcrrd_msg_to_data(msg: CommsRCGCRRD) -> bytes:
    """Convert RCGCRRD ROS msg to binary data."""

    container = Container(
        behaviours_info=Container(
            sec=msg.behaviours_info.timestamp.sec,
            player_number=msg.behaviours_info.player_number,
            robot_pos_x=msg.behaviours_info.robot_pos_x,
            robot_pos_y=msg.behaviours_info.robot_pos_y,
            heading=msg.behaviours_info.heading,
            ball_pos_x=msg.behaviours_info.ball_pos_x,
            ball_pos_y=msg.behaviours_info.ball_pos_y,
            confidence=msg.behaviours_info.confidence,
            kick_success=msg.behaviours_info.kick_success,
            player_role=msg.behaviours_info.player_role,
            my_ball=msg.behaviours_info.my_ball,
            kicking_team=msg.behaviours_info.kicking_team,
            ref_detected=msg.behaviours_info.ref_detected
        )
    )

    data = RoboCupGameControlRobotReturnData.build(container)
    return data


