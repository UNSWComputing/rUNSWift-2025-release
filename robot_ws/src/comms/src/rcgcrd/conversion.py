from construct import Container

from rcgcrd.robocup_game_control_return_data import RoboCupGameControlReturnData
from runswift_interfaces.msg import CommsRCGCRD


def rcgcrd_msg_to_data(msg: CommsRCGCRD) -> bytes:
    """Convert RCGCRD ROS msg to binary data."""
    msg.pose[0] *= 1000 # Convert to mm
    msg.pose[1] *= 1000 # Convert to mm

    container = Container(
        playerNum=msg.player_num,
        teamNum=msg.team_num,
        fallen=msg.fallen,
        pose=msg.pose,
        ballAge=msg.ball_age,
        ball=msg.ball,
    )
    data = RoboCupGameControlReturnData.build(container)
    return data

