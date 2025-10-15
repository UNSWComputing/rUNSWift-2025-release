from src.utils.Constants import (
    HALF_FIELD_WIDTH,
    HALF_FIELD_LENGTH,
    OUTFIELD_POS_X,
    OUTFIELD_POS_Y,
    OUTFIELD_NEG_X,
    OUTFIELD_NEG_Y,
)
from src.datamodels.RobotPose import RobotPose
from geometry_msgs.msg import Point


def InBounds(RobotPose):
    if abs(RobotPose.point.y) <= HALF_FIELD_WIDTH and abs(RobotPose.point.x) <= HALF_FIELD_LENGTH:
        return True
    else:
        return False


def InBoundsOutfieldTolerance(RobotPose):
    if RobotPose.point.y < (-HALF_FIELD_WIDTH - OUTFIELD_NEG_Y) or RobotPose.point.y > (
        HALF_FIELD_WIDTH + OUTFIELD_POS_Y
    ):
        return False
    elif RobotPose.point.x < (-HALF_FIELD_LENGTH - OUTFIELD_NEG_X) or RobotPose.point.x > (
        HALF_FIELD_LENGTH + OUTFIELD_POS_X
    ):
        return False
    else:
        return True
