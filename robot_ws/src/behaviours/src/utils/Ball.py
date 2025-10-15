import math
from src.utils.Timer import Timer
from src.datamodels.RobotPose import RobotPose
from geometry_msgs.msg import Point
from src.utils.FieldCheck import InBoundsOutfieldTolerance
from dataclasses import dataclass

_ballLostTime = None

@dataclass
class BallMeasurement:
    timestamp: float
    x: float
    y: float

    @staticmethod
    def from_balls_absolute_msg(msg) -> "BallMeasurement | None":
        ball = getBall(msg.balls_absolute)

        if ball is None:
            return None

        return BallMeasurement(
            x=ball.pos_x,
            y=ball.pos_y,
            timestamp=float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9  # Corrected scaling
        )
        
def updateBallLostTime():
    """
    Initializes the _ballLostTime timer if not already initialized.
    """
    global _ballLostTime
    if _ballLostTime is None:
        _ballLostTime = Timer()

def getBall(ball_list):
    """
    Finds the ball with the highest confidence in the given array.

    Args:
        ball_list (list): Array of balls.

    Returns:
        SeBall: The SeBall object with the highest confidence score, or None if the array is empty.
    """
    if not ball_list:
        return None

    best_ball = None
    max_confidence = float('-inf')
    min_confidence = 5

    for ball in ball_list:
        if ball.confidence > max_confidence and ball.confidence > min_confidence and InBoundsOutfieldTolerance(RobotPose(point = Point(x = ball.pos_x, y = ball.pos_y), heading = 0.0)):
            max_confidence = ball.confidence
            best_ball = ball

    return best_ball

def getRelativeBall(blackboard):
    """
    Finds the ball with the highest confidence in the array of relative ball values.

    Returns:
        SeBall: The SeBall object with the highest confidence score, or None if the array is empty.
    """
    if blackboard.ball_base is None:
        return None
    return getBall(blackboard.ball_base.balls_relative)


def getAbsoluteBall(blackboard):
    """
    Finds the ball with the highest confidence in the given array of absolute ball values.

    Returns:
        SeBall: The SeBall object with the highest confidence score, or None if the array is empty.
    """
    if blackboard.ball_world is None:
        return None
    return getBall(blackboard.ball_world.balls_absolute)

def getVisionBall(blackboard):
    """
    Finds the ball with the highest confidence in the given array of vision ball values.

    Returns:
        Vision Ball Feature: The Vision Ball Feature object with the highest confidence score, or None if the array is empty.
    """
    if blackboard.vision_ball is None:
        return None
    ball_list = blackboard.vision_ball.ball_features
    if not ball_list:
        return None

    best_ball = None
    max_confidence = float('-inf')

    for ball in ball_list:
        if ball.confidence_score > max_confidence:
            max_confidence = ball.confidence_score
            best_ball = ball

    return best_ball


def heading(ball):
    """
    Calculates the heading of a ball relative to the world.

    Args:
        ball (SeBall): The ball object.

    Returns:
        float: The heading of the ball in radians.
    """
    if ball is None:
        raise ValueError("Ball object cannot be None")

    return math.atan2(ball.vel_y, ball.vel_x)

def distance(ball):
    """
    Calculates the Euclidean distance between the base and the ball.

    Args:
        ball (SeBall): The ball object.

    Returns:
        float: The distance between the base and the ball.
    """
    if ball is None:
        raise ValueError("Ball object cannot be None")

    return math.sqrt(ball.pos_x**2 + ball.pos_y**2)

def distanceTo(ball, robotPos):
    if ball is None:
        raise ValueError("Ball object cannot be None")

    return math.sqrt((ball.pos_x - robotPos.point.x)**2 + (ball.pos_y - robotPos.point.y)**2)

def ballLostTime(ball):
    """
    Calculates the time elapsed since the ball was last seen.

    Args:
        ball (SeBall): The ball object.

    Returns:
        float: The time in seconds since the ball was last seen.
    """
    updateBallLostTime()  # Ensure the timer is initialized
    if canSeeBall(ball):
        _ballLostTime.restart()
    return _ballLostTime.elapsedSeconds()

def canSeeBall(ball):
    """
    Determines if the ball is visible based on its confidence score.

    Args:
        ball (SeBall): The ball object.

    Returns:
        bool: True if the ball is visible, False otherwise.
    """
    if ball and ball.confidence > 50:
        return True
    return False
