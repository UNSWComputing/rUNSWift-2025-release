from src.datamodels.RobotPose import RobotPose, SeBallToPoint
from math import radians, cos, sin
from src.utils.Ball import getAbsoluteBall, distanceTo
from src.utils.Constants import (
    FIELD_LENGTH,
    GOAL_POST_DIAMETER,
    GOAL_POST_ABS_X,
    GOAL_POST_ABS_Y,
    PENALTY_CROSS_ABS_X,
    GOAL_BOX_LENGTH,
    GOAL_BOX_WIDTH,
    PENALTY_AREA_LENGTH,
    PENALTY_AREA_WIDTH,
)
from src.utils.MathUtil import clamp, normalisedTheta
from geometry_msgs.msg import Point

blackboard = None

# Variables that can be accessed from other classes
_ball_near_our_goal = False
_ball_in_front_of_enemy_goal = False

######## NEEDS TO BE CHANGED FOR RobotPose instead of Vector2D #################

# Enemy goal vectors.
ENEMY_GOAL_CENTER = RobotPose(Point(x=FIELD_LENGTH / 2.0, y=0.0))
# +100 offset so angles aren't too sharp near goals
ENEMY_GOAL_BEHIND_CENTER = RobotPose(Point(x=FIELD_LENGTH / 2.0 + 100, y=0.0))
ENEMY_GOAL_INNER_LEFT = RobotPose(Point(x=FIELD_LENGTH / 2.0, y=GOAL_POST_ABS_Y - (GOAL_POST_DIAMETER / 2)))
ENEMY_GOAL_INNER_RIGHT = RobotPose(Point(x=FIELD_LENGTH / 2.0, y=-GOAL_POST_ABS_Y + (GOAL_POST_DIAMETER / 2)))
ENEMY_GOAL_OUTER_LEFT = RobotPose(Point(x=FIELD_LENGTH / 2.0, y=GOAL_POST_ABS_Y + (GOAL_POST_DIAMETER / 2)))
ENEMY_GOAL_OUTER_RIGHT = RobotPose(Point(x=FIELD_LENGTH / 2.0, y=-GOAL_POST_ABS_Y - (GOAL_POST_DIAMETER / 2)))
ENEMY_PENALTY_CENTER = RobotPose(Point(x=PENALTY_CROSS_ABS_X))
ENEMY_LEFT_POST = RobotPose(Point(x=GOAL_POST_ABS_X, y=GOAL_POST_ABS_Y))
ENEMY_RIGHT_POST = RobotPose(Point(x=GOAL_POST_ABS_X, y=-GOAL_POST_ABS_Y))

# Own goal vectors.
OWN_GOAL_CENTER = RobotPose(Point(x=-FIELD_LENGTH / 2.0))
# +100 offset so angles aren't too sharp near goals
OWN_GOAL_BEHIND_CENTER = RobotPose(Point(x=-FIELD_LENGTH / 2.0 - 100))

OUR_GOAL_CENTRE = RobotPose(Point(x=-FIELD_LENGTH / 2.0))
OUR_GOAL_BEHIND_CENTRE = RobotPose(Point(x=-FIELD_LENGTH / 2.0 - 100))
OUR_LEFT_POST = RobotPose(Point(x=-GOAL_POST_ABS_X, y=GOAL_POST_ABS_Y))
OUR_RIGHT_POST = RobotPose(Point(x=-GOAL_POST_ABS_X, y=-GOAL_POST_ABS_Y))

def update_field_geometry(newBlackboard):
    """
    Updates the FieldGeometry.py global variables, i.e. the blackboard.

    Callable via `FieldGeometry.update_field_geometry(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = newBlackboard
    update_ball_near_our_goal()
    update_ball_in_front_of_enemy_goal()


def calculateTimeToReachBall(robot_pos, robot_heading):
    opponentGoal = ENEMY_GOAL_CENTER
    myPose = RobotPose(robot_pos, robot_heading)
    ballPos = SeBallToPoint(getAbsoluteBall())
    interceptToGoalHeading = normalisedTheta(opponentGoal.angleTo(ballPos))
    return calculateTimeToReachPose(myPose, ballPos, interceptToGoalHeading)


TURN_RATE = radians(60.0)  # radians/second
WALK_RATE = 300.0  # mm/second
CIRCLE_STRAFE_RATE = radians(40.0)  # radians/second


#def calculateTimeToReachPose(myPos, myHeading, targetPos, targetHeading=None):
def calculateTimeToReachPose(myPose, ballPos, targetHeading):
    # potential error in order of subtraction
    toTargetHeading = normalisedTheta(myPose.angleTo(ballPos))

    # How far we need to turn to point at the targetPos
    toTargetTurn = abs(normalisedTheta(toTargetHeading - myPose.heading))

    # The straightline distance to walk to the targetPos
    toTargetDistance = myPose.subtract(ballPos).length()
    
    # How far we need to turn once we get to the targetPos so that we are
    # facing the targetHeading
    if targetHeading is None:
        toTargetHeadingTurn = 0.0
    else:
        toTargetHeadingTurn = abs(normalisedTheta(toTargetHeading - targetHeading))

    return toTargetTurn / TURN_RATE + toTargetDistance / WALK_RATE + toTargetHeadingTurn / CIRCLE_STRAFE_RATE


# Point is in format RobotPose, absCoord in format Point
def angleToPoint(point, absCoord):
    return point.absThetaTo(absCoord)



# Whether something is inside our goalbox
def isInOurGoalBox(pos, buffx=0, buffy=0):
    return pos.x < -FIELD_LENGTH / 2 + GOAL_BOX_LENGTH + buffx and abs(pos.y) < GOAL_BOX_WIDTH / 2 + buffy


# Whether something is in the opponent goalbox
def isInOpponentGoalBox(pos, buffx=0, buffy=0):
    return pos.x > FIELD_LENGTH / 2 - GOAL_BOX_LENGTH - buffx and abs(pos.y) < GOAL_BOX_WIDTH / 2 + buffy


# Whether something is inside our goalbox
def isInOurPenaltyBox(pos, buffx=0, buffy=0):
    return pos.x < -FIELD_LENGTH / 2 + PENALTY_AREA_LENGTH + buffx and abs(pos.y) < PENALTY_AREA_WIDTH / 2 + buffy


# Whether something is in the opponent goalbox
def isInOpponentPenaltyBox(pos, buffx=0, buffy=0):
    return pos.x > FIELD_LENGTH / 2 - PENALTY_AREA_LENGTH - buffx and abs(pos.y) < PENALTY_AREA_WIDTH / 2 + buffy


def addRrToRobot(robotPos, rx, ry):
    x = robotPos.point.x + cos(robotPos.heading) * rx - sin(robotPos.heading) * ry
    y = robotPos.point.y + sin(robotPos.heading) * rx + cos(robotPos.heading) * ry
    return x, y
# heading = theta

# used once in headlookatglobalpoint 
def globalPointToRobotRelativePoint(globalVector,myPose):
    # myPose is from robotpose
    robotPos = myPose.point
    robotHeading = myPose.heading
    
    return globalVector.subtract(robotPos).rotate(-robotHeading)


# Closes y-value from ball to our goal line between posts
def closest_goal_y():
    return clamp(getAbsoluteBall().pos_y, -GOAL_POST_ABS_Y, GOAL_POST_ABS_Y)


# Closest point from the ball to our goal line between posts
def closest_our_goal_point():
    return RobotPose(Point(-GOAL_POST_ABS_X, closest_goal_y(), 0))


# Closest point from the ball to our goal line between posts
def closest_opponent_goal_point():
    return RobotPose(Point(GOAL_POST_ABS_X, closest_goal_y(), 0))


# Update whether ball is near our goal, with a noise margin
def update_ball_near_our_goal():
    dist_ball_to_our_goal = distanceTo(getAbsoluteBall(), closest_goal_y())

    global _ball_near_our_goal
    if _ball_near_our_goal:
        if dist_ball_to_our_goal > 1500:
            _ball_near_our_goal = False
    else:
        if dist_ball_to_our_goal < 1100:
            _ball_near_our_goal = True


# Update whether ball is in front of opponent goal, with a noise margin
def update_ball_in_front_of_enemy_goal():
    global _ball_in_front_of_enemy_goal
    if _ball_in_front_of_enemy_goal:
        if getAbsoluteBall().pos_x < FIELD_LENGTH / 2 - GOAL_BOX_LENGTH - 500 or abs(getAbsoluteBall().pos_y) > GOAL_POST_ABS_Y + 100:
            _ball_in_front_of_enemy_goal = False
    else:
        if (
            getAbsoluteBall().pos_x > FIELD_LENGTH / 2 - GOAL_BOX_LENGTH - 250
            and abs(getAbsoluteBall().pos_y) < GOAL_POST_ABS_Y - 100
        ):
            _ball_in_front_of_enemy_goal = True


def ball_near_our_goal():
    return _ball_near_our_goal


def ball_in_front_of_enemy_goal():
    return _ball_in_front_of_enemy_goal