import math 

# not sure if this will work or not
# from std_msgs.msg import ColorRGBA

# not sure (hard coded)
# values taken from the fieldParams ( 2024 release)
FIELD_WIDTH = 6020
HALF_FIELD_WIDTH = FIELD_WIDTH / 2.0
FIELD_LENGTH = 9010
HALF_FIELD_LENGTH = FIELD_LENGTH / 2.0
GOAL_BOX_WIDTH = 2220
GOAL_BOX_LENGTH = 615
CENTER_CIRCLE_DIAMETER = 1500
GOAL_WIDTH = 1565
BALL_RADIUS = 50
GOAL_POST_ABS_X = (FIELD_LENGTH / 2.0) - (50 / 2.0) + (90 / 2.0)
GOAL_POST_ABS_Y = (GOAL_WIDTH / 2)
GOAL_POST_DIAMETER = 40
ROBOTS_PER_TEAM = 7
PENALTY_CROSS_ABS_X = (FIELD_LENGTH / 2 - 1290)
PENALTY_CROSS_DISTANCE = 1290
PENALTY_AREA_LENGTH = 1650
PENALTY_AREA_WIDTH = 4000
GOAL_KICK_ABS_X = PENALTY_CROSS_ABS_X
GOAL_KICK_ABS_Y = (GOAL_BOX_WIDTH / 2)
CORNER_KICK_ABS_X = (FIELD_LENGTH / 2)
CORNER_KICK_ABS_Y =  (FIELD_WIDTH / 2)

# These contstants describe the area outside the field of play and should be at
# a minimum of 500 as per SPL rules
OUTFIELD_POS_X = 500
OUTFIELD_POS_Y = 500
OUTFIELD_NEG_X = 500
OUTFIELD_NEG_Y = 500

# Head joint limits
MIN_HEAD_YAW = math.radians(-119.5)
MAX_HEAD_YAW = math.radians(119.5)
MIN_HEAD_PITCH = math.radians(-38.5)
MAX_HEAD_PITCH = math.radians(29.5)

# Foot geometry, used for dribblin/approaching the ball
TOE_CENTRE_X = 105
HIP_OFFSET = 50


# LED Colours. ignored for now 
# class LEDColour(object):
#     off = robot.rgb(False, False, False)
#     red = robot.rgb(True, False, False)
#     green = robot.rgb(False, True, False)
#     blue = robot.rgb(False, False, True)
#     yellow = robot.rgb(True, True, False)
#     cyan = robot.rgb(False, True, True)
#     magenta = robot.rgb(True, False, True)
#     white = robot.rgb(True, True, True)
# class LEDColour:
#     def rgb(r, g, b):
#         """Returns a ColorRGBA message with specified RGB values."""
#         return ColorRGBA(r=r, g=g, b=b, a=1.0)  # Alpha is set to 1.0 (opaque)

#     off = rgb(0.0, 0.0, 0.0)
#     red = rgb(1.0, 0.0, 0.0)
#     green = rgb(0.0, 1.0, 0.0)
#     blue = rgb(0.0, 0.0, 1.0)
#     yellow = rgb(1.0, 1.0, 0.0)
#     cyan = rgb(0.0, 1.0, 1.0)
#     magenta = rgb(1.0, 0.0, 1.0)
#     white = rgb(1.0, 1.0, 1.0)

# Time you have to wait before you can enter the circle in a defensive kickoff.
KICKOFF_MIN_WAIT = 10 * 1000 * 1000