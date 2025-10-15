import subprocess
from src.skills.Skill import Skill
from src.skills.leaf_skills.Walk import Walk
from src.skills.leaf_skills.Stand import Stand
from src.skills.leaf_skills.Kick import Kick
from src.skills.main_skills.SoloFindBall import SoloFindBall
from src.datamodels.RobotPose import RobotPose, SeBallToRobotPose, VBallToRobotPose
from src.utils.Constants import HALF_FIELD_LENGTH
from math import radians, sqrt, degrees
from geometry_msgs.msg import Point
from src.utils.Ball import getAbsoluteBall, getVisionBall, heading
from src.utils.MathUtil import normalisedTheta
import math


class Lineup(Skill):
    """
    Description:
    Fine-tuned positioning skill that lines up the robot's left foot with the ball
    for precise kicking. Uses absolute ball coordinates for consistent positioning.
    Similar to ApproachBall but with finer margins for final adjustment.
    """


    DEFAULT_AIM = float(HALF_FIELD_LENGTH)  # mm

    IN_POSITION_DISTANCE_X = 80   # mm
    IN_POSITION_DISTANCE_Y = 80   # mm
    NOT_IN_POSITION_DISTANCE = 150  # mm

    # Movement speeds (slower for precision)
    WALK_SPEED = 180.0  # mm/s
    SIDE_STEP_SPEED = 60.0  # mm/s
    TURN_RATE = 0.5  # rad/s

    # Heading tolerances
    HEADING_ERROR_TO_ADJUST = radians(20)  # rad

    # Left foot positioning offset from robot center
    ROBOT_FOOT_X = 90.0  # mm - forward offset to left foot
    ROBOT_FOOT_Y = 50.0  # mm - left offset to left foot

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Walk": Walk(self),
            "Stand": Stand(self),
            "Kick": Kick(self),
            "SoloFindBall": SoloFindBall(self),
        }

    def _reset(self):
        self._current_sub_skill = "Stand"
        self._foot = None
        self._final_pose = None
        self._position_close = False
        self._heading_close = False

    def _check_finished(self):
        if self._current_sub_skill == "Kick":
            return True
        return False

    def _tick(self, aim_point=RobotPose(Point(x=DEFAULT_AIM, y=0.0)), foot="left", distance=None):

        my_pose = self.ctx.blackboard.robot_world

        robot_world_x = my_pose.point.x
        robot_world_y = my_pose.point.y
        robot_heading = my_pose.heading

        relative_ball = None
        if self.ctx.blackboard.vision_ball is not None:
            relative_ball = getVisionBall(self.ctx.blackboard)
            relative_ball = VBallToRobotPose(relative_ball.ball_coordinates)

        if relative_ball is None or relative_ball.length() > 500:
            # Get absolute ball position
            abs_ball = getAbsoluteBall(self.ctx.blackboard)

            if abs_ball is None:
                self._current_sub_skill = "SoloFindBall"
                self._tick_sub_skill()
                return
            # Calculate difference in world coordinates
            abs_ball_pose = SeBallToRobotPose(abs_ball)
            world_diff_x = abs_ball.pos_x - robot_world_x
            world_diff_y = abs_ball.pos_y - robot_world_y

            # Transform to robot's local coordinate frame
            # Robot's local frame: x = forward, y = left
            cos_heading = math.cos(robot_heading)
            sin_heading = math.sin(robot_heading)

            # Rotation matrix transformation
            relative_x = world_diff_x * cos_heading + world_diff_y * sin_heading  # forward
            relative_y = -world_diff_x * sin_heading + world_diff_y * cos_heading  # left
            relative_ball = RobotPose(Point(x=relative_x, y=relative_y))


        if foot == "left":
            self._foot = 1
        elif foot == "right":
            self._foot = -1

        self._final_pose = relative_ball
        self._final_pose.heading = abs_ball_pose.angleTo(aim_point.point)

        # Adjust for foot position
        self._final_pose.point.y -= self._foot * self.ROBOT_FOOT_Y
        self._final_pose.point.x -= self.ROBOT_FOOT_X

        self._pos_difference = sqrt(self._final_pose.point.x**2 + self._final_pose.point.y**2)

        # Calculate position and heading errors correctly
        heading_error = normalisedTheta(self._final_pose.heading - (my_pose.heading + math.pi))

        # Check if in position
        if (
            not self._position_close
            and self._final_pose.point.x < self.IN_POSITION_DISTANCE_X
            and self._final_pose.point.y < self.IN_POSITION_DISTANCE_Y
        ):
            self._position_close = True
        elif self._position_close and self._pos_difference >= self.NOT_IN_POSITION_DISTANCE:
            self._position_close = False

        # Determine movement strategy
        if self._position_close:
            self._current_sub_skill = "Kick"
            forward = 0
            left = 0
            turn = 0
        elif abs(heading_error) > self.HEADING_ERROR_TO_ADJUST:
            # Focus on heading correction
            self._current_sub_skill = "Walk"
            forward = 0
            left = 0
            turn = self.TURN_RATE if heading_error > 0 else -self.TURN_RATE

        else:
            self._current_sub_skill = "Walk"
            x_speed = self.WALK_SPEED if self._final_pose.point.x > 0 else -self.WALK_SPEED
            y_speed = self.SIDE_STEP_SPEED if self._final_pose.point.y > 0 else -self.SIDE_STEP_SPEED

            if abs(self._final_pose.point.x) < 300.0:
                x_speed *= 0.5
            elif abs(self._final_pose.point.x) < 150:
                x_speed *= 0.25

            forward = x_speed
            left = y_speed
            turn = 0

        if self._current_sub_skill == "Stand":
            self._tick_sub_skill()
        elif self._current_sub_skill == "Kick":
            self._tick_sub_skill(foot=foot, distance=distance)
        elif self._current_sub_skill == "SoloFindBall":
            self._tick_sub_skill()
        else:
            self._tick_sub_skill(forward, left, turn)
