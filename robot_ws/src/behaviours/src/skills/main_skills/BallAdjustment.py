import subprocess
from src.skills.Skill import Skill
from src.skills.head_skills.HeadLookStraight import head_look_straight
from src.skills.leaf_skills.Walk import Walk
from src.skills.leaf_skills.Stand import Stand
from src.skills.leaf_skills.Kick import Kick
from src.datamodels.RobotPose import RobotPose, VBallToRobotPose
from src.utils.Ball import getVisionBall
from math import radians, atan2, sqrt
from geometry_msgs.msg import Point


class BallAdjustment(Skill):
    """
    Description:

    Lines up the foot with the ball ready for kicking. Uses left foot for kicking.
    Will kick the ball after lining up.
    Uses vision robot relative ball.
    """

    # Closeness parameters

    IN_POSITION_DISTANCE_X = 130  # mm
    IN_POSITION_DISTANCE_Y = 70  # mm

    NOT_IN_POSITION_DISTANCE = 200  # mm

    # Speeds of walk (used to adjust the position of the robot to the ball)
    # Note: Slower than walk to point for micro adjustments
    WALK_SPEED = 150.0  # mm / s
    SIDE_STEP_SPEED = 120.0  # mm / s

    TURN_RATE = 0.3  # rad / s

    HEADING_ERROR_TO_ONLY_TURN = radians(60)  # rad
    # Smaller than WalkToPoint
    HEADING_ERROR_TO_ADJUST = radians(40)  # rad

    TIME_TO_FIX_HEADING = 10.0  # how much time we allow to align heading (s)

    # Positioning for center of robot to right foot (added 50mm for the center of the ball)
    ROBOT_FOOT_X = 50.0  # mm
    ROBOT_FOOT_Y = 50.0  # mm

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Walk": Walk(self),
            "Stand": Stand(self),
            "Kick": Kick(self)
        }

    def _reset(self):
        self._current_sub_skill = "Walk"
        self._position_close = False
        self._heading_close = False
        self._final_pos = None

    def _check_finished(self):
        if self._current_sub_skill == "Kick":
            return True
        return False

    def _tick(self, heading=None, speed=1.0, foot="left", distance=None):
        self._vision_ball = getVisionBall(self.ctx.blackboard)
        if self._vision_ball is None:
            # head_look_straight(self.ctx.blackboard)
            self._current_sub_skill = "Walk"
            if self._final_pos is not None and self._final_pos.point.x < 100:
                self._current_sub_skill = "Walk"
                walk_speed = -self.WALK_SPEED
            else:
                walk_speed = 0
                self._tick_sub_skill(forward=walk_speed)
            return
        else:
            self._current_sub_skill = "Walk"
        self._final_pos = VBallToRobotPose(self._vision_ball.ball_coordinates)

        # Multiply by positive or negative depending on the foot
        if foot == "left":
            self._foot = 1
        elif foot == "right":
            self._foot = -1

        # Adjustment relative to foot
        self._final_pos.point.y -= self._foot * self.ROBOT_FOOT_Y
        self._final_pos.point.x -= self.ROBOT_FOOT_X

        self._pos_difference = sqrt(self._final_pos.point.x**2 + self._final_pos.point.y**2)
        if heading is None:
            heading = atan2(self._final_pos.point.y, self._final_pos.point.x)

        # Checking if the distance is within the acceptable range or not
        if (
            not self._position_close
            and abs(self._final_pos.point.x) < self.IN_POSITION_DISTANCE_X
            and abs(self._final_pos.point.y) < self.IN_POSITION_DISTANCE_Y
        ):
            self._position_close = True
        elif self._position_close and self._pos_difference >= self.NOT_IN_POSITION_DISTANCE:
            self._position_close = False

        self._heading_diff = heading - self.ctx.blackboard.robot_world.heading

        if self._position_close:
            self._current_sub_skill = "Kick"
            forward = 0
            left = 0
            turn = 0
        # elif abs(heading) > self.HEADING_ERROR_TO_ONLY_TURN:
        # TODO: heading is not relative at the moment, make it relative (calculate the angle with normalisedTheta)
        elif abs(self._heading_diff) > self.HEADING_ERROR_TO_ADJUST:
            # If heading is very off, just turn, walking forwards with a big
            # turn is very unstable
            forward = -30
            left = 0
            # Might be backwards, I changed it ages ago
            turn = self.TURN_RATE if self._heading_diff > 0 else -self.TURN_RATE
            # turn = heading / self.TIME_TO_FIX_HEADING
        else:
            x_speed = self.WALK_SPEED if self._final_pos.point.x > 0 else -self.WALK_SPEED
            y_speed = self.SIDE_STEP_SPEED if self._final_pos.point.y > 0 else -self.SIDE_STEP_SPEED
            x_speed = x_speed - 30 if x_speed < 0 else x_speed
            # Slow down if we're close to the final position
            # (to prevent overshooting)
            if abs(self._final_pos.point.x) < 300.0:
                x_speed *= 0.4
            forward = x_speed
            left = y_speed
            turn = 0

        if self._current_sub_skill == "Stand":
            self._tick_sub_skill()
        elif self._current_sub_skill == "Kick":
            self._tick_sub_skill(foot=foot, distance=distance)
        else:
            self._tick_sub_skill(forward, left, turn, speed=speed)
