import math
from src.skills.Skill import Skill
from src.skills.leaf_skills.Walk import Walk
from src.skills.leaf_skills.Stand import Stand
from src.datamodels.RobotPose import RobotPose
from math import radians
from geometry_msgs.msg import Point
from src.utils.MathUtil import normalisedTheta


class WalkToPoint(Skill):
    """
    Description:
    Skill associated with walking to a global/relative point (to be confirmed).

    Note: This is heavily based on the previous WalkToPoint
    """

    # Definition of close i.e. how close to the point the robot has to be to be
    # considered there (Current parameters from previous code)
    IN_POSITION_DISTANCE = 50  # mm
    NOT_IN_POSITION_DISTANCE = 150  # mm

    # Constants, later on these should be made as parameters as a metric we can
    # use where it factors in risk, where the greater the variables, the more
    # risky it is to execute
    WALK_SPEED = 250.0  # mm / s
    TURN_RATE = 1.5  # rad / s
    SLOW_TURN_RATE = 0.4

    HEADING_ERROR_TO_ONLY_TURN = radians(40)  # rad
    HEADING_ERROR_TO_ADJUST = radians(15)  # rad

    # Initialise the subtask as the walk leaf skill
    def _initialise_sub_skills(self):
        self._sub_skills = {"Walk": Walk(self), "Stand": Stand(self)}

    def _reset(self):
        self._current_sub_skill = "Walk"
        self._position_close = False
        self._heading_close = False

    def _check_finished(self, final_pos=None, *args, **kwargs):
        if final_pos is None:
            return False
        self._update_position_close(final_pos)
        return self._position_close

    def _tick(self, final_pos=RobotPose(), speed=1.0, forward_only=True):
        # print(f"            {self._current_sub_skill}: {self._position_close}: {self._check_finished(final_pose=final_pos)}")

        self._update_position_close(final_pos)

        heading = RobotPose().angleTo(self._pos_difference.point)
        headingError = normalisedTheta(heading - self.ctx.blackboard.robot_world.heading)

        if self._position_close:
            self._current_sub_skill = "Stand"
            self.parent.finished = True
            return

        if not forward_only:
            dx = self._pos_difference.point.x
            dy = self._pos_difference.point.y
            h = self.ctx.blackboard.robot_world.heading
            cos_h = math.cos(h)
            sin_h = math.sin(h)

            # rotate into robot-frame
            forward = -(dx * cos_h + dy * sin_h)
            left = -(-dx * sin_h + dy * cos_h)
            turn = 0.0

            self._current_sub_skill = "Walk"
            self._tick_sub_skill(forward, left, turn, speed=speed)
            return

        if abs(headingError) > self.HEADING_ERROR_TO_ONLY_TURN:
            # If heading is very off, just turn, walking forwards with a big
            # turn is very unstable
            forward = 0
            left = 0
            turn = self.TURN_RATE if headingError > 0 else -self.TURN_RATE
        else:
            point = Point()
            point.x = self.WALK_SPEED
            point.y = 0.0
            point.z = 0.0
            walkPose = RobotPose(point)

            # Slow down if we're close to the final position
            # (to prevent overshooting)
            if self._pos_difference.length() < 200:
                walkPose.scale(0.5)

            forward = walkPose.point.x
            left = walkPose.point.y

            if abs(headingError) > self.HEADING_ERROR_TO_ADJUST:
                # Aim to correct the heading error over an amount of time
                turn = self.SLOW_TURN_RATE if headingError > 0 else -self.SLOW_TURN_RATE
            else:
                # If the error is small, just keep walking straight,
                # constantly changing turn causes instability
                turn = 0
        if self._current_sub_skill == "Walk":
            self._tick_sub_skill(forward, left, turn, speed=speed)
        else:
            self._tick_sub_skill()

    def _update_position_close(self, final_pos):
        self._pos_difference = self.ctx.blackboard.robot_world.subtract(final_pos.point)

        if not self._position_close and self._pos_difference.length() < self.IN_POSITION_DISTANCE:
            self._position_close = True
        elif self._position_close and self._pos_difference.length() >= self.NOT_IN_POSITION_DISTANCE:
            self._position_close = False
