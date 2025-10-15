from unittest import skipUnless
from src.skills.Skill import Skill
from src.skills.leaf_skills.Walk import Walk
from src.skills.leaf_skills.Stand import Stand
from src.skills.leaf_skills.Kick import Kick
from src.datamodels.RobotPose import RobotPose, SeBallToRobotPose, VBallToRobotPose
from src.utils.Ball import getAbsoluteBall, getVisionBall
from math import radians, atan2, sqrt
from geometry_msgs.msg import Point
import math


class FollowBall(Skill):
    """
    Description:
    Walks towards the ball.
    """

    # Closeness parameters
    IN_POSITION_DISTANCE = 70  # mm
    NOT_IN_POSITION_DISTANCE = 250  # mm

    # Speeds of walk (used to adjust the position of the robot to the ball)
    # Note: Slower than walk to point for micro adjustments
    WALK_SPEED = 150.0  # mm / s
    TURN_RATE = 1.0  # rad / s
    HEADING_SLOW_ERROR = radians(35)

    TIME_TO_FIX_HEADING = 1.0  # how much time we allow to align heading (s)

    HEADING_ERROR_TO_ONLY_TURN = radians(45)  # rad
    # Smaller than WalkToPoint
    HEADING_ERROR_TO_ADJUST = radians(25)  # rad

    TIME_TO_FIX_HEADING = 1.0  # how much time we allow to align heading (s)

    # Positioning for center of robot to right foot (added 50mm for the center of the ball)
    ROBOT_FOOT_X = 90.0  # mm
    ROBOT_FOOT_Y = 40.0  # mm
    CHANGE_FOOT_THRESHOLD = 100.0  # mm

    # How close the robot should be to the ball in the x direction before sidestepping
    SIDE_STEP_DISTANCE = 6000  # mm

    # Parameters for head scanning
    HEAD_TURN_RATE = 0.05
    MAX_HEAD_YAW = radians(70)
    HEAD_SCAN_PITCH = 0.4

    def _initialise_sub_skills(self):
        self._sub_skills = {"Walk": Walk(self), "Stand": Stand(self), "Kick": Kick(self)}

    def _reset(self):
        self._current_sub_skill = "Walk"
        self._position_close = False
        self._heading_close = False
        self._head_hit_left = False
        self._chosen_foot = None
    def _check_finished(self):
        if self._current_sub_skill == "Kick":
            return True
        return False

    # Expecting final_pos as RobotPose or 0
    def _tick(self, final_pos=0, speed=1.0, foot="left"):
        if final_pos == 0:
            self._vision_ball = getVisionBall(self.ctx.blackboard)
            if self._vision_ball is None:
                # self._current_sub_skill = "Stand"
                forward = 0
                left = 0
                turn = 0
                self._tick_sub_skill()
                return
            else:
                self._current_sub_skill = "Walk"
            self._final_pos = VBallToRobotPose(self._vision_ball.ball_coordinates)
        else:
            self._final_pos = final_pos



        # if self._final_pos.point.y < 0 and self._chosen_foot is not None:
        #     self._chosen_foot = "right"
        # elif self._final_pos.point.y > 0 and self._chosen_foot is not None:
        #     self._chosen_foot = "left"
        # else:
        #     self._chosen_foot = "left"

        abs_ball = getAbsoluteBall(self.ctx.blackboard)
        if abs_ball is None:
            self._chosen_foot = "left"
        else:
            abs_ball_pose = SeBallToRobotPose(abs_ball)
            my_pose = self.ctx.blackboard.robot_world
            heading_to_ball = my_pose.angleTo(abs_ball_pose.point)
            if heading_to_ball > math.pi:
                self._chosen_foot = "left"
            else:
                self._chosen_foot = "right"

        foot = self._chosen_foot

        # Multiply by positive or negative depending on the foot
        if foot == "left":
            self._foot = 1
        elif foot == "right":
            self._foot = -1

        # Adjustment relative to foot
        self._final_pos.point.y -= self._foot * self.ROBOT_FOOT_Y
        self._final_pos.point.x -= self.ROBOT_FOOT_X

        self._pos_difference = sqrt(self._final_pos.point.x**2 + self._final_pos.point.y**2)
        heading = atan2(self._final_pos.point.y, self._final_pos.point.x)

        # Checking if the distance is within the acceptable range or not
        if not self._position_close and self._pos_difference < self.IN_POSITION_DISTANCE and self._final_pos.point.x > 0:
            self._position_close = True
        elif self._position_close and self._pos_difference >= self.NOT_IN_POSITION_DISTANCE:
            self._position_close = False

        if self._position_close:
            if foot == "right" and self._final_pos.point.y > self.CHANGE_FOOT_THRESHOLD:
                foot == "left"
            elif foot == "left" and self._final_pos.point.y < -self.CHANGE_FOOT_THRESHOLD:
                foot == "right"

            self._current_sub_skill = "Kick"
            forward = 0
            left = 0
            turn = 0

        elif abs(heading) > self.HEADING_ERROR_TO_ONLY_TURN:
            # If heading is very off, just turn, walking forwards with a big
            # turn is very unstable
            forward = 0
            left = 0
            turn = self.TURN_RATE if heading > 0 else -self.TURN_RATE
            if self.HEADING_SLOW_ERROR:
                turn *= 0.5
        else:
            point = Point()
            point.x = self.WALK_SPEED
            point.y = self._final_pos.point.y * 0.7 if self._final_pos.point.x < self.SIDE_STEP_DISTANCE else 0.0
            point.z = 0.0
            walkPose = RobotPose(point)

            # Slow down if we're close to the final position
            # # (to prevent overshooting)
            if 70.0 < self._pos_difference < 220.0:
                walkPose.scale(0.5)
            # elif self._pos_difference < 160.0:
            #     walkPose.scale(0.25)

            forward = walkPose.point.x
            left = walkPose.point.y

            if abs(heading) > self.HEADING_ERROR_TO_ADJUST:
                # Aim to correct the heading error over an amount of time
                turn = heading / self.TIME_TO_FIX_HEADING
            else:
                # If the error is small, just keep walking straight,
                # constantly changing turn causes instability
                turn = 0
        if self._current_sub_skill == "Walk":
            self._tick_sub_skill(forward, left, turn, speed=speed)
        else:
            self._tick_sub_skill(foot=foot)

    def _head_scan(self):
        self.ctx.blackboard.motion_command.head_command.is_relative = False
        yaw = self.ctx.blackboard.motion_command.head_command.yaw
        if yaw > 0 and self._head_hit_left:
            self._head_hit_left = False
            return True
        if yaw > self.MAX_HEAD_YAW:
            self.ctx.blackboard.head_yaw_increment = -self.HEAD_TURN_RATE
        elif yaw < -self.MAX_HEAD_YAW:
            self.ctx.blackboard.head_yaw_increment = self.HEAD_TURN_RATE
            self._head_hit_left = True
        self.ctx.blackboard.motion_command.head_command.pitch = self.HEAD_SCAN_PITCH
        self.ctx.blackboard.motion_command.head_command.yaw += self.ctx.blackboard.head_yaw_increment
        return False

