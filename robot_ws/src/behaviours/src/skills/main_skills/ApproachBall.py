import subprocess
from src.skills.Skill import Skill
from src.skills.main_skills.WalkToPose import WalkToPose
from src.skills.leaf_skills.Stand import Stand
from src.skills.main_skills.BallAdjustment import BallAdjustment
from src.skills.main_skills.Lineup import Lineup
from src.skills.main_skills.FollowBall import FollowBall
from src.skills.main_skills.SideStepInCircle import SideStepInCircle
from src.datamodels.RobotPose import RobotPose, SeBallToRobotPose
from src.skills.main_skills.SoloFindBall import SoloFindBall
from math import radians
from geometry_msgs.msg import Point
from src.utils.Constants import HALF_FIELD_LENGTH, TOE_CENTRE_X, HIP_OFFSET
from src.utils.MathUtil import normalisedTheta, distancePointToPoint
from src.utils.Ball import getAbsoluteBall, getVisionBall
import math


class ApproachBall(Skill):
    """
    Description:
    Given a target to kick to, robot will approach ball and lineup behind
    the ball so that a straight kick will kick to target.
    """

    # Default aim of where the robot should kick to, set to enemy goal
    DEFAULT_AIM = float(HALF_FIELD_LENGTH)  # mm

    # Constant to scale the normalised vector by, used to offset the target position
    # for the robot so it doesn't walk into the ball
    TARGET_LOCATION_OFFSET_BALL = 400

    TURN_RATE = 1.5  # rad / s

    # Distance to offset the intermediate point by so robot doesn't walk into ball
    INTERMEDIATE_POINT_OFFSET_DIST = 450
    TRAJECTORY_THRESHOLD_OFFSET = 100

    # How close to the ball the robot needs to be to circle to pose
    CIRCLE_TO_POSE_THRESHOLD = 200
    CIRCLE_TO_POSE_HEADING = math.radians(50)
    STOP_CIRCLE_TO_POSE_THRESHOLD = 300
    STOP_CIRCLE_TO_POSE_HEADING = math.radians(10)
    FACING_THE_BALL_HEADING = math.radians(50)
    STOP_FACING_THE_BALL_HEADING = math.radians(30)
    CIRCLING_SPEED = 250.0
    CIRLCE_TO_POSE_HEADING_SLOW = math.radians(70)
    CIRCLE_RADIUS_CORRECTION = 0  # mm

    # Do not change the end pose unless it has changed by this much
    OVERRIDE_END_POSE_THRESHOLD = 0

    # Close to ball region size
    CLOSE_TO_BALL_REGION_PERPENDICULAR = 200
    CLOSE_TO_BALL_REGION_PARALLEL = TARGET_LOCATION_OFFSET_BALL + 200

    # Prevent rapid switching when distances are close
    HYSTERESIS_THRESHOLD = 200

    # Initialise the subtask as the walkToPoint skill
    def _initialise_sub_skills(self):
        self._sub_skills = {
            "WalkToPose": WalkToPose(self),
            "Stand": Stand(self),
            "BallAdjustment": BallAdjustment(self),
            "SoloFindBall": SoloFindBall(self),
            "Lineup": FollowBall(self),
            "SideStepInCircle": SideStepInCircle(self),
        }

    def _reset(self):
        self._current_sub_skill = "Stand"
        self._position_close = False
        self._chosen_side = None
        self._BallAdjustment = False
        self._previous_end_pose = None
        self.end_pose = None
        self.aim_point = None
        self._chosen_side = None
        self.distance = None
        self._kicked = False
        self._foot = None
        self._is_circling = False
        self._is_sidestepping = False

    def _transition(self):
        # Claim ball
        self.ctx.blackboard.my_ball = True

        if self.aim_point is None:
            return

        if self.end_pose is not None:
            self._previous_end_pose = self.end_pose

        in_region = False

        my_pose = self.ctx.blackboard.robot_world
        abs_ball = getAbsoluteBall(self.ctx.blackboard)

        if abs_ball is None:
            self._current_sub_skill = "SoloFindBall"
        else:
            ball_pose = SeBallToRobotPose(abs_ball)

            # get vector between ball and aim point
            offset_vector = ball_pose.subtract(self.aim_point.point)
            offset_pose = offset_vector.normalise_copy().scale(self.TARGET_LOCATION_OFFSET_BALL)
            # robots end point is an offset distance from the ball
            self.end_pose = ball_pose.add(offset_pose.point)
            self.end_pose.heading = self.aim_point.angleTo(self.end_pose.point)

            # Calculate how far away from the ball we are
            distance_to_ball = ball_pose.subtract(my_pose.point).length()
            heading_diff = (self.end_pose.heading - my_pose.heading + math.pi) % (2 * math.pi) - math.pi

            # Check if we should circle to pose
            # if distance to the ball is less than 1m and the angle to the final pose is less than 30 degrees

            if distance_to_ball < self.CIRCLE_TO_POSE_THRESHOLD and abs(heading_diff) > self.CIRCLE_TO_POSE_HEADING and not self._is_circling:
                self._is_circling = True
            elif (
                abs(heading_diff) < self.STOP_CIRCLE_TO_POSE_HEADING and self._is_circling or
                distance_to_ball > self.STOP_CIRCLE_TO_POSE_THRESHOLD and self._is_circling
            ):
                self._is_circling = False

            if self._is_circling:

                # compute direction to ball
                angle_to_ball = math.atan2(ball_pose.point.y, ball_pose.point.x)

                # error between where we're facing and the ball
                heading_error = ((my_pose.heading - math.radians(180) - my_pose.angleTo(ball_pose.point) + math.pi) % (2 * math.pi)) - math.pi

                if abs(heading_error) > self.FACING_THE_BALL_HEADING and not self._is_sidestepping:
                    self._is_sidestepping = True
                elif abs(heading_error) < self.STOP_FACING_THE_BALL_HEADING and self._is_sidestepping:
                    self._is_sidestepping = False

                if self._is_sidestepping:
                    # rotate in place to face ball
                    self._current_sub_skill = "WalkToPose"
                    self.end_pose = my_pose
                    self.end_pose.heading = my_pose.angleTo(ball_pose.point) + math.radians(180) % math.radians(360)
                else:
                    # now that we're facing it, sidestep in circle
                    self._current_sub_skill = "SideStepInCircle"
                    self.left = self.CIRCLING_SPEED if heading_diff < 0 else -self.CIRCLING_SPEED
                    if heading_diff < self.CIRLCE_TO_POSE_HEADING_SLOW:
                        self.left *= 0.5
                    self.radius = max(distance_to_ball - self.CIRCLE_RADIUS_CORRECTION, self.CIRCLE_RADIUS_CORRECTION)
                return

            # Calculate if we are in the threshold for going to an intermediate point
            trajectory_threshold = offset_pose.normal_vector()
            theta = offset_vector.signed_angle_between(offset_pose)
            if my_pose.subtract(self.aim_point.point).length() * math.cos(theta) < offset_vector.length():
                # Calculate intermediate points
                scaled_trajectory_threshold = trajectory_threshold.normalise_copy().scale(
                    self.INTERMEDIATE_POINT_OFFSET_DIST
                )
                scaled_offset_pose = offset_pose.normalise_copy().scale(self.TRAJECTORY_THRESHOLD_OFFSET)

                left_pose = scaled_trajectory_threshold.add(ball_pose.point).add(scaled_offset_pose.point)
                right_pose = scaled_trajectory_threshold.scale(-1).add(ball_pose.point).add(scaled_offset_pose.point)

                left_pose_distance = left_pose.subtract(my_pose.point).length()
                right_pose_distance = right_pose.subtract(my_pose.point).length()

                if self._chosen_side is None:
                    self._chosen_side = "left" if left_pose_distance < right_pose_distance else "right"
                elif (
                    self._chosen_side == "left" and right_pose_distance < left_pose_distance - self.HYSTERESIS_THRESHOLD
                ):
                    self._chosen_side = "right"
                elif (
                    self._chosen_side == "right"
                    and left_pose_distance < right_pose_distance - self.HYSTERESIS_THRESHOLD
                ):
                    self._chosen_side = "left"

                # Use the chosen side
                self.end_pose = left_pose if self._chosen_side == "left" else right_pose

            else:
                # Check if we are in the "close region" behind the ball
                ball_to_end_point = self.end_pose.subtract(ball_pose.point)
                ball_to_robot = my_pose.subtract(ball_pose.point)
                theta = ball_to_end_point.signed_angle_between(ball_to_robot)

                ball_to_robot_length = ball_to_robot.length()
                perpendicular_distance = abs(ball_to_robot_length * math.sin(theta))
                parallel_distance = ball_to_robot_length * math.cos(theta)

                if (
                    perpendicular_distance < self.CLOSE_TO_BALL_REGION_PERPENDICULAR
                    and parallel_distance < self.CLOSE_TO_BALL_REGION_PARALLEL
                    and parallel_distance > -100
                ):
                    # I am in a position to shoot, just spin to the target then shoot
                    self.end_pose.point = my_pose.point
                    self._current_sub_skill = "WalkToPose"
                    in_region = True

            if (
                not in_region
                and self._previous_end_pose is not None
                and self.end_pose.subtract(self._previous_end_pose.point).length() < self.OVERRIDE_END_POSE_THRESHOLD
            ):
                self.end_pose = self._previous_end_pose

            self._current_sub_skill = "WalkToPose"
            if self._sub_skill_finished(skill="WalkToPose", final_pos=self.end_pose):
                self._current_sub_skill = "Lineup"
            if self._sub_skill_finished(skill="Lineup") and not self._kicked:
                self._kicked = True
                self.ctx.blackboard.kick_success = True
                self.ctx.controller.publish_comms_command()

    def _tick(self, aim_point=RobotPose(Point(x=DEFAULT_AIM, y=0.0))):
        # def _tick(self, aim_point=RobotPose(Point(x=-750.0, y=3000.0))):
        self.aim_point = aim_point
        if self.end_pose is not None and self.aim_point is not None and self.aim_point.point is not None and self.end_pose.point is not None:
            self.distance = distancePointToPoint(self.aim_point, self.end_pose)
        if self._current_sub_skill == "WalkToPose":
            self._tick_sub_skill(final_pos=self.end_pose, heading_tolerance=30)
        elif self._current_sub_skill == "BallAdjustment":
            self._tick_sub_skill(heading=self.end_pose.heading if self.end_pose is not None else None, distance=self.distance)
        elif self._current_sub_skill == "Lineup":
            self._tick_sub_skill()
        elif self._current_sub_skill == "SideStepInCircle":
            self._tick_sub_skill(self.radius, self.left)
        else:
            self._tick_sub_skill()
