import time
import math
from src.skills.Skill import Skill
import traceback
# Delete as unnecessary (head skills)
from src.skills.head_skills.HeadLookStraight import head_look_straight
from src.skills.head_skills.HeadNod import head_nod
from src.skills.head_skills.HeadScanBall import head_scan_ball
from src.skills.head_skills.HeadTrackBallPixel import head_track_ball_pixel

# Delete as unnecessary (leaf skills)
from src.skills.leaf_skills.Stand import Stand
from src.skills.leaf_skills.Walk import Walk
from src.skills.leaf_skills.Kick import Kick
from src.skills.leaf_skills.Sit import Sit

# Delete as unnecessary (main skills)
from src.skills.main_skills.ApproachBall import ApproachBall
from src.skills.main_skills.SoloFindBall import SoloFindBall
from src.skills.main_skills.WalkToPose import WalkToPose

# Delete as unnecessary (utils)
from src.utils.Ball import getAbsoluteBall
import src.utils.Constants
import src.utils.FieldCheck
from src.utils.FieldGeometry import ENEMY_GOAL_BEHIND_CENTER
from src.utils.MathUtil import distancePointToPoint
from src.utils.TeamInfo import getClosestRobotToTeamBall, getTeamBall
import src.utils.Timer
from src.utils.Constants import HALF_FIELD_WIDTH
from src.utils.Ball import getVisionBall

from runswift_interfaces.msg import BehavioursRobotInfo
from src.datamodels.RobotPose import RobotPose
from geometry_msgs.msg import Point

from rclpy.clock import Clock

class Attacker(Skill):
    """
    Description:
    Attacker:
    - Robot that isn't going for ball will move to a supporting position behind and on the opposite half of the ball
    - Shoot after doing a pass
    - If we haven't passed we should pass to other attacker
    """

    SUPPORT_OFFSET_X = 100
    SUPPORT_OFFSET_Y = 1000
    POSITION_TOLERANCE = 500

    # How long the robot should not see the ball for before returning to their default positions
    RETURN_TIMEOUT = 20  # s

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Stand": Stand(self),
            "ApproachBall": ApproachBall(self),
            "WalkToPose": WalkToPose(self),
            "SoloFindBall": SoloFindBall(self),
        }

    def _reset(self):
        self._current_sub_skill = "Stand"
        self.target = ENEMY_GOAL_BEHIND_CENTER
        self.support_pos = RobotPose()
        self.sent_packet = False
        self.last_seen_ball_time = time.time()
        self.forward_only = True

        self._default_positions = [
            RobotPose(point = Point(x=0.0, y=-HALF_FIELD_WIDTH / 2 - 500), heading = math.pi/2),
            RobotPose(point = Point(x=0.0, y=HALF_FIELD_WIDTH / 2 + 500), heading = -math.pi/2),
        ]

    def _transition(self):
        if self.should_send_packet():
            self.ctx.controller.publish_comms_command()

        ball = getAbsoluteBall(self.ctx.blackboard)
        if ball is None and time.time() - self.ctx.blackboard._last_vision_ball > self.RETURN_TIMEOUT:
            self._current_sub_skill = "WalkToPose"
            self.forward_only = True
            for robot in self.ctx.blackboard.robot_comms.values():
                if robot.player_role == BehavioursRobotInfo.ATTACKER and robot.player_number != self.ctx.blackboard.player_info.player_number:
                    if self.ctx.blackboard.player_info.player_number < robot.player_number:
                        self.support_pos = self._default_positions[0]
                    else:
                        self.support_pos = self._default_positions[1]
            if self.support_pos is None:
                self.support_pos = self._default_positions[0]

        elif ball is None:  # and the timeout is not already reached
            self._current_sub_skill = "SoloFindBall"
        else:
            self._current_sub_skill = "ApproachBall"
            # If we are not the closest to the ball
            # closest_teammate_number = getClosestRobotToTeamBall(self.ctx.blackboard)
            # closest_teammate = self.ctx.blackboard.robot_comms.get(closest_teammate_number, None)
            # closest_teammate_role = closest_teammate.player_role if closest_teammate is not None else None
            # if closest_teammate_number is None:
            #     self._current_sub_skill = "ApproachBall"
            #     self.target = ENEMY_GOAL_BEHIND_CENTER
            # elif self.ctx.blackboard.player_info.player_number != closest_teammate_number:
            #
            #     self.support_pos = RobotPose()
            #     # If ball is on the right, support position is left and behind ball
            #     if ball.pos_y < 0:
            #         self.support_pos.point.y = ball.pos_y + self.SUPPORT_OFFSET_Y
            #     else:
            #         self.support_pos.point.y = ball.pos_y - self.SUPPORT_OFFSET_Y
            #     self.support_pos.point.x = ball.pos_x - self.SUPPORT_OFFSET_X
            #
            #     my_pos = self.ctx.blackboard.robot_world
            #     if distancePointToPoint(my_pos, self.support_pos) > self.POSITION_TOLERANCE:
            #         self._current_sub_skill = "WalkToPose"
            #         self.forward_only = True
            #     else:
            #         self._current_sub_skill = "SoloFindBall"

            # Pass to the other attacker if pass has not been made
            # elif self.ctx.blackboard.kick_success == False:
            #     teammate_pos = RobotPose()
            #     for robot in self.ctx.blackboard.robot_comms.values():
            #         if robot.player_role == BehavioursRobotInfo.ATTACKER and robot.player_number != self.ctx.blackboard.player_info.player_number:
            #             teammate_pos.point.x = robot.robot_pos_x
            #             teammate_pos.point.y = robot.robot_pos_y
            #     if teammate_pos != RobotPose():
            #         self.target = teammate_pos
            #     else:
            #         self.target = ENEMY_GOAL_BEHIND_CENTER
            #     self._current_sub_skill = "ApproachBall"
            # else:
            #     self.target = ENEMY_GOAL_BEHIND_CENTER
            #     self._current_sub_skill = "ApproachBall"

    def _tick(self):
        if self._current_sub_skill == "ApproachBall":
            self._tick_sub_skill(aim_point=self.target)
        elif self._current_sub_skill == "WalkToPose":
            if getVisionBall(self.ctx.blackboard) is None:
                head_scan_ball(self.ctx.blackboard)
            self._tick_sub_skill(final_pos=self.support_pos, forward_only=self.forward_only)
        else:
            self._tick_sub_skill()

    # Returns true if no other robot is going for ball
    def ball_free(self):
        for player in self.ctx.blackboard.robot_comms.values():
            if player.my_ball == True and player.player_number != self.ctx.blackboard.player_info.player_number:
                return False
        return True

    # Returns true if no packet has been sent in last 5 seconds
    def should_send_packet(self):
        for player in self.ctx.blackboard.robot_comms.values():
            if player.player_number == self.ctx.blackboard.player_info.player_number:
                clock = Clock()
                if clock.now().to_msg().sec - player.timestamp.sec < 5:
                    self.sent_packet = False
                elif clock.now().to_msg().sec - player.timestamp.sec > 5 and self.sent_packet == False:
                    self.sent_packet = True
                    return True
        return False
