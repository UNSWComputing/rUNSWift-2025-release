from runswift_interfaces.msg import SeBall, SeBallsAbsolute

from src.datamodels.RobotPose import RobotPose
from src.skills.Skill import Skill
import math
import time
from collections import deque
import traceback

# Delete as unnecessary (head skills)
from src.skills.head_skills.HeadLookStraight import head_look_straight
from src.skills.head_skills.HeadNod import head_nod
from src.skills.head_skills.HeadScanBall import head_scan_ball
from src.skills.head_skills.HeadTrackBallPixel import head_track_ball_pixel
from src.skills.head_skills.HeadScanBallGoalie import head_scan_ball_goalie


# Delete as unnecessary (leaf skills)
# from src.skills.leaf_skills.Stand import Stand
# from src.skills.leaf_skills.Walk import Walk
# from src.skills.leaf_skills.Kick import Kick
# from src.skills.leaf_skills.Sit import Sit
# from src.skills.leaf_skills.Dive import Dive


# Delete as unnecessary (main skills)
from src.skills.leaf_skills.Dive import Dive
from src.skills.main_skills.ApproachBall import ApproachBall
from src.skills.main_skills.WalkToPose import WalkToPose

# Delete as unnecessary (utils)
from src.utils import Constants
import src.utils.Ball
import src.utils.Constants
import src.utils.FieldCheck
import src.utils.FieldGeometry
import src.utils.Hysteresis
import src.utils.MathUtil
import src.utils.TeamInfo
import src.utils.Timer

# Delete as unnecessary (utils)
from src.utils.Ball import getAbsoluteBall, getBall, getVisionBall, BallMeasurement
from geometry_msgs.msg import Point
from src.utils.Constants import HALF_FIELD_LENGTH, GOAL_POST_ABS_Y

# from src.utils.FieldCheck import ...
# from src.utils.FieldGeometry import ...
# from src.utils.Hysteresis import ...
# from src.utils.MathUtil import ...
# from src.utils.TeamInfo import ...
# from src.utils.Timer import ...
# from src.utils.Constants import 

our_goal_x = -HALF_FIELD_LENGTH
our_goal_y = 0.0
goalie_pose = RobotPose(point=Point(x=our_goal_x, y=our_goal_y), heading = 0) 

class Goalie(Skill):
    """
    Description:
    This skill is intended to be used in conjunction with Attacker.py,
    Defender.py and Midfielder.py. It dictates the strategy of the robot
    depending on what role it is given

    Goalie:
    - Stay in the goal (roughly the middle half of the goal)
    - covering just moves to block goal
    - active takes a more active approach, moving to the ball
    """

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "ApproachBall": ApproachBall(self),
            "WalkToPose": WalkToPose(self),
            "Dive": Dive(self)
        }

    def _reset(self):
        self.ball_last_seen = 0.0

        # prevents the robot from diving repeatedly when the ball is really close
        self.can_dive = True

        self.BALL_CLOSE_THRESHOLD = 1500
        self.BALL_FAR_THRESHOLD = 2000
        
        self._current_sub_skill = "WalkToPose"
        self._sub_skill_parameters = { "final_pos": goalie_pose }

    def _transition(self):
        try:
            if self._current_sub_skill == "Dive":
                # get out of dive
                self._current_sub_skill = "WalkToPose"
                self._sub_skill_parameters = { "final_pos": goalie_pose }
                return

            # make sure we've got a ball
            if self.ctx.blackboard.vision_ball is None:
                head_scan_ball_goalie(self.ctx.blackboard)
                
                if (time.time() - self.ball_last_seen) > 3.0:
                    # we haven't seen ball in 3s, go back to goal to defend
                    self._current_sub_skill = "WalkToPose"
                    self._sub_skill_parameters = { "final_pos": goalie_pose }
            else:
                head_track_ball_pixel(self.ctx.blackboard)
                self.ball_last_seen = time.time()

                if self.can_dive and self._current_sub_skill != "ApproachBall":
                    # check if any vision balls are really close to the goal
                    for feature in self.ctx.blackboard.vision_ball.ball_features:
                        # 30cm within the goal
                        ball_rel = feature.ball_coordinates
                        me = self.ctx.blackboard.robot_world
                        # me: RobotPose

                        if (abs(me.heading) <= math.radians(30) or abs(me.heading) >= math.radians(330)) and our_goal_x < me.point.x < our_goal_x + 400 and ball_rel.x < 300 and abs(ball_rel.y) < 300:
                            self._current_sub_skill = "Dive"
                            self._sub_skill_parameters = { "direction": "centre" }
                            self.can_dive = False
                            self.ctx.get_logger().error(f"Diving on ball ({ball_rel.x}, {ball_rel.y})!")
                            return
                        else:
                            self.ctx.get_logger().error(f"Not diving on ball ({ball_rel.x}, {ball_rel.y}), btw my heading is {math.degrees(me.heading)}")

            if self.ctx.blackboard.ball_world is None:
                return
            
            ball = BallMeasurement.from_balls_absolute_msg(self.ctx.blackboard.ball_world)

            if ball is None:
                return
            
            is_ball_close = (ball.x - our_goal_x) <= self.BALL_CLOSE_THRESHOLD and abs(ball.y - our_goal_y) <= self.BALL_CLOSE_THRESHOLD
            is_ball_far   = (ball.x - our_goal_x)  > self.BALL_FAR_THRESHOLD   and abs(ball.y - our_goal_y) <= self.BALL_FAR_THRESHOLD

            if self._current_sub_skill != "ApproachBall":
                if is_ball_close:
                    # ball is getting close, kick it!
                    self._current_sub_skill = "ApproachBall"
                    self._sub_skill_parameters = {} # default aim point is good enough
                else:
                    self._current_sub_skill = "WalkToPose"
                    target_y = min(max(ball.y, -Constants.GOAL_POST_ABS_Y + 300), Constants.GOAL_POST_ABS_Y - 300)
                    self._sub_skill_parameters = { "final_pos": RobotPose(point=Point(x=our_goal_x + 300, y=target_y), heading = 0) }

                    # ball got reasonably far away, we can dive again
                    self.can_dive = True
            
            elif is_ball_far:
                # we've kicked the ball far enough, return to goal
                self._current_sub_skill = "WalkToPose"
                self._sub_skill_parameters = { "final_pos": goalie_pose, "forward_only": False }

                # ball got far away, we can dive again
                self.can_dive = True

        except Exception:
            # no exception or race condition (:skull:) shall ever stop the goalie in its crusade
            print(traceback.format_exc())
            pass

    def _tick(self):
        self._tick_sub_skill(**self._sub_skill_parameters)