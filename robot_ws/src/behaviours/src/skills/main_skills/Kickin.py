from src.skills.Skill import Skill

# Delete as unnecessary (head skills)
from src.skills.head_skills.HeadLookStraight import head_look_straight
from src.skills.head_skills.HeadNod import head_nod
from src.skills.head_skills.HeadScanBall import head_scan_ball
from src.skills.head_skills.HeadTrackBallPixel import head_track_ball_pixel

# Delete as unnecessary (leaf skills)
from src.skills.leaf_skills.Stand import Stand

# Delete as unnecessary (main skills)
from src.skills.main_skills.ApproachBall import ApproachBall
from src.utils.TeamInfo import getClosestRobotToTeamBall
from src.datamodels.RobotPose import RobotPose
from geometry_msgs.msg import Point
from src.utils.Constants import HALF_FIELD_LENGTH
from runswift_interfaces.msg import CommsRCGCD

# Delete as unnecessary (utils)
import src.utils.Ball
import src.utils.Constants
import src.utils.FieldCheck
import src.utils.FieldGeometry
import src.utils.Hysteresis
import src.utils.MathUtil
import src.utils.TeamInfo
import src.utils.Timer
import math
# Delete as unnecessary (utils)
# from src.utils.Ball import ...
# from src.utils.Constants import ...
# from src.utils.FieldCheck import ...
# from src.utils.FieldGeometry import ...
# from src.utils.Hysteresis import ...
# from src.utils.MathUtil import ...
# from src.utils.TeamInfo import ...
# from src.utils.Timer import ...


class Kickin(Skill):
    """
    Description:
    Kickin setplay
    """
    
    RUNSWIFT = 18

    def _initialise_sub_skills(self):
        self._sub_skills = {"Stand": Stand(self), "ApproachBall": ApproachBall(self)}

    def _reset(self):
        self._current_sub_skill = "Stand"
        self.target = HALF_FIELD_LENGTH

    def _transition(self):
        # check kicking team 
        if self.ctx.blackboard.kicking_team == self.RUNSWIFT:
            # closest teammate to approach ball
            closest_teammate_number = getClosestRobotToTeamBall(self.ctx.blackboard)
            if self.ctx.blackboard.player_info.player_number == closest_teammate_number:  
                # aim a bit closer if corner kick      
                if self.ctx.blackboard.gameinfo.set_play == CommsRCGCD.SET_PLAY_CORNER_KICK:
                    self.target = RobotPose(Point(x=float(HALF_FIELD_LENGTH - 500), y=0.0))
                else:
                    self.target = RobotPose(Point(x=float(HALF_FIELD_LENGTH), y=0.0))
                self._current_sub_skill = "ApproachBall" 
            else: 
                self._current_sub_skill = "Stand" 
            # if its is not our kick apprach close 
        else:
            # calcuate relative distance
            my_pose = self.ctx.blackboard.robot_world
            abs_ball = getAbsoluteBall(self.ctx.blackboard)
            if abs_ball is not None:
                dx = abs_ball.x - my_pose.point.x
                dy = abs_ball.y - my_pose.point.y

                # Rotate (dx, dy) into robot's frame
                rel_x = math.cos(-my_pose.heading) * dx - math.sin(-my_pose.heading) * dy
                rel_y = math.sin(-my_pose.heading) * dx + math.cos(-my_pose.heading) * dy

                # Relative distance to ball
                relative_distance = math.hypot(rel_x, rel_y)
                # if close stop approaching
                if relative_distance < 1000:
                    self._current_sub_skill = "Stand"
                else:
                    self._current_sub_skill = "ApproachBall"
            else: 
                self._current_sub_skill = "ApproachBall" 

    def _tick(self):
        if self._current_sub_skill == "ApproachBall":
            self._tick_sub_skill(aim_point = self.target)
        else:
            self._tick_sub_skill()
