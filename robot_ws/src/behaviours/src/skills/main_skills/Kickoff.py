from geometry_msgs.msg import Point
from src.datamodels.RobotPose import RobotPose
from src.skills.Skill import Skill
from src.skills.leaf_skills.Stand import Stand
from src.skills.main_skills.ApproachBall import ApproachBall
from src.skills.main_skills.WalkToPose import WalkToPose
from src.utils.FieldGeometry import ENEMY_GOAL_BEHIND_CENTER
from math import pi

class Kickoff(Skill):
    """
    Description:
    Kickoff play. Player in centre circle will pass to player on wing.
    Player on wing will shoot the ball to the goal
    """
    # rUNSWift team number
    RUNSWIFT = 18

    # Point just in front of player on wing
    PASS_TO_WING = RobotPose(Point(x=1500.0, y=2000.0))

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Stand": Stand(self),
            "ApproachBall": ApproachBall(self),
            "WalkToPose": WalkToPose(self),
        }

    def _reset(self):
        self._current_sub_skill = "Stand"
        self._kick_aim_point = ENEMY_GOAL_BEHIND_CENTER

    def _transition(self):
        player_number = self.ctx.blackboard.player_info.player_number
        # player 5 is in centre circle
        if player_number == 4:
            self._player_5_transition()
        # player 4 is on the wing
        elif player_number == 5:
            self._player_4_transition()
        else:
            self._current_sub_skill = "Stand"

    def _tick(self):
        if self._current_sub_skill == "ApproachBall":
            self._tick_sub_skill(aim_point=self._kick_aim_point)
        elif self._current_sub_skill == "WalkToPose":
            final_pos = RobotPose(point=Point(x=800.0, y=1500.0), heading=-pi/6)
            self._tick_sub_skill(final_pos=final_pos)
        else:
            self._tick_sub_skill()

    def _player_5_transition(self):
        self._current_sub_skill = "ApproachBall"
        self._kick_aim_point = self.PASS_TO_WING

    def _player_4_transition(self):
        if not self.ctx.blackboard.kick_success:
            self._current_sub_skill = "Stand"
        elif self.ctx.blackboard.vision_ball is not None:
            self._current_sub_skill = "ApproachBall"
            self._kick_aim_point = ENEMY_GOAL_BEHIND_CENTER
        else:
            self._current_sub_skill = "WalkToPose"

