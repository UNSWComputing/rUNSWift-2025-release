import math

from src.skills.Skill import Skill
from src.skills.leaf_skills.Stand import Stand
from src.skills.main_skills.FollowBall import FollowBall
from src.skills.roles.Goalie import Goalie

from src.skills.head_skills.HeadTrackBallPixel import head_track_ball_pixel

class Demo(Skill):
    """
    Description:
    Run when robot is not connected to gamecontroller. i.e. when it is in "demo mode"
    """

    def _initialise_sub_skills(self):
        self._sub_skills = {"Stand": Stand(self), "FollowBall": FollowBall(self), "Goalie": Goalie(self)}

    def _reset(self):
        self._current_sub_skill = "Goalie"

    def _transition(self):
        pass

    def _tick(self):
        self._tick_sub_skill()
