from src.skills.Skill import Skill

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
from src.skills.main_skills.BallAdjustment import BallAdjustment
from src.skills.main_skills.FollowBall import FollowBall
from src.skills.main_skills.KickOffSetPlay import KickOffSetPlay
from src.skills.main_skills.Playing import Playing
from src.skills.main_skills.Ready import Ready
from src.skills.main_skills.SoloFindBall import SoloFindBall
from src.skills.main_skills.WalkInCircle import WalkInCircle
from src.skills.main_skills.WalkToPoint import WalkToPoint
from src.skills.main_skills.WalkToPose import WalkToPose

# Delete as unnecessary (utils)
import src.utils.Ball
import src.utils.Constants
import src.utils.FieldCheck
import src.utils.FieldGeometry
import src.utils.Hysteresis
import src.utils.MathUtil
import src.utils.TeamInfo
import src.utils.Timer

# Delete as unnecessary (utils)
# from src.utils.Ball import ...
# from src.utils.Constants import ...
# from src.utils.FieldCheck import ...
# from src.utils.FieldGeometry import ...
# from src.utils.Hysteresis import ...
# from src.utils.MathUtil import ...
# from src.utils.TeamInfo import ...
# from src.utils.Timer import ...


class Template(Skill):
    """
    Description:
    Tell us what your skill does!
    """

    def _initialise_sub_skills(self):
        self._sub_skills = {"Stand": Stand(self)}

    def _reset(self):
        self._current_sub_skill = "Stand"

    def _transition(self):
        pass

    def _tick(self):
        self._tick_sub_skill()
