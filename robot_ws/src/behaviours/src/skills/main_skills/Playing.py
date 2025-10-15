from src.skills.Skill import Skill
from src.skills.main_skills.Kickoff import Kickoff
from src.skills.leaf_skills.Stand import Stand
from src.skills.leaf_skills.Walk import Walk
from src.datamodels.RobotPose import RobotPose
from geometry_msgs.msg import Point
from src.skills.leaf_skills.Stand import Stand
from src.skills.leaf_skills.Kick import Kick
from src.skills.leaf_skills.Sit import Sit
from src.skills.main_skills.ApproachBall import ApproachBall
from src.skills.main_skills.Kickin import Kickin
from src.skills.roles.Attacker import Attacker
from src.skills.roles.Goalie import Goalie
from src.skills.roles.Defender import Defender
from src.skills.roles.Midfielder import Midfielder
from src.skills.main_skills.WalkToPose import WalkToPose
from src.skills.main_skills.GoalieKick import GoalieKick
from src.skills.head_skills.HeadScanBall import head_scan_ball
from runswift_interfaces.msg import CommsRCGCD, VisionPipelineSwitchAsk
from src.datamodels.RobotPose import RobotPose
from src.utils.Constants import PENALTY_CROSS_ABS_X
from geometry_msgs.msg import Point
from math import pi
from src.skills.main_skills.CheckRef import CheckRef
import subprocess


from runswift_interfaces.msg import CommsRCGCD

from src.utils.Timer import Timer
from src.utils.Ball import getAbsoluteBall, getVisionBall


class Playing(Skill):
    """
    Description:
    What the robot does in playing state.
    """

    RUNSWIFT = 18
    NOT_RUNSWIFT = 0
    KICKOFF_TIME = 15

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Stand": Stand(self),
            "Kickoff": Kickoff(self),
            "Kick": Kick(self),
            "Sit": Sit(self),
            "GoalieKick": GoalieKick(self),
            "WalkToPose": WalkToPose(self),
            "ApproachBall": ApproachBall(self),
            "Attacker": Attacker(self),
            "Defender": Defender(self),
            "CheckRef": CheckRef(self),
            "Walk": Walk(self),
            "Goalkeeper": Goalie(self),
            "Midfielder": Midfielder(self),
            "Kickin": Kickin(self),
        }

    def _reset(self):
        self._current_sub_skill = "Stand"
        self._timer = Timer()
        self._started = False
        self._unpenalised_timer = False
        self._played_penalised_sound = False

    def _transition(self):
        if not self._started:
            self._timer.restart()
            self._started = True

    def _tick(self):
        # Penalised logic
        if self.ctx.blackboard.penalised:
            self._current_sub_skill = "Stand"
            self._unpenalised_timer = False
            self._tick_sub_skill()
            if not self._played_penalised_sound:
                subprocess.Popen(["aplay", "/home/nao/bin/penalised.wav"])
                self._played_penalised_sound = True
            return

        # Unpenalised logic
        if self.ctx.blackboard.unpenalised and self._unpenalised_timer == False:
            self._timer.restart()
            self._unpenalised_timer = True

        if self.ctx.blackboard.unpenalised and self._timer.elapsed_seconds() < 15:
            self._current_sub_skill = "Walk"
            self._tick_sub_skill(forward=250.0)
            head_scan_ball(self.ctx.blackboard)
            self._played_penalised_sound = False
            return

        # Playing logic
        if self._timer.elapsed_seconds() < self.KICKOFF_TIME:
            self._current_sub_skill = "Kickoff"
        elif self.ctx.blackboard.player_info.player_number == 5 or self.ctx.blackboard.player_info.player_number == 4:
            self._current_sub_skill = "Attacker"
        elif self.ctx.blackboard.player_info.player_number == 3 or self.ctx.blackboard.player_info.player_number == 2:
            self._current_sub_skill = "Defender"
        else:
            self._current_sub_skill = "Goalkeeper"

        # Set play logic
        # only handle these set‐plays
        SET_PLAYS = {
            CommsRCGCD.SET_PLAY_GOAL_KICK,
            CommsRCGCD.SET_PLAY_PUSHING_FREE_KICK,
            CommsRCGCD.SET_PLAY_CORNER_KICK,
            CommsRCGCD.SET_PLAY_KICK_IN,
        }

        if self.ctx.blackboard.gameinfo.set_play in SET_PLAYS:
            is_goalie = self.ctx.blackboard.player_info.player_number == 1
            is_goal_kick = self.ctx.blackboard.gameinfo.set_play == CommsRCGCD.SET_PLAY_GOAL_KICK

            if not is_goalie:
                # if not goalie check if we have already seen the signal
                if self.ctx.blackboard.kicking_team is None:
                    # if we haven't look for ref
                    self._current_sub_skill = "ApproachBall"
                else:
                    self._current_sub_skill = "ApproachBall"
            else:
                # goalie, check for if it is a goal kick and we haven't already kicked
                if is_goal_kick:
                    self._current_sub_skill = "ApproachBall"
                    # goalie kick
                else:
                    # after the kick (or for any other special play)
                    self._current_sub_skill = "Goalkeeper"
        else:
            # if not in any set play return the kicking team to None and turn off ref

            referee_signal_msg = VisionPipelineSwitchAsk()
            referee_signal_msg.is_ref = False

            # doesnt matter
            referee_signal_msg.check_mode = 2
            self.ctx.blackboard.kicking_team = None
            self.ctx.blackboard.referee_signal = referee_signal_msg

        # Tell other robots when we stop going for ball
        if "ApproachBall" not in self.ctx.behaviour_hierarchy:
            self.ctx.blackboard.my_ball = False
        self._tick_sub_skill()
