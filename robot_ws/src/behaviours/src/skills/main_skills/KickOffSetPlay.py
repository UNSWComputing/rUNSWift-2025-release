import math
import subprocess

from geometry_msgs.msg import Point
from src.datamodels.RobotPose import RobotPose
from src.skills.Skill import Skill
from src.skills.leaf_skills.Stand import Stand
from src.skills.main_skills.ApproachBall import ApproachBall
from src.skills.main_skills.WalkToPose import WalkToPose
from src.utils.Constants import HALF_FIELD_LENGTH
from src.utils.Timer import Timer


class KickOffSetPlay(Skill):
    """
    Description:
    Ideal kick off play: Player 1 will kick the ball to player 2 and
    player 2 will shoot the ball to the goal
    """

    # Define posotion targets
    GOAL = RobotPose(Point(x=float(HALF_FIELD_LENGTH), y=0.0))  # The enemy goal position
    # PASS_TO_PLAYER_2 = RobotPose(Point(x=1500.0, y=2000.0))  # Point just in front of player 2
    PASS_TO_PLAYER_2 = GOAL  # Point just in front of player 2
    PLAYER_2_INTERCEPTION = RobotPose(Point(x=300.0, y=2500.0), 5.7)  # Point just in front of player 2

    # Define timers for set play
    # After this time, the pass should be complete and this skill is finished for player 1
    PLAYER_1_FINISHED_PASS_TIME = 15
    PLAYER_1_FINISHED_PASS_TIME_MAX = PLAYER_1_FINISHED_PASS_TIME + 5
    # If a goal is not scored in this time, this set play is not longer idea, and we abort
    ABORT_TIMEOUT = 120

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Stand": Stand(self),
            "ApproachBall": ApproachBall(self),
            "WalkToPose": WalkToPose(self),
        }

    def _reset(self):
        self._current_sub_skill = "Stand"
        self.started = False
        self._timer = Timer()

    def _transition(self):
        if not self.started:
            self._timer.restart()
            self.started = True
        player_number = self.ctx.blackboard.player_info.player_number
        if player_number == 1:
            self._player_1_transition()
        elif player_number == 2:
            self._player_2_transition()
        else:
            self._current_sub_skill = "Stand"

    def _check_finished(self, *args, **kwargs):
        if self._timer.elapsed_seconds() > self.ABORT_TIMEOUT:
            return True
        if (
            self.ctx.blackboard.player_info.player_number == 1
            and self._timer.elapsed_seconds() > self.PLAYER_1_FINISHED_PASS_TIME_MAX
        ):
            return True
        return False

    def _tick(self):
        if self._current_sub_skill == "ApproachBall":
            self._tick_sub_skill(aim_point=self._kick_aim_point)
        elif self._current_sub_skill == "WalkToPose":
            self._tick_sub_skill(final_pos=self.PLAYER_2_INTERCEPTION)
        else:
            self._tick_sub_skill()

    def _player_1_transition(self):
        if self._timer.elapsed_seconds() < self.PLAYER_1_FINISHED_PASS_TIME_MAX:
            self._current_sub_skill = "ApproachBall"
            self._kick_aim_point = self.PASS_TO_PLAYER_2
            # subprocess.Popen(["flite", "-t", "Passing"])
        else:
            self.current_sub_skill = "Stand"

    def _player_2_transition(self):
        # if (
        #     self._timer.elapsed_seconds() < self.PLAYER_1_FINISHED_PASS_TIME
        #     and not self._sub_skill_finished(skill="WalkToPose", final_pos=self.PLAYER_2_INTERCEPTION)
        # ):
        #     self._current_sub_skill = "WalkToPose"
        #     # subprocess.Popen(["flite", "-t", "Receiving"])
        if self._timer.elapsed_seconds() < self.PLAYER_1_FINISHED_PASS_TIME:
            self._current_sub_skill = "Stand"
        else:
            self._current_sub_skill = "ApproachBall"
            self._kick_aim_point = self.GOAL
            # subprocess.Popen(["flite", "-t", "Shooting"])
