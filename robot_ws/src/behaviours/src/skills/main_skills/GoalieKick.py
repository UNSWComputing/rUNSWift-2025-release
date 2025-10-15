from geometry_msgs.msg import Point
from src.skills.Skill import Skill
from src.skills.main_skills.ApproachBall import ApproachBall
from src.skills.main_skills.WalkToPose import WalkToPose
from src.skills.leaf_skills.Kick import Kick
from src.skills.leaf_skills.Stand import Stand
from src.utils.Ball import getAbsoluteBall
from src.utils.Constants import PENALTY_CROSS_ABS_X
from src.datamodels.RobotPose import RobotPose
import math


class GoalieKick(Skill):
    """
    Description:
    Defending robot goes to the outer rectangle of the goal post and kicks it to one of the attackers
    (for now) and then one of the three attackers kicks it to the opponent's goal

    Assumptions:   - the attacking robots know where to turn to score a goal
                - the kick power is strong enough to the point where it will reach to the other end of the field
                - two attackers and three defenders
                - if the ball is in one of the attackers regions then they pick it up to score a goal
    """

    # Position robot goes to at start to find ball
    LOOK_FOR_BALL_POS = RobotPose(Point(x=-(PENALTY_CROSS_ABS_X - 100), y=0.0), heading=-(math.pi))

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "ApproachBall": ApproachBall(self),
            "Stand": Stand(self),
        }

    def _reset(self):
        self._current_sub_skill = "Stand"
        self.the_chosen_one = None  # Chosen attacker to pass to
        self.ball_found = False
        # Define 2 attacker positions
        self._attacker_positions = [
            RobotPose(Point(x=4000.0, y=-1500.0)),
            RobotPose(Point(x=4000.0, y=1500.0)),
        ]

    def _tick(self):
        self._current_sub_skill = "ApproachBall"
        self._tick_sub_skill()
        return
        target = self._get_best_attacker()
        if getAbsoluteBall(self.ctx.blackboard) is not None and target is not None:
            self._current_sub_skill = "ApproachBall"

        if self._current_sub_skill == "ApproachBall":
            self._tick_sub_skill(aim_point=target)
        else:
            self._tick_sub_skill()

    def _get_best_attacker(self):
        if self.the_chosen_one is not None:
            return self.the_chosen_one

        ball_pos = getAbsoluteBall(self.ctx.blackboard)
        if ball_pos is None:
            self.the_chosen_one = self._attacker_positions[0]
            return self.the_chosen_one
        attacker_1 = self._attacker_positions[0]
        attacker_2 = self._attacker_positions[1]

        dx1 = attacker_1.point.x - ball_pos.pos_x
        dy1 = attacker_1.point.y - ball_pos.pos_y
        dist_1 = math.sqrt(dx1 * dx1 + dy1 * dy1)

        dx2 = attacker_2.point.x - ball_pos.pos_x
        dy2 = attacker_2.point.y - ball_pos.pos_y
        dist_2 = math.sqrt(dx2 * dx2 + dy2 * dy2)

        if dist_1 <= dist_2:
            self.the_chosen_one = attacker_1
        else:
            self.the_chosen_one = attacker_2

        return self.the_chosen_one
