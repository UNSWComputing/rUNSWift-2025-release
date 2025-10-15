from math import pi
from src.skills.Skill import Skill
from src.utils.Constants import GOAL_BOX_WIDTH, HALF_FIELD_WIDTH, HALF_FIELD_LENGTH, PENALTY_AREA_LENGTH, PENALTY_CROSS_ABS_X, CENTER_CIRCLE_DIAMETER, PENALTY_AREA_LENGTH, GOAL_BOX_WIDTH
from src.skills.main_skills.WalkToPose import WalkToPose
from src.datamodels.RobotPose import RobotPose
from geometry_msgs.msg import Point


class Ready(Skill):
    """
    Description:
    Skill associated with robots walking to their initial starting
    positions based on their player number.
    """

    def _initialise_sub_skills(self):
        self._sub_skills = {"WalkToPose": WalkToPose(self)}

    def _reset(self):
        self.attack_positions = [
            RobotPose(),     # Placeholder for player number = 0
            RobotPose(point = Point(x=-HALF_FIELD_LENGTH + 300.0, y=0.0), heading = 0.0),
            RobotPose(point = Point(x=-PENALTY_CROSS_ABS_X, y=-HALF_FIELD_WIDTH / 4), heading = 0.0),
            RobotPose(point = Point(x=-PENALTY_CROSS_ABS_X, y=HALF_FIELD_WIDTH / 4), heading = 0.0),
            RobotPose(point = Point(x=-400.0, y=-400.0), heading = pi / 4),
            RobotPose(point = Point(x=-300.0, y=HALF_FIELD_WIDTH / 2), heading = 0.0),
        ]
        self.defence_positions = [
            RobotPose(),
            RobotPose(point = Point(x=-HALF_FIELD_LENGTH + 300.0, y=0.0), heading = 0.0),  # goalkeeper
            RobotPose(point = Point(x=-HALF_FIELD_LENGTH + PENALTY_AREA_LENGTH - 100, y=-(GOAL_BOX_WIDTH * 0.5) + 60.0), heading = 0.0),  # defender  # noqa
            RobotPose(point = Point(x=-HALF_FIELD_LENGTH + PENALTY_AREA_LENGTH - 70, y=(GOAL_BOX_WIDTH * 0.5) - 60.0), heading = 0.0),  # defender2  # noqa
            RobotPose(point = Point(x=-HALF_FIELD_LENGTH * 0.25, y=-HALF_FIELD_WIDTH * 0.75), heading = 0.0),  # midfield1  # noqa
            RobotPose(point = Point(x=-HALF_FIELD_LENGTH * 0.20, y=HALF_FIELD_WIDTH * 0.75), heading = 0.0)  # kickoff player  # noqa
        ]

        self._current_sub_skill = "WalkToPose"

    def _tick(self):
        player_number = self.ctx.blackboard.player_info.player_number
        if self.ctx.blackboard.gameinfo is None:
            self._tick_sub_skill(final_pos=self.attack_positions[player_number])
        elif self.ctx.blackboard.gameinfo.kicking_team == 18:
            self._tick_sub_skill(final_pos=self.attack_positions[player_number])
        else:
            self._tick_sub_skill(final_pos=self.defence_positions[player_number])
