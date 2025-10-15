from src.skills.Skill import Skill
import traceback
from src.datamodels.RobotPose import RobotPose

from src.skills.leaf_skills.Stand import Stand

from src.skills.main_skills.ApproachBall import ApproachBall
from src.skills.main_skills.WalkToPose import WalkToPose
from src.skills.main_skills.SoloFindBall import SoloFindBall

from src.skills.head_skills.HeadScanBall import head_scan_ball
from src.skills.head_skills.HeadTrackBallPixel import head_track_ball_pixel

from src.utils.Ball import getAbsoluteBall, BallMeasurement, getVisionBall, getBall

from src.utils.TeamInfo import getClosestRobotToTeamBall, getTeamBall
from src.utils.Constants import PENALTY_CROSS_ABS_X, HALF_FIELD_WIDTH

from runswift_interfaces.msg import BehavioursRobotInfo
from runswift_interfaces.msg import SeBall, SeBallsAbsolute

from geometry_msgs.msg import Point

from dataclasses import dataclass

@dataclass
class BallMeasurement:
    timestamp: float
    x: float
    y: float

    @staticmethod
    def from_balls_absolute_msg(msg: SeBallsAbsolute) -> "BallMeasurement | None":
        ball: SeBall | None = getBall(msg.balls_absolute)

        if ball is None:
            return None

        return BallMeasurement(
            x = ball.pos_x,
            y = ball.pos_y,
            timestamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 10e-9
        )


class Defender(Skill):
    """
    Description:
    Simple defender skill to build off of. Go for ball if we are closest to ball, otherwise go to default position
    """

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Stand": Stand(self),
            "ApproachBall": ApproachBall(self),
            "WalkToPose": WalkToPose(self),
            "SoloFindBall": SoloFindBall(self),
        }

    def _reset(self):
        self._current_sub_skill = "Stand"
        self._default_positions = [
            RobotPose(point = Point(x=-2500.0, y=HALF_FIELD_WIDTH / 4), heading = 0.0),
            RobotPose(point = Point(x=-2500.0, y=-HALF_FIELD_WIDTH / 4), heading = 0.0),
            RobotPose(point = Point(x=-2500.0, y=0.0), heading = 0.0),
        ]
        self._my_default_position = None
        self.BALL_CLOSE_THRESHOLD = 4000

    def _transition(self):
        ball = BallMeasurement.from_balls_absolute_msg(self.ctx.blackboard.ball_world)
        if ball is None:
            self._current_sub_skill = "WalkToPose"
        else:
            self._current_sub_skill = "ApproachBall"
            # is_ball_close = (ball.x - self.ctx.blackboard.robot_world.point.x) <= self.BALL_CLOSE_THRESHOLD and abs(ball.y - self.ctx.blackboard.robot_world.point.y) <= self.BALL_CLOSE_THRESHOLD
            # if is_ball_close:
            #     self._current_sub_skill = "ApproachBall"
            # else:
            #     self._current_sub_skill = "WalkToPose"

        # Choose a default position to fall back to opposite from other defender
        # for robot in self.ctx.blackboard.robot_comms.values():
        #     if robot.player_role == BehavioursRobotInfo.DEFENDER and robot.player_number != self.ctx.blackboard.player_info.player_number:
        #         if robot.robot_pos_y < 0:
        #             self._my_default_position = self._default_positions[0]
        #         else:
        #             self._my_default_position = self._default_positions[1]
        # if self._my_default_position is None:
        #     self._my_default_position = self._default_positions[2]

        if self.ctx.blackboard.player_info.player_number == 2:
            self._my_default_position = self._default_positions[1]
        elif self.ctx.blackboard.player_info.player_number == 3:
            self._my_default_position = self._default_positions[0]

        # Track ball if we aren't going for it
        if self._current_sub_skill == "WalkToPose":
            if getVisionBall(self.ctx.blackboard) is None:
                head_scan_ball(self.ctx.blackboard)
            else:
                head_track_ball_pixel(self.ctx.blackboard)
        if self._sub_skill_finished(skill="WalkToPose", final_pos=self._my_default_position):
            self._current_sub_skill = "SoloFindBall"

    def _tick(self):
        if self._current_sub_skill == "WalkToPose":
            self._tick_sub_skill(final_pos=self._my_default_position)
        else:
            self._tick_sub_skill()
