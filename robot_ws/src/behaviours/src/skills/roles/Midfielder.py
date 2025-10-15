from src.skills.Skill import Skill

from src.datamodels.RobotPose import RobotPose

from src.skills.leaf_skills.Stand import Stand

from src.skills.main_skills.ApproachBall import ApproachBall
from src.skills.main_skills.WalkToPose import WalkToPose

from src.skills.head_skills.HeadScanBall import head_scan_ball
from src.skills.head_skills.HeadTrackBallPixel import head_track_ball_pixel

from src.utils.Ball import getAbsoluteBall, getVisionBall, getBall
from src.utils.TeamInfo import getClosestRobotToTeamBall, getTeamBall
from src.utils.Constants import FIELD_LENGTH, HALF_FIELD_WIDTH

from runswift_interfaces.msg import BehavioursRobotInfo, SeBallsAbsolute, SeBall

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

class Midfielder(Skill):
    """
    Exact same as defender but x pushed closer towards the centre circe, spread
    spread the robots further out to form a Y shape.
    """

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Stand": Stand(self),
            "ApproachBall": ApproachBall(self),
            "WalkToPose": WalkToPose(self),
        }

    def _reset(self):
        self._current_sub_skill = "Stand"
        self._default_positions = [
            RobotPose(point = Point(x=-FIELD_LENGTH/4, y=HALF_FIELD_WIDTH / 1.5), heading = 0.0),
            RobotPose(point = Point(x=-FIELD_LENGTH/4, y=-HALF_FIELD_WIDTH / 1.5), heading = 0.0),
            RobotPose(point = Point(x=-FIELD_LENGTH/4, y=0.0), heading = 0.0),
        ]
        self._my_default_position = None
        self.BALL_CLOSE_THRESHOLD = 1500

    def _transition(self):
        ball = BallMeasurement.from_balls_absolute_msg(self.ctx.blackboard.ball_world)
        if ball is None:
            self._current_sub_skill = "WalkToPose"
        else:
            is_ball_close = (ball.x - self.ctx.blackboard.robot_world.point.x) <= self.BALL_CLOSE_THRESHOLD and abs(ball.y - self.ctx.blackboard.robot_world.point.y) <= self.BALL_CLOSE_THRESHOLD
            if is_ball_close:
                self._current_sub_skill = "ApproachBall"
            else:
                self._current_sub_skill = "WalkToPose"

        # Choose a default position to fall back to opposite from other defender
        for robot in self.ctx.blackboard.robot_comms.values():
            if robot.player_role == BehavioursRobotInfo.DEFENDER and robot.player_number != self.ctx.blackboard.player_info.player_number:
                if robot.robot_pos_y < 0:
                    self._my_default_position = self._default_positions[0]
                else:
                    self._my_default_position = self._default_positions[1]
        if self._my_default_position is None:
            self._my_default_position = self._default_positions[2]

        # Track ball if we aren't going for it
        if self._current_sub_skill == "WalkToPose":
            if getVisionBall(self.ctx.blackboard) is None:
                head_scan_ball(self.ctx.blackboard)
            else:
                head_track_ball_pixel(self.ctx.blackboard)

    def _tick(self):
        if self._current_sub_skill == "WalkToPose":
            self._tick_sub_skill(final_pos=self._my_default_position)
        else:
            self._tick_sub_skill()
