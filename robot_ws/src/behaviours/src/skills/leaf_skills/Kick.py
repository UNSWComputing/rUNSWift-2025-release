import subprocess
from src.utils.MathUtil import clamp
from src.utils.FieldGeometry import ENEMY_GOAL_BEHIND_CENTER
from src.utils.Timer import Timer
from src.skills.leaf_skills.Stand import Stand
from src.skills.Skill import Skill
from runswift_interfaces.msg import BodyCommand
from src.datamodels.RobotPose import RobotPose
from geometry_msgs.msg import Point
from src.utils.MathUtil import distancePointToPoint
from src.utils.Timer import Timer


class Kick(Skill):

    KICK_TIME = 1

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Stand": Stand(self)
        }

    def _reset(self):
        self._kicked = False
        self._timer = Timer()
        self._started = False

    def _tick(
        self,
        foot="left",
        distance=None
    ):

        # subprocess.Popen(["flite", "-t", "Kicking"])
        power = self._calculate_power(distance)
        self.ctx.blackboard.motion_command.body_command.action_type = BodyCommand.KICK
        self.ctx.blackboard.motion_command.body_command.power = power
        self.ctx.blackboard.motion_command.body_command.foot = foot == "right"

    def _calculate_power(self, distance):
        if distance is None:
            # Max power for default kick
            return 1.0
        else:
            if distance <= 2000:
                return 0.0
            elif distance >= 4275:
                return 1.0
            else:
                return 0.5

            # Otherwise interpolate based on kickTarget

            # Numbers derived with... SCIENCE!
            # (actually derived by making the robot with different powers,
            #  and fitting a linear function)
            # return clamp(0.00029648 * target_distance - 0.63829, 0.0, 1.0)
