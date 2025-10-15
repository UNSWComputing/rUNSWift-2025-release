from src.skills.Skill import Skill
from runswift_interfaces.msg import BodyCommand

class Sit(Skill):
    """
    Decription:
    Skill for the robot to sit.
    """

    def _tick(self):
        self.ctx.blackboard.motion_command.body_command.action_type = BodyCommand.SIT
        self.ctx.blackboard.motion_command.body_command.twist.linear.x = 0.0
        self.ctx.blackboard.motion_command.body_command.twist.linear.y = 0.0
        self.ctx.blackboard.motion_command.body_command.twist.angular.z = 0.0

