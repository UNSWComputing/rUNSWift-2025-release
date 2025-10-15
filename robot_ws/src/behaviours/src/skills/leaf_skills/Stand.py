from src.skills.Skill import Skill
from runswift_interfaces.msg import BodyCommand


class Stand(Skill):
    """
    Description:
    A skill associated with standing (migrated to ROS2)
    """

    def _tick(self):
        self.ctx.blackboard.motion_command.body_command.action_type = BodyCommand.STAND
        self.ctx.blackboard.motion_command.body_command.twist.linear.x = 0.0
        self.ctx.blackboard.motion_command.body_command.twist.linear.y = 0.0
        self.ctx.blackboard.motion_command.body_command.twist.angular.z = 0.0

