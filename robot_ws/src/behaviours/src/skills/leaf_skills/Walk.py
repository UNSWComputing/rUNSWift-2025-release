from src.skills.Skill import Skill
from runswift_interfaces.msg import BodyCommand


class Walk(Skill):
    """
    Description:
    Skill associated with directly creating the walk actioncommand.
    SPEED should be increased for serious games.
    Currently there are issues with low SPEED values on v5 robots.
    """

    # Range between 0.0 and 1.0. Use 0 in lab and 1 in serious games. v5 robots
    # do not work with low speed - 0.5 is functional. v6 need further testing.
    #
    # Lower speeds can be good sometimes, e.g. if the carpet/astroturf is really slippery
    # e.g. in Thailand 2022 a value of 0.4 on v6s was used
    # as any higher and the robots just fell over all the time
    # if the robot falls over in regular walking, then it'll be far slower going from point A to B
    # for logic when no config set see default value of walk_speed in options.cpp

    def _tick(self, forward=0, left=0, turn=0, speed=1.0, allow_shuffle=True, cap_speed=True):
        self.ctx.blackboard.motion_command.body_command.action_type = BodyCommand.WALK
        self.ctx.blackboard.motion_command.body_command.twist.linear.x = float(forward)
        self.ctx.blackboard.motion_command.body_command.twist.linear.y = float(left)
        self.ctx.blackboard.motion_command.body_command.twist.angular.z = float(turn)
        # self.ctx.blackboard.motion_command.body_command.use_shuffle = allow_shuffle
