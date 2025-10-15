import subprocess
from src.skills.Skill import Skill
from src.skills.leaf_skills.Stand import Stand
from runswift_interfaces.msg import BodyCommand


class Dive(Skill):
    """
    Description:
    Leaf skill for performing a dive. The dive can be directed to the left or right.
    """

    def _reset(self):
        self.flite_ran = False

    def _initialise_sub_skills(self):
        # Optional fallback sub-skill.
        self._sub_skills = {"Stand": Stand(self)}

    def _tick(self, direction="left"):
        if not self.flite_ran:
            self.ctx.get_logger().error(f"I AM GONNA DIVE {direction}")
            subprocess.Popen(["flite", "-t", f"I AM GONNA DIVE {direction}"])
            self.flite_ran = True

        if direction == "centre":
            self.ctx.blackboard.motion_command.body_command.action_type = BodyCommand.GOALIE_CENTRE
        elif direction == "left":
            self.ctx.blackboard.motion_command.body_command.action_type = BodyCommand.GOALIE_DIVE_LEFT
        else:
            self.ctx.blackboard.motion_command.body_command.action_type = BodyCommand.GOALIE_DIVE_RIGHT
