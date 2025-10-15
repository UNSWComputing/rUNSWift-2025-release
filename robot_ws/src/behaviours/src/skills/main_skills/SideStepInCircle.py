from src.skills.Skill import Skill
from src.skills.leaf_skills.Walk import Walk


class SideStepInCircle(Skill):
    """
    Description:
    Skill to side step in a circle.
    """

    def _initialise_sub_skills(self):
        self._sub_skills = {"Walk": Walk(self)}

    def _reset(self):
        self._current_sub_skill = "Walk"

    def _tick(self, radius, speed):
        self._tick_sub_skill(forward=0, left=speed, turn=-speed/radius)

