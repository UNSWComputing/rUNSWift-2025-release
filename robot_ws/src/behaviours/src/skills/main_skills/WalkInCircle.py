from src.skills.Skill import Skill
from src.skills.leaf_skills.Walk import Walk


class WalkInCircle(Skill):
    """
    Description:
    Skill to walk in a circle.
    """

    def _initialise_sub_skills(self):
        self._sub_skills = {"Walk": Walk(self)}

    def _reset(self):
        self._current_sub_skill = "Walk"

    def _tick(self, forward=150, left=0, turn=0.3, speed=1.0):
        self._tick_sub_skill(forward, left, turn, speed)

