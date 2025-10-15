class Skill:
    """
    Definitions of a Skill.
    "Leaf" - A node with no children. Publishes an actioncommand.
    "Internal node" - A node with at least one child. Calls other subskills.
    """

    def __init__(self, parent=None, ctx=None):
        """__init__

        Override of constructor.

        If a parent class exists, copy context from parent. Otherwise, context
        should be provided as an argument. (Root Skill case)

        Initialise:
        - self._current_sub_skill
        - self._sub_skills
        - Other attributes as required (in _reset())

        DO NOT Override this.
        """
        if parent is not None:
            self.parent = parent
            self.ctx = self.parent.ctx
        else:
            self.ctx = ctx

        self._current_sub_skill = None
        self._initialise_sub_skills()
        self._reset()

    def _initialise_sub_skills(self):
        """_initialise_sub_skills

        Private function that initialises a map of sub skills.

        Overriding:
        - Leaf: Don't override
        - Internal Node: Do override
        """
        self._sub_skills = None

    def _reset(self):
        """_reset

        Private function to initialise and reset class attributes.

        Overriding:
        - Leaf: Optional
        - Internal Node: Optional
        """
        pass

    def _transition(self):
        """_transition.

        Private function for switching between sub skills.

        Overriding:
        - Leaf: Don't override
        - Internal Node: Do override
        """
        pass

    def _tick(self, *args, **kwargs):
        """_tick.

        Private function to process information, calculate parameters to pass
        to sub skills.

        Overriding:
        - Leaf: Do override
        - Internal Node: Optional. If overriden, ensure self._tick_sub_skill()
                         is called.
        """
        if self._current_sub_skill is not None:
            self._tick_sub_skill()

    def _check_finished(self, *args, **kwargs):
        """_check_finished

        Private function for checking if the current skill is finished.

        Overriding:
        - Leaf: Don't override
        - Internal Node: Do override
        """
        return False

    def reset(self):
        """reset.

        Public function to call _reset() and reset() of current sub skill.

        DO NOT Override this.
        """

        self._reset()

        if self._current_sub_skill is not None:
            self._sub_skills[self._current_sub_skill].reset()

    def tick(self, *args, **kwargs):
        """tick.

        Public function to:
        - Leaf: call _tick()
        - Internal Node: Call _transition(), reset skills if necessary,
                         then calls _tick()

        DO NOT Override this.
        """

        self._update_behaviour_hierarchy()

        if self._current_sub_skill is not None:
            previous_sub_skill = self._current_sub_skill
            self._transition()

            if self._current_sub_skill != previous_sub_skill:
                self._sub_skills[self._current_sub_skill].reset()

        self._tick(*args, **kwargs)

    def _tick_sub_skill(self, *args, **kwargs):
        """_tick_sub_skill

        Private function that ticks the current sub skill with provided
        arguments.

        DO NOT Override this.
        """
        self._sub_skills[self._current_sub_skill].tick(*args, **kwargs)

    def _sub_skill_finished(self, *args, **kwargs):
        """_sub_skill_finished

        Private function that checks the finished condition of current
        sub skill.

        DO NOT Override this.
        """

        skill = kwargs.pop("skill", self._current_sub_skill)
        return self._sub_skills[skill]._check_finished(*args, **kwargs)

    def _update_behaviour_hierarchy(self):
        """_update_behaviour_hierarchy

        Private function to update the behaviour hierarchy to reflect the current
        skill and its parents.
        """
        hierarchy = []
        current_skill = self

        while current_skill is not None:
            hierarchy.append(current_skill.__class__.__name__)
            if hasattr(current_skill, "_current_sub_skill") and current_skill._current_sub_skill is not None:
                current_skill = current_skill._sub_skills[current_skill._current_sub_skill]
            else:
                current_skill = None

        self.ctx.behaviour_hierarchy = ".".join(hierarchy)
