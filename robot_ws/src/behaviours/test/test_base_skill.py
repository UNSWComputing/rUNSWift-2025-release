from src.skills.Skill import Skill

class TestSkill:
    """
    Test suite for the Skill superclass to verify core functionalities.
    """

    GREEN = "\033[92m"  # ANSI code for green
    RED = "\033[91m"    # ANSI code for red
    RESET = "\033[0m"   # ANSI code to reset color

    def __init__(self):
        """Initialize test context."""
        self.ctx = self.TestContext()
        self.root_skill = self.MockInternalSkill(None, self.ctx)
        self.leaf_skill = self.MockLeafSkill(self.root_skill)

    class TestContext:
        """
        A mock context object with a behaviour_hierarchy attribute for testing.
        """
        def __init__(self):
            self.behaviour_hierarchy = ""

    class MockLeafSkill(Skill):
        """
        A mock leaf skill for testing.
        """
        def _tick(self, *args, **kwargs):
            self.ctx.behaviour_hierarchy += "LeafTick."

    class MockLeafSkill2(Skill):
        """
        A mock leaf skill for testing.
        """
        def _tick(self, *args, **kwargs):
            self.ctx.behaviour_hierarchy += "LeafTick2."

    class MockInternalSkill(Skill):
        """
        A mock internal skill with two child skills for testing transitions.
        """
        def _initialise_sub_skills(self):
            self._sub_skills = {
                "child_1": TestSkill.MockLeafSkill(self),
                "child_2": TestSkill.MockLeafSkill2(self)
            }
            self._current_sub_skill = "child_1"

        def _transition(self):
            if self._current_sub_skill == "child_1":
                self._current_sub_skill = "child_2"
            else:
                self._current_sub_skill = "child_1"

        def _reset(self):
            self.ctx.behaviour_hierarchy = ""

    def print_result(self, test_name, success):
        """Print the test result with color."""
        if success:
            print(f"{self.GREEN}[SUCCESS] {test_name}{self.RESET}")
        else:
            print(f"{self.RED}[FAIL] {test_name}{self.RESET}")

    def test_leaf_tick(self):
        """
        Test that a leaf skill correctly appends its tick behavior to the hierarchy.
        """
        self.ctx.behaviour_hierarchy = ""
        self.leaf_skill.tick()
        assert self.ctx.behaviour_hierarchy == "MockLeafSkill.LeafTick.", "Leaf skill tick did not execute correctly."

    def test_internal_tick_and_transition(self):
        """
        Test that an internal skill transitions between subtasks and ticks them.
        """
        self.ctx.behaviour_hierarchy = ""
        self.root_skill.tick()
        assert self.ctx.behaviour_hierarchy == "MockInternalSkill.MockLeafSkill2.LeafTick2.", "First subskill did not tick correctly."

        self.ctx.behaviour_hierarchy = ""
        self.root_skill.tick()
        assert self.ctx.behaviour_hierarchy == "MockInternalSkill.MockLeafSkill.LeafTick.", "Second subskill did not tick correctly."

    def test_reset(self):
        """
        Test that reset correctly resets the behaviour_hierarchy and subtasks.
        """
        self.ctx.behaviour_hierarchy = "SomeState."
        self.root_skill.reset()
        assert self.ctx.behaviour_hierarchy == "", "Reset did not clear behaviour hierarchy."

    def run_tests(self):
        """Run all tests."""
        tests = [
            ("Leaf Skill Tick", self.test_leaf_tick),
            ("Internal Skill Tick and Transition", self.test_internal_tick_and_transition),
            ("Reset Functionality", self.test_reset)
        ]

        for test_name, test_func in tests:
            try:
                test_func()
                self.print_result(test_name, True)
            except AssertionError as e:
                self.print_result(test_name, False)
                print(f"    {e}")


if __name__ == "__main__":
    tester = TestSkill()
    tester.run_tests()
