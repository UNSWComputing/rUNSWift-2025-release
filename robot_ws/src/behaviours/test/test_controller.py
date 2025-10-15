from src.Controller import Controller


class TestController:
    """
    Test suite for the updated Controller skill.
    """

    GREEN = "\033[92m"  # ANSI code for green
    RED = "\033[91m"  # ANSI code for red
    RESET = "\033[0m"  # ANSI code to reset color

    def __init__(self):
        """
        Initialize the test context and Controller instance.
        """
        self.ctx = self.TestContext()
        self.controller = Controller(ctx=self.ctx)

    class TestContext:
        """
        Mock context object with minimal behavior hierarchy tracking.
        """

        def __init__(self):
            self.behaviour_hierarchy = ""

    def print_result(self, test_name, success):
        """
        Print test result with colored output.
        """
        if success:
            print(f"{self.GREEN}[SUCCESS] {test_name}{self.RESET}")
        else:
            print(f"{self.RED}[FAIL] {test_name}{self.RESET}")

    def test_default_strategy_id_usage(self):
        """
        Test that the default strategy is used when an invalid ID is provided.
        """
        strategy = self.controller._load_transition_strategy(99)  # Invalid ID
        assert (
            strategy == self.controller._static_transition
        ), "Default strategy (ID 1) should be used for invalid strategy ID."

    def test_specific_strategy_resolution(self):
        """
        Test that valid strategy IDs resolve to the correct strategy.
        """
        assert (
            self.controller._load_transition_strategy(0) == self.controller._legacy_transition
        ), "Strategy ID 0 should resolve to _legacy_transition."
        assert (
            self.controller._load_transition_strategy(1) == self.controller._static_transition
        ), "Strategy ID 1 should resolve to _static_transition."
        assert (
            self.controller._load_transition_strategy(2) == self.controller._dynamic_transition
        ), "Strategy ID 2 should resolve to _dynamic_transition."

    def test_transition_strategy_execution(self):
        """
        Test that the current transition strategy executes without error.
        """
        self.controller._transition_strategy = self.controller._legacy_transition
        try:
            self.controller._transition_strategy()  # Should execute without error
            assert True, "_legacy_transition executed successfully."
        except Exception as e:
            assert False, f"_legacy_transition failed: {e}"

        self.controller._transition_strategy = self.controller._static_transition
        try:
            self.controller._transition_strategy()  # Should execute without error
            assert True, "_static_transition executed successfully."
        except Exception as e:
            assert False, f"_static_transition failed: {e}"

        self.controller._transition_strategy = self.controller._dynamic_transition
        try:
            self.controller._transition_strategy()  # Should execute without error
            assert True, "_dynamic_transition executed successfully."
        except Exception as e:
            assert False, f"_dynamic_transition failed: {e}"

    def test_initialise_sub_skills(self):
        """
        Test that subskills are initialized correctly.
        """
        self.controller._initialise_sub_skills()
        assert isinstance(self.controller._sub_skills, dict), "Subskills should be initialized as a dictionary."
        
    def run_tests(self):
        """
        Run all tests for the Controller skill.
        """
        tests = [
            ("Default Strategy ID Usage", self.test_default_strategy_id_usage),
            ("Specific Strategy Resolution", self.test_specific_strategy_resolution),
            ("Transition Strategy Execution", self.test_transition_strategy_execution),
            ("Subskill Initialization", self.test_initialise_sub_skills),
        ]

        for test_name, test_func in tests:
            try:
                test_func()
                self.print_result(test_name, True)
            except AssertionError as e:
                self.print_result(test_name, False)
                print(f"    {e}")


if __name__ == "__main__":
    tester = TestController()
    tester.run_tests()
