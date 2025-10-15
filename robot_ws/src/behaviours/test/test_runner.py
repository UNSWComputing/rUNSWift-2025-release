import os
import runpy
import traceback

def run_all_python_files():
    """Run all tests inside the test directory."""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    script_name = os.path.basename(__file__)

    RED = "\033[91m"  # ANSI code for red
    GREEN = "\033[92m"  # ANSI code for green
    RESET = "\033[0m"  # ANSI code to reset color

    # Find all Python files in the directory except this script
    python_files = [
        f for f in os.listdir(current_dir)
        if f.endswith(".py") and f != script_name
    ]

    for python_file in python_files:
        print(f"Running {python_file}...")
        try:
            # Run the Python file as a script
            runpy.run_path(os.path.join(current_dir, python_file), run_name="__main__")
            print(f"{GREEN}[SUCCESS]{RESET} {python_file} ran successfully.\n")
        except Exception as e:
            print(f"{RED}[FAIL]{RESET} Error running {python_file}:")
            print(f"{RED}{traceback.format_exc()}{RESET}")

if __name__ == "__main__":
    run_all_python_files()
