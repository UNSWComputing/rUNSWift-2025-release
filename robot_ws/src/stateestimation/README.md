## Useful Commands

|Command|What it does|
|-------|------------|
|`make build`|Build `stateestimation` ROS2 package and dependencies |
|`source ./install/setup.sh`|Add the built `stateestimation` package to your `ros2` command|
|`make runself`|Run the `SelfEstimator` node|
|`make debugself`|Run the `SelfEstimator` node with `gdbserver` on port `3000`.|

## Setting up analysis and autocomplete with `clangd`
1. Build the project at least once: `make build`.
2. Run `apt install -y clangd`.
3. Install clangd extension from vscode.
4. You should now (hopefully) have autocomplete, type hints, etc. when browsing the codebase.

If you want to debug in vscode, you should also install the microsoft C/C++ extension (cpptools), but
just make sure its intellisense is off and clangd isn't giving you any errors.

> Martin: from my experience, clangd is way faster than MS Cpptools's intellisense. I've disabled
> the cpptools intellisense in `.vscode/settings.json`, and use clangd myself. If you want to use
> cpptools instead, you can remove the line in `settings.json` that says `"C_Cpp.intelliSenseEngine": "disabled"`. Just be warned that it WILL guzzle up all your memory and max out your CPU.

### with CLion
The following guide can be used: https://www.jetbrains.com/help/clion/ros2-tutorial.html
```bash
colcon build --packages-select stateestimation --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Tips:
- it's recommended you support colcon build outside the devcontainer as compile_commands.json will otherwise point to paths relative to the container
    - or alternatively run CLion inside the devcontainer via gateway (not good for performance)
- If you're using WSL, you must run via JetBrains gateway as the paths will not resolve correctly otherwise


## Debugging with `gdbserver`
Note: make sure you built the binary with debug info, eg. by running `make builddebug`.

### In VSCode (recommended)
- Step 1: Make sure you are in `/workspace/robot_ws/src/stateestimation`.
- Step 2: Set any breakpoints you want by clicking next to the line numbers in the source code.
- Step 3: Run `make debugself`.
- Step 4: Press F5 to start debugging. 
- Step 5. Make any changes, and then repeat from step 2. You may wish to rebuild with `make build` before re-running.

### In raw gdb command line
- Step 1: Make sure you are in `/workspace/robot_ws/src/stateestimation`.
- Step 2: Run `make debugself`.
- Step 3: Run `gdb`.
- Step 4: In GDB, run the command `target remote :3000`. You should now be connected to the debugger session of the ROS node.
- Step 5: Run any GDB commands you want, eg. `break main`, `info registers`, etc.
