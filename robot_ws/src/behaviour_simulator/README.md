
# rUNSWift Behaviours Simulation Tool

## Features
- Real time simulation of robot behaviours, independent of other subsystems.
- Runs the real behaviours code in simulation, in the same way it runs on a robot.
- Support for multiple robots on the field.
- Visualisation of the robots and ball.
- Visualisation of the robots perception.
- Sidebar with live simulation data.
- Configurable initial conditions through scripting.

## Directory Structure
```bash
.
├── src1/                # Core simulation logic
├── simulation_scripts/  # Custom scenario scripts
├── README.md            # Documentation file
└── requirements.txt     # Dependencies
```

## Architecture Description
There are two components to the complete simulation:
1. simulation engine (sim)
2. simulation renderer (bviz)

These two components are both independent ROS nodes. The simulation engine publishes the simulation state to
the `/simulation/state` topic. The renderer subscribes to `/simulation/state` and renders the state provided.

This allows us to decouple the renderer from the simulator. This means that they both run asynchronously, and do not block each other (e.g. long rendering time does not block simulation from ticking).

A launchfile is used to to start a new `BehaviourNode` for each robot. This launchfile remaps the topics it subscribes
and publishes to into a namespace unique to the robot. The remappings for a robot are as follows
(`behaviour_node.launch.xml:16-18`):
```xml
<remap from="/motion_command" to="/robot$(var player_number)/motion_command" />
<remap from="/ball_world" to="/robot$(var player_number)/ball_world" />
<remap from="/ball_base" to="/robot$(var player_number)/ball_base" />
```

This is necessary to avoid multiple simulated robots publishing and subscribing to the same topics.

## Getting Started
### Prerequisites
- Python 3.10 or higher
- ROS 2 Humble
- Pygame

Install dependencies using:
```
pip3 install -r requirements.txt
```

### Useage
Please use the docker container to proceed, or work on a native Ubuntu 22.04 system.
1. Build:
	```bash
	colcon build
	```
2. Source (for each new terminal opened in subsequent steps):
	```bash
	cd robot_ws
	source install/setup.bash
	```
3. Start the renderer (bviz):
	```bash
	ros2 run behaviour_simulator bviz
	```
4. Verify that is it running:
	- If running natively, you should see the simulator pop up with the field.
	- If running on the docker, use the vnc server to view:
		1.  Run the entryfile:
			```bash
			cd
			./entryfile.sh
			```
		2. Export your display
			```bash
			export DISPLAY=:1
			```
		3. Visit `localhost:8080` in your browser
			**Troubleshooting:** if there are issues here, check on Docker Desktop network settings to allow localhost bridging.

5. Start the simulator
	```bash
	ros2 run behaviour_simulator sim --scenario <simulation-script>
	```
	Here, you shuold replace `<simulation-script>` with the name of the script with the desired initial conditions. The script must be located in the `simulation_scripts` directory. There are example scripts included. For example, you can run:
	```bash
	ros2 run behaviour_simulator sim --scenario solo_ball_chase.py
	```
	You may also write your own simulation scripts. You must place them in the `simulation_scripts` directory, and you will need to `colcon build` before it can be parsed by the simulator. Writing the scripts should be trivial by referencing the examples.

## Assumptions and Limitations
The goal of this behaviour simulator was to create a _temporary_ tool that can operate independently of other subsystems to test behaviours. This was necessary due to the very tight timeline during the 2024-2025 ROS2 transition, and behaviours encountered blockers waiting on other subsystems. Thus, this is a very lightweight and simplified simulation. It should be replaced in the future by "proper" simulation tools, such as those created in the simulation league for RoboCup.

In this light, there were rigorous simplifications made in the development of this tool listed below.

### Assumptions
- The ball is seen so long as it is within the robot's field of view. This is 56.3 degrees ([source](https://github.com/UNSWComputing/rUNSWift/blob/42ab6e305afaa6e27b93694312792b81d1c38be1/robot/perception/vision/VisionDefinitions.hpp#L60-L64)).
- The robot has instantaneous response to motion commands.
- Robots can see the ball across the entire field, so long as it lies it ints field of view.

### Limitations
- Ball deceleration due to friction is arbitrary. It is not a scientifically tested or calculated value.
- Collisions between robots and the ball are simplified, with deterministic outcomes based on relative velocities.
- Robots are free to exit and enter the bounds of the field, as well as the rendered frame.

## Visualisation
- Robot: represented by a large blue circle
- Ball: represented by a small yellow circle
- Robot heading: represented by a straight white radius inside the blue circle
- Robot yaw and field of view: represented by the cone which extends from the centre of the robot circle
- Player number: represented by the white number inside the blue robot circle
- Perceived ball positions: represented by grey circles. The number inside the cirlce is the player number of the robot that estimated ball position correlates to.
