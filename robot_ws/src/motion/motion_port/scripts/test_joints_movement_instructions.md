# Joint Movement Tester ROS Node

This ROS 2 Python node provides the same functionality as the original `test_joints_movement.py` but uses ROS 2 topics instead of direct socket communication.

## Usage

### Building
```bash
cd robot_ws
colcon build --packages-up-to motion_port
source install/setup.bash
```

### Running the Test
```bash
# Make sure the robot is connected and LoLA is running. If not, you can run with
ros2 run nao_lola_client nao_lola_client
```
```bash
# Place the robot flat on ground, eyes up

ros2 run motion_port test_joints_movement_ros.py
```
```bash
# or if you want to disable the initial move so it moves quicker

ros2 run your_package test_joints_movement_node.py --ros-args -p move_to_initial:=false
```

### What it does
1. Publishes to `/effectors/joint_positions` and `/effectors/joint_stiffnesses` topics
2. Subscribes to `/sensors/joint_positions` for joint feedback
3. Tests each joint by moving it through its full range of motion
4. Reports any joints that don't reach their target positions (>5 degrees difference)

### Topics Used
- **Publishers:**
  - `/effectors/joint_positions` (nao_lola_command_msgs/JointPositions)
  - `/effectors/joint_stiffnesses` (nao_lola_command_msgs/JointStiffnesses)
- **Subscribers:**
  - `/sensors/joint_positions` (nao_lola_sensor_msgs/JointPositions)

### Compatibility
This node works with both:
- Real NAO robots (via nao_lola_client)
- Simulated robots (via IsaacSim bridge)

The advantage of using ROS 2 topics is that it can work in simulation environments and doesn't require direct socket access to the robot. It can also test how well nao_lola_client is responding.
