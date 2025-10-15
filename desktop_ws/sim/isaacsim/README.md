I will update this more thoroughly eventually but here's some basic steps so I don't forget later

Key things:
- you need to be using wsl since isaacsim doesn't work with devcontainers. This means you won't have a bunch of dependencies enabled by default. This is a TODO, but we should create a script which essentially runs everything in the dockerfile so we can have all the dependencies. So far the dependencies I have needed include the madgwick filter (motion launch issues) and the foxglove bridge to connect to the robot.
- in our motion I've made the pos stuff relative so it works on dev container and wsl. This means you need to launch from rUNSWift dir not robot_ws dir

I like to use foxglove to communicate with robot
1. install foxglove app (through web or whatever)
2. install foxglove bridge on your system (for me inside wsl) since it's probably not there using
```sudo apt install ros-humble-foxglove-bridge```


to do the thing using motion code:
1. build and source robot_ws
2. build and source desktop_ws
3. ```ros2 launch isaacsim isaacsim_launch.py```

or if you don't have foxglove installed
3. ```ros2 launch isaacsim isaacsim_launch.py foxglove:=false```

To do the thing with non-motion code:
1. build and source robot_ws
2. build and source desktop_ws
3. ```ros2 run isaacsim isaacsim_bridge.py```
3. Or as needed set parameters. For example when running test joints movement you'd run ```ros2 run isaacsim isaacsim_bridge --ros-args -p bridge_joint_positions:=true -p bridge_joint_stiffnesses:=true```, then run ```ros2 run motion_port test_joints_movement_node.py  --ros-args -p move_to_initial:=false``` in a separate terminal 