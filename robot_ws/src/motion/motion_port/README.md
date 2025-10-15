# Launch Notes for motion port

### 1. compile
``` bash
colcon build [--packages-up-to motion_port]
```

## Connect to the robot [OPTION 1 - LAN]
### 2. plug in ethernet cable

### 3. set IP configuration:
- IP: `10.1.18.2xx` (less than 255)
- Subnet mask: `255.255.0.0`

## Connect to the robot [OPTION 2 - WIFI]
### 2/3. connect to wifi SPL-A as robot and configure IP address
- IP: `10.0.18.2xx` (less than 255)
- Subnet mask: `255.255.0.0`

## Sync Package to Robot

### 4. turn on robot and sync package*
``` bash
make sync [robot_name] [motion_port]
```

## Run Robot
### 5. in docker terminal or kodos
``` bash
ssh nao@[robot_name]
```

### 6. launch motion (after ssh to robot in a separate terminal)
``` bash
ros2 launch motion_port motion_launch.py
```

### 7. use keyboard to control robot (after ssh to robot in a separate terminal)
See robot_ws/src/keyboard_teleop/README.md for more detail
- WASD to increase forward/left/backward/right speed
- U/J for standing/sitting
- K/L for L/R kicking
``` bash
ros2 run keyboard_teleop go
```

## FoxGlove to Control Parameters

### 8. build foxglove bridge
``` bash
ssh nao@[robot_name]
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### 9. using FoxGlove Studio
- start FoxGlove studio
- click open connection and enter `ws://[robot_name/robot_ip]:8765`

## Troubleshooting

### If unable to ssh to a robot by its name, try using its ip address instead
- when using WIFI to connect with the robot, it would be something like `10.0.18.xxx`
- when connecting through LAN, using `10.1.18.xxx`
``` bash
# robot will shout out its ip, just listen ... °□°
ssh nao@[robot_ip]
```

### If `make sync` not working, doublecheck [Makefile](../../../../Makefile)