# Motion Calibration System

This README explains how to use the motion calibration system for adjusting kick parameters on robots.

## Overview

The motion calibration system allows you to fine-tune kick lean parameters while visualizing the results in real-time. By placing a ball in front of the robot, you can observe the kicking behavior and adjust parameters until you achieve optimal performance.

## Dependencies

Before starting the calibration process, ensure that you can have the following systems running:

1. **Kinematics System**: Provides joint control and robot pose information
2. **Vision System**: Enables ball detection and tracking
3. **Motion System with Calibration**: Controls robot movement and enables parameter adjustment

## Starting the Calibration System

Launch the required components in separate terminals:

### 1. Start the Motion System with Calibration Enabled
```bash
ros2 launch motion_port motion_launch.py kick_calibration:=True
```

### 2. Start the Kinematics System
```bash
ros2 launch bringup kinematics_bringup.py
```

### 3. Start the Vision System
```bash
ros2 launch bringup vision_bringup.py
```



## Calibration Workflow

1. **Position the Ball**: Place a ball in front of the robot within the detection zone (x: 0-300mm, y: -300-300mm from the robot's perspective).

2. **Observe Behavior**: The robot will automatically kick the ball when detected in the target zone.

3. **Adjust Parameters**: While the system is running, modify kick parameters using the command line(or foxglove or rqt):
   ```bash
   ros2 param set /motion_calibration kick_lean_offset_l 0.35
   ```
   
   ```bash
   ros2 param set /motion_calibration kick_lean_offset_r 0.45
   ```
   
   ```bash
   ros2 param set /motion_calibration kick_foot left
   ```

   ```bash
   ros2 param set /motion_calibration kick_foot right
   ```
   

4. **Test & Repeat**: After changing parameters, the robot will use the new values for subsequent kicks. Place the ball again to test the adjusted values.

5. **Automatic Saving**: Parameter changes are automatically saved to the robot's body configuration file, so they will persist across restarts.

## Available Parameters

The following parameters can be adjusted during calibration:

- **kick_lean_offset_l**: Left foot lean offset for kicking (default: 0.3)
- **kick_lean_offset_r**: Right foot lean offset for kicking (default: 0.4)
- **kick_foot**: which foot to use for kick(deafult:left, unknown input will also be default to left)

## Stopping the Calibration

When calibration is complete, you can stop the processes:

1. Press `Ctrl+C` in each terminal to stop the respective launch files.
2. Verify that parameters were saved correctly by checking the config file at the specified location.
