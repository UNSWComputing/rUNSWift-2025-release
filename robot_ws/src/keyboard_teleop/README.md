# keyboard_teleop

## Motivation

This `keyboard_teleop` is built on the principle of inertia, more formally stated in Sir Isaac Newton's First Law of Motion:

> A body in motion remains in motion or a body at rest remains at rest, unless acted upon by a force.

## Usage

NB: If you haven't already, make sure to `source robot_ws/install/setup.bash` or similar in all SSH sessions to the Nao.

Firstly, check [sys-d motion](../../../image/home/nao/sys-d/README.md) is NOT running:

```
sudo systemctl status runswift_motion.service   # Motion control
```
If it is, stop it with:
```
sudo systemctl stop runswift_motion.service
```
You may need to wait, say 10 seconds for the status to report `inactive`.

Secondly, launch motion manually (which also populates `ros2 topic list`):
```
ros2 launch motion_port motion_launch.py
```

Thirdly, in another terminal SSH to the nao, launch keyboard_teleop:
```
ros2 run keyboard_teleop go
```

Fourthly, stiffen the robot, e.g. via head tap.

Finally, use the keyboard keys 'uj' and 'wasdqe ' as follows.

```
u/j are stand/sit
w/s are forward     increase by 10mm/s/-10mm/s
a/d are left        increase by 10mm/s/-10mm/s 
q/e are turn left   increase by 0.1rad/s/-0.1rad/s
t/y are big kick    with power=1
g/h are small kick  with power=0
'SPACE' is set walking speeds to 0 
```

NB: Expect to repeat the steps above each time you reboot, as sys-d services start on boot.
