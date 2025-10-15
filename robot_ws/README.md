# robot_ws

Welcome to the rUNSWift ros2 workspace! To get started, see the steps below

## Prerequisites
0. Have [ros2 humble](https://docs.ros.org/en/humble/Installation.html) installed or be running our docker container by first running ```make dev``` in the `rUNSWift` root directory
1. If you don't have it in your bashrc, source ros2 by running ```source /opt/ros/humble/setup.bash``` in your terminal
2. Install dependencies by running ```./src/3rdparty/scripts/install-dependencies.sh``` in `robot_ws` directory
3. Build workspace by running ```colcon build``` in `robot_ws` directory
4. In every terminal do `source robot_ws/install/setup.bash` or add this command to your ~/.bashrc

## Prepare a real robot

1. Flash a robot - with usb by running ```make build-image``` in the `rUNSWift` root directory, then ```./bin/make-usb.sh``` (following the instructions in that bash file)
2. ssh into the robot (if you don't have access run ./bin.update-hosts.sh) and snoop around. If you've made an update since the last flash run ```make nao-sync <robot/all>```, or ```make nao-sync-pkg <robot/all> <package-name>```
3. Don't forget to `source robot_ws/install/setup.bash` before you run anything you need to. The main launch file is `ros2 launch bringup runswift_launch.py`

- If you want specifics for each package, check for a README! If there isn't one, ask someone on discord as everything is brand sparkly new. Even better, why not document it yourself once you have it figured out for the next you to come along and read!

### Simulated robot
Note: this section has been copied from something old and isn't working quite yet but if you're reading this why not give it a go and fix it up I want a simulator <3

#### Prerequisites
1. Install SimSpark by running `setup-simspark.sh``

#### Running
1. In one terminal run `rcsoccersim3d`
2. In another, run `ros2 run rcss3d_nao rcss3d_nao`

