## Use the simulator

## Installation

Follow [the Cyberbotics guide][webots-install], however for native Ubuntu 22.04, a quick start is:

```
sudo snap install webots
```

## Download the repo containing .wbt file
Use whichever has the least issues. As of 03/04/2024, the bembelbots one doesn't work for us
- https://github.com/nomadz-ethz/WebotsLoLaController
- https://github.com/Bembelbots/WebotsLoLaController


## Running sim
If using docker, most things should be set up for you. Make sure you run subsequent commands in the docker as well. If running natively, make sure you run this in any terminals which would normally require access to the robots image/ folder.
'''
export RUNSWIFT_CHECKOUT_DIR = <your runswift filepath>
'''
1. Type in the following cmd to launch the simulator with the appropriate file path.
```
webots <path to your .wbt file>
```
If you are running the belmbelbots:

'''
webots ./WebotsLoLaController/worlds/nao_robocup.wbt
'''

## Platform support

### MACOS (silicon chips)
As the simulator is not support linux/arm64 and emulation has a poor performance. It is recommanned to use kodos if you are 
using apple silicon macbook. Refer to rUNSWift WIKI how to get display working via ssh.


### Linux/AMD64
The display should be forward properly if you are using runswift devcontainer(last checked 2nd March 2025). Should also work natively.

### Linux/ARM64
Not supported. Use kodos

### Windows
Untested but should work.

On windows 
## Kill the simulator
pkill -9 webots|| true

[webots-install]: https://cyberbotics.com/doc/guide/installing-webots
