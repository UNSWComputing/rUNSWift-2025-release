[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

Welcome to the rUNSWift git repository.


## Directory structure

The directory structure is:

* **Makefile**: This file acts as a doc/cheat sheet for the most common commands used in the rUNSWift repository, such
  as building the code, syncing to the robot, and launching tools.
* **bin**:
    This is where any executables are stored, such as configuration scripts.
* **image**:
    The contents of this directory are synced to the robot with nao-sync, put custom configuration files
    or libraries independent of ROS2 here.
* **firmware**
    Related to building the actual ROS2 OS image using NaoImage. Contains NaoImage snippets which define how
    this happens.
* **desktop_ws**:
    This workspace will contain tools, scripts, and packages that are only used for debugging, monitoring, and 
    simulation on development computers. It will be layered on top of the robot_ws, meaning it can access the
    robot-related code without needing to recompile it.
* **robot_ws**:
    This workspace will contain all the robot-related code that runs on the physical robot and is critical for its 
    operation.
* **utils**:
    This is the source code for any off-robot utilities, such as migration and conversion scripts or
    offline debugging utilities which don't need to interface directly with robot-related code.


## Documentation

Documentation can be found in several places:
 - On [ReadTheDocs][read-the-docs] for heaps of high-level overviews on different areas, of note:
    - [Setup](https://runswift.readthedocs.io/en/latest/setup/index.html)
    - [Architecture](https://runswift.readthedocs.io/en/latest/architecture.html)
    - [Code Release / Team Reports](https://runswift.readthedocs.io/en/latest/code_releases_team_reports.html)

 - On the [CSE RoboCup Site](https://cgi.cse.unsw.edu.au/~robocup/) for game videos, older team reports and code releases

 <!-- search link is relative to repo home.  won't work when looking at `README.md` as a blob. -->
 - [In README.md files in many directories](../../search?q=filename%3AREADME) such as for [pos files](image/home/nao/data/pos/README.md) and [vision regions](robot/perception/vision/Region/README.md), great for implementation overviews
 
 - In docstrings or multiline comments at the top of files, such as explaining how [behaviour.py](image/home/nao/data/behaviours/behaviour.py#L1-L17) bridges C++ and Python

 - In code comments as appropriate to the language
 
 <!-- wiki link is relative to repo home.  won't work when looking at `README.md` as a blob. -->
 - In the [wiki](../../wiki), typically just for private/secret things (most of the public stuff has been migrated to [ReadTheDocs][read-the-docs], though if you find something that hasn't been please help migrate it)

[read-the-docs]: https://runswift.readthedocs.io/

