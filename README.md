# RG2021_projects

This repository houses the games for the final project of the lecture Robotic Games.
The games are implemented using the [rogata_engine](https://rogata-engine.readthedocs.io/en/latest/what_is_rogata.html) which can be installed by following the tutorials [here](https://rogata-engine.readthedocs.io/en/latest/usage.html)

Our team was working on the game Heist from the point of view of the Evader agent.

# Install Requirements 
```
pip install -r requirements.txt
```

```
sudo apt-get install ros-noetic-pointcloud-to-laserscan
```

```
sudo apt-get install ros-noetic-robot-localization
```

# How to run
```
roslaunch heist evader_map_1_pointcloud.launch
```

```
roslaunch heist evader_map_1_laserscan.launch
```

```
roslaunch heist evader_map_2_pointcloud.launch
```

```
roslaunch heist evader_map_2_laserscan.launch
```

A number of arguments are available for use with the launch files and can be found within them One, which you should keep in mind is the ```clearing``` parameter. By default it is 1 and this means that the costmpa clearing is enabled. When set to 0, this will be disabled. It is recommended not to set it to 0, as this means that the entire map may eventually be covered in obstacles, which don't really exist.

# Contact
Milena Bruseva: https://github.com/milenabruseva

Leo Nguyen: https://github.com/Xenovortex


# License
 [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
