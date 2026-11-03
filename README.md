# Hero_chassis_controller

## Overview

This is controller  for the assignment to control the chassis of Hero Robot. 

**Keywords:** RoboMaster, ROS, ros_control

### License

**Author: DengJiejing
Maintainer: DengJiejing,djj12091025@qq.com**

The hero_chassis_controller package has been tested under [ROS] Noetic on respectively Ubuntu 20.04. 

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- controller_interface
- forward_command_controller
- hardware_interface
- pluginlib

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone https://github.com/DJdarkstorm1/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build

## Usage

Run the simulation and controller with:

	roslaunch hero_chassis_controller hero_chassis_controller.launch

Use this to simulate motion control for robots：

```
roslaunch hero_chassis_controller teleop_key_hero.launch
```



## Config files

Config file config

* **controllers.yaml**  Params of hero_chassis_controller and joint_state_controller.
* **config.rviz** config for rviz used in hero_chassis_controller.launch

## Launch files

* **hero_chassis_controller:** Hero chassis only simulation and simple chassis controller
* **teleop_key_hero.launch:** A keyboard controller specially designed for hero_chassis_controller

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/DJdarkstorm1/hero_chassis_controller/issues).

[ROS]: http://www.ros.org
