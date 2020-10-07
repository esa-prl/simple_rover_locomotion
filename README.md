# Simple Rover Locomotion

## Overview

This packages computes the inverse kinematics (rover_velocities to joint_commands) for a rover with 6 steerable wheels. Ackermann, crabbing, spot turn and a combination of all these locomotion gaits are possible by simply varying the desired linear and angular velocity.

The 3D robot model is simplified (assumes bogie and deployment joints to be fixed) and treated as a 2D model.

**Keywords:** locomotion, package, crab, ackerman, spot-turn

### License

<!-- The source code is released under a [BSD 3-Clause license](ros_package_template/LICENSE). -->

**Author: Miro Voellmy<br />
Affiliation: [Planetary Robotic Laboratory](http://www.esa.int/Enabling_Support/Space_Engineering_Technology/Planetary_Robotics_Laboratory)<br />
Maintainer: Miro Voellmy, miro.voellmy@esa.int**

The Simple Rover Locomotion package has been tested under [ROS2] Foxy Fitzroy and Ubuntu 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Locomotion Mode Package](https://github.com/esa-prl/locomotion_mode)

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/simple_rover_locomotion.git
	cd ../
	colcon build

## Usage

Run the main node with:

	ros2 launch simple_rover_locomotion simple_locomotion.launch.py

To run and test the node in context check out the [marta_launch](https://github.com/esa-prl/marta_launch) package.

## Launch files

* **simple_locomotion.launch.py:** launches the node alone.

## Nodes

### simple_rover_locomotion_node

Inherited from `locomotion_mode.hpp` base class. Computes the steering joint positions and the driving joint velocities given a robot model and a desired rover velocity.

#### Subscribed Topics

See [locomotion_mode](https://github.com/esa-prl/locomotion_mode) for topics and services used.

## Bugs & Feature Requests

Please report bugs and request features using the github issues.

[ROS2]: https://index.ros.org/doc/ros2/
[geometry_msgs/twist]: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[rover_msgs/joint_commmand_array]: https://github.com/esa-prl/rover_msgs
