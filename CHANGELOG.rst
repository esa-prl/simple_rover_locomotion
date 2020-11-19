^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_rover_locomotion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO
----
* go through TODO's in code
* potentially introduce mode_name
* send velocities=0 to steering motors

NICE TO HAVE
------------
* Steering angle correctin procedure could be simplified by moving step 5 before 4.

KNOWN BUGS
----------


0.0.1 (unreleased)
------------------
* Waits until the wheels are close to desired position before sending the velocities for driving.
* Split Locomotion Mode Library from Simple Rover Locomotion
* Fixed steering
* Fixed driving directions
* added joint_state_publisher, so joints can be changed using a gui
* create MaRTA Xacro Model
* reading out limits from urdf file works.
* implement generic rover library
* publish joint message
* function to find transform of steering in respect to base.
* load paths from config file
* load xacro file
* added launch file
* loaded rover model from config file
* add realistic messages
* inherited from locomotion_mode class
* test subscriber
* test service