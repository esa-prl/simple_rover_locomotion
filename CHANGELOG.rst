^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_rover_locomotion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO
----
* Add README.
* Switch to Abstract Model so people don't need to use URDF specific calls incase we want to later switch to the SDF model
* go through TODO's in code
* implement real services for activation and deactivation of locomotion mode

NICE TO HAVE
------------

KNOWN BUGS
----------
* changing speed ratio while driving does not update joint velocities correctly
* robot_state_publisher does not publish tf frames even though it is receiving joint states.
* Message Definitions don't compile with Header variable...
* robot_state_publisher still prints messages after removing output='screen' in launch file.

0.0.0 (2019-11-20)
------------------
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