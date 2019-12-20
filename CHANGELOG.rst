^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_rover_locomotion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO
----
* Switch to Abstract Model so people don't need to use URDF specific calls incase we want to later switch to the SDF model
* Make sure steering works
* go through TODO's in code
* implement real services
* Split Locomotion Mode Library from Simple Rover Locomotion

NICE TO HAVE
------------
* make it work with a simulator

KNOWN BUGS
----------
* Message Definitions don't compile with Header variable...
* robot_state_publisher still prints messages after removing output='screen' in launch file.

w.i.p (2019-11-20)
------------------
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