^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_rover_locomotion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO
----
* potentially introduce mode_name

NICE TO HAVE
------------
* It's quite messy how stop_rover, steering_in_progress_ and the non_steerable_wheels influence the pipeline. Would be good to refactor the code for improved readability.
* Load Steering Margin from Config
* Compute the wheel speeds from the current wheel orientations and not the set wheel orientations.
* Steering angle correctin procedure could be simplified by moving step 5 before 4.
* Check in which direction the non steerable wheels point and apply that to the input limitation. Currently it is assumed that the wheels rotate around the y axis.


KNOWN BUGS
----------


0.0.1 (unreleased)
------------------
* Send velocities=0 to steering motors
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