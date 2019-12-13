from launch import LaunchDescription
from launch_ros.actions import Node

import os

import xacro
import tempfile

namespace_ = 'marta'

def generate_launch_description():
	

	log_level = 'INFO'
	log_level_arg = '--log-level ' + log_level

	xacro_model_dir  = "/home/freki/rover_wss/ros2_ws/src/rover_config/urdf/"
	xacro_model_name = "rover_dummy.xacro"

	xacro_model_path = os.path.join(xacro_model_dir, xacro_model_name)

	urdf_model_path = to_urdf(xacro_model_path)

	urdf_arg = '--ros-args --param urdf_model_path:=' + urdf_model_path

	cmd_args = '--ros-args ' + log_level_arg

	return LaunchDescription([
		Node(
			package='joy',
			# node_namespace=namespace_,
			node_executable='joy_node',
			node_name='joy_node',
			remappings=[
				('joy', 'gamepad')
			],
			output='screen',
			emulate_tty=True,
			arguments=[(cmd_args)]
		),
		Node(
			package='gamepad_parser',
			# node_namespace=namespace_,
			node_executable='gamepad_parser_node',
			# node_name='gamepad_parser_node',
			output='screen',
			emulate_tty=True,
			arguments=[(cmd_args)]
		) ,
		Node(
			package='simple_rover_locomotion',
			# node_namespace=namespace_,
			node_executable='simple_rover_locomotion_node',
			# node_name='simple_rover_locomotion_node',
			output='screen',
			emulate_tty=True,
			arguments=[(urdf_arg)]
		)
		])



def to_urdf(xacro_path, urdf_path=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * urdf_path -- the path to the urdf file
    """
    # If no URDF path is given, use a temporary file
    if urdf_path is None:
        urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path  # Return path to the urdf file
