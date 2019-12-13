from launch import LaunchDescription
from launch_ros.actions import Node

namespace_ = 'marta'

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='joy',
			# node_namespace=namespace_,
			node_executable='joy_node',
			node_name='joy_node',
			remappings=[
				('joy', 'gamepad')
			],
			output='screen'
		),
		Node(
			package='gamepad_parser',
			# node_namespace=namespace_,
			node_executable='gamepad_parser_node',
			# node_name='gamepad_parser_node',
			output='screen'
		) ,
		Node(
			package='simple_rover_locomotion',
			# node_namespace=namespace_,
			node_executable='simple_rover_locomotion_node',
			# node_name='simple_rover_locomotion_node',
			output='screen'
		)

		])