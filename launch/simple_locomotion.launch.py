from launch import LaunchDescription
from launch_ros.actions import Node

namespace_ = 'marta'

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='simple_rover_locomotion',
			node_namespace=namespace_,
			node_executable='simple_rover_locomotion_node',
			node_name='simple_rover_locomotion_node',
			output='screen'
		)

		])