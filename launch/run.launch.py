from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	pkg_sg_slam = get_package_share_directory('sg_slam')
	pkg_gazebo_simulator = get_package_share_directory('robot_gazebo')
	pkg_slam_algorithm = get_package_share_directory('lio_sam')

	max_radius_arg = DeclareLaunchArgument('max_radius', default_value='5.0', description='Maximum scanning radius for the graph')

	gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_simulator, 'launch', 'robot_sim.launch.py')
        )
    )
	
	slam = TimerAction(
			period=2.0,
			actions=[
					IncludeLaunchDescription(
							PythonLaunchDescriptionSource(
									os.path.join(pkg_slam_algorithm, 'launch', 'run.launch.py')
							)
					)
			]
	)
	
	semantic_graph_node = TimerAction(
		period=5.0, 
		actions=[
			Node(
				package='sg_slam',
				executable='semantic_graph_node_exec',
				name='semantic_graph_node_exec',
				output='screen',
				parameters=[
						{'max_radius': LaunchConfiguration('max_radius')}
						])
				]
		)
	
	rviz_node = Node(
				package='rviz2',
				executable='rviz2',
				name='rviz2',
				output='screen',
				arguments=['-d', os.path.join(pkg_slam_algorithm, 'config', 'rviz2.rviz')],
				parameters=[{'use_sim_time': True}]
		)
	
	return LaunchDescription([
				max_radius_arg,
				gazebo,
				rviz_node,
				semantic_graph_node,
				slam
		])
	
