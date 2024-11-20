from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_sg_slam = get_package_share_directory('sg_slam')
    pkg_gazebo_simulator = get_package_share_directory('robot_gazebo')
    pkg_slam_algorithm = get_package_share_directory('lio_sam')

    # Аргумент для параметра max_radius
    max_radius_arg = DeclareLaunchArgument(
        'max_radius',
        default_value='5.0',  # Значение по умолчанию
        description='Maximum scanning radius for the graph'
    )

    # Подключение других пакетов
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_simulator, 'launch', 'robot_sim.launch.py')
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_algorithm, 'launch', 'run.launch.py')
        )
    )

    joy_to_cmd_vel_node = Node(
        package='sg_slam',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel',
        output='screen'
    )

    semantic_graph_node = Node(
        package='sg_slam',
        executable='semantic_graph_node_exec',
        name='semantic_graph_node_exec',
        output='screen',
        parameters=[
            {'max_radius': LaunchConfiguration('max_radius')}
        ]
    )

    return LaunchDescription([
        max_radius_arg, 
        gazebo,
        slam,
        joy_to_cmd_vel_node,
        semantic_graph_node
    ])
