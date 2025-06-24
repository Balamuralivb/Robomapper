from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get paths
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    description_dir = get_package_share_directory('turtlebot3_description')
    rviz_config_path = os.path.join(get_package_share_directory('turtlebot3_launcher'), 'rviz', 'turtlebot3.rviz')

    return LaunchDescription([
        # Launch Gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
            )
        ),

        # Start teleop node (runs in terminal)
        ExecuteProcess(
            cmd=['ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
            output='screen',
            shell=True
        ),

        # RViz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
