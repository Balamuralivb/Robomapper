from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('slam_exploration')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='')
    
    # Paths
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'slam_config.rviz')
    world_file_path = os.path.join(pkg_share, 'worlds', 'office.world')
    
    # Read robot description
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'world_file',
            default_value=world_file_path,
            description='Path to world file'
        ),
        
        # Gazebo server
        ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so', 
                 '-s', 'libgazebo_ros_factory.so', world_file_path],
            output='screen'
        ),
        
        # Gazebo client
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }]
        ),
        
        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'mobile_robot', 
                      '-topic', 'robot_description',
                      '-x', '0.0', '-y', '0.0', '-z', '0.1']
        ),
        
        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
        ),
        
        # Teleop twist keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='xterm -e'
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])