from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch_ros.actions import Node

def generate_launch_description():
    # Package paths
    turtlebot3_gazebo = Path(get_package_share_directory('turtlebot3_gazebo'))
    nav2_bringup = Path(get_package_share_directory('nav2_bringup'))
    slam_toolbox = Path(get_package_share_directory('slam_toolbox'))

    turtlebot3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    
    # Launch robot in Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(turtlebot3_gazebo / 'launch' / 'turtlebot3_world.launch.py'))
    )
    
    # Launch Nav2 with delay
    nav2_launch = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to start
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_bringup / 'launch' / 'navigation_launch.py')),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )
    
    # Launch SLAM Toolbox with delay
    slam_launch = TimerAction(
        period=8.0,  # Wait 8 seconds for Nav2 to start
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(slam_toolbox / 'launch' / 'online_async_launch.py')),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )
    
    # Launch RViz with delay
    rviz_launch = TimerAction(
        period=10.0,  # Wait 10 seconds for SLAM to start
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'],
                output='screen'
            )
        ]
    )
    
    # Launch teleop (in new terminal window)
    teleop_launch = TimerAction(
        period=12.0,  # Wait 12 seconds before launching teleop
        actions=[
            ExecuteProcess(
                cmd=['xterm', '-e', 'ros2 run turtlebot3_teleop teleop_keyboard'],
                output='screen'
            )
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        turtlebot3_model,  # Set the model first
        gazebo_launch,
        nav2_launch,
        slam_launch,
        rviz_launch,
        teleop_launch
    ])