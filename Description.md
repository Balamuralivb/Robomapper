# Approach:
I approached this project by creating a launch file that automates the startup of a complete SLAM simulation system using Turtlebot3 in Gazebo.

# Concepts:
- SLAM-Toolbox which I used in my project is a grid-map based approach where the world is divided into grids. SLAM helps the robot to figure out its position in an environment and at the same time map the environment. I the case of TurtleBot3, it uses a LiDAR to observe the surrounding.
- In my approach I am using "online_async_launch.py" which is the main launch file in SLAM Toolbox for building a map using real-time laserscan data. It launches the SLAM node along with parameters for real-time SLAM.
- But before launching SLAM, Navigation stack is launched using "navigation_launch.py" from "Nav2_bringup" package. In my case this prepares the system for localization (Nav2 can later be used to implement autonomous navigation). Nav2 subscribes to /map and /tf topics which will be published by SLAM.
- Visual tools like gazebo and RViz are used for visualization.

# Code explanation:
### Robot Model:
Burger model from TurtleBot3 is set as the environment variable so that the dependent nodes know which robot configuration to use.
### Get shared directories:
Get the share/ directories of turtlebot3_gazebo, nav2_bringup and slam-toolbox.
### Gazebo Simulation:
Launching Gazebo with TurtleBot3 Burger inside the predefined turtlebot3_world. The robot model spawns with a LiDAR which will be used to map the world.
### Launch Nav2(Navigation Stack):
Here Nav2 is only used to subscribe to /map and /tf.
### Start SLAM Toolbox:
- Starting SLAM Toolbox in online asynchronous mode.
- Subscribes to /scan. (from LiDAR)
- Subscribes to /odom.
- Builds map in real-time.
- Publishes /map and /tf.(Nav2 subscribes to these topics)
### Launch RViz2:
I am launching RViz2 using the nav2_default_view from nav2_bringup, this loads Robot-model, Lasser scan, Map display, Costmaps and TF tree. This lets us see where the robot is, what the robot sees(LiDAR) and what SLAM algorithm has mapped so far.
### Launch Keyboard Teleop:
Teleop requires keyboard inputs, so I am using xterm which creates a seperate window which accepts keyboard inputs, in this way we are able to manually explore the environment.
