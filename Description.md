# Approach:
I approached this project by creating a launch file that automates the startup of a complete SLAM simulation system using Turtlebot3 in Gazebo.

# Concepts:
- SLAM-Toolbox which I used in my project is a grid-map based approach where the world is divided into grids. SLAM helps the robot to figure out its position in an environment and at the same time map the environment. I the case of TurtleBot3, it uses a LiDAR to observe the surrounding
- In my approach I am using "online_async_launch.py" which is the main launch file in SLAM Toolbox for building a map using real-time laserscan data. It launches the SLAM node along with parameters for real-time SLAM.
- But before launching SLAM, Navigation stack is launched using "navigation_launch.py" from "Nav2_bringup" package. In my case this prepares the system for localization (Nav2 can later be used to implement autonomous navigation). Nav2 subscribes to /map topic which will be published by SLAM.

# Code explanation:
### Robot Model:
