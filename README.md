# Robomapper
 This project is an attempt to learn ROS2 and particualrly robomapping using SLAM Algorithms.

# Prerequisites
 - ROS2 humble
 - RViz2
 - Gazebo
 - TurtleBot3
 - Navigation2
 - Nav2_bringup
 - Slam-Toolbox

# Problems
- My approach to this project was to create a launch file which automates the entire SLAM simulation using TurtleBot3 in Gazebo. It enables mapping using SLAM and visualization using RViz2, also teleop is used to manually control the robot.
- The problem I am facing here is that I got an unexpected error "No such file or directory: '' ", This error caught me offgaurd and I coudn't find a way to debug it. For now this launch file momentarily starts up gazebo with the waffle model in turtlebot3_world but then the error is encountered and the program ends.
- Because of this problem I am unable to get any screenshots or recordings for demonstration. However I have included the concepts that I used and the flow of code in Description.md file.
