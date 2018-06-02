#!/bin/sh
# Launch the turtlebot in a gazebo environment
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$HOME/catkin_ws/src/RoboND-HomeServiceRobot-Project/home_service_robot/worlds/MyWorld.world" &
sleep 5
#launch gmapping_demo to perform SLAM
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 5
#Launch view_navigation to view the map in rviz
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
#Launch keyboard_teleop to manually control the robot with keyboard commands
xterm -e   " roslaunch turtlebot_teleop keyboard_teleop.launch"

