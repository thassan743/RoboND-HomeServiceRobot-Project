#!/bin/sh
# Launch the turtlebot in a gazebo environment
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$HOME/catkin_ws/src/RoboND-HomeServiceRobot-Project/home_service_robot/worlds/MyWorld.world" &
sleep 5
#launch gmapping_demo to perform SLAM
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$HOME/catkin_ws/src/RoboND-HomeServiceRobot-Project/home_service_robot/worlds/MyWorldMap.yaml 3d_sensor:=kinect" & 
sleep 5
#Launch view_navigation to view the map in rviz
xterm  -e  " roslaunch home_service_robot view_navigation.launch" &
sleep 5
xterm  -e  " rosrun add_markers markers" &
sleep 5
xterm  -e  " rosrun pick_objects pick_objects"

