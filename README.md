# RoboND-HomeServiceRobot-Project
Home Service Robot project for RoboND Term 2 

### catkin_ws is the name of the active ROS Workspace, if your workspace name is different, change the commands accordingly

First, update your system with `sudo apt-get update`.

Then, install the ROS navigation stack with `sudo apt-get install ros-kinetic-navigation`

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Next, clone the following repositories to `catkin_ws/src`:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-perception/slam_gmapping.git
$ git clone https://github.com/turtlebot/turtlebot.git
$ git clone https://github.com/turtlebot/turtlebot_interactions.git
$ git clone https://github.com/turtlebot/turtlebot_simulator.git
```

Install package dependencies with `rosdep install [package-name]`

Clone this repository to `catkin_ws/src`
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/thassan743/RoboND-HomeServiceRobot-Project.git
```

Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Finally run the project:
```sh
$ cd ~/catkin_ws/src/RoboND-HomeServiceRobot-Project/ShellScripts
$ ./home_service.sh
````

### Note
A different directory structure was used from what was suggested in the project.
The directory structure used was as follows:

```
catkin_ws/src
    |-- slam_gmapping
        |-- gmapping
        |-- ...
    |-- turtlebot
        |-- turtlebot_teleop
        |-- ...
    |-- turtlebot_interactions
        |-- turtlebot_rviz_launchers
        |-- ...
    |-- turtlebot_simulator
        |-- turtlebot_gazebo
        |-- ...
    |-- RoboND-HomeServiceRobot-Project
        |-- add_markers
            |-- src/add_markers.cpp
            |-- ...
        |-- home_service_robot
            |-- worlds
            |-- ...
        |-- pick_objects
            |-- src/pick_objects.cpp
            |-- ...
        |-- ShellScripts
            |-- ...
        |-- wall_follower
            |-- src/add_markers.cpp
            |-- ...
```
As can be seen, all packages, nodes and scripts written are found in the `RoboND-HomeServiceRobot-Project` directory which is this repository.
