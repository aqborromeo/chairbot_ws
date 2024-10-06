# Localization

Instruction below will cover both map creation and testing the localization for chairbot.

The team rely on AMCL (Adaptive Monte Carlo Localization) is a probabilistic algorithm used in robotics for determining a robot's position within a known map. It uses a set of weighted hypotheses (particles) to estimate the robot's location by comparing sensor data (like laser scans) with the map. The algorithm adjusts these particles dynamically, focusing more on likely locations and updating the robot's position as it moves and gathers more data.


# Self Organizing Chair Environment Localization
![chairbot_localization](https://github.com/aqborromeo/chairbot_ws/tree/main/src/chairbot_localization)

## Prerequisites
1. ROS and Gazebo
2. CMake and the GCC compiler
3. ROS dependencies
```console
$ sudo apt-get update && sudo apt-get upgrade -y
$ sudo apt-get install ros-${ROS_DISTRO}-map-server
$ sudo apt-get install ros-${ROS_DISTRO}-amcl
$ sudo apt-get install ros-${ROS_DISTRO}-move-base
```

## Build
1. Iinitialize catkin workspace
```
$ cd ~/chairbot_ws/src && catkin_init_workspace
```

2. Within `src`, clone the `teleop` package
```
$ cd ~/chairbot_ws/src
$ git clone
```

3. Move back to the project directory and build
```
$ cd ..
$ catkin_make
```

4. Launch the world and robot
```
$ source ~/chairbot_ws/devel/setup.bash
$ roslaunch chairbot_localization single_chair.launch
```

5. SLAM

```
roslaunch chairbot_localization chairbot_localization_slam.launch

```

NOTE: you will need to adjust the view further to the left side in the rviz window to see the map.

6. Open another terminal, and run the `teleop` node and move around, then save the map.
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
rosrun map_server map_saver -f ~/chairbot_ws/src/chairbot_slam/maps/office_map

```

7. Close all terminal, to test localization use run the following command.

terminal_1
```
roslaunch chairbot_localization chairbot_localization.launch
```

terminal_2
```
$ roslaunch chairbot_localization chairbot_localization.launch
```

terminal_3
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

NOTE: notice the logs for the localization, when the chairbot is out of the map you will get a notification.

## Structure
```
.
├── CMakeLists.txt
├── README.md
├── config
│   ├── base_local_planner_params.yaml
│   ├── costmap_common_params_waffle_pi.yaml
│   ├── dwa_local_planner_params_waffle_pi.yaml
│   ├── global_costmap_params.yaml
│   ├── gmapping_params.yaml
│   ├── local_costmap_params.yaml
│   └── move_base_params.yaml
├── include
│   └── chairbot_localization
├── launch
│   ├── amcl.launch
│   ├── chairbot_localization.launch
│   ├── chairbot_localization_gmapping.launch
│   ├── chairbot_localization_slam.launch
│   ├── chairbots.launch
│   ├── move_base.launch
│   └── single_chair.launch
├── maps
│   ├── office_map.pgm
│   └── office_map.yaml
├── models
├── package.xml
├── param
│   ├── base_local_planner_params.yaml
│   ├── costmap_common_params_waffle_pi.yaml
│   ├── dwa_local_planner_params_waffle_pi.yaml
│   ├── global_costmap_params.yaml
│   ├── local_costmap_params.yaml
│   └── move_base_params.yaml
├── rviz
│   ├── chairbot_localization_gmapping.rviz
│   └── chairbot_localization_gmapping_confroom.rviz
├── src
│   └── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── urdf
│   ├── turtlebot3_waffle.gazebo.xacro
│   └── turtlebot3_waffle_pi.urdf.xacro
└── worlds
    ├── office1.world
    └── office2.world

```
