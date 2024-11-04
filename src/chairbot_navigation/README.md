# Navigation
![chairbot_navigation](https://github.com/aqborromeo/chairbot_ws/tree/main/src/chairbot_navigation)

When a robot encounters a new environment, it needs to both create a map and localize itself within it. Mapping algorithms that can be applied include Occupancy Grid Mapping, Grid-based FastSLAM, Graph-SLAM, and RTAB-Map.

RTAB-Map (Real-Time Appearance-Based Mapping) is a SLAM (Simultaneous Localization and Mapping) algorithm used for creating 3D maps of environments while simultaneously tracking a robot's position. It combines visual, laser, or RGB-D sensor data to build detailed, large-scale maps and efficiently manage memory by prioritizing more relevant areas for exploration. RTAB-Map is particularly useful for robotics applications requiring real-time navigation and localization in complex, dynamic environments.

In this module [rtabmap-ros](http://wiki.ros.org/rtabmap_ros) (a ROS wrapper around the RTAB-Map) will be used with a RGB-D camera which can generate 3D point clouds of the environment and/or create a 2D occupancy grid map for navigation. We will be generating a 2D occupancy grid map.

## Prerequisites
1. ROS and Gazebo
2. CMake and the gcc compiler
3. The rtabmap-ros package

Install the `rtabmap-ros` package
```console
$ sudo apt-get install ros-${ROS_DISTRO}-rtabmap-ros
```

## Build
1. Clone project and initialize a catkin workspace
```
$ cd ~/chairbot_ws/src && catkin_init_workspace
```

2. Move back to the project folder and build
```
$ cd ..
$ catkin_make
```

3. In a terminal Launch the world and robot
```
$ source ~/chairbot_ws/devel/setup.bash
$ roslaunch chairbot_navigation single_chair.launch
```

## SLAM Mapping

1. In a second terminal launch the `chairbot_navigation_slam.launch` file to start the creating SLAM mapping.
```
$ roslaunch chairbot_navigation chairbot_navigation_slam.launch

```

2. Open third terminal, and run the `teleop` node and move around, then save the map.
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

3. Open fourth terminal, save the map on the intended localtion
```
rosrun map_server map_saver -f ~/chairbot_ws/src/chairbot_navigation/maps/office_map2
```
4. Stop and close all terminal, then Move to the navigation part

## Navigation

1. In a terminal Launch the world and robot
```
$ source ~/chairbot_ws/devel/setup.bash
$ roslaunch chairbot_navigation single_chair.launch
```

2. Open second terminal Launch `chairbot_navigation.launch`
```
$ source ~/chairbot_ws/devel/setup.bash
$ roslaunch chairbot_navigation chairbot_navigation.launch map_file:=$HOME/chairbot_ws/src/chairbot_navigation/maps/office_map2.yaml
```

3. Go to rviz and test the navigation by clicking "2D Post Estinamate" and "2D Nav Goal".


## Structure
```
.
├── CMakeLists.txt
├── config
│   └── gmapping_params.yaml
├── launch
│   ├── amcl.launch
│   ├── amcl_v1.launch
│   ├── chairbot_navigation_gmapping.launch
│   ├── chairbot_navigation.launch
│   ├── chairbot_navigation_slam.launch
│   ├── chairbots.launch
│   ├── move_base.launch
│   ├── move_base_v1.launch
│   ├── multi_chairbot.launch
│   ├── multi_chair_rviz.launch
│   ├── one_robot_v2.launch
│   ├── robot1_amcl.launch
│   ├── robot1_navigation.launch
│   ├── robot2_amcl.launch
│   ├── robot2_navigation.launch
│   ├── robot3_amcl.launch
│   ├── robot3_navigation.launch
│   └── single_chair.launch
├── maps
│   ├── office_map1.pgm
│   ├── office_map1.yaml
│   ├── office_map2.pgm
│   ├── office_map2.yaml
│   ├── office_map.pgm
│   └── office_map.yaml
├── package.xml
├── param
│   ├── amcl_params.yaml
│   ├── base_local_planner_params.yaml
│   ├── costmap_common_params_waffle_pi.yaml
│   ├── dwa_local_planner_params_waffle_pi.yaml
│   ├── global_costmap_params.yaml
│   ├── local_costmap_params.yaml
│   └── move_base_params.yaml
├── README.md
├── rviz
│   ├── chairbot_navigation_gmapping_confroom1.rviz
│   ├── chairbot_navigation_gmapping_confroom2.rviz
│   ├── chairbot_navigation_gmapping_confroom.rviz
│   ├── chairbot_navigation_gmapping.rviz
│   ├── chairbot_navigation_multi.rviz
│   ├── chairbot_navigation.rviz
│   ├── chairbots_navigation.rviz
│   └── single_chair_model.rviz
├── urdf
│   ├── turtlebot3_waffle.gazebo.xacro
│   └── turtlebot3_waffle_pi.urdf.xacro
└── worlds
    └── office2.world

7 directories, 46 files

```
