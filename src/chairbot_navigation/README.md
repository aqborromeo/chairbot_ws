# Mapping
![features]()

When a robot finds itself in a new environment it must create a map and localise itself within it. Mapping algorithms that can be used include Occupancy Grid Mapping, Grid-based FastSLAM, Graph-SLAM and RTAB-Map.

[RTAB-Map (Real-Time Appearance-Based Mapping)](http://introlab.github.io/rtabmap/) is a Graph-SLAM approach that performs [Loop Closure](http://www.cds.caltech.edu/~murray/courses/me132-wi11/me132a_lec16.pdf) with [Visual Bag-of-Words](https://www.youtube.com/watch?v=a4cFONdc6nc). Loop closure occurs inside working memory based on features detected with [SURF (Speeded Up Robust Features)](https://people.ee.ethz.ch/~surf/eccv06.pdf) esimating how likely a new image comes from a previous location or a new location. When a loop closure hypothesis is accepted, a new constraint is added to the map’s graph and an optimizer minimizes the mapping errors. A memory management is used to limit the number of locations used for loop closure detection and graph optimization, so that real-time constraints on large-scale environnements are respected.

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

3. Go to rviz and test the navigation by clicking "2D Post Estinamate" and "2D Nav Goal".

## Structure
```
.
├── CMakeLists.txt
├── README.md
├── config
│   └── gmapping_params.yaml
├── include
│   └── chairbot_navigation
├── launch
│   ├── amcl.launch
│   ├── chairbot_navigation.launch
│   ├── chairbot_navigation_gmapping.launch
│   ├── chairbot_navigation_slam.launch
│   ├── move_base.launch
│   └── single_chair.launch
├── maps
│   ├── office_map.pgm
│   ├── office_map.yaml
│   ├── office_map1.pgm
│   ├── office_map1.yaml
│   ├── office_map2.pgm
│   └── office_map2.yaml
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
│   ├── chairbot_navigation.rviz
│   ├── chairbot_navigation_gmapping.rviz
│   ├── chairbot_navigation_gmapping_confroom.rviz
│   ├── chairbot_navigation_gmapping_confroom1.rviz
│   ├── chairbot_navigation_gmapping_confroom2.rviz
│   └── single_chair_model.rviz
├── src
├── urdf
│   ├── turtlebot3_waffle.gazebo.xacro
│   └── turtlebot3_waffle_pi.urdf.xacro
└── worlds
    └── office2.world
```
