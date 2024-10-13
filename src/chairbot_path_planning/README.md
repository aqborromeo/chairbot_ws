# Path_Planning
![chairbot_path_planning](https://github.com/aqborromeo/chairbot_ws/tree/main/src/chairbot_path_planning)

This experiment will use a phython script for path_planning and obstacle avoidance

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
$ roslaunch chairbot_path_planning single_chair.launch
```

## Path_Planning

1. In a terminal Launch the world and robot
```
$ source ~/chairbot_ws/devel/setup.bash
$ roslaunch chairbot_path_planning single_chair.launch
```

2. Open second terminal Launch `chairbot_path_planning.launch`
```
$ source ~/chairbot_ws/devel/setup.bash
$ roslaunch chairbot_path_planning chairbot_path_planning.launch map_file:=$HOME/chairbot_ws/src/chairbot_path_planning/maps/office_map2.yaml

3. Go to rviz and test the path_planning by clicking "2D Post Estinamate" and "2D Nav Goal".

## Structure
```
.
