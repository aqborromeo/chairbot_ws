# Multi Chairbot Running

This experiment will use a phython script for multiple turtlebot3 to run, simulating chairs arranging themselves

## Prerequisites
1. ROS and Gazebo
2. Turtlebot3 Packages
2. CMake and the gcc compiler

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

3. Ensure package multi_hairbot installed


## Running Simulation

1. In a terminal Launch the world and robot
```
$ roslaunch multi_chairbot multi_chair.launch
```

## Running RViz

1. Open terminal Launch RViz launch file
```
$ roslaunch multi_chairbot multi_chair_rviz.launch
```

## Running Self arranging
1. Open terminal and rosrun python file
```
$ chown 755 /home/aqborromeo/chairbot_ws/src/multi_chairbot/src/test_path.py
$ rosrun multi_chairbot test_path.py 
```
