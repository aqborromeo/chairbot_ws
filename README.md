# Chairbot - Self Organizing Chair

What pain point you want to address?

Meeting rooms are often the important part of an organizational operations, here we hosted various types of gatherings from small team discussions to large presentations. Most often than not these spaces suffer from dis-organization specially after meeting where chairs are left in random location in the room.

Manual arrangement of chairs creates inefficiencies when preparing the room for the next use, space underutilization due to inconsistent chair arrangement. user inconvenience and dissatisfaction, reliance on facilities personnel to manage the setup diverting their attention to other pressing matters and can even cause potential safety hazards.

What is the goal of the project?

The primary goal is to autonomously rearrange at least 3 meeting chairs to a predefined configuration before and after each meeting. 


## Prerequisites
1. ROS and Gazebo
2. CMake & g++/gcc, C++11

Refer to below instruction for installing the dependencies.

![install_dependencies](https://github.com/aqborromeo/chairbot_ws/blob/main/Instructions%20README/1.%20Install%20Dependencies)

# Downloading the Chairbot Workspace

![download_chairbot](https://github.com/aqborromeo/chairbot_ws/blob/main/Instructions%20README/2.%20Download%20Chairbot%20Package)

# Experimentation

This repository have multiple package each representing experementation stages taken to create the chairbot simulation environment.

## Simulation: 
This package show the experimentation taken to create models and create world for chairbot. A launch file was created for single chairbot and multiple chairbot.

Instruction on how to test the worlds can be seen on this link.
![chairbot_simulation_world](https://github.com/aqborromeo/chairbot_ws/blob/main/Instructions%20README/3.%20Experimentation_Simulation_World)

## Localization: 
Instruction below will cover both map creation and testing the localization for chairbot.
The localization system of the chairbot uses algorithms like Adaptive Monte Carlo Localization (AMCL) to estimate its position within a map. It relies on sensor data, primarily from lidar and odometry, to compare current readings with the map, adjusting its estimated pose accordingly. This process allows the robot to accurately track its location in real-time, facilitating effective navigation and interaction with its environment.


Instruction on how to test the worlds can be seen on this link.
![chairbot_localization](https://github.com/aqborromeo/chairbot_ws/blob/main/Instructions%20README/4.%20Experimentation_Localization)

## Navigation:

The navigation system of the chair allows it to autonomously move within an environment using SLAM (Simultaneous Localization and Mapping) or a pre-built map. It relies on sensor data, primarily from its lidar, to avoid obstacles, plan paths, and localize itself within the map. The navigation is controlled through ROS packages which we have adopted like `move_base`, which integrates global and local planners for pathfinding and obstacle avoidance.

Instruction on how to test the worlds can be seen on this link.
![chairbot_navigation](https://github.com/aqborromeo/chairbot_ws/blob/main/Instructions%20README/4.%20Experimentation_Localization)

## Path Planning:

The path planning system of the TurtleBot3 Waffle Pi is responsible for generating efficient routes for the robot to navigate from a start position to a goal while avoiding obstacles. It utilizes algorithms such as Dijkstra's or A* for global path planning, which calculate optimal paths based on the map data, and local planners to dynamically adjust the route in real-time as the robot encounters obstacles. This integrated approach ensures that the TurtleBot3 can navigate complex environments smoothly and safely.

![chairbot_path_planning](https://github.com/aqborromeo/chairbot_ws/blob/main/Instructions%20README/6.%20Experiment_Path_Planning)

## Chairbot:

UNDER CONSTRUCTION

![chairbot](https://github.com/aqborromeo/chairbot_ws/blob/main/Instructions%20README/7.%20Chairbot%20(Self%20Organizing%20Chair)%20)
