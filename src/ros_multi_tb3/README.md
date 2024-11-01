Developed a novel approach to collaborative Simultaneous Localization and Mapping (SLAM) by utilizing a multi-robot system. The experiment employed two turtlebot3 robots operating in a simulated gazebo environment, with ROS serving as the platform for seamless integration. Leveraging the gmapping algorithm, we achieved highly precise mapping of the environment. Additionally, we equipped each robot with autonomous exploration capabilities using explore lite. The maps generated by the robots were skillfully merged using the multirobot map merger, resulting in a comprehensive and detailed representation of the environment.

## Installation

mkdir catkin_ws

cd catkin_ws

mkdir src

cd src

git clone https://github.com/GutlapalliNikhil/Collaborative_SLAM.git

git clone https://github.com/ros-perception/slam_gmapping.git

git clone https://github.com/ros-perception/openslam_gmapping.git

cd ..

catkin_make

source devel/setup.bash

## Launching 2 turtlebots in gazebo env

roslaunch ros_multi_tb3 2_tb3_house.launch

## Running the exploration on those 2 robots

roslaunch explore_lite explore_1.launch
 
roslaunch explore_lite explore_2.launch

## Results

Gazebo Env

![gazebo](https://user-images.githubusercontent.com/33520288/235723214-09aa93bd-2478-4509-9bc8-b71512997224.jpeg)

Map generated by Robot 1

![map_1](https://user-images.githubusercontent.com/33520288/235722569-8fae87ce-ef7d-474a-912d-a5c214500aa5.jpeg)

Mag generated by Robot 2

![map_2](https://user-images.githubusercontent.com/33520288/235722694-e799f873-585b-4128-b0e1-e66b77624062.jpeg)

Combined map

![map](https://user-images.githubusercontent.com/33520288/235722815-bd8caec6-7162-47b4-a83c-bf72196d67e2.jpeg)