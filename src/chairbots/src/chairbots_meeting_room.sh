#!/bin/sh

xterm  -e "
roslaunch chairbot_simulation multi_chair.launch " &
sleep 20

xterm  -e "
roslaunch chairbot_navigation multi_chair_rviz.launch" &
sleep 20

xterm  -e "
#sudo ln -s /usr/bin/python3 /usr/bin/python
chmod 755 ~/chairbot_ws/src/chairbot_path_planning/src/chairbot_path_planning_sim.py
chmod 755 ~/chairbot_ws/src/chairbot_path_planning/src/chairbot_path_planning_seq.py
" &
sleep 10

xterm  -e "
# rosrun chairbot_path_planning chairbot_path_planning_seq.py
rosrun chairbot_path_planning chairbot_path_planning_sim.py
" &
sleep 5