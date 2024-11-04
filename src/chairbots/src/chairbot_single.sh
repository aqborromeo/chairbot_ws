#!/bin/sh

xterm  -e "
roslaunch chairbot_simulation single_chair.launch " &
sleep 20

xterm  -e "
roslaunch chairbot_navigation chairbot_navigation.launch" &
sleep 20

xterm  -e "
#sudo ln -s /usr/bin/python3 /usr/bin/python
chmod 755 ~/chairbot_ws/src/chairbot_path_planning/src/path_planning.py
" &
sleep 10

xterm  -e "
rosrun chairbot_path_planning path_planning.py
" &
sleep 5
