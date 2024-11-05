# Chairbot - Practice Module 

Chairbot utilizes multiple TurtleBot3 Waffle Pi robots and applies ROS packages developed through extensive experimentation with simulation, localization, navigation, and path planning. The system enables multiple TurtleBots to autonomously park at specified waypoints. This setup is achieved with a Python script that coordinates the movement of multiple TurtleBot3s, simulating a self-arranging chair setup.

### Run the Chairbot script (Meeting Room)
```
$ source ~/chairbot_ws/src/chairbots/src/chairbots_meeting_room.sh
```
# Testing and Evaluation

### Single Chair
```
$ source ~/chairbot_ws/src/chairbots/src/chairbot_single.sh 
```

### Office Room (Single Chair)
```
$
```

### Office Room (Double Chairs)
```
$
```

## Structure
```
.
├── CMakeLists.txt
├── include
│   └── chairbots
├── package.xml
├── README.md
└── src
    ├── chairbot_office1.sh
    ├── chairbot_office2.sh
    ├── chairbot_single.sh
    └── chairbots_meeting_room.sh

3 directories, 7 files
```