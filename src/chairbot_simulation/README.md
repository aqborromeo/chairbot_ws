# Simulation
This gazebo package experimentation was only use to test and update the gazebo world to be use for this project. 

# Self Organizing Chair Environment Simulation
![chairbot_simulation](https://github.com/aqborromeo/chairbot_ws/tree/main/src/chairbot_simulation)

## Prerequisites
This project requires that ROS and Gazebo are installed along with the gcc compiler.
please refer to the main Readme file to install the packages.

## Build
1. Change to the project folder 
```
$ cd ~/chairbot_ws/src/chairbot_simulation
```
2. Create a build folder 
```
$ mkdir build && cd build
```
3. Build with cmake 
```
$ cmake .. && make
```
4. Add the build folder to the Gazebo plugin path: 

```
GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/chairbot_ws/src/chairbot_simulation/build
```
5. Change to the project folder 
```
$ cd ~/chairbot_ws/src/chairbot_simulation/
```

6. Launch the simulation world for a single chairbot.
    `gazebo worlds/office_empty.world`
    `gazebo worlds/office_table.world`
    `gazebo worlds/office.world`
    `gazebo worlds/office1.world`
    `gazebo worlds/office2.world`

7. Close all terminal and reopen and new one. Launch a new terminal to launch multiple chairbot.
```
$ roslaunch chairbot_simulation chairbots.launch
```

NOTE: For the custom meeting room table use for the office2.world here are the geometric dimensions.
```
Counter Top
X: 1.2192
Y: 4.2672
Z: 0.0508

Base
X: 0.6096
Y: 4.2672 - (0.6096) = 3.6576
Z: 0.6858
```

# Structure
```
.
├── build
│   ├── atomic_configure
│   │   ├── env.sh
│   │   ├── local_setup.bash
│   │   ├── local_setup.sh
│   │   ├── local_setup.zsh
│   │   ├── setup.bash
│   │   ├── setup.sh
│   │   ├── _setup_util.py
│   │   └── setup.zsh
│   ├── catkin
│   │   └── catkin_generated
│   │       └── version
│   │           └── package.cmake
│   ├── catkin_generated
│   │   ├── env_cached.sh
│   │   ├── generate_cached_setup.py
│   │   ├── installspace
│   │   │   ├── chairbot_simulationConfig.cmake
│   │   │   ├── chairbot_simulationConfig-version.cmake
│   │   │   ├── chairbot_simulation.pc
│   │   │   ├── env.sh
│   │   │   ├── local_setup.bash
│   │   │   ├── local_setup.sh
│   │   │   ├── local_setup.zsh
│   │   │   ├── setup.bash
│   │   │   ├── setup.sh
│   │   │   ├── _setup_util.py
│   │   │   └── setup.zsh
│   │   ├── ordered_paths.cmake
│   │   ├── package.cmake
│   │   ├── pkg.develspace.context.pc.py
│   │   ├── pkg.installspace.context.pc.py
│   │   ├── setup_cached.sh
│   │   └── stamps
│   │       └── chairbot_simulation
│   │           ├── interrogate_setup_dot_py.py.stamp
│   │           ├── package.xml.stamp
│   │           ├── pkg.pc.em.stamp
│   │           └── _setup_util.py.stamp
│   ├── CATKIN_IGNORE
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   │   ├── 3.16.3
│   │   │   ├── CMakeCCompiler.cmake
│   │   │   ├── CMakeCXXCompiler.cmake
│   │   │   ├── CMakeDetermineCompilerABI_C.bin
│   │   │   ├── CMakeDetermineCompilerABI_CXX.bin
│   │   │   ├── CMakeSystem.cmake
│   │   │   ├── CompilerIdC
│   │   │   │   ├── a.out
│   │   │   │   └── CMakeCCompilerId.c
│   │   │   └── CompilerIdCXX
│   │   │       ├── a.out
│   │   │       └── CMakeCXXCompilerId.cpp
│   │   ├── clean_test_results.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── cmake.check_cache
│   │   ├── CMakeDirectoryInformation.cmake
│   │   ├── CMakeRuleHashes.txt
│   │   ├── download_extra_data.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── doxygen.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── Makefile2
│   │   ├── Makefile.cmake
│   │   ├── progress.marks
│   │   ├── roscpp_generate_messages_cpp.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── roscpp_generate_messages_eus.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── roscpp_generate_messages_lisp.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── roscpp_generate_messages_nodejs.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── roscpp_generate_messages_py.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── rosgraph_msgs_generate_messages_cpp.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── rosgraph_msgs_generate_messages_eus.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── rosgraph_msgs_generate_messages_lisp.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── rosgraph_msgs_generate_messages_nodejs.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── rosgraph_msgs_generate_messages_py.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── run_tests.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── std_msgs_generate_messages_cpp.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── std_msgs_generate_messages_eus.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── std_msgs_generate_messages_lisp.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── std_msgs_generate_messages_nodejs.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── std_msgs_generate_messages_py.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── TargetDirectories.txt
│   │   └── tests.dir
│   │       ├── build.make
│   │       ├── cmake_clean.cmake
│   │       ├── DependInfo.cmake
│   │       └── progress.make
│   ├── cmake_install.cmake
│   ├── CTestConfiguration.ini
│   ├── CTestCustom.cmake
│   ├── CTestTestfile.cmake
│   ├── devel
│   │   ├── cmake.lock
│   │   ├── env.sh
│   │   ├── lib
│   │   │   └── pkgconfig
│   │   │       └── chairbot_simulation.pc
│   │   ├── local_setup.bash
│   │   ├── local_setup.sh
│   │   ├── local_setup.zsh
│   │   ├── setup.bash
│   │   ├── setup.sh
│   │   ├── _setup_util.py
│   │   ├── setup.zsh
│   │   └── share
│   │       └── chairbot_simulation
│   │           └── cmake
│   │               ├── chairbot_simulationConfig.cmake
│   │               └── chairbot_simulationConfig-version.cmake
│   ├── gtest
│   │   ├── CMakeFiles
│   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   └── progress.marks
│   │   ├── cmake_install.cmake
│   │   ├── CTestTestfile.cmake
│   │   ├── googlemock
│   │   │   ├── CMakeFiles
│   │   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   │   ├── gmock.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   └── progress.make
│   │   │   │   ├── gmock_main.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   └── progress.make
│   │   │   │   └── progress.marks
│   │   │   ├── cmake_install.cmake
│   │   │   ├── CTestTestfile.cmake
│   │   │   └── Makefile
│   │   ├── googletest
│   │   │   ├── CMakeFiles
│   │   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   │   ├── gtest.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   └── progress.make
│   │   │   │   ├── gtest_main.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   └── progress.make
│   │   │   │   └── progress.marks
│   │   │   ├── cmake_install.cmake
│   │   │   ├── CTestTestfile.cmake
│   │   │   └── Makefile
│   │   └── Makefile
│   └── Makefile
├── CMakeLists.txt
├── launch
│   ├── multi_chair.launch
│   ├── multi_chair_orig.launch
│   ├── single_chair.launch
│   └── spawn_model.launch
├── models
│   ├── box_table
│   │   └── model.config
│   └── box_table1
│       └── model.config
├── package.xml
├── README.md
├── urdf
│   ├── turtlebot3_waffle_pi.gazebo.xacro
│   └── turtlebot3_waffle_pi.urdf.xacro
└── worlds
    ├── office1.world
    ├── office2.world
    ├── office_empty.world
    ├── office_table.world
    └── office.world

55 directories, 205 files
```