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

Counter Top
X: 1.2192
Y: 4.2672
Z: 0.0508

Base
X: 0.6096
Y: 4.2672 - (0.6096) = 3.6576
Z: 0.6858

# Structure
```
.
├── CMakeLists.txt
├── README.md
├── build
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
│   │   │   │   ├── CMakeCCompilerId.c
│   │   │   │   ├── a.out
│   │   │   │   └── tmp
│   │   │   └── CompilerIdCXX
│   │   │       ├── CMakeCXXCompilerId.cpp
│   │   │       ├── a.out
│   │   │       └── tmp
│   │   ├── CMakeDirectoryInformation.cmake
│   │   ├── CMakeError.log
│   │   ├── CMakeOutput.log
│   │   ├── CMakeRuleHashes.txt
│   │   ├── CMakeTmp
│   │   ├── Makefile.cmake
│   │   ├── Makefile2
│   │   ├── TargetDirectories.txt
│   │   ├── clean_test_results.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── cmake.check_cache
│   │   ├── download_extra_data.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── doxygen.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── progress.marks
│   │   ├── roscpp_generate_messages_cpp.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── roscpp_generate_messages_eus.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── roscpp_generate_messages_lisp.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── roscpp_generate_messages_nodejs.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── roscpp_generate_messages_py.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── rosgraph_msgs_generate_messages_cpp.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── rosgraph_msgs_generate_messages_eus.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── rosgraph_msgs_generate_messages_lisp.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── rosgraph_msgs_generate_messages_nodejs.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── rosgraph_msgs_generate_messages_py.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── run_tests.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── std_msgs_generate_messages_cpp.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── std_msgs_generate_messages_eus.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── std_msgs_generate_messages_lisp.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── std_msgs_generate_messages_nodejs.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   ├── std_msgs_generate_messages_py.dir
│   │   │   ├── DependInfo.cmake
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   └── progress.make
│   │   └── tests.dir
│   │       ├── DependInfo.cmake
│   │       ├── build.make
│   │       ├── cmake_clean.cmake
│   │       └── progress.make
│   ├── CTestConfiguration.ini
│   ├── CTestCustom.cmake
│   ├── CTestTestfile.cmake
│   ├── Makefile
│   ├── atomic_configure
│   │   ├── _setup_util.py
│   │   ├── env.sh
│   │   ├── local_setup.bash
│   │   ├── local_setup.sh
│   │   ├── local_setup.zsh
│   │   ├── setup.bash
│   │   ├── setup.sh
│   │   └── setup.zsh
│   ├── bin
│   ├── catkin
│   │   └── catkin_generated
│   │       └── version
│   │           └── package.cmake
│   ├── catkin_generated
│   │   ├── env_cached.sh
│   │   ├── generate_cached_setup.py
│   │   ├── installspace
│   │   │   ├── _setup_util.py
│   │   │   ├── chairbot_simulation.pc
│   │   │   ├── chairbot_simulationConfig-version.cmake
│   │   │   ├── chairbot_simulationConfig.cmake
│   │   │   ├── env.sh
│   │   │   ├── local_setup.bash
│   │   │   ├── local_setup.sh
│   │   │   ├── local_setup.zsh
│   │   │   ├── setup.bash
│   │   │   ├── setup.sh
│   │   │   └── setup.zsh
│   │   ├── ordered_paths.cmake
│   │   ├── package.cmake
│   │   ├── pkg.develspace.context.pc.py
│   │   ├── pkg.installspace.context.pc.py
│   │   ├── setup_cached.sh
│   │   └── stamps
│   │       └── chairbot_simulation
│   │           ├── _setup_util.py.stamp
│   │           ├── interrogate_setup_dot_py.py.stamp
│   │           ├── package.xml.stamp
│   │           └── pkg.pc.em.stamp
│   ├── cmake_install.cmake
│   ├── devel
│   │   ├── _setup_util.py
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
│   │   ├── setup.zsh
│   │   └── share
│   │       └── chairbot_simulation
│   │           └── cmake
│   │               ├── chairbot_simulationConfig-version.cmake
│   │               └── chairbot_simulationConfig.cmake
│   ├── gtest
│   │   ├── CMakeFiles
│   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   └── progress.marks
│   │   ├── CTestTestfile.cmake
│   │   ├── Makefile
│   │   ├── cmake_install.cmake
│   │   ├── googlemock
│   │   │   ├── CMakeFiles
│   │   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   │   ├── gmock.dir
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   ├── progress.make
│   │   │   │   │   └── src
│   │   │   │   ├── gmock_main.dir
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   ├── progress.make
│   │   │   │   │   └── src
│   │   │   │   └── progress.marks
│   │   │   ├── CTestTestfile.cmake
│   │   │   ├── Makefile
│   │   │   └── cmake_install.cmake
│   │   ├── googletest
│   │   │   ├── CMakeFiles
│   │   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   │   ├── gtest.dir
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   ├── progress.make
│   │   │   │   │   └── src
│   │   │   │   ├── gtest_main.dir
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   ├── progress.make
│   │   │   │   │   └── src
│   │   │   │   └── progress.marks
│   │   │   ├── CTestTestfile.cmake
│   │   │   ├── Makefile
│   │   │   └── cmake_install.cmake
│   │   └── lib
│   └── test_results
├── include
│   └── chairbot_simulation
├── launch
│   └── chairbots.launch
├── models
│   ├── box_table
│   │   ├── model.config
│   │   └── model.sdf
│   └── box_table1
│       ├── model.config
│       └── model.sdf
├── package.xml
├── src
├── urdf
│   ├── turtlebot3_waffle_pi.gazebo.xacro
│   └── turtlebot3_waffle_pi.urdf.xacro
└── worlds
    ├── office.world
    ├── office1.world
    ├── office2.world
    ├── office_empty.world
    └── office_table.world
```