# ROS_Instinct 

 ROS_Instinct is an implementation of Instict: Biologically inspired reactive planner. This implementation is specifically for ROSBot 2.0, however can adjusted for any ROS device.

##Installation 

* follow the [Husarion Tutorial](https://husarion.com/tutorials/ros-tutorials/2-creating-nodes/) for creating nodes
* Copy all the files from instinct_pln_pkg/src to your packages src folder
* If you have chosen a different naming convention, rename instinct_pln_pkg_node.cpp to mach your package node 
* in the CMakeLists.txt replace the add_executable(...) line with 
add_executable(${PROJECT_NAME}_node src/instinct_pln_pkg_node.cpp src/Instinct.cpp src/CmdPlanner.cpp src/PlanManager.cpp src/Names.cpp src/stdafx.cpp src/InstinctVCPlusPlusHelpers.cpp)

To install instinct generetator for dia

* Install dia 
* Copy the instinctgen.py file from extras, in the approrpiate location in dia program files


##License
The Instinct Robot World is Open Source Software distributed under the GNU GPL licence.

Copyright (c) 2018-19 Rezo Tsulukidze
