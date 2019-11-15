#!/bin/bash

PROJECT_HOME=$(pwd) #TODO: Change to export from config file 
ROS2_SETUP=/opt/ros/dashing/setup.bash

if [[ $1 == clear ]] ; then
    rm -r ros2/build
    rm -r ros2/install
    rm -r ros2/log
fi

source $ROS2_SETUP
cd ros2
colcon build
source install/setup.bash
cd $PROJECT_HOME