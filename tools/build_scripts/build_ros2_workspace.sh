#!/bin/bash

PROJECT_HOME=$(pwd) #TODO: Change to export from config file 
ROS2_SETUP=/opt/ros/dashing/setup.bash

if [[ $1 == clear ]] ; then
    rm -r ros1/build
    rm -r ros1/install
    rm -r ros1/log
fi

source $ROS2_SETUP
cd ros1
colcon build
cd $PROJECT_HOME