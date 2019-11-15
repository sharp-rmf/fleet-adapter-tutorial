#!/bin/bash

PROJECT_HOME=$(pwd) #TODO: Change to export from config file 
MIR_EXAMPLE_INIT=ros1/install/setup.bash
ROS1_SETUP=/opt/ros/melodic/setup.bash

source $ROS1_SETUP
source $MIR_EXAMPLE_INIT
roslaunch mir_vendor_setup main.launch 
