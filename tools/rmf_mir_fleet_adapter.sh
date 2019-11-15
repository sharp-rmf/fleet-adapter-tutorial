#!/bin/bash

PROJECT_HOME=$(pwd) #TODO: Change to export from config file 
ROS2_SETUP=/opt/ros/dashing/setup.bash

source $ROS2_SETUP
source $PROJECT_HOME/ros2/install/setup.bash
ros2 run mir_rmf_fleet_adapter fleet_adapter -f $1 -g maps/$2/$2_nav_0.yaml
