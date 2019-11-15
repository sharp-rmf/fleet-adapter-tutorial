#!/bin/bash 

PROJECT_HOME=$(pwd) #TODO: Change to export from config file
ROS1_SETUP=/opt/ros/melodic/setup.bash
ROS2_SETUP=/opt/ros/dashing/setup.bash

echo "Sourcing Environment"
source $ROS1_SETUP
source $ROS2_SETUP  > /dev/null 2>&1
source external/soss-bridging/ros2/install/setup.bash  > /dev/null 2>&1
source external/soss-bridging/ros1/install/setup.bash  > /dev/null 2>&1
echo "Running Bridge!"
soss external/soss-bridging/bridge.yaml