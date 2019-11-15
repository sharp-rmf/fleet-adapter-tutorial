#!/bin/bash 
sudo apt install ros-dashing-test-msgs

PROJECT_HOME=$(pwd) #TODO: Change to export from config file
ROS1_SETUP=/opt/ros/melodic/setup.bash
ROS2_SETUP=/opt/ros/dashing/setup.bash

cd external/soss-bridging

cd ros2
source $ROS2_SETUP
colcon build
source install/setup.bash

cd ../ros1
source $ROS1_SETUP
colcon build
source install/setup.bash

cd $PROJECT_HOME