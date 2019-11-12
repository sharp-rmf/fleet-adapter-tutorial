#!/bin/bash 
cd ros2
source /opt/ros/dashing/setup.bash
colcon build
source install.setup.bash

cd ../ros1
source /opt/ros/melodic/setup.bash
colcon build
source install.setup.bash
cd ..