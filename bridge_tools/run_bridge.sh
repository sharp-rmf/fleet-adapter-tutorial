#!/bin/bash
echo "Sourcing Environment"
source /opt/ros/melodic/setup.bash
source /opt/ros/dashing/setup.bash  > /dev/null 2>&1
source bridge_tools/ros2/install/setup.bash  > /dev/null 2>&1
source bridge_tools/ros1/install/setup.bash  > /dev/null 2>&1
echo "Running Bridge!"
soss bridge_tools/bridge.yaml