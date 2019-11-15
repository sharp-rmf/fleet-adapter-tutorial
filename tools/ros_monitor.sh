#/bin/bash

# This script aims to be a one-push way to create an interface to monitor ros1 and ros2 stuff
# Remember to start running a roscore node first

PROJECT_HOME=$(pwd) #TODO: Change to export from config file
ROS1_SETUP=/opt/ros/melodic/setup.bash
ROS2_SETUP=/opt/ros/dashing/setup.bash

gnome-terminal -- bash -c "source $ROS1_SETUP; watch 'rostopic list | pr --columns=2'; exec $SHELL"
gnome-terminal -- bash -c "source $ROS2_SETUP; watch 'ros2 topic list | pr --columns=2'; exec $SHELL"