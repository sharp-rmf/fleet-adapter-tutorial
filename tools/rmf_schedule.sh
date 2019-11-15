PROJECT_HOME=$(pwd) #TODO: Change to export from config file 
ROS2_SETUP=/opt/ros/dashing/setup.bash

source $ROS2_SETUP
source $PROJECT_HOME/ros2/install/setup.bash
ros2 run rmf_traffic_ros2 rmf_traffic_schedule
