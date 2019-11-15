cmake_minimum_required(VERSION 2.8.3)

project(mir_fleet_manager)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  rospy
  std_msgs
  mir_navigation
)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs rospy std_msgs
)

catkin_install_python(
  PROGRAMS nodes/fleet_manager
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/../mir_navigation/share/mir_navigation/rviz
)
