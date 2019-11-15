cmake_minimum_required(VERSION 3.5)
project(mir_rmf_fleet_adapter)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include(GNUInstallDirs)

add_executable(fleet_adapter src/fleet_adapter.cpp src/FleetAdapterNode.cpp src/ParseGraph.cpp)

target_link_libraries(fleet_adapter
  PUBLIC
    rmf_traffic::rmf_traffic
    rmf_traffic_ros2::rmf_traffic_ros2
    yaml-cpp
    ${rclcpp_LIBRARIES}
)

target_include_directories(fleet_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS fleet_adapter
  EXPORT  fleet_adapter
  # RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
