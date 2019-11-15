#!/bin/bash
PROJECT_HOME=$(pwd) #TODO: Change to export from config file

gnome-terminal -- bash -c "$PROJECT_HOME/tools/build_scripts/build_ros1_workspace.sh clear"
gnome-terminal -- bash -c "$PROJECT_HOME/tools/build_scripts/build_ros2_workspace.sh clear"
gnome-terminal -- bash -c "$PROJECT_HOME/tools/build_scripts/build_traffic_editor.sh"
gnome-terminal -- bash -c "$PROJECT_HOME/tools/build_scripts/build_soss.sh"