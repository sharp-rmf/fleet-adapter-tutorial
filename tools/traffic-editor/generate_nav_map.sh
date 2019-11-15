#!/bin/bash

PROJECT_HOME=$(pwd) #TODO: Change to export from config file 

if [[ $# -eq 0 ]] ; then
    echo 'Enter name of map as first parameter'
    exit 0
fi

# x.yaml files need to have filename path point to x.png files from the project root
# output will be x_nav_0.yaml file, for rmf_core graphs
cd $PROJECT_HOME/external/src/traffic-editor/generators
./nav_generator.py $PROJECT_HOME/maps/$1/$1.yaml $1_nav
mv $1_nav_0.yaml $PROJECT_HOME/maps/$1