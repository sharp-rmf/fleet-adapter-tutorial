#!/bin/bash

PROJECT_HOME=$(pwd) #TODO: Change to export from config file 

cd external
mkdir build || (rm -r build && mkdir build)
cd build
cmake .. && make || exit 1
cd $PROJECT_HOME