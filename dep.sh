#!/bin/bash
#execute in catkin workspace

rosdep update
rosdep install ex3_solution

sudo apt-get -y install ros-indigo-driver-common

sudo apt-get -y install ros-indigo-gazebo-ros-pkgs

