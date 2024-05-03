#!/bin/sh

sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control

echo "source ~/ware_ws/install/setup.bash" >> ~/.bashrc
