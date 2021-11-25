#!/bin/bash

gnome-terminal -- roscore
sleep 3
gnome-terminal -- roslaunch final_project main.launch
sleep 20
gnome-terminal -- python2 turtlebot_cmd.py
