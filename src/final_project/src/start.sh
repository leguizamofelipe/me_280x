#!/bin/bash

gnome-terminal -- roscore
sleep 3
gnome-terminal -- roslaunch final_project main.launch
