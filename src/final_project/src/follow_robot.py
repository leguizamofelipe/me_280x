#!/usr/bin/env python
# import necessary packages below
from generic_robot import TurtleBot
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import time
import os
import subprocess
import time
import os
import random
#import pandas as pd

low_boundary = 0
high_boundary = 10

if __name__ == '__main__':

    target_1 = [str(random.randint(low_boundary, high_boundary)), str(random.randint(low_boundary, high_boundary))]
    target_2 = [str(random.randint(low_boundary, high_boundary)), str(random.randint(low_boundary, high_boundary))]

    print('Target for robot 1:' + str(target_1))
    print('Target for robot 2:' + str(target_2))

    robot_1_process = subprocess.Popen(['python2', 'generic_robot.py', 'goto_xy', '1', target_1[0], target_1[1]])
    robot_2_process = subprocess.Popen(['python2', 'generic_robot.py', 'goto_xy', '2', target_2[0], target_2[1]])


    robot_3_process = subprocess.Popen(['python2', 'generic_robot.py', 'follower', '1'])
    robot_4_process = subprocess.Popen(['python2', 'generic_robot.py', 'follower', '2'])
    
