#!/usr/bin/env python2.7
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
import pandas as pd

low_boundary = 8
high_boundary = 10

if __name__ == '__main__':

    target_1 = [str(random.randint(low_boundary, high_boundary)), str(random.randint(low_boundary, high_boundary))]

    print('Target for robot 1:' + str(target_1))

    robot_1_process = subprocess.Popen(['python2', 'generic_robot.py', 'goto_xy', '4', target_1[0], target_1[1]])

    
