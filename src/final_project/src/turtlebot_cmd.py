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
#import pandas as pd

low_boundary = 0
high_boundary = 10

if __name__ == '__main__':
    rng = np.random.default_rng(12345)

    target_1 = [str(int(rng.integers(low=low_boundary, high=high_boundary, size=1))), str(int(rng.integers(low=low_boundary, high=high_boundary, size=1)))]
    target_2 = [str(int(rng.integers(low=low_boundary, high=high_boundary, size=1))), str(int(rng.integers(low=low_boundary, high=high_boundary, size=1)))]
    target_3 = [str(int(rng.integers(low=low_boundary, high=high_boundary, size=1))), str(int(rng.integers(low=low_boundary, high=high_boundary, size=1)))]
    target_4 = [str(int(rng.integers(low=low_boundary, high=high_boundary, size=1))), str(int(rng.integers(low=low_boundary, high=high_boundary, size=1)))]

    robot_1_process = subprocess.Popen(['python2', 'generic_robot.py', '1', target_1[0], target_1[1]])
    robot_2_process = subprocess.Popen(['python2', 'generic_robot.py', '2', target_2[0], target_2[1]])
    robot_3_process = subprocess.Popen(['python2', 'generic_robot.py', '3', target_3[0], target_3[1]])
    robot_4_process = subprocess.Popen(['python2', 'generic_robot.py', '4', target_4[0], target_4[1]])
    