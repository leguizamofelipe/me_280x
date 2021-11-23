#!/usr/bin/env python
# import necessary packages below
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import time
import os
import subprocess
import pandas as pd

from turtlesim_PID_go2goal import *

if __name__ == '__main__':
	#os.system('rosrun turtlesim turtlesim_node &')
	# Baseline takes ~3s
	baseline = [8,8,1,0,0,2,0,0,0.25]

	results_dict = {}

	runs = [
		[8 , 8 , 10 , 0.5 , 0.2 , 15 , 1.2 , 0 , 0.25],
	]
	
	run_times = []
	vals_list = []
	
	for val in runs:
		process = subprocess.Popen(['rosrun', 'turtlesim', 'turtlesim_node'])
		time.sleep(1)

		x = TurtleSim()
		run_time = x.move2goal(val, 3)
		
		run_times.append(run_time)	

		print("Complete in:")
		print(run_time)
		process.kill()
		
		vals_list.append(val)
	
	results_dict = {"Vals":vals_list,
			"Runtime":run_times}

	results_dict = pd.DataFrame(results_dict)

	results_dict.to_csv('output/results_' + str(round(time.time()))+ '.csv', index = False)
	
	print(run_times)
	print(results_dict)
