#!/usr/bin/env python
# import necessary packages below
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

from turtlebot_PID_go2goal import *

if __name__ == '__main__':
	baseline = [8,8,1,0,0,2,0,0,0.25]

	results_dict = {}

	runs = [
		[8 , 8 , 10 , 0.5 , 0.2 , 15 , 1.2 , 0 , 0.25],
	]
	
	run_times = []
	vals_list = []
	
	for val in runs:
		os.environ['TURTLEBOT3_MODEL'] = "waffle_pi"

		time.sleep(1)
		
		process2 = subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_empty_world.launch'])	
		time.sleep(5)

		x = TurtleBot()
		run_time = x.move2goal(val, 3)
		
		run_times.append(run_time)	

		print("Complete in:")
		print(run_time)
		process2.kill()
		
		vals_list.append(val)
	
	results_dict = {"Vals":vals_list,
			"Runtime":run_times}

	#results_dict = pd.DataFrame(results_dict)

	#results_dict.to_csv('output/results_' + str(round(time.time()))+ '.csv', index = False)
	
	print(run_times)
	print(results_dict)
