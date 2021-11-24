#!/usr/bin/env python
# import necessary packages below
from final_project.src.generic_robot import TurtleBot
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

if __name__ == '__main__':
    robot_1_process = subprocess.Popen(['python', 'generic_robot.py', '1', '5', '5'])
    robot_2_process = subprocess.Popen(['python', 'generic_robot.py', '2', '5', '5'])
    robot_3_process = subprocess.Popen(['python', 'generic_robot.py', '3', '5', '5'])
    robot_4_process = subprocess.Popen(['python', 'generic_robot.py', '4', '5', '5'])
    
    '''
    robot_1 = TurtleBot(1)
    robot_2 = TurtleBot(2)
    robot_3 = TurtleBot(3)
    robot_4 = TurtleBot(4)

    robot_1.move2goal(5, 5)
    robot_2.move2goal(5, 5)
    robot_3.move2goal(5, 5)
    robot_4.move2goal(5, 5)
    '''
    
    '''
    for val in runs:

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

    print(run_times)
    print(results_dict)

    '''
    