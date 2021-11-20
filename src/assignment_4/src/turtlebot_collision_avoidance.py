#! /usr/bin/env python

import rospy # import rospy, ROS python wrapper
from sensor_msgs.msg import LaserScan # import sensor message package
from geometry_msgs.msg import Twist  # import geometry message package. Twist is the data type used to describe linear and angular velocities

def callback(msg):
	print('------------------')
	print('Front-direction laser scan:', msg.ranges[0])
	print('15 deg laser scan:', msg.ranges[15])
	print('Left-direction laser scan:', msg.ranges[90])
	print('Right-direction laser scan:', msg.ranges[270])
	print('345 deg laser scan:', msg.ranges[345])

	r_sample = [345, 330, 315, 300, 285, 270]
	l_sample = [0, 15, 30, 45, 60, 75, 90]

	# check if there is no obstacle in front and to the front-left and front-right.....
	if msg.ranges[0] > 0.75 and msg.ranges[15] > 0.75 and msg.ranges[345] > 0.75:
		move.linear.x = 0.5 #move forward in the x direction with a velocity of ...
		move.angular.z = 0 #rotate in the z axis with a angular velocity of ...
	# else, if there's obstacle, do something else	
	else:
		move.linear.x = 0 #move forward in the x direction with a velocity of ...
		move.angular.z = 0.5 #rotate in the z axis with a angular velocity of ...

		

	pub.publish(move)

rospy.init_node('avoid_obstacles')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist)
move = Twist()

rospy.spin()
