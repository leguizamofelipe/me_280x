#!/usr/bin/env python
# import necessary packages below
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import time

# For turtlesim resets

from std_srvs.srv import Empty
clear_bg = rospy.ServiceProxy('reset', Empty)

class TurtleSim:

	def __init__(self):
		# Creates a node with name 'turtlesim_controller' and make sure it is a
		# unique node (using anonymous=True).
		rospy.init_node('turtlesim_controller', anonymous=True)

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
												  Twist, queue_size=10)

		# A subscriber to the topic '/turtle1/pose'. self.update_pose is called
		# when a message of type Pose is received.
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
												Pose, self.update_pose)

		# Create a pose object attributed to turtlebot
		self.pose = Pose()
		# Set the rate at which we publish/suscribe in Hz
		self.rate = rospy.Rate(10)
		# We need a short pause to allow self.pose to suscribe from the topic and accurate display turtlebots pose
		# Try commenting out rospy.sleep() below and observe the print out of the pose if we didn't pause
		rospy.sleep(0.5)
		print('Initiliazing at x:{}, y:{}'.format(self.pose.x, self.pose.y))

		self.distance_error = 0
		self.previous_distance_error = 0 
		self.sum_distance_error = 0

		self.angular_error = 0
		self.previous_angular_error = 0
		self.sum_angular_error = 0

	def update_pose(self, data):
		"""Callback function which is called when a new message of type Pose is
		received by the subscriber."""
		self.pose = data
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)

	def euclidean_distance(self, goal_pose):
		"""Euclidean distance between current pose and the goal."""
		return sqrt(pow((goal_pose.x - self.pose.x), 2) +
					pow((goal_pose.y - self.pose.y), 2))

	def linear_vel(self, goal_pose, k_p=1.5, k_i=0, k_d=0): 
		# Store previous error in a variable
		self.previous_distance_error = self.distance_error
		# Update the distance error with respect to current pose
		self.distance_error = self.euclidean_distance(goal_pose)
		# Sum cumulative distance error over time
		self.sum_distance_error += self.distance_error

		return k_p*self.distance_error + k_i*self.sum_distance_error + k_d*(self.distance_error-self.previous_distance_error)

	def steering_angle(self, goal_pose):
		"""Steering angle between current pose and the goal"""
		return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

	def angular_vel(self, goal_pose, k_p=6, k_i=0, k_d=0):
		self.previous_angular_error = self.angular_error
		self.angular_error = self.steering_angle(goal_pose) - self.pose.theta
		self.sum_angular_error += self.angular_error

		return k_p*(self.angular_error) + k_i*(self.sum_angular_error) + k_d*(self.angular_error - self.previous_angular_error)

	def move2goal(self, values, max_time):
		"""Moves the turtle to the goal."""

		# Creates a pose object
		goal_pose = Pose()

		# Get the input from the user.
		goal_pose.x = float(input("Set your x goal: "))
		goal_pose.y = float(input("Set your y goal: "))
		linear_k_p = float(input("Set constant for linear K_p: "))
		linear_k_i = float(input("Set constant for linear K_i: "))
		linear_k_d = float(input("Set constant for linear K_d: "))
		angular_k_p = float(input("Set constant for angular K_p: "))
		angular_k_i = float(input("Set constant for angular K_i: "))
		angular_k_d = float(input("Set constant for angular K_d: "))

		# Insert a number slightly greater than 0 (e.g. 0.01).
		distance_tolerance = float(input("Set your tolerance for goal: "))

		start_time = time.time()
		# Instantiate Twist object to send mesg to turtlebot
		vel_msg = Twist()

		# feedback loop to keep sending control signal while distance > tolerance
		while self.euclidean_distance(goal_pose) >= distance_tolerance and time.time()-start_time < max_time:

			# Linear velocity in the x-axis.
			vel_msg.linear.x = self.linear_vel(goal_pose, linear_k_p, linear_k_i, linear_k_d)
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0

			# Angular velocity in the z-axis.
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = self.angular_vel(goal_pose, angular_k_p, angular_k_i, angular_k_d)

			# Publishing our vel_msg
			self.velocity_publisher.publish(vel_msg)

			# Publish at the desired rate.
			self.rate.sleep()

		# Stopping our robot after the movement is over.
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		end_time = time.time()
		self.velocity_publisher.publish(vel_msg)
		
		print('Arrived at location x:{}, y:{}'.format(self.pose.x, self.pose.y))
		dt = end_time - start_time
		if dt > max_time:
			return -1
		else:
			return dt

if __name__ == '__main__':

	x = TurtleSim()
	x.move2goal()

