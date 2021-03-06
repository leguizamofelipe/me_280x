#! /usr/bin/env python2.7
# we need 2.7 to import tf

import rospy # import rospy, a ROS-python wrapper 
from turtlesim.msg import Pose # import turtlesim message package. Pose is the daya type used to store x,y coordinates
from math import pow, atan2, sqrt, pi # import auxiliary math functions
from sensor_msgs.msg import LaserScan # import sensor message package to utilize the Lidar sensor
from geometry_msgs.msg import Twist  # import geometry message package. Twist is the data type used to describe linear and angular velocities
from nav_msgs.msg import Odometry # import navigation message package. Odometry is the data type to extract turtlebot's position in quaternions
import tf
from tf.transformations import euler_from_quaternion #import transformation package, which allows us to convert from quaternions to eulerian coordinates
import sys
import time
import trouve as tv
import numpy as np

angles_list = np.array(range(-90,91,5))
angles_list[angles_list<0] +=360
angles_list = np.flip(angles_list, 0)

class TurtleBot:

    def __init__(self, robot_number):
        # Creates a node with name 'turtlebot_controller' and make sure it is a unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.robot_number = robot_number
        self.obstacle_avoidance = "Straight"

        # Publisher which will publish to the topic 'robotX/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('robot'+str(robot_number)+'/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/odom'. self.update_pose is called when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('robot'+str(robot_number)+'/odom', Odometry, self.update_pose)
        self.laser_suscriber = rospy.Subscriber('robot'+str(robot_number)+'/scan', LaserScan, self.update_scan)

        # Create a pose object attributed to turtlebot
        self.odom = Odometry
        # Set the rate at which we publish/suscribe in Hz
        self.rate = rospy.Rate(10)
        # We need a short pause to allow self.pose to suscribe from the topic and accurate display turtlebots pose
        rospy.sleep(0.5)
        print('Initiliazing at x:{}, y:{}'.format(self.pos_x, self.pos_y))

        self.distance_error = 0
        self.previous_distance_error = 0 
        self.sum_distance_error = 0

        self.angular_error = 0
        self.previous_angular_error = 0
        self.sum_angular_error = 0

        self.previous_angular_speed = 0
        self.previous_linear_speed = 0

    def update_pose(self, data):
        """Callback function that is called when a new message of type Pose is received by the pose_subscriber."""
        self.odom = data
        self.pos_x = round(self.odom.pose.pose.position.x, 4)
        self.pos_y = round(self.odom.pose.pose.position.y, 4)
        # convert quaternion coordinates in the form of (x,y,z,w) to eulerian coordinates (roll, pitch, yaw)
        self.roll,self.pitch,self.yaw = euler_from_quaternion((self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, \
                                                               self.odom.pose.pose.orientation.z,self.odom.pose.pose.orientation.w))
        # we only use yaw angle since in this case, turtlebot is constrained to rotation around the z-axis
        self.theta = self.yaw

    def target_open_space(self, input_data, distance_threshold):
        pass_zones = tv.find_events(input_data > distance_threshold, period = 1)
        #blocked_zones = tv.find_events(input_data <= distance_threshold, period = 1)

        true_center = len(input_data)/2

        # print(input_data)
        target = 0

        for zone in pass_zones:
            center = (zone.stop + zone.start) / 2
            # print(str(zone.start) + "," + str(zone.stop))
            # print('*')
            if abs(true_center - center) < abs(true_center - target):
                target = center

        angular_vel = 1
        linear_vel = 0.25

        # print(input_data)

        if target > true_center and abs(target-true_center)>3: #Turn Right
            self.obstacle_avoidance = "Right"
            # print("Robot {} - Right".format(str(self.robot_number)))
            return linear_vel, angular_vel * -1
        elif target < true_center and abs(target-true_center)>3:
            self.obstacle_avoidance = "Left"
            # print("Robot {} - Left".format(str(self.robot_number)))
            return linear_vel, angular_vel
        else:
            print("Robot {} - Straight".format(str(self.robot_number)))
            self.obstacle_avoidance = "Straight"
            return 0.75, 0


    def update_target_pose(self, data):
        self.target_odom = data
        self.target_robot_pos_x = round(self.target_odom.pose.pose.position.x, 4)
        self.target_robot_pos_y = round(self.target_odom.pose.pose.position.y, 4)

    def update_scan(self, data):
        """Callback function that is called when a new message of type LaserScan is received by the laser_subscriber."""

        # updates distance to any obstacle in front of turtlebot
        self.front_laser = data.ranges[0]
        #self.left_laser = data.ranges[15]
        #self.right_laser = data.ranges[-15]

        self.front_array = np.array([data.ranges[315], data.ranges[330], data.ranges[345], data.ranges[0], data.ranges[15], data.ranges[30], data.ranges[45]])
    
        self.scanning_array = np.zeros(len(angles_list))

        for count, angle in enumerate(angles_list):
            self.scanning_array[count] = data.ranges[angle]

        #print(self.scanning_array)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pos_x), 2) + pow((goal_pose.y - self.pos_y), 2))

    def linear_vel(self, goal_pose, k_p=0.2, k_i=0, k_d=0): 
        # Store previous error in a variable
        self.previous_distance_error = self.distance_error
        # Update the distance error with respect to current pose
        self.distance_error = self.euclidean_distance(goal_pose)
        # Sum cumulative distance error over time
        self.sum_distance_error += self.distance_error

        return k_p*self.distance_error + k_i*self.sum_distance_error + k_d*(self.distance_error-self.previous_distance_error)

    def steering_angle(self, goal_pose):
        """Steering angle between current pose and the goal"""
        angle = atan2(goal_pose.y - self.pos_y, goal_pose.x - self.pos_x)
        return angle

    def angular_vel(self, goal_pose, k_p=0.5, k_i=0, k_d=0):
        # Store previous error in a variable
        self.previous_angular_error = self.angular_error
        # Update the distance error with respect to current pose
        self.angular_error = self.steering_angle(goal_pose) - self.theta
        # Sum cumulative distance error over time
        self.sum_angular_error += self.angular_error

        return k_p*(self.angular_error) + k_i*(self.sum_angular_error) + k_d*(self.angular_error - self.previous_angular_error)

    def move2goal(self, goal_x, goal_y, request_input = False, update_pose_for_following = False):
        """Moves the turtlebot to the goal."""

        # Creates a pose object
        goal_pose = Pose()
    
        '''
        Default PID Tunes - from 11/27 Teams call. They work for the most part, but are slow

        k_p = 0.06
        k_i = 0.001
        k_d = 0.001

        k_p_angular = 0.3
        k_i_angular = 0.001
        k_d_angular = 0.001
        
        
        k_p = 0.1
        k_i = 0.003 #0.003
        k_d = 0.1

        k_p_angular = 0.5
        k_i_angular = 0 #0.003
        k_d_angular = 0.005
        '''
        distance_tolerance = 0.2

        speed_limit = 2
        angular_speed_limit = 1.5

        goal_pose.x = goal_x
        goal_pose.y = goal_y

        if request_input:
            # Get the input from the user.
            goal_pose.x = float(input("Set your x goal: "))
            goal_pose.y = float(input("Set your y goal: "))
            k_p = float(input("Set constant for K_p: "))
            k_i = float(input("Set constant for K_i: "))
            k_d = float(input("Set constant for K_d: "))

            k_p_angular = float(input("Set constant for K_p angular: "))
            k_i_angular = float(input("Set constant for K_i angular: "))
            k_d_angular = float(input("Set constant for K_d angular: "))

            # Insert a number slightly greater than 0 (e.g. 0.01).
            distance_tolerance = float(input("Set your tolerance for goal: "))

        # Instantiate Twist object to send mesg to turtlebot
        vel_msg = Twist()

        permanent_fine_tune = False
        smoothing = True
        ran_collision = False

        # feedback loop to keep sending control signal while distance > tolerance

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            
            #Precision tunes when approaching target
            
            if (self.euclidean_distance(goal_pose) <= 1.5 and vel_msg.linear.x > 2) or permanent_fine_tune == True:
                # Use different set of tunes when coming in at high speeds to target
                # Set Boolean flag to ground the robot to using these tunes when close to target
                k_p = 0.03
                k_i = 0.001
                k_d = 0

                k_p_angular = 0.3
                k_i_angular = 0
                k_d_angular = 0

                permanent_fine_tune = True
                smoothing = False
            else:
                k_p = 0.08
                k_i = 0.001
                k_d = 0.01

                k_p_angular = 0.5
                k_i_angular = 0
                k_d_angular = 0.005
                

            if (self.front_array < 1.5).sum() > 1: #self.front_laser < 1.5 or self.left_laser < 1.5 or self.right_laser < 1.5:
                # print('Obstacle detected in front. Stopping...')
                # vel_msg.linear.x = 0
                # vel_msg.angular.z = 0
                
                # Return angular velocity depending on obstacle to the right or left
                vel_msg.linear.x, vel_msg.angular.z = self.target_open_space(self.scanning_array, 1.75) 
                self.sum_angular_error = 0
                self.sum_distance_error = 0

                ran_collision = True
                collision_start = time.time()
            else:
                # Linear velocity in the x-axis.
                if smoothing:
                    vel_msg.linear.x = (self.linear_vel(goal_pose, k_p, k_i, k_d) + self.previous_linear_speed) / 2

                    # Angular velocity in the z-axis.
                    vel_msg.angular.z = (self.angular_vel(goal_pose, k_p_angular, k_i_angular, k_d_angular) + self.previous_angular_speed) / 2
                else:
                    vel_msg.linear.x = self.linear_vel(goal_pose, k_p, k_i, k_d)

                    # Angular velocity in the z-axis.
                    vel_msg.angular.z = self.angular_vel(goal_pose, k_p_angular, k_i_angular, k_d_angular)

                # Getting out of control with linear speed
                if vel_msg.linear.x > speed_limit:
                    vel_msg.linear.x = speed_limit
                    # print('Robot {} hit linear velocity limit'.format(str(self.robot_number)))
                    self.sum_distance_error = self.sum_distance_error * 0.15

                if ran_collision:
                    if time.time() - collision_start > 20: ran_collision = False

                # Getting out of control with angular speed
                if abs(vel_msg.angular.z) > angular_speed_limit * 0.5:
                    if not ran_collision:
                        vel_msg.linear.x = vel_msg.linear.x * 0.1
                        direction = np.sign(vel_msg.angular.z)
                        vel_msg.angular.z = direction * angular_speed_limit

                    self.sum_angular_error = self.sum_angular_error * 0.25
                    # print('Robot {} hit angular limit'.format(str(self.robot_number)))
            
            # print('Robot {}. Linear {}'.format(str(self.robot_number), str(vel_msg.linear.x)))
            # print('Robot {}. Angular {}'.format(str(self.robot_number), str(vel_msg.angular.z)))
            # print('*************************************')
            

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

            if update_pose_for_following:
                goal_pose.x = self.target_robot_pos_x
                goal_pose.y = self.target_robot_pos_y

            # Store commanded linear and angular speed 
            self.previous_linear_speed = vel_msg.linear.x
            self.previous_angular_speed = vel_msg.angular.z
                

        # Stopping our robot after destination is reached
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        print('Arrived at location x:{}, y:{}'.format(self.pos_x, self.pos_y))

    def follow_robot(self):
        start_time = time.time()
        while(time.time() - start_time < 45):
            # print('Robot {} targets x:{}, y:{}'.format(self.robot_number, self.target_robot_pos_x, self.target_robot_pos_y))
            self.move2goal(self.target_robot_pos_x, self.target_robot_pos_y, update_pose_for_following = True)

            
if __name__ == '__main__':
    try:
        # Arguments:
        # 1 - select mode (str)
        # 2 - robot number (int)
        # 3 - x_pos target (int)
        # 4 - y_pos target (int)
        if sys.argv[1] == 'goto_xy':
            robot_number = int(sys.argv[2])
            x_pos = int(sys.argv[3])
            y_pos = int(sys.argv[4])
            x = TurtleBot(robot_number)
            x.move2goal(x_pos, y_pos)

        # Arguments:
        # 1 - select mode (str)
        # 2 - robot number (int)
        # 3 - x_pos target (int)
        # 4 - y_pos target (int)
        elif sys.argv[1] == 'leader':
            robot_number = int(sys.argv[2])
            x_pos = int(sys.argv[3])
            y_pos = int(sys.argv[4])
            x = TurtleBot(robot_number)
            x.move2goal(x_pos, y_pos)

        # Arguments:
        # 1 - select mode (str)
        # 2 - robot number (int)
        # 3 - target robot (int)
        elif sys.argv[1] == 'follower':
            robot_number = int(sys.argv[2])
            x = TurtleBot(robot_number)
            #time.sleep(15)
            x.leader_subscriber = rospy.Subscriber('robot'+str(sys.argv[3])+'/odom', Odometry, x.update_target_pose)
            x.target_odom = Odometry
            rospy.sleep(0.5)

            x.follow_robot()

        # Arguments:
        # 1 - select mode (str)
        # 2 - robot number (int)
        elif sys.argv[1] == 'collision-test':
            robot_number = int(sys.argv[2])
            x = TurtleBot(robot_number)
            #time.sleep(15)
            while(True): 
                x.target_open_space(x.scanning_array, 2)

            #x.follow_robot()

    except rospy.ROSInterruptException:
        pass
