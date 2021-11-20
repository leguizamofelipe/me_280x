#!/usr/bin/env python
# import necessary packages below
import rospy

# For turtlesim resets
from std_srvs.srv import Empty
clear_bg = rospy.ServiceProxy('reset', Empty)

if __name__ == '__main__':
	try:
		clear_bg()
	except rospy.ROSInterruptException:
		pass
