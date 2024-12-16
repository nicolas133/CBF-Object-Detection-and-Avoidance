#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

def callback(msg):
	position = msg.pose.pose.position
	print(msg)
	speed = Twist()
	speed.linear.x = 0
	vel_pub.publish(speed)
	
rospy.init_node("odom_sub")
rospy.Subscriber("/odom_raw", Odometry, callback)
vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

rospy.spin()
