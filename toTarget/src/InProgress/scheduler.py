#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import LaserScan


def callback(data):
	rospy.logininfo(ropsy.get_caller_id()+ "I heard%s", data.ranges)


def listener():
	rospy.init_node('lidar_listner', anonymous=True)
	rospy.Subscriber("scan",LaserScan, callback)
	rospy.spin()

if_name_=='_main_:
	listener()

