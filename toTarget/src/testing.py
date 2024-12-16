#!/usr/bin/env python

import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class controllerNode:

    def __init__(self):

        rospy.init_node("controller_node")
	self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	
	
        self.i = 0
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.Controller_callback)

	#self.odom_pub = rospy.Publisher("/odom", Odometry, self.Controller_callback)

        rate = rospy.Rate(30)

        
    def Controller_callback(self, msg):
        speed = Twist()
	if(self.i==0):
		self.startPosition = msg.pose.pose.position
		self.i = 1
		
	
	self.currentPosition = msg.pose.pose.position

	speed.linear.x = 0

	if(self.currentPosition.x > self.startPosition.x):
		speed.linear.x = -0.5
		print("back")
	else:
		speed.linear.x = 0.5
		print("forward")
	
	self.vel_pub.publish(speed)

	print("x: %f, y: %f, z: %f", self.currentPosition.x, self.currentPosition.y, self.currentPosition.z)
        
            
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = controllerNode()
    speed = Twist()
    speed.linear.x = 0.5
    node.vel_pub.publish(speed)
    time.sleep(5)
    node.run()


