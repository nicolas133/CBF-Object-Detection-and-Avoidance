#!/usr/bin/env python

import rospy
import time
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class controllerNode:
    def __init__(self):
        rospy.init_node("controller_node")
        
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.Controller_callback)

	self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

	#self.startTime = time.time()
	
	self.i = 0

	self.timedFB = time.time()

	
	rate = rospy.Rate(100)

        
    def Controller_callback(self, msg):
        speed = Twist()

	if self.i == 0:
	    self.startTime = time.time()
	    self.startPosition = msg.pose.pose.position
	    self.i = 1
	if (time.time() > self.timedFB):
	    print("CurX: %d", msg.pose.pose.position.x)
	    self.timedFB = self.timedFB + 1

	#print(((msg.pose.pose.position.x - 0.95)**2))
	#print(((self.startPosition.x)**2))

 	if ((msg.pose.pose.position.x - 0.5) < (self.startPosition.x)):
	   # print(((msg.pose.pose.position.x - 0.95)**2))
	    # print(((self.startPosition.x)**2))
            speed.linear.x = 0.5
            speed.angular.z = 0.0
	    #print("Position: %d", msg.pose.pose.position)
            self.vel_pub.publish(speed)
	    #self.odom_pub.publish(position)
            #print("forward --- %d --- %d", startTime, time.time() - startTime)
            
        #if ((msg.pose.pose.position >= self.startPosition) and (time.time() - self.startTime >= 3)):
         #   speed.linear.x = -0.5
          #  speed.angular.z = 0.0
           # self.vel_pub.publish(speed)
            #print("backward")
            
	#if (msg.pose.pose.position.x < self.startPosition.x):    
       	 #   speed.linear.x = 0.0
          #  self.vel_pub.publish(speed)
	    #rospy.signal_shutdown()
	else:
	    print("done")
	    speed.linear.x = 0.0
	    self.vel_pub.publish(speed)
            #print("CurX: %d, StartX: %d", round(msg.pose.pose.position.x, 4), round(self.startPosition.x, 4))
            
            
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = controllerNode()
   # speed = Twist()
    #speed.linear.x = 0.5
    #node.vel_pub.publish(speed)
    #time.sleep(5)
    node.run()
