#!/usr/bin/env python

import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class controllerNode:
    def __init__(self):
        rospy.init_node("controller_node")
        
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.Controller_callback)

	self.odom_pub = rospy.Publisher("/odom", Odometry, self.Controller_callback)

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
    def Controller_callback(self, msg):
        speed = Twist()
        self.vel_pub.publish(speed)
            
        startTime = time.time()
        startPosition = msg.pose.pose.position
            
        while (time.time() - startTime < 1):
            speed.linear.x = 0.5
            speed.angular.z = 0.0
	    print("Position: %d", msg.pose.pose.position)
	    #print("Orientation: %d", msg.pose.pose.orientation)
            self.vel_pub.publish(speed)
	    #self.odom_pub.publish(odom)
            #print("forward --- %d --- %d", startTime, time.time() - startTime)
                
        speed.linear.x = 0.0
        self.vel_pub.publish(speed)
            
    def run(self):
                rospy.spin()

if __name__ == '__main__':
    node = controllerNode()
    node.run()
