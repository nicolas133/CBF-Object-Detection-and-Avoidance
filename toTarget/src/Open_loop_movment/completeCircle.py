#!/usr/bin/env python
import rospy       
import time  
import argparse
from geometry_msgs.msg import Twist

parser = argparse.ArgumentParser(description='time')
parser.add_argument('--tim', action='store',type=float,default=3)
args = parser.parse_args()

rospy.init_node("speed_controller")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
speed = Twist()
startTime = time.time()

while(time.time()-startTime < args.tim):
    speed.linear.x = 0.1
    speed.angular.z = 1
    pub.publish(speed)

speed.linear.x = 0.0
pub.publish(speed)
