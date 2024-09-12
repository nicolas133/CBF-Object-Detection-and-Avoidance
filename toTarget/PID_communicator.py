#!/usr/bin/env python
import rospy
import math
from math import atan2
import time
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from singlePID import SinglePID
from geometry_msgs.msg import Twist
import tf
from PID_edit import angle_calc, angle_verifier

class controllerNode:

    def __init__(self):

	# initialize node
        rospy.init_node('PID', anonymous = True)

	#subsribers and publishers
        rospy.Subscriber('/odom', Odometry, self.callback)
	self.pid_pub = rospy.Publisher('/pid', Twist, queue_size=1)
        #self.pid_sa_pub = rospy.Publisher('/pid_sa', Twist, queue_size=1)

        self.Speed_PID = SinglePID(P=1,I=0,D=0) # create single PID object 
	self.SA_PID = SinglePID(P=1,I=0,D=0)

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	
	# refresh rate of callback function
        rate = rospy.Rate(30)

	# current velocity variables
        self.current_vel_x=0
	self.current_vel_y=0
        
        # keeps track of waypoint list index
        self.wayindex = 0

	# boolean variable for keeping track of whether car has reached final waypoint
	self.finished = False
    
    def callback(self, msg):

	# define current velocity
        self.current_vel_x = msg.twist.twist.linear.x
        print("msg: " + str(msg.twist.twist.linear.x))
	self.current_vel_y = msg.twist.twist.linear.y
	self.current_sa= msg.twist.twist.angular.z

	#update current location
        self.current_pose = msg.pose.pose
	
	# publisher sends data to /cmd_vel 
        self.speed_Publisher()
        
    def speed_Publisher(self):
     
	# pre-defined waypoints for robot to reach
        #self.waypoint = [[0,0], [1, 0], [2, 0], [3, 1], [4, 1], [4, 0], [4,-2]]
        #self.waypoint = [[0,0], [3, 0]]
        self.waypoint = [[0,0], [1.5, 0.8], [3, 0]]

        # when the car is within 0.07m away of a waypint, move on to the next waypoint  
        self.distance_threshold = 0.07

	# tolerance for x and y coordinates - adjust as needed
        self.x_threshold = 0.1
	self.y_threshold = 0.1

	# defines current goal coordinates
        current_target = self.waypoint[self.wayindex]

	# print current waypoint for debugging purposes
        print("Current WAYPOINT: " + str(current_target))

        # get current x, y, and z position values
        x_coord = self.current_pose.position.x
	y_coord = self.current_pose.position.y
	
	# print current x and y values
        print("X-coord: " + str(x_coord))
        print("Y-coord: " + str(y_coord))

	# defines the current error of x and y coordinates
        x_error = abs(x_coord - current_target[0])
	y_error = abs(y_coord - current_target[1])

	# if car is close enough to goal destination, sets goal to be next desired waypoint
        if (x_error < self.x_threshold  and y_error < self.y_threshold):

	    # sets goal waypoint to next designated waypoint
	    if (self.wayindex + 1 < len(self.waypoint)):
                self.wayindex = (self.wayindex + 1)
                current_target = self.waypoint[self.wayindex]

                print("currently going towards " + str(self.waypoint[self.wayindex]))

	    # sets finished boolean variable to true if final waypoint has been achieved
	    else:
		self.finished = True
                print("FInished")

	# initializing vars in Twist object
        new_vel = Twist()

        desired_angle = angle_verifier(angle_calc([x_coord, y_coord], current_target))

	# used to acquire a theta value from the car's current orientation
	rot = self.current_pose.orientation

	(roll, pitch, theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
	# defines a new linear and angular velocity dependent on the orientation and goal destination of car
        pidData = Twist()
	#if(not self.finished):

        pid_x = self.Speed_PID.pid_compute(.3, self.current_vel_x)

	pid_sa = self.SA_PID.pid_compute(desired_angle, theta)

	print("Theta: ", theta)

        if self.finished:
                pidData.linear.y = 1
                pid_x = 0.0
                pid_sa = 0.0
		print ("reached final point")
               

        
        # publishes new PID data nd location data to CBF node
	#pidData = Twist()

	#pidData.linear.x = pid_x + self.current_vel_x
	#pidData.linear.z = pid_sa + self.current_sa


        pidData.linear.x = pid_x #+ self.current_vel_x
	pidData.angular.z = pid_sa #+ self.current_sa
	
	#pidAndLocationData.pose.pose.position.x = x_coord
	#pidAndLocationData.pose.pose.position.y = y_coord
	#pidAndLocationData.twist.twist.linear.x = new_vel.linear.x
	#pidAndLocationData.twist.twist.linear.y = new_vel.linear.y

	print("current v_x: " + str(self.current_vel_x))
	print("pid_x: " + str(pid_x))
        print("SA: " + str(pid_sa))
	#self.vel_pub.publish(pidData)
        self.pid_pub.publish(pidData)
	

if __name__ == '__main__':
	print("new callback run--------------------------------------------------------------")
	controller = controllerNode()
        rospy.spin()

