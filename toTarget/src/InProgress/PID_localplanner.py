#!/usr/bin/env python
import rospy
import math
from math import atan2
import time
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from singlePID import SinglePID
from geometry_msgs.msg import Twist
import tf
#from cbf_filter import quaternion_to_euler

# calculates the desired angle of rotation to get from current coordinates to goal coordinates
def angle_calc(curr_Coord, goal_Coord):
	diff_y = goal_Coord[1] - curr_Coord[1]
	return atan2(diff_y, diff_x)

# ensures that calculated angle is within the bounds of [-pi, pi]
def angle_verifier(angle):

	if (angle > math.pi):
	    return angle - (2 * math.pi)
	elif (angle < -math.pi):
	    return angle + (2 * math.pi)
	else:
	    return angle

class controllerNode:
    

    def __init__(self):
     
        print("begin intialization")  
        # initialize node
        rospy.init_node('contoller_node', anonymous = True)

	# subscriber to robot topic
        rospy.Subscriber('/odometry/filtered', Odometry, self.callback)
	
        # publisher to robot topic
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.Speed_PID = SinglePID(P=1,I=0,D=0) # create single PID object 
	
	# rate at which robot calculates error and updates publisher data
        rate = rospy.Rate(100)

	# current velocity and steering angles vars
        self.current_vel_x=0
	self.current_vel_y=0
        self.current_steering_angle=0

        # bool that checks if current iteration was the first one
        self.firstRun = True
        self.initial_x = 0
        self.initial_y = 0

        # keeps track of waypoint list index
        self.wayindex = 0

        # keeps track of last waypoints position error
        self.old_x_position_error = 0
        self.errorFlag = False

	# boolean variable for keeping track of whether car has reached final waypoint
	self.finished = False


        # Initializes finished value to 0, will change to 1 once robot has reached final waypoint
        self.finished = 0
        self.plan_index = 0
            
        #intialize the start of the plan 
        self.plan = None
            
      
 
    def setPlan(self,plan):
        self.plan=plan
        #create an object of plan class
        self.plan_index=0
        #setting waypoints from global planner

    def callback(self, msg):

	# define current velocity
        current_vel_x = msg.twist.twist.linear.x
	current_vel_y = msg.twist.twist.linear.y
	# define current steering angle
        self.current_steering_angle = msg.twist.twist.angular.z

	# sets the steering angle to 0 if small enough to have minimal effect on car
        if ((self.current_steering_angle < 1) and (self.current_steering_angle > -1)):
            self.current_steering_angle = 0
        
	# used to debug if waypoint is changing correctly
	print("Current wayIndex: " + str(self.wayindex))

	# set initial x pos to 0
        self.current_pose = msg.pose.pose
	
	# publisher sends data to /cmd_vel 
        self.computeVEL()
        
    def computeVEl(self):
        if self.plan is None or self.plan_index >=len(self.plan.poses):#check if we reached dest and if we even have a plan 
            new_V.linear.x = 0.0
            new_V.angular.z = 0.0
	


        #grab the current position from the global planner 
        current_target= self.plan.poses[self.plan_index].pose.position
        print("the current target is"+ str(current_target))

        # when the car is within 0.07m away of a waypint, move on to the next waypoint  
        self.distance_threshold = 0.07

	# tolerance for x and y coordinates - adjust as needed
        self.x_threshold = 0.09
	self.y_threshold = 0.09

	# defines current goal coordinates
        current_target = self.waypoint[self.wayindex]
	desired_heading = 0

	# print current waypoint for debugging purposes
        print("Current WAYPOINT: " + str(current_target))

        # get current x, y, and z position values
        x_coord = self.current_pose.position.x
	y_coord = self.current_pose.position.y
        z_coord = self.current_pose.position.z
	
	# print current x and y values
        print("X-coord: " + str(x_coord))
        print("Y-coord: " + str(y_coord))

	# defines the current error of x and y coordinates
        x_error = abs(x_coord - current_target.x[0])
	y_error = abs(y_coord - current_target.y[1])

	# if car is close enough to goal destination, sets goal to be next desired waypoint
        if (x_error < self.x_threshold  and y_error < self.y_threshold):

	    # sets goal waypoint to next designated waypoint
	    if (self.plan_index + 1 < len(self.plan.poses)):
                self.plan_index = (self.plan_index + 1)
                current_target = self.plan.poses[self.plan_index].pose.position
                desired_heading = 0
                print("currently going towards " + str(self.waypoint[self.wayindex]))

	    # sets finished boolean variable to true if final waypoint has been achieved
	    elif (self.plan_index + 1 == len(self.plan.poses)):
                self.finished = 1

	# arbitrary optimal velocity // to be altered to depend on x and y errors
        optimal_vel_x = 0.4

	# initializing vars in Twist object
        new_vel = Twist()

	# desired angle of rotation based on current coordinates and goal coordinates
	desired_angle = angle_verifier(angle_calc([x_coord, y_coord], current_target))

	# used to acquire a theta value from the car's current orientation
	rot = self.current_pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
	#print("tf: " + str(theta))
	#print("gpt: " + str(quaternion_to_euler(rot.x, rot.y, rot.z, rot.w)))
	
	
	# defines a new linear and angular velocity dependent on the orientation and goal destination of car
        new_vel.linear.x = self.current_vel_x + self.Speed_PID.pid_compute(optimal_vel_x, self.current_vel_x)
	#new_vel.linear.y = self.current_vel_y + self.Speed_PID.pid_compute(y_error, self.current_vel_y)
        new_vel.angular.z = self.Speed_PID.pid_compute(desired_angle, theta)

	# checks if car has reached final waypoint and stops if true
	if (self.finished == 1):
	    new_vel = Twist()

        # publishes new speed to motor
	self.velocity_publisher.publish(new_vel)
	

if __name__ == '__main__':
	print("new callback run--------------------------------------------------------------")
	controller = controllerNode()
        rospy.spin()
      


