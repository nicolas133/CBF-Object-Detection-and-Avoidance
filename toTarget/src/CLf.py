#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from singlePID import SinglePID
from geometry_msgs.msg import Twist
import numpy as np
from numpy import array, dot
from qpsolvers import solve_qp
from CBFHelperFunctions_of import *
from std_msgs.msg import Float64
from math import atan2


# calculates the desired angle of rotation to get from current coordinates to goal coordinates
def angle_calc(curr_Coord, goal_Coord):
    diff_x = goal_Coord[0] - curr_Coord[0]
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



# Function that converts quaternions to euler and returns yaw, which is the only desired value
def quaternion_to_euler(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y ** 2 + z ** 2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return yaw


class CBF_CLF_safetyFilter:
    def __init__(self):
        # Initializes CBF_Filter node
        rospy.init_node('CBF_CLF_Filter', anonymous=False)

        # Subscribe to the odom output
        rospy.Subscriber('/odom', Odometry, self.callback_odom)

        # Publishes info back to sensors
        self.velpub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Establishes rate
        rate = rospy.Rate(30)

       
        # Initializes finished value to 0, will change to 1 once robot has reached final waypoint
        self.finished = 0

        self.wayindex = 0  # Initialize index (add initialization)

    def callback_odom(self, msg):
        self.current_pose = msg.pose.pose
        self.current_V = msg.twist.twist.linear.x
        self.current_Vy = msg.twist.twist.linear.y
        self.current_Vz = msg.twist.twist.angular.z

        self.optimizer()

    def optimizer(self):
        # Tolerance for x and y coordinates - adjust as needed
        x_threshold = 0.2
        y_threshold = 0.2

        # Waypoints
        waypoint = [[1,0],[2,2]]

        # Defines current goal coordinates
        current_target = waypoint[self.wayindex]

        # Print current waypoint for debugging purposes
        print("Current WAYPOINT: " + str(current_target))

        # Get current x, y, and z position values
        x_coord = self.current_pose.position.x
        y_coord = self.current_pose.position.y

        # Print current x and y values
        print("X-coord: " + str(x_coord))
        print("Y-coord: " + str(y_coord))

        # Calculate errors
        x_error = current_target[0] - x_coord
        y_error = current_target[1] - y_coord


	#total postion error 
        total_error= x_error + y_error 

        # If car is close enough to goal destination, sets goal to be next designated waypoint
        if abs(x_error) < x_threshold and abs(y_error) < y_threshold:
            if self.wayindex + 1 < len(waypoint):
                self.wayindex += 1
                current_target = waypoint[self.wayindex]
                print("Currently going towards " + str(waypoint[self.wayindex]))
            else:
                self.finished = 1
                print("Finished")

        #### Clf_SA ####
        desired_angle = angle_verifier(angle_calc([x_coord, y_coord], current_target))

        # Used to acquire a theta value from the car's current orientation
        rot = self.current_pose.orientation
        theta = quaternion_to_euler(rot.x, rot.y, rot.z, rot.w)
        theta = angle_verifier(theta)
        #obliterate tiny steering angles 
        #if (abs(theta) < .2 and abs(theta) != 0):
        #    theta = 0.0
        #print("tiny steering angle destroyed")
	
        theta_error = angle_verifier(theta-desired_angle)
        clf_theta = (theta - desired_angle) ** 2
        clf_theta_dot = 2 * (theta - desired_angle) * self.current_Vz

        #### Clf_Vx ####
        clf_x = (x_error) ** 2
        clf_x_dot = 2 * (x_error) * self.current_V

	
	###total_CLF###
        #clf_total = (total_error) ** 2 + (clf_theta)
        clf_total = (x_error) ** 2 + (y_error) ** 2 + (clf_theta)

        #### Define First Object ####
        circular_obstacle = obstacle(R=0.5, x=[3, 0])

        # Defines an array of the current coordinates of the car
        coords = np.array([self.current_pose.position.x, self.current_pose.position.y])

        # Compute CBF h-value
        hvalue = circular_obstacle.h_of_x(coords)

        # Define gradient based on current coordinates
        Grad = circular_obstacle.Grad(coords)

        print("coords: " + str(coords))
        print("h val: " + str(hvalue))

        # Defines current safety_gain which will impact output of CBF
        safety_gain = 10
        Vx_gain = 1
        SA_gain = 1
        kv = 0.5
        kth = 0.5

       
        P = np.identity(2)
        #P = array([[1,0,0],[0,1,0],[0,0,100]])
        P = P.astype(np.double)
        #q = np.zeros(2) # Zero vector because we are minimizing u directly
        #q = array([0,0])
        #q=q.astype(np.double)
        v_clf = kv*((x_error)**2+(y_error)**2)**0.5
        w_clf = kth*(theta_error)
        q = array([-v_clf,-w_clf])
        q=q.astype(np.double)
        #q = array([0,0,1])
        #q=q.astype(np.double)
        #q = array([[1,1],[1,1],[1,1]])

        #a = array([
            #[0, 2 * np.cos(theta) * (-x_coord + 3) + 2 * np.sin(theta) * (-y_coord + 0)],
            #[0, clf_x_dot],
            #[clf_theta_dot, 0]
        #])
	
        l = 0.15
        #a = array([
        #    [-(2*(x_coord+l*np.cos(theta)-2)*np.cos(theta)+2*(y_coord+l*np.sin(theta)-2)),-(-2*(x_coord+l*np.cos(theta)-2)*l*np.cos(theta)+2*(y_coord+l*np.sin(theta)-2)*l*np.sin(theta)),0],
        #    [2*x_error*np.cos(theta)+2*y_error*np.sin(theta), 2*theta_error,-1],
        #])

        #h = array([
        #    safety_gain *((x_coord+l*np.cos(theta)-2)**2+2*(y_coord+l*np.sin(theta)-2)**2-0.5**2),
        #    -Vx_gain * clf_total
        #])
	
        #a = array([
        #    [2*x_error*np.cos(theta)+2*y_error*np.sin(theta), 2*theta_error],
        #])

        #h = array([
        #    -Vx_gain * clf_total
        #])

        #B = h

        # Solve QP
        #x = solve_qp(P, q, a=None, B=None, A=None, b=None)
        x = solve_qp(P,q)

        # Initializing variables in Twist object
        new_V = Twist()
        
        
         

        # Verifies whether car has reached final waypoint, if true, stops the car
        if self.finished != 1:
            #new_V.linear.x = self.current_V + x[0]
            new_V.linear.x = x[0]
            new_V.angular.z = -x[1]
        else:
            new_V.linear.x = 0.0
            new_V.angular.z = 0.0


        # Setting max velocity to be 2m/s
        if (x[0] > 2):
           x[0] = 2 

        print("Cur v: " + str(self.current_V))
        print("SA: " + str(theta))
        print("th_error: " + str(theta_error))
        print("x_error: " + str(x_error))
        print("y_error: " + str(y_error))
        print("new velocity for x: " + str(new_V.linear.x))
        print("new angle for z: " + str(new_V.angular.z))
        self.velpub.publish(new_V)


if __name__ == '__main__':
    print("new callback run--------------------------------------------------------------")
    controller = CBF_CLF_safetyFilter()
    rospy.spin()

