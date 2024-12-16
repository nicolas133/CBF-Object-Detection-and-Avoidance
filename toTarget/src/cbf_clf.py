#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from singlePID import SinglePID
from geometry_msgs.msg import Twist
import numpy as np
from numpy import array, dot
from qpsolvers import solve_qp
from CBFHelperFunctions import *
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

        # Initializes starting x_velocity and steering angle values to 0.0
        self.pid_x = 0.0
        self.pid_sa = 0.0

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
        x_threshold = 0.1
        y_threshold = 0.1

        # Waypoints
        waypoint = [[0, 0], [5, 0]]

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

        clf_theta = (theta - desired_angle) ** 2
        clf_theta_dot = 2 * (theta - desired_angle) * self.current_Vz

        #### Clf_Vx ####
        clf_x = (x_error) ** 2
        clf_x_dot = 2 * (x_error) * self.current_V

        #### Define First Object ####
        circular_obstacle = obstacle(R=0.5, x=[1.5, 0])

        # Defines an array of the current coordinates of the car
        coords = np.array([self.current_pose.position.x, self.current_pose.position.y])

        # Compute CBF h-value
        hvalue = circular_obstacle.h_of_x(coords)

        # Define gradient based on current coordinates
        Grad = circular_obstacle.Grad(coords)

        print("coords: " + str(coords))
        print("h val: " + str(hvalue))

        # Defines current safety_gain which will impact output of CBF
        safety_gain = 1
        Vx_gain = 1
        SA_gain = 1

       
        P = np.identity(2)
        q = np.zeros(2) # Zero vector because we are minimizing u directly
        a = array([
            [0, 2 * np.cos(theta) * (-x_coord + 3) + 2 * np.sin(theta) * (-y_coord + 0)],
            [0, clf_x_dot],
            [clf_theta_dot, 0]
        ])
        h = array([
            safety_gain *-hvalue,
            SA_gain * clf_theta,
            Vx_gain * clf_x
        ])
        B = h

        # Solve QP
        x = solve_qp(P, q, a, B, A=None, b=None)

        # Initializing variables in Twist object
        new_V = Twist()

        # Verifies whether car has reached final waypoint, if true, stops the car
        if self.finished != 1:
            new_V.linear.x = self.current_V + x[1]
            new_V.angular.z = x[0]
        else:
            new_V.linear.x = 0.0
            new_V.angular.z = 0.0

        print("Cur v: " + str(self.current_V))
        print("SA: " + str(theta))
        print("new velocity for x: " + str(new_V.linear.x))
        print("new angle for z: " + str(new_V.angular.z))
        self.velpub.publish(new_V)


if __name__ == '__main__':
    print("new callback run--------------------------------------------------------------")
    controller = CBF_CLF_safetyFilter()
    rospy.spin()

