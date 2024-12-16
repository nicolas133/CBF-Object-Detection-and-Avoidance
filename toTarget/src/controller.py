#!/usr/bin/env python
import rospy 
import math 
from nav_msgs.msg import Odometry
from singlePID import SinglePID
from geometry_msgs.msg import Twist
import numpy as np
from numpy import array, dot
#from qpsolvers import solve_qp
from CBFHelperFunctions import *
from std_msgs.msg import Float64


# Function that converts quaternions to euler and returns yaw, which is the only desired value
def quaternion_to_euler(x, y, z, w):
	
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

class CBF_safetyFilter:


    def __init__(self):
        self.initalizz=False

    def initialize(self):
        if not self.initalizz



        # Initializes CBF_Filter node
        rospy.init_node('Local_PLanner',anonymous= True) 

	

        # Subscribe to the pid output
        rospy.Subscriber('/pid', Twist, self.callback_x)

        # Subscribe to the pid output
        #rospy.Subscriber('/pid_y', Float64, self.callback_y)

        # Subscribe to the odom output
        rospy.Subscriber('/odom', Odometry, self.callback_odom) 

        # Publishes info back to sensors
        self.velpub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        # Establishes rate 
        rate=rospy.Rate(30)

        # Initializes starting x_velocity and steering angle values to 0.0
        self.pid_x = 0.0
        self.pid_sa = 0.0

        # Initializes finished value to 0, will change to 1 once robot has reached final waypoint
        self.finished = 0
        self.plan_index = 0
        #intialize the start of the plan 
        self.plan = None
        #self.costmap_ros = costmap_ros

        self.initalizz=True

        
    def setPlan(self,plan):
        self.plan = plan
        self.plan_index=0

    
    def callback_x(self,msg):
        self.pid_x = msg.linear.x
        self.pid_sa = msg.angular.z
        self.finished = msg.linear.y
        print(self.finished)
	


    
    def callback_odom(self,msg):
        #self.pid = msg
        self.current_pose = msg.pose.pose
        self.current_V = msg.twist.twist.linear.x
        self.current_Vy = msg.twist.twist.linear.y
        self.computeVel()

    def computeVel(self):

        if self.plan is None or self.plan_index>=len(self.plan.poses):
            new_V.linear.x = 0.0
            new_V.angular.z = 0.0

            
	
        ####Define First  Object#####
        circular_obstacle = obstacle(R=0.5,x=[1.5,0]) 

        # Defines an array of the current coordinates of the car
        coords = np.array([self.current_pose.position.x, self.current_pose.position.y])


        #grab the current position from the global planner 
        current_target= self.plan.poses[self.plan_index].pose.position
        print('hello')


        # Compute CBF h-value
        hvalue= circular_obstacle.h_of_x(coords) 

        # Define gradient based on current coordinates
        Grad= circular_obstacle.Grad(coords)

        print("coords: " + str(coords))
        print("h val: " + str(hvalue))
        #print("grad: " + str(Grad))

        # Defines current coordinates
        x_cord= self.current_pose.position.x
        y_cord= self.current_pose.position.y

        # Defines orientation in quaternions to be converted into euler
        rot = self.current_pose.orientation
        theta = quaternion_to_euler(rot.x, rot.y, rot.z, rot.w)

        # Defines current SA based on theta obtained from quaternion_to_euler conversion
        current_SA = thetacv
        safety_gain=1
 
        accel_PID=  array([self.pid_sa, self.pid_x])
        #print(accel_PID)

        f_modified= array([-self.pid_sa, -self.pid_x])
        #print(f_modified)

        P = np.identity(2)
        #print("P: " + str(P))

        # Transpose of linear term in onjective function used in quadprog --term can be used to bias
        # solution should be var(f) in quadprog
        q = f_modified 
        
        a = array([0, 2*np.cos(current_SA)*(-x_cord + 3) + 2*np.sin(current_SA)*(-y_cord + 0)])
        #print("a: ", str(a))

        # Should be b in matlab quadprog minimum threshold
        h = array([safety_gain*hvalue])
        B = -h 
        #print("h: " + str(h))
        #print("B: " + str(B))

        x = solve_qp(P, q, a, B, A= None, b= None)
        #print("QP soln: " + str(x))

        # Initializing variables in Twist object
        new_V=Twist()
        
        # Verifies whether car has reached final waypoint, if true, stops the car
        if (self.finished != 1): 
            new_V.linear.x = self.current_V + x[1]
            new_V.angular.z = x[0]
        else:
            new_V.linear.x = 0.0
            new_V.angular.z = 0.0

        #print("Cur v: " + str(self.current_V))
        #print("pid_x: " + str(self.pid_x))
        #print("SA: " + str(current_SA))
        #print("new velocity for x: " + str(new_V.linear.x))
        #print("new angle for z: " + str(new_V.angular.z))
        self.velpub.publish(new_V)

if __name__ == '__main__':
        print("new callback run--------------------------------------------------------------")
        controller = CBF_safetyFilter()
        rospy.spin()
	
