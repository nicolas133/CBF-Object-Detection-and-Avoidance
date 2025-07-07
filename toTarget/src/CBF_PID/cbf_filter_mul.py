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
from geometry_msgs.msg import Pose, PoseArray
from datetime import datetime, timedelta

#centroid_xlist = np.zeros(20)
#centroid_ylist = np.zeros(20)
#radius_list = np.zeros(20)



# Function that converts quaternions to euler and returns yaw, which is the only desired value
def quaternion_to_euler(x, y, z, w):
	
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

class CBF_safetyFilter:


    def __init__(self):

        # Initializes CBF_Filter node
        rospy.init_node('CBF_Filter',anonymous= False)

        # Initializes starting x_velocity and steering angle values to 0.0
        self.pid_x = 0.0
        self.pid_sa = 0.0

        # Initializes finished value to 0, will change to 1 once robot has reached final waypoint
        self.finished = 0


        self.num_clusters = 0 

        # Subscribe to the pid output
        rospy.Subscriber('/pid', Twist, self.callback_x)

        # Subscribe to the pid output
        #rospy.Subscriber('/pid_y', Float64, self.callback_y)


        #subscribe to num clusters 
        rospy.Subscriber('/num_clusters',Twist,self.callback_numcluster)

        #print("reseting centroids to 0 appeasement")
        self.centroid_xlist = np.zeros(20)
        self.centroid_ylist = np.zeros(20)
        self.radius_list = np.zeros(20)
        rospy.Subscriber('/WrappedObjs', PoseArray, self.callback_wrapped) 

  

        # Subscribe to the odom output
        rospy.Subscriber('/odom', Odometry, self.callback_odom)
	


        # Publishes info back to sensors
        self.velpub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        # Establishes rate 
        rate=rospy.Rate(30)

        



    def callback_numcluster(self, msg):
        self.num_clusters = msg.linear.x
	

    def callback_x(self, msg):
        self.pid_x = msg.linear.x
        self.pid_sa = msg.angular.z
        self.finished = msg.linear.y
        print(self.finished)

    #def callback_y(self, msg):
     #   self.pid_y = msg.data

    def callback_odom(self,msg):
        self.full_timer = datetime.now()
        #self.pid = msg
        self.current_pose = msg.pose.pose
        self.current_V = msg.twist.twist.linear.x
        self.current_Vy = msg.twist.twist.linear.y
        
        if (self.num_clusters != 0):
            self.optimizer()
        else:
            new_V2 = Twist()
            new_V2.linear.x = self.pid_x + self.current_V
            new_V2.angular.z = self.pid_sa
            self.velpub.publish(new_V2)

    def callback_wrapped(self,msg):

        

        #print(msg)
        for i, pose in enumerate(msg.poses):
             x = pose.position.x
             y = pose.position.y
             z = pose.position.z
             #print(f"Pose {i} x = {x} y = {y} z = {z}")
             #print(x)
             self.centroid_xlist[i] = x
             self.centroid_ylist[i] = y
             self.radius_list[i] = z
	#self.centroid_x=
	#self.centroid_y=
	#self.radius=
        #print(centroid_xlist)
	

    def optimizer(self):
    
        for i in range(len(self.centroid_ylist)):
            if i >= self.num_clusters:
                self.centroid_ylist[i] = 0
        print(self.centroid_ylist)
        min_y_indexes = []
        for i in range(len(self.centroid_ylist)):
           if(self.centroid_ylist[i] != 0):
           	min_y_indexes.append(abs(self.centroid_ylist[i]))

        min_y_indexes = sorted(range(len(min_y_indexes)), key=lambda i: min_y_indexes[i])
        
        ####Define First  Object#####
        print("radius_bruh", self.radius_list[0])
        print("centroidx", self.centroid_xlist[0])
        print("centroidy", self.centroid_ylist[0])

        circular_obstacle_list = []
        for i in range(int(self.num_clusters)):
           circular_obstacle_list.append(obstacle(R=self.radius_list[i],x=[self.centroid_xlist[i], self.centroid_ylist[i]]))
        

        # Defines an array of the current coordinates of the car
        coords = np.array([self.current_pose.position.x, self.current_pose.position.y])
        print(coords)

        # Compute CBF h-value
        hval_list = []
        for i in range(int(self.num_clusters)):
           hval_list.append(circular_obstacle_list[i].h_of_x()) 
        
        # Define gradient based on current coordinates
        #Grad= circular_obstacle.Grad()

        print("coords: " + str(coords))
        print("h val: " + str(hval_list))
        #print("grad: " + str(Grad))

        # Defines current coordinates
        x_cord= self.current_pose.position.x
        y_cord= self.current_pose.position.y

        # Defines orientation in quaternions to be converted into euler
        rot = self.current_pose.orientation
        theta = quaternion_to_euler(rot.x, rot.y, rot.z, rot.w)

        # Defines current SA based on theta obtained from quaternion_to_euler conversion
        current_SA = theta

        # Defines current safety_gain which will impact output of CBF
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
        
        #a = array([0, 2*np.cos(current_SA)*(-x_cord + 3) + 2*np.sin(current_SA)*(-y_cord + 0)])
        #a = array([0, 2*np.cos(current_SA)*(-x_cord + self.centroid_xlist[0]) + 2*np.sin(current_SA)*(-y_cord + self.centroid_ylist[0])])
        #a = array([0, 2 * self. np.cos(current_SA)*(-self.centroid_xlist[0]) + 2*np.sin(current_SA)*(-self.centroid_ylist[0])])
        a = []
        for i in range(int(self.num_clusters)):
             
            a.append([0,self.current_V* (2*np.cos(current_SA)*(self.centroid_xlist[i]) +    2*np.sin(current_SA)*(self.centroid_ylist[i]))])

        #a = array([0,(2*np.cos(current_SA)*(self.centroid_xlist[0]) + 2*np.sin(current_SA)*(self.centroid_ylist[0]))])


        print("a: ", str(a))

        # Should be b in matlab quadprog minimum threshold
        #h_list = hval_list * safety_gain
        h_list = []
        print(min_y_indexes)
        print(self.num_clusters)
        if(int(self.num_clusters) != 0):
            for i in range(len(hval_list)):
                h_list.append(hval_list[i] * (safety_gain / (1+abs(self.centroid_ylist[i])))**2)
                

        #h = array([safety_gain*hvalue])
        #if (h >= -1):
        #    h = abs(h)
        #    print("dealing with adversity")
        for i in range(len(h_list)):
           if h_list[i] >= -.75:
              h_list[i] = abs(h_list[i])
              print("hvalue is about to go unsafe so just make it safe and trigger control")

         
        B = -1 * np.array(h_list)
        #print("h: " + str(h))
        #print("B: " + str(B))
        qp_curr_time = datetime.now()
        a=np.array(a)
        x = solve_qp(P, q, a, B, A= None, b= None)
        qp_end_time = datetime.now()
        qp_time_interval = qp_end_time - qp_curr_time
        print("Time interval for solving quadratic " + str(qp_time_interval))
        full_time_interval = qp_end_time - self.full_timer
        print("Full Time Interval :" + str(full_time_interval))
                
        #print("QP soln: " + str(x))

        # Initializing variables in Twist object
        new_V=Twist()
        
        # Verifies whether car has reached final waypoint, if true, stops the car
        if (self.finished != 1): 
            new_V.linear.x = self.current_V + x[1]
            #new_V.linear.x = x[1]
            print("qp_output",x[1])
            new_V.angular.z = x[0]
        else:
            new_V.linear.x = 0.0
            new_V.angular.z = 0.0

        print("Cur v: " + str(self.current_V))
        print("pid_x: " + str(self.pid_x))
        #print("SA: " + str(current_SA))
        print("new velocity for x: " + str(new_V.linear.x))
        #print("new angle for z: " + str(new_V.angular.z))
        self.velpub.publish(new_V)

if __name__ == '__main__':
        print("new callback run--------------------------------------------------------------")
        controller = CBF_safetyFilter()
        rospy.spin()
	
