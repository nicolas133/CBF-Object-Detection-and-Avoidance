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


def angle_Calc(curr_Coord, goal_Coord):
	diff_x = goal_Coord[0] - curr_Coord[0]
	diff_y = goal_Coord[1] - curr_Coord[1]
	
	return atan2(diff_y, diff_x)

class controllerNode:
    def __init__(self):
        print("loop")
        rospy.init_node('contoller_node', anonymous = True)
        rospy.Subscriber('/odom', Odometry, self.callback)
	
        #rospy.spin()
	# publisher to robot topic
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.Speed_PID = SinglePID(P=1,I=0,D=0) # create single PID object
        
	
        rate = rospy.Rate(30)# rate at which robot calculates error and updates publisher data
	# current velocity and steering angles vars
        self.currentVx=0
        self.currentSA=0
        # bool that checks if current iteration was the first one
        self.firstRun = True
        self.initial_x = 0
        self.initial_y = 0
        # keeps track of waypoint list index
        self.wayindex = 0
        # keeps track of last waypoints position error
        self.old_x_position_error = 0
        self.errorFlag = False

	self.currTime = time.time()

    def callback(self, msg):

        print("---------------------")
	#Get current V
        currentVx=msg.twist.twist.linear.x
        currentSA=msg.twist.twist.angular.z
        if (currentSA < 1) and (currentSA > -1):
            currentSA = 0
            print("I JUST OBLITEREATED A TINY STEERING MESSAGE")
        print("Current Steering Message:" + str(currentSA))

	# set initial x pos to 0
        self.current_pose = msg.pose.pose
	
	# publisher call sends data to /cmd_vel 
        print("Publishing...")
        self.speed_Publisher()
        
    def speed_Publisher(self):
	#self.orientation=[0, 0, 0,(math.pi)/2, (math.pi)/2, (math.pi)/2, (math.pi)/2, (math.pi)/2, 0, (math.pi)/2, 0,0,0,0]
	self.orientation=[0,0,0,(math.pi)/2, 0, 0, 0, 0]
        #self.orientation = [0,0,0,(math.pi)/2,(math.pi)/2,(math.pi)/2,(math.pi)/2,0,(math.pi)/2,0,0,0,0,0]
        # keep track of when making turnS
        #print("TEST 7th index: " + str(orientation[7]))
#[0, 0, 0, 270, 270, 270, 270, 270, 0, 270, 0, 0, 0]
	# Calculate D_ERROR in order to determine what goal the location is
        #self.waypoint = [[0,0], [1, 0], [2, 0], [2,1], [2,2], [2, 3]]
        self.waypoint = [[0,0], [1, 0], [2, 0], [3, 1], [4, 1], [4, 0]]
        #self.waypoint = [[0,0], [1, 0], [2, 0], [2,1], [2,2], [2, 3], [2, 4], [2, 5], [3, 5], [3, 6]]
#[4, 6], [5, 6], [6, 6], [7, 6]]
        #for elem in self.waypoint:
         #   elem[0] = .1175*elem[0]
          #  elem[1] = .1342*elem[1]
        print(self.waypoint)
        # when the robot is within 0.7m away of a waypint, move on to the next waypoint
        
        self.distancethresh = 0.07
        self.x_threshold = 0.1 # adjust as needed
	self.y_threshold = 0.1
 # adjust as needed


        current_way = self.waypoint[self.wayindex]
	desired_heading=self.orientation[self.wayindex]
        print("Current WAYPOINT: " + str(current_way))
        print("WAYINDEX = " + str(self.wayindex))


        # STORE THE INITIAL POSITION - ensures we start at (0,0)
        #if self.firstRun:
         #   self.firstRun = False
         #   SA_optimal = 0

        # get current position
        # subtract x error
        x_cord = self.current_pose.position.x
        print("original X cord:")
        print(x_cord)
        #if (self.wayindex >= 4):
            # average error for left turns
        #    x_cord -= .385
        #    print("X_CORD MINUS ERROR = " + str(x_cord))

        #x_cord -= self.initial_x
        y_cord = self.current_pose.position.y
        #y_cord -= self.initial_y
        z_cord = self.current_pose.position.z
        print("Y cord:")
        print(y_cord)

        # DISTANCE FROM WAYPOINT - calculate position error based off current waypoint
        #distance_from_waypoint = math.sqrt((current_way[0]) ** 2 + (current_way[1]) ** 2)
        # how does this change after you hit a waypoint?
        #total_dist_traveled = math.sqrt((x_cord**2) + (y_cord**2))
        #position_error = math.sqrt((x_cord-current_way[0]) ** 2 + (y_cord-current_way[1]) ** 2)
        	
        #print("POSITION ERROR:")
        #print(position_error)
        #print("DISTANCE THRESHHOLD:")
        #print(self.distancethresh)
        #if position_error < self.distancethresh:
            #print("execute next state yay!")
            #self.wayindex = (self.wayindex + 1)
            #current_way=self.waypoint[self.wayindex]
            #desired_heading=self.orientation[self.wayindex]
            #print("currently going towards " + str(self.waypoint[self.wayindex]))
        
        # Get current heading from quaternion orientation
        quaternion = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
            # generates the yaw pitch and roll
        euler = tf.transformations.euler_from_quaternion(quaternion)
        current_heading = euler[2]  # yaw
        # THIS ISN'T 0?
        #if (current_heading < 0.1) and (current_heading > -0.1):
         #   current_heasding = 0
          #  print("correct heading bc too smol")

        x_error = abs(x_cord - current_way[0])
	y_error = abs(y_cord - current_way[1])

	'''
        print("I AM CURRENTLY FACING " + str(current_heading))
        if (abs(current_heading - desired_heading) < 0.1) or self.errorFlag:
            if (self.wayindex >= 3):
                self.errorFlag = True
                # average error for left turns
                #x_cord -= .385
                x_cord -= .4
                y_cord -= .3
                #y_cord -= .24
                x_error = abs(x_cord - current_way[0])
	        y_error = abs(y_cord - current_way[1])
                print("X_CORD MINUS ERROR = " + str(x_cord))
                print("Y_CORD MINUS ERROR = " + str(y_cord))
                print
	'''   
        # - pi is RIGHT, + pi is LEFT

        #if x_error < self.x_threshold or (self.waypoint>2 and y_error < self.y_threshold):
        if x_error < self.x_threshold  and y_error < self.y_threshold:

            print("Eeecute next state yay!")
	    if (self.wayindex + 1 < len(self.waypoint)):
                self.wayindex = (self.wayindex + 1)
                current_way=self.waypoint[self.wayindex]
                desired_heading=self.orientation[self.wayindex]
                print("currently going towards " + str(self.waypoint[self.wayindex]))
		


	# Calculate desired heading
        #desired_heading = math.atan2(current_way[1] - y_cord, current_way[0] -x_cord)
        print("My desired heaading is "+ str(desired_heading))
        # atan(0 - -0.0102, 0 - 0.0394)
        
        # Calculate steering angle as difference between desired and current heading
        SA_optimal = desired_heading - current_heading
        print("BEFORE ADJUSTING IT, I WILL TURN MY WHEELS THIS MUCH: " + str(SA_optimal))

        # normalize pi values withing [-pi, pi]
        if SA_optimal > math.pi:
            print(SA_optimal)
            print("SA OPTIMAL WAS > MATH.PI")
            SA_optimal -= 2*math.pi
        elif SA_optimal <= -math.pi:
            print(SA_optimal)
            print("SA OPTIMAL WAS < -MATH.PI")
            SA_optimal += 2*math.pi
        else:
            print("DID NOT NORMALIZE")

        # why do twice??
        #SA_optimal = math.atan2(current_way[1] - y_cord, current_way[0] - x_cord)
        #SA_optimal = math.atan2(current_waypoint[1], current_waypoint[0]) - math.atan2(y_cord, x_cord);

        #print("CALCULATED ANGLE AFTER NORMALIZING = " + str(SA_optimal))
        #SA_optimal = round(SA_optimal, 2)



        print("Optimal Steering Angle = " + str(SA_optimal))
	#SA_optimal=1 #use this line to test an angle you want without cord list 
        Vx_optimal = 0.3
        
        #print("Current state = " + str(self.wayindex))
	   
        new_V=Twist()# initializing vars in Twist object
	 


        if current_heading == desired_heading:
            print("END turn TURN")
            SA_optimal = 0

	
	desired_angle = angle_Calc([x_cord, y_cord], current_way)
	rot = self.current_pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        new_V.linear.x = self.Speed_PID.pid_compute(Vx_optimal,self.currentVx)
        new_V.angular.z = (self.currentSA)+self.Speed_PID.pid_compute(desired_angle, theta)
	#new_V.angular.z = self.Speed_PID.pid_compute(SA_optimal,self.currentSA)
        print("Z val = " + str(new_V.angular.z))
	# publishes new speed to motor
	#print("we have published")
         
        #print(new_V)
	print(len(self.waypoint))
	if (self.wayindex + 1 > len(self.waypoint)):
	    new_V = Twist()

	if (time.time() - self.currTime > 40):
	    new_V = Twist()

	self.velocity_publisher.publish(new_V)
	
        #rate.sleep()

if __name__ == '__main__':
	print("new callback run--------------------------------------------------------------")
	controller = controllerNode()
        rospy.spin()
	#controller.run

