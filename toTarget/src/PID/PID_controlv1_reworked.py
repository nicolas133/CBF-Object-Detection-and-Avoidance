#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from singlePID import SinglePID
from geometry_msgs.msg import Twist


class controllerNode:
    def __init__(self):
        print("loop")
        rospy.init_node('contoller_node', anonymous = True)
        rospy.Subscriber('/odom', Odometry, self.callback)
	
        #rospy.spin()
	# publisher to robot topic
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=8)
        self.Speed_PID = SinglePID(P=1,I=0,D=0) # create single PID object
	
        rate = rospy.Rate(10)# rate at which robot calculates error and updates publisher data
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

    def callback(self, msg):
	
	#Get current V
        currentVx=msg.twist.twist.linear.x
        print("Current Vx:")
        print(currentVx)
	#Get current Angle
        currentSA=msg.twist.twist.angular.z
        print("Steering Angle:")
        print(currentSA)


	# set initial x pos to 0
        self.current_pose = msg.pose.pose

	
	# publisher call sends data to /cmd_vel 
        print("Publishing...")
        self.speed_Publisher()
        
        
    def speed_Publisher(self):

	# Calculate D_ERROR in order to determine what goal the location is
        self.waypoint = [(0,0), (0, 1), (0, 2), (1, 2), (2, 2), (3, 2), (4, 2), (5, 2), (5, 3), (6, 3), (6, 4), (6, 5), (6, 6), (6, 7)]
        # when the robot is within 0.3m away of a waypint, move on to the next waypoint
        self.distancethresh = 0.2
        
        current_way = self.waypoint[self.wayindex]

        print("CURRENT WAYPOINT and Next Waypoint")
        print(current_way)
        print(self.waypoint[self.wayindex + 1])




        # STORE THE INITIAL POSITION - ensures we start at (0,0)
        if self.firstRun:
            self.firstRun = False
            print(self.firstRun)
            print("CURRENT X POSITION: " + self.current_pose.position.x)
            print("CURRENT Y POSITION: " + self.current_pose.position.y)
            
            self.initial_x = self.current_pose.position.x
            self.initial_y = self.current_pose.position.y

        # get current position
        x_cord = self.current_pose.position.x
        x_cord -= self.initial_x
        y_cord = self.current_pose.position.y
        y_cord -= self.initial_y
        z_cord = self.current_pose.position.z

        # DISTANCE FROM WAYPOINT - calculate position error based off current waypoint
        '''
         distance_from_waypoint = math.sqrt((current_way[0]) ** 2 + (current_way[1]) ** 2)
        # how does this change after you hit a waypoint?
        total_dist_traveled = math.sqrt((x_cord**2) + (y_cord**2))
        '''
        
        position_error = dist(current_way, [x_cord, y_cord])

        #print("X cord: ", x_cord , "Y cord: " , y_cord)
        print("---------------------")
        print("POSITION ERROR: ")
        print(position_error)
        print("DISTANCE THRESHOLD: ")

        print(self.distancethresh)
        if position_error < self.distancethresh:
            print("execute next state yay!")
            self.wayindex += 1
        current_waypoint=self.waypoint[self.wayindex]
        print(current_waypoint)
        SA_optimal = math.atan2(current_waypoint[1] - y_cord, current_waypoint[0] - x_cord)
	#SA_optimal=1 #use this line to test an angle you want without cord list 
        Vx_optimal =0.0
        

	   
        new_V=Twist()# initializing vars in Twist object
	 
        new_V.linear.x = self.Speed_PID.pid_compute(Vx_optimal,self.currentVx)
        new_V.angular.z = self.Speed_PID.pid_compute(SA_optimal,self.currentSA)
	# publishes new speed to motor
	#print("we have published")
        #print(new_V)
        self.velocity_publisher.publish(new_V)
        #rate.sleep()
 


	    
                    
'''
Issue: can't gauge distance from current waypoint and next one soley based on your x-cord., what if you're turning?
Solution idea: keep track of current and next waypoints, and grab distance from those two with an
incrementing value -> after 'next state yay!' is triggered, save current_x and current_y, and calculate
position error as the differneces of those values and current. pos_err = sqrt(cur^2) - (old_x + old_y)^2)
Goodluck!
'''

    
def dist(current_way, currentCoords):
        x_cord = currentCoords[0]
        y_cord = currentCoords[1]
        distance_from_waypoint = math.sqrt((current_way[0]) ** 2 + (current_way[1]) ** 2)
        # how does this change after you hit a waypoint?
        total_dist_traveled = math.sqrt((x_cord**2) + (y_cord**2))
        return distance_from_waypoint - total_dist_traveled

if __name__ == '__main__':
        print("new callback run--------------------------------------------------------------")
        controller = controllerNode()
        rospy.spin()
        
	#!/usr/bin/env python
