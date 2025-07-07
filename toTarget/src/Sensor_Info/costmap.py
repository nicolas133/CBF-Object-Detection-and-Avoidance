#!/usr/bin/env python 
import rospy 
from nav_msgs.msg import OccupancyGrid 

def costmap_callback(msg): 
# Callback function to handle incoming laser scan messages  

        width = msg.info.width
        height = msg.info.height

        resolution = msg.info.resolution
        origin = msg.info.origin

        grid_data = msg.data

        center_index = width // 2 + (height // 2) * width
        occupancy = grid_data[center_index]

        #print(f"Received occupancy data at origin: {occupancy}")
        #for i in range(1, 60):
        #    print("Iteration " + str(i) + ": ")
        #    for j in range(1 * i, 60 * i):
        #        print(str(grid_data[j]) + ", ", end='')
        for i in range(1, 3600):
            if (i % 60 == 0):
		print(grid_data[i])
  	    	print(" ")
  	    else:
	        print grid_data[i],
              

def costmap_subscriber(): 
	rospy.init_node('costmap', anonymous=True) 
	rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, costmap_callback) 	
	rospy.spin() # Keeps Python from exiting until this node is stopped 

if __name__ == '__main__': 
	try: 
		costmap_subscriber() 
	except rospy.ROSInterruptException: 
		pass
