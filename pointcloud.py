#!/usr/bin/env python
import rospy 
import math
from geometry_msgs.msg import Point32 
from sensor_msgs.msg import PointCloud #msg.points is a list of gemotrty_msgs


def PC_sub(): 
	rospy.init_node("controller_node", anonymous = True) #start the controller node 

        #rospy.Rate(0.001)
        rospy.Subscriber('/point_cloud',PointCloud, Cloudcallback)#create a point cloud subscriber 
        print("cloud___loading")
        	
	 



def Cloudcallback(msg):
        print("-------------------------------------")
        point_list=[]

        for point in msg.points: 
            x=point.x
            y=point.y
            point_list.append((round(x, 3), round(y, 3)))
            #print(point_list)
        #if len(point_list) > 0:
         #   print()
          #  print()
           # print("data is here")
            #print()
            #print()
	#print(point_list[0])
        #if (time.time() - startTime > 2):
    	    #startTime = time.time()
        #for i in range(0, 9):
        #    if (i % 3 == 0):
        #        print(point_list[i])
        #        print(" ")
  	#    else:
        #        print point_list[i],
	print(point_list)
        print(" ")
        print("-------------------------------------")
        



if __name__ == '__main__': 
    #startTime = time.time()
    
    


    while not rospy.is_shutdown():
        PC_sub()
        rate=rospy.Rate(2)
        rate.sleep()
    #rospy.spin()
    
        






