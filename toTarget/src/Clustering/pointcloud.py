#!/usr/bin/env python
import rospy 
import math
from geometry_msgs.msg import Point32 
from sensor_msgs.msg import PointCloud #msg.points is a list of gemotrty_msgs
import matplotlib.pyplot as plt
import numpy as np


def PC_sub(): 
	rospy.init_node("controller_node", anonymous = True) #start the controller node 

        #rospy.Rate(0.001)
        rospy.Subscriber('/point_cloud',PointCloud, Cloudcallback)#create a point cloud subscriber 
        print("cloud___loading")
        	
	 



def Cloudcallback(msg):
        print("-------------------------------------")
        point_list=[]

        x_list = []
        y_list = []
        inverted_ylist = []

        for point in msg.points: 
           
            x=point.x
            y=point.y
            point_list.append((round(x, 3), round(y, 3)))
            if (abs(x) <= 3 and x >= 0 and abs(y) <=3):
               x_list.append(round(x, 3)) 
               y_list.append(round(y, 3))
               inverted_y = -1 * y
               inverted_ylist.append(round(inverted_y, 3))
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
	#print(point_list)
	#inverted_xlist = -1 * x_list
        plt.scatter(inverted_ylist, x_list)
        plt.savefig("mypointclouddata.jpg")
        #plt.show()
        ourData = []
        for i in range (len(x_list)):
           ourData.append([inverted_ylist[i] , x_list[i]])
        #print(ourData)

        with open("ourData.txt", "w") as file:
           #file.writelines(f"{num}\n" num for num in ourData)
           for i in ourData:
              #print("i is :" + i)
              file.write(str(i) + ", ") 
        file.close()
        
        print(" ")
        print("-------------------------------------")
        



if __name__ == '__main__': 
    #startTime = time.time()
    
    


    while not rospy.is_shutdown():
        PC_sub()
        rate=rospy.Rate(2)
        rate.sleep()
    #rospy.spin()
    
        






