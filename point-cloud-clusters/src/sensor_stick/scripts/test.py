#!/usr/bin/env python3

from pcl_helper import *
from filtering_helper import *
from sensor_msgs.msg import PointCloud, PointCloud2
from geometry_msgs.msg import Twist
from sensor_msgs import point_cloud2
import struct
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm
from std_msgs.msg import Float64MultiArray
from scipy.spatial import ConvexHull
from sensor_msgs.msg import PointField
import matplotlib.patches as patches
from geometry_msgs.msg import Pose, PoseArray
import math 
from datetime import datetime, timedelta


time_delta = 1.0
#init_time = datetime.now()


# clusters = []

class Clustering_Shaping:

	def __init__(self):

             self.full_timer = datetime.now()
             print("enterd init of clustering")
             # ROS node initialization
             rospy.init_node('clustering', anonymous=True)

             rospy.Subscriber("/point_cloud", PointCloud, self.pcl_callback, queue_size=1)

             # Create Publishers
             self.clusters_publisher = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

             #self.wrapped_obj_pub = rospy.Publisher("WrappedObjs",Twist,queue_size=1)
             #self.wrapped_obj_pub = rospy.Publisher("WrappedObjs",Float64MultiArray,queue_size=1)
             self.wrapped_obj_pub = rospy.Publisher("WrappedObjs",PoseArray,queue_size=1)


             #publish # of clusters
             self.num_clusters_pub = rospy.Publisher("num_clusters",Twist, queue_size=1)
             

             self.init_time = datetime.now()

             # Initialize color_list
             get_color_list.color_list = []
          





	def filtering(self,cloud):
            filtering_time = datetime.now()
            x_limit = 2
            y_limit = 2
	    # Downsample the cloud to lower resolution to make algo run faster
            downsampled_cloud = do_voxel_grid_filter(point_cloud=cloud, LEAF_SIZE=0.01)
            """
            print("---------------------------------------------------------")
            print(downsampled_cloud[5])
            print("---------------------------------------------------------")
            print("After voxel grid filter:", downsampled_cloud.size)
 

	    # Spatial RANGE FILTER-implement
            filtered_cloud = do_passthrough_filter(point_cloud=downsampled_cloud,
		                                   name_axis='x', min_axis=-1,  max_axis=1.0)

            filtered_cloud = do_passthrough_filter(point_cloud=downsampled_cloud,
		                                   name_axis='y', min_axis=-1, max_axis=1)
            print(filtered_cloud[5])
            print("---------------------------------------------------------")
	    # print("After passthrough filter:", filtered_cloud.size)

            print("After Bounding Box", filtered_cloud)
            
            objects_cloud = filtered_cloud

	    # cloud is an array with each cell having three numbers corresponding to x, y, z position
	    # Returns list of [x, y, z, color]
            new_filter_time = datetime.now()
            print("Filtering Function Time:", new_filter_time - filtering_time)
            """
            objects_cloud = []
            for i in range(downsampled_cloud.size):
               if(downsampled_cloud[i][0] < x_limit and abs(downsampled_cloud[i][1]) < y_limit):
                   objects_cloud.append(downsampled_cloud[i])

            print(len(objects_cloud))
            return objects_cloud


	def db_scan(self,cloud, tolerance, min_size, max_size, debug=False):
	    # 'clusters' is a list of lists, with each incidiy representing the cluster it belongs to
	    curr_time = datetime.now()

	    tree = cloud.make_kdtree()
            
	    extraction_object = cloud.make_EuclideanClusterExtraction()

	    extraction_object.set_ClusterTolerance(tolerance)
	    extraction_object.set_MinClusterSize(min_size)
	    extraction_object.set_MaxClusterSize(max_size)
	    extraction_object.set_SearchMethod(tree)
	    cluster_indices = extraction_object.Extract()

	    #cluster inidices cluster_indices[0] = [0, 1, 2] means points at indices 0, 1, and 2 in cloud belong to cluster 0.


	    # Get the X and Y data points for the cluster
	    clusters = []

	    points = cloud.to_array() #convert to numpy array but shape is stll nx4

	    # Run through the indices and get the X, Y coordinates of the points
	    for indices in cluster_indices:
		# for cluster 0 go to 1,2,3 the rows which have points that belong to cluster 0
                cluster_points = points[indices][:, 0:2] #grab all rows but only grab columns 0 and 1 up to but not including 2

                clusters.append(cluster_points) # now we have xy array


	    # print only when debug
	    if debug == True:
                print("There are a total number of:", {len(clusters)}, " Clusters")
	    numcluster = Twist()
	    numcluster.linear.x = len(clusters)
            
	    self.num_clusters_pub.publish(numcluster)

	    # print(clusters)

	    # Create the graph for the three clusters and save the figure
	    colors = cm.viridis(np.linspace(0, 1, len(clusters)))

	    #fig, ax =plt.subplots()


	    #for i, cluster in enumerate(clusters):
                #ax.scatter(cluster[:, 0], cluster[:, 1], color=colors[i], alpha=0.6)

	    #plt.savefig('dbscan_clusters.png')

	    new_time = datetime.now()
	    time_interval = new_time - curr_time
	    print("Time interval is " + str(time_interval))
            

	    return cluster_indices, clusters


	def bounding_box(self,cloud, clusters):

            box_curr_time = datetime.now()
            fig, ax2 = plt.subplots()
            obj_array=PoseArray()
            
            for i in clusters:
                cluster_points = []
                cluster_points_x = []
                cluster_points_y = []
                for j in i:
                    cluster_points.append(j)
                    cluster_points_x.append(j[0])
                    cluster_points_y.append(j[1])

		#print(cluster_points)

		
                #ax2.scatter(cluster_points_x, cluster_points_y, label=f'Cluster {i}', s=10)

                cluster_points_x = np.array(cluster_points_x)
                cluster_points_y = np.array(cluster_points_y)

                min_x = cluster_points_x.min()
                min_y = cluster_points_y.min()
                max_x = cluster_points_x.max()
                max_y = cluster_points_y.max()

                width = max_x - min_x
                length = max_y - min_y

                half_width = width/2
                half_length = length/2

                centroid_x = (max_x + min_x) / 2
                centroid_y = (max_y + min_y) / 2
                centroid_pair = (centroid_x, centroid_y)

		# width_length_pair = (abs(width), abs(length))
		# radius = max(width_length_pair)/2

		#np_max_pair = ([half_width,half_length])
		# if use this then (0,0) is second point in Euclidian distance calucation
		# as this is already distance refrenced from centroid

                np_max_pair = np.array([max_x, max_y])

                np_centroid_pair = np.array(centroid_pair)

		#radius = np.linalg.norm(np_max_pair-np_centroid_pair)
                result= ((np_max_pair[0]-np_centroid_pair[0])**2+(np_max_pair[1]-np_centroid_pair[1])**2)

                radius = (math.sqrt(result))*.9

                print("Centroid x: ", centroid_x)
                print("Centroid y: ", centroid_y)
                print("Centroid pair: ", centroid_pair)
                print("radius: ", radius)

                print("min_x and min_y" + str(min_x) + "," + str(min_y))
                print("max_x and max_y" + str(max_x) + "," + str(max_y))

	       # rectangle = np.array([[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]])##npt used to print anymore

		#fig, ax = plt.subplots()
                circle = patches.Circle(centroid_pair, radius, fill=False, edgecolor="blue")
                rectangle= patches.Rectangle((min_x,min_y),width,length,linewidth=1, edgecolor='r', facecolor='none',fill=False)


                #ax2.add_patch(rectangle)
                #ax2.add_patch(circle)


                #wrapped_objs = Twist()
                #wrapped_objs.linear.x = centroid_x
                #wrapped_objs.linear.y = centroid_y
                #wrapped_objs.linear.z = radius
                #self.wrapped_obj_pub.publish(wrapped_objs)
              
                wrapped_objs = Pose()
                wrapped_objs.position.x = centroid_x
                wrapped_objs.position.y = centroid_y
                wrapped_objs.position.z = radius
                obj_array.poses.append(wrapped_objs)
                box_end_time = datetime.now()
                box_time_interval = box_end_time - box_curr_time
                print("Time interval for wrapping is " + str(box_time_interval))
                full_time_interval = box_end_time - self.full_timer
                print("Full Time Interval :" + str(full_time_interval))
                
                
                
                
               


		#plt.fill(rectangle[:, 0], rectangle[:, 1], linewidth=2, alpha=0.5, label=f'Cluster {i}')



	    # save final plot
            self.wrapped_obj_pub.publish(obj_array)
            #plt.savefig('clusters_with_bounding_box')
            return


	# clusters is a list of lists each list containing indices of the cloud

	def get_colored_clusters(clusters, cloud):
	    # Get a random unique colors for each object
	    number_of_clusters = len(clusters)
	    colors = get_color_list(number_of_clusters)

	    colored_points = []

	    # Assign a color for each point
	    # Points with the same color belong to the same cluster
	    for cluster_id, cluster in enumerate(clusters):
               for c, i in enumerate(cluster):
                    x, y, z = cloud[i][0], cloud[i][1], cloud[i][2]
                    color = rgb_to_float(colors[cluster_id])
                    colored_points.append([x, y, z, color])

	    return colored_points


	# Callback function for your Point Cloud Subscriber

	def pointcloud_to_pointcloud2(self,pointcloud_msg):
	    header = pointcloud_msg.header

	    # Define the fields for PointCloud2, including 'rgb'
	    fields = [
		PointField('x', 0, PointField.FLOAT32, 1),
		PointField('y', 4, PointField.FLOAT32, 1),
		PointField('z', 8, PointField.FLOAT32, 1),
		PointField('rgb', 12, PointField.FLOAT32, 1),
	    ]

	    # Create a list of points with default RGB values
	    points = []
	    for point in pointcloud_msg.points:
                x, y, z = point.x, point.y, point.z
		# Set default color to white (255, 255, 255)
                r, g, b = 255, 255, 255
                rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                rgb_float = struct.unpack('f', struct.pack('I', rgb))[0]
                points.append([x, y, z, rgb_float])

	    # Create PointCloud2 message
	    pcl2_msg = point_cloud2.create_cloud(header, fields, points)
	    return pcl2_msg




	def pcl_callback(self,pcl_msg):
	    self.full_timer = datetime.now()
	    print("entered pcl_callback")

	    pcl2_msg = self.pointcloud_to_pointcloud2(pcl_msg)  # convert to pcl2

	    # Convert ROS msg to PCL data
	    cloud = ros_to_pcl(pcl2_msg)

	    frame_id = pcl2_msg.header.frame_id

	    print("Initial__cloud size:", cloud.size)

	    # IMPLEMENT BOUNDING BOX AND LOWER RESOLUTION
	    objects_cloud = self.filtering(cloud)

	    # Get a point cloud of only the position information without color information
	    colorless_cloud = XYZRGB_to_XYZ(objects_cloud)

            #init_time = datetime.now() #change to subscribed time
	    next_time = self.init_time + timedelta(seconds=time_delta)

	    current_time = datetime.now()

	    if (current_time >= next_time):

	    # This is effectively a list of lists, with each list containing indices of the cloud
	    	clusters_indicies, clusters = self.db_scan(colorless_cloud, tolerance=0.05, min_size=20, max_size=1500, debug=True)

	    	self.bounding_box(cloud, clusters)
	    	self.init_time = datetime.now()

	    current_time = datetime.now()
	    


	    # Assign a unique color float for each point (x, y, z)
	    # Points with the same color belong to the same cluster
	    colored_points = get_colored_clusters(clusters, colorless_cloud)

	    # Create a cloud with each cluster of points having the same color
	    clusters_cloud = pcl.PointCloud_PointXYZRGB()
	    clusters_cloud.from_list(colored_points)

	    # Convert pcl data to ros messages

	    clusters_msg = pcl_to_ros(clusters_cloud, frame_id)

	    # Publish ROS messages
	    self.clusters_publisher.publish(clusters_msg)


	


if __name__ == '__main__':
    print("new callback run--------------------------------------------------------------")
    Detected_Objects = Clustering_Shaping()
    rospy.spin()
