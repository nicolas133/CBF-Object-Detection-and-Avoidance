#!/usr/bin/env python3

from pcl_helper import *
from filtering_helper import *
from sensor_msgs.msg import PointCloud, PointCloud2
from sensor_msgs import point_cloud2
import struct
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm
from scipy.spatial import ConvexHull
from sensor_msgs.msg import PointField
import matplotlib.patches as patches
#clusters = []



def filtering(cloud):

  # Downsample the cloud to lower resolution to make algo run faster 
  downsampled_cloud = do_voxel_grid_filter(point_cloud = cloud, LEAF_SIZE = 0.01)
  print("After voxel grid filter:", downsampled_cloud.size)


  #Spatial RANGE FILTER-implement  
  filtered_cloud = do_passthrough_filter(point_cloud = downsampled_cloud, 
                                         name_axis = 'x', min_axis = -1, max_axis = 1.0)

  filtered_cloud = do_passthrough_filter(point_cloud = downsampled_cloud, 
                                         name_axis = 'y', min_axis = -1, max_axis = 1)
  #print("After passthrough filter:", filtered_cloud.size)

 
   
  print("After Bounding Box",filtered_cloud)

  objects_cloud = filtered_cloud
 
# cloud is an array with each cell having three numbers corresponding to x, y, z position
# Returns list of [x, y, z, color]

  return objects_cloud 



def db_scan(cloud, tolerance, min_size, max_size,debug=False):

  # 'clusters' is a list of lists, with each index representing the cluster it belongs to 

  tree = cloud.make_kdtree()
  extraction_object = cloud.make_EuclideanClusterExtraction()

  extraction_object.set_ClusterTolerance(tolerance)
  extraction_object.set_MinClusterSize(min_size)
  extraction_object.set_MaxClusterSize(max_size)
  extraction_object.set_SearchMethod(tree)
  cluster_indices = extraction_object.Extract()
  
  # Get the X and Y data points for the cluster
  clusters = []
  points = cloud.to_array()
  
  # Run through the indices and get the X, Y coordinates of the points
  for indices in cluster_indices:
      cluster_points = points[indices][:, :2]
      clusters.append(cluster_points)

  # print only when debug 
  if debug == True:
      print("There are a total number of:", {len(clusters)}," Clusters")
  
     # print(clusters)
  
  # Create the graph for the three clusters and save the figure
  colors = cm.viridis(np.linspace(0, 1, len(clusters)))
  for i, cluster in enumerate(clusters):
      plt.scatter(cluster[:,0], cluster[:,1], color = colors[i], alpha=0.6)
  
  plt.savefig('dbscan_clusters.png')
  
  return cluster_indices, clusters


def visualize_convex_hull(cloud, clusters):
	
	for i in clusters:
		cluster_points = []
		cluster_points_x = []
		cluster_points_y  = []  
		for j in i:
			cluster_points.append(j)
			cluster_points_x.append(j[0])
			cluster_points_y.append(j[1])

		print(cluster_points)
		plt.scatter(cluster_points_x, cluster_points_y, label=f'Cluster {i}', s = 10)


		cluster_points_x = np.array(cluster_points_x)
		cluster_points_y = np.array(cluster_points_y)

		min_x = cluster_points_x.min()
		min_y = cluster_points_y.min()
		max_x = cluster_points_x.max()
		max_y = cluster_points_y.max()

		width = max_x + min_x
		length = max_y + min_y

		centroid_x = width/2
		centroid_y = length/2
		centroid_pair = (centroid_x, centroid_y)
		
		#width_length_pair = (abs(width), abs(length))
		#radius = max(width_length_pair)/2

		if (width > length) :
			np_max_pair = np.array([max_x,0])
		else:
			np_max_pair = np.array([0,max_y])
		np_max_pair = np.array(max_x, max_y)
		np_centroid_pair = np.array(centroid_pair)

		radius = np.linalg.norm(np_centroid_pair - np_max_pair)

		print("Centroid x: " , centroid_x)
		print("Centroid y: " , centroid_y)
		print("Centroid pair: " , centroid_pair)
		print("radius: " , radius)

		print("min and min" + str(min_x) + "," + str(min_y))
		print("max and max"+ str(max_x)  + "," + str(max_y))
             
                	

		rectangle = np.array([[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]])
		fig,ax =plt.subplots()
		circle=patches.Circle(centroid_pair,radius,fill=False,edgecolor="blue")	
		ax.add_patch(circle)
		plt.savefig("circle_image")
		

		plt.fill(rectangle[:,0], rectangle[:,1], linewidth=2,alpha=0.5, label=f'Cluster {i}')
	#plt.show()
	plt.savefig('Shape_Wrapping.png')
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
def pcl_callback(pcl_msg):

  print("entered pcl_callback")

  pcl2_msg = pointcloud_to_pointcloud2(pcl_msg) #convert to pcl2

  # Convert ROS msg to PCL data
  cloud = ros_to_pcl(pcl2_msg) 

  frame_id = pcl2_msg.header.frame_id

  print("Initial__cloud size:", cloud.size)


  # IMPLEMENT BOUNDING BOX AND LOWER RESOLUTION 
  objects_cloud = filtering(cloud) 

  # Get a point cloud of only the position information without color information
  colorless_cloud = XYZRGB_to_XYZ(objects_cloud)
  

  # This is effectively a list of lists, with each list containing indices of the cloud
  clusters_indicies, clusters = db_scan(colorless_cloud, tolerance = 0.05, min_size = 20, max_size = 1500,debug=True)
  
  visualize_convex_hull(cloud, clusters)

  # Assign a unique color float for each point (x, y, z)
  # Points with the same color belong to the same cluster
  colored_points = get_colored_clusters(clusters, colorless_cloud)

  # Create a cloud with each cluster of points having the same color
  clusters_cloud = pcl.PointCloud_PointXYZRGB()
  clusters_cloud.from_list(colored_points)

  # Convert pcl data to ros messages
 
  clusters_msg = pcl_to_ros(clusters_cloud, frame_id)

  # Publish ROS messages
  clusters_publisher.publish(clusters_msg)
 
def pointcloud_to_pointcloud2(pointcloud_msg):
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

if __name__ == '__main__':
  print("enterd main of clustering")
  # ROS node initialization
  rospy.init_node('clustering', anonymous = True)

    
  subscriber = rospy.Subscriber("/point_cloud", PointCloud, pcl_callback, queue_size = 1)

  # Create Publishers
  clusters_publisher = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)
  
  # Initialize color_list
  get_color_list.color_list = []

  # Spin while node is not shutdown
  while not rospy.is_shutdown():
    rospy.spin()
