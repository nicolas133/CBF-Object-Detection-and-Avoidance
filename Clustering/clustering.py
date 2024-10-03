#!/usr/bin/env python3

from pcl_helper import *
from filtering_helper import *
from sensor_msgs.msg import PointCloud, PointCloud2
from sensor_msgs import point_cloud2
import struct
from sensor_msgs.msg import PointField




def filtering(cloud):

  # Downsample the cloud to lower resolution to make algo run faster 
  downsampled_cloud = do_voxel_grid_filter(point_cloud = cloud, LEAF_SIZE = 0.01)
  print("After voxel grid filter:", downsampled_cloud.size)


  #Spatial RANGE FILTER-implement  
  filtered_cloud = do_passthrough_filter(point_cloud = downsampled_cloud, 
                                         name_axis = 'x', min_axis = -2, max_axis = 2.0)

  filtered_cloud = do_passthrough_filter(point_cloud = downsampled_cloud, 
                                         name_axis = 'y', min_axis = -2, max_axis = 2)
  #print("After passthrough filter:", filtered_cloud.size)

 
   
  print("post_bounding_box",filtered_cloud)

  objects_cloud = filtered_cloud
 
# cloud is an array with each cell having three numbers corresponding to x, y, z position
# Returns list of [x, y, z, color]

  return objects_cloud 



def db_scan(cloud, tolerance, min_size, max_size,debug=False):

  # 'clusters' is a list of lists, with each incidiy representing the cluster it belongs to 

  tree = cloud.make_kdtree()
  extraction_object = cloud.make_EuclideanClusterExtraction()

  extraction_object.set_ClusterTolerance(tolerance)
  extraction_object.set_MinClusterSize(min_size)
  extraction_object.set_MaxClusterSize(max_size)
  extraction_object.set_SearchMethod(tree)
  clusters = extraction_object.Extract()


  # print only when debug 
  if debug == True:
      print("There are a total number of:", {len(clusters)}," CLUSTERS")
  
      print(clusters) 
  return clusters
  

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
  clusters = db_scan(colorless_cloud, tolerance = 0.05, min_size = 20, max_size = 1500,debug=True)

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
        rgb = 0xFFFFFF  # Set RGB to white
        rgb_float = struct.unpack('f', struct.pack('I', rgb))[0]
      # expects a Float32 
      # first we need to pack python val to 32bit int turn
      # next we unpack and turn into float
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
