#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

class controllerNode:
    def __init__(self):
        rospy.init_node('ekf_data_listener')
        print("Started 2")
        rospy.Subscriber('/odometry/filtered', Odometry, self.ekf_callback)

    def ekf_callback(self, msg): 
        print("Started 4")
        # Print the received EKF data (e.g., pose, twist)
        print("Received EKF data:")
        print("Position (x, y, z): {:.3f}, {:.3f}, {:.3f}".format(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z))
        print("Orientation (quaternion): {:.3f}, {:.3f}, {:.3f}, {:.3f}".format(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w))
        print("Linear Velocity (x, y, z): {:.3f}, {:.3f}, {:.3f}".format(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z))
        print("Angular Velocity (x, y, z): {:.3f}, {:.3f}, {:.3f}".format(
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z))
        print("")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = controllerNode()
    node.run()


