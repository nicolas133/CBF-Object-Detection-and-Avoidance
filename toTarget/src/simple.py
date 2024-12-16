import rospy
from nav_msgs import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0.0
y = 0.0
theta = 0.0

def getInfo(msg):
    global x, y, theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odometry/filtered", Odometry, getInfo)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
speed = Twist()

goal = Point()

goal.x = 5
goal.y = 5

while not rospy.is_shutdown():
    angle = atan2(goal.x - x, goal.y - y)
    if(abs(angle) > 0.2):
        speed.linear.x = 0.0
        speed.angular = 0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0
    pub.publish(speed)
