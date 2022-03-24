#!/usr/bin/env python3

# from re import X
from math import atan2
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    rot_q = msg.pose.pose.orientation
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node("speed_controller1")
sub = rospy.Subscriber("/robot1/odom", Odometry, newOdom)
pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size = 10)

speed = Twist()
goal = Point()
r = rospy.Rate(4)
goal.x = 5
goal.y = 5
while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y
    angle_to_goal = atan2(inc_y, inc_x)
    if abs(angle_to_goal - theta) > 0.2:
        speed.linear.x = 0.0
        speed.angular.z = 0.5
    else:
        speed.linear.x = 1.5
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()
