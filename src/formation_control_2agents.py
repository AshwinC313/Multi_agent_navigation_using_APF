#!/usr/bin/env python3
 
import rospy 
import math
from math import atan2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

x1 = 0.0
y1 = 0.0
theta1 = 0.0
theta1_err = 0.0

x2 = 0.0
y2 = 0.0
theta2 = 0.0
theta2_err = 0.0

# Setting initial values for the gains
# increase kp till oscillations occur
K1 = 0.05
K2 = 0.05
K_12 = 15.0
K_t = 0.4

def newOdom1(msg):
    #subscriber function which is called when a new message of type Odometry of robot1 is received by the subscriber
    global x1
    global y1
    global theta1

    rotation_quaternion1 = msg.pose.pose.orientation
    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y
    (roll1, pitch1, theta1) = euler_from_quaternion([rotation_quaternion1.x, rotation_quaternion1.y, rotation_quaternion1.z, rotation_quaternion1.w])

def newOdom2(msg):
    #subscriber function which is called when a new message of type Odometry of robot2 is received by the subscriber
    global x2
    global y2
    global theta2

    rotation_quaternion2 = msg.pose.pose.orientation
    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y
    (roll2, pitch2, theta2) = euler_from_quaternion([rotation_quaternion2.x, rotation_quaternion2.y, rotation_quaternion2.z, rotation_quaternion2.w])

rospy.init_node("formation_control_for_2agents")
sub1 = rospy.Subscriber("/robot1/odom", Odometry, newOdom1)
sub2 = rospy.Subscriber("/robot2/odom", Odometry, newOdom2)

pub1 = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=1)
pub2 = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=1)

speed1 = Twist()
speed2 = Twist()

goal = Point()
r = rospy.Rate(1)
goal.x = 5
goal.y = 5
t = 1.0

while not rospy.is_shutdown():
    xf1 = goal.x
    yf1 = goal.y + 1 
    xf2 = goal.x
    yf2 = goal.y - 1

    inc_x1 = xf1 - x1
    inc_y1 = yf1 - y1
    inc_x2 = xf2 - x2
    inc_y2 = yf2 - y2

    theta1_err = atan2(inc_y1, inc_x1)
    theta2_err = atan2(inc_y2, inc_x2)

    dist1 = abs(math.sqrt(inc_x1**2 + inc_y1**2))
    dist2 = abs(math.sqrt(inc_x2**2 + inc_y2**2))

    lin_speed = K1*dist1
    lin_speed1 = K2*dist2

    des_ang_goal = theta1_err - theta1
    des_ang_goal1 = theta2_err - theta2

    angular_speed = K_t*(des_ang_goal)
    angular_speed1 = K_t*(des_ang_goal1)

    speed1.linear.x = lin_speed
    speed1.angular.z = angular_speed

    speed2.linear.x = lin_speed1
    speed2.angular.z = angular_speed1

    pub1.publish(speed1)
    pub2.publish(speed2)

    print('x1= ', x1, 'y1= ', y1, 'distance to goal for agent 1=  ', dist1)
    print('x2= ', x2, 'y2= ', y2, 'distance to goal for agent 2=  ', dist2)

    if dist1 < 0.1:
        speed1.linear.x = 0.0
        speed1.angular.z = 0.0
        speed2.linear.x = 0.0
        speed2.angular.z = 0.0
        pub1.publish(speed1)
        pub2.publish(speed2)
        break

    r.sleep()