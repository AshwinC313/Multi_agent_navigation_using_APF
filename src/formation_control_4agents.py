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

x3 = 0.0
y3 = 0.0
theta3 = 0.0
theta3_err = 0.0

x4 = 0.0
y4 = 0.0
theta4 = 0.0
theta4_err = 0.0

# Setting initial values for the gains
# increase kp till oscillations occur
K1 = 0.05
K2 = 0.05
K3 = 0.05
K4 = 0.05
#K_12 = 15.0
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

def newOdom3(msg):
    #subscriber function which is called when a new message of type Odometry of robot3 is received by the subscriber
    global x3
    global y3
    global theta3

    rotation_quaternion3 = msg.pose.pose.orientation
    x3 = msg.pose.pose.position.x
    y3 = msg.pose.pose.position.y
    (roll3, pitch3, theta3) = euler_from_quaternion([rotation_quaternion3.x, rotation_quaternion3.y, rotation_quaternion3.z, rotation_quaternion3.w])

def newOdom4(msg):
    #subscriber function which is called when a new message of type Odometry of robot4 is received by the subscriber
    global x4
    global y4
    global theta4

    rotation_quaternion4 = msg.pose.pose.orientation
    x4 = msg.pose.pose.position.x
    y4 = msg.pose.pose.position.y
    (roll4, pitch4, theta4) = euler_from_quaternion([rotation_quaternion4.x, rotation_quaternion4.y, rotation_quaternion4.z, rotation_quaternion4.w])


rospy.init_node("formation_control_for_4agents")
sub1 = rospy.Subscriber("/robot1/odom", Odometry, newOdom1)
sub2 = rospy.Subscriber("/robot2/odom", Odometry, newOdom2)
sub3 = rospy.Subscriber("/robot3/odom", Odometry, newOdom3)
sub4 = rospy.Subscriber("/robot4/odom", Odometry, newOdom4)

pub1 = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=1)
pub2 = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=1)
pub3 = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=1)
pub4 = rospy.Publisher("/robot4/cmd_vel", Twist, queue_size=1)

speed1 = Twist()
speed2 = Twist()
speed3 = Twist()
speed4 = Twist()

goal = Point()
r = rospy.Rate(1)
goal.x = 5
goal.y = 5
t = 1.0

while not rospy.is_shutdown():
    xf1 = goal.x + 1
    yf1 = goal.y + 1 
    xf2 = goal.x - 1
    yf2 = goal.y + 1
    xf3 = goal.x - 1
    yf3 = goal.y - 1
    xf4 = goal.x + 1
    yf4 = goal.y - 1

    inc_x1 = xf1 - x1
    inc_y1 = yf1 - y1
    inc_x2 = xf2 - x2
    inc_y2 = yf2 - y2
    inc_x3 = xf3 - x3
    inc_y3 = yf3 - y3
    inc_x4 = xf4 - x4
    inc_y4 = yf4 - y4


    theta1_err = atan2(inc_y1, inc_x1)
    theta2_err = atan2(inc_y2, inc_x2)
    theta3_err = atan2(inc_y3, inc_x3)
    theta4_err = atan2(inc_y4, inc_x4)

    dist1 = abs(math.sqrt(inc_x1**2 + inc_y1**2))
    dist2 = abs(math.sqrt(inc_x2**2 + inc_y2**2))
    dist3 = abs(math.sqrt(inc_x3**2 + inc_y3**2))
    dist4 = abs(math.sqrt(inc_x4**2 + inc_y4**2))

    lin_speed = K1*dist1
    lin_speed1 = K2*dist2
    lin_speed2 = K3*dist3
    lin_speed3 = K4*dist4

    des_ang_goal = theta1_err - theta1
    des_ang_goal1 = theta2_err - theta2
    des_ang_goal2 = theta3_err - theta3
    des_ang_goal3 = theta4_err - theta4

    angular_speed = K_t*(des_ang_goal)
    angular_speed1 = K_t*(des_ang_goal1)
    angular_speed2 = K_t*(des_ang_goal2)
    angular_speed3 = K_t*(des_ang_goal3)

    speed1.linear.x = lin_speed
    speed1.angular.z = angular_speed

    speed2.linear.x = lin_speed1
    speed2.angular.z = angular_speed1

    speed3.linear.x = lin_speed2
    speed3.angular.z = angular_speed2

    speed4.linear.x = lin_speed3
    speed4.angular.z = angular_speed3

    pub1.publish(speed1)
    pub2.publish(speed2)
    pub3.publish(speed3)
    pub4.publish(speed4)

    print('x1= ', x1, 'y1= ', y1, 'distance to goal for agent 1=  ', dist1)
    print('x2= ', x2, 'y2= ', y2, 'distance to goal for agent 2=  ', dist2)
    print('x3= ', x3, 'y3= ', y3, 'distance to goal for agent 3=  ', dist3)
    print('x4= ', x4, 'y4= ', y4, 'distance to goal for agent 4=  ', dist4)

    if dist1 < 0.1:
        speed1.linear.x = 0.0
        speed1.angular.z = 0.0
        speed2.linear.x = 0.0
        speed2.angular.z = 0.0
        speed3.linear.x = 0.0
        speed3.angular.z = 0.0
        speed4.linear.x = 0.0
        speed4.angular.z = 0.0

        pub1.publish(speed1)
        pub2.publish(speed2)
        pub3.publish(speed3)
        pub4.publish(speed4)
        break

    r.sleep()