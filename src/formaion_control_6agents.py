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

x5 = 0.0
y5 = 0.0
theta5 = 0.0
theta5_err = 0.0

x6 = 0.0
y6 = 0.0
theta6 = 0.0
theta6_err = 0.0

# Setting initial values for the gains
# increase kp till oscillations occur
K1 = 0.05
K2 = 0.05
K3 = 0.05
K4 = 0.05
K5 = 0.05
K6 = 0.05

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

def newOdom5(msg):
    #subscriber function which is called when a new message of type Odometry of robot5 is received by the subscriber
    global x5
    global y5
    global theta5

    rotation_quaternion5 = msg.pose.pose.orientation
    x5 = msg.pose.pose.position.x
    y5 = msg.pose.pose.position.y
    (roll5, pitch5, theta5) = euler_from_quaternion([rotation_quaternion5.x, rotation_quaternion5.y, rotation_quaternion5.z, rotation_quaternion5.w])

def newOdom6(msg):
    #subscriber function which is called when a new message of type Odometry of robot6 is received by the subscriber
    global x6
    global y6
    global theta6

    rotation_quaternion6 = msg.pose.pose.orientation
    x6 = msg.pose.pose.position.x
    y6 = msg.pose.pose.position.y
    (roll6, pitch6, theta6) = euler_from_quaternion([rotation_quaternion6.x, rotation_quaternion6.y, rotation_quaternion6.z, rotation_quaternion6.w])
    
rospy.init_node("formation_control_for_6agents")
sub1 = rospy.Subscriber("/robot1/odom", Odometry, newOdom1)
sub2 = rospy.Subscriber("/robot2/odom", Odometry, newOdom2)
sub3 = rospy.Subscriber("/robot3/odom", Odometry, newOdom3)
sub4 = rospy.Subscriber("/robot4/odom", Odometry, newOdom4)
sub5 = rospy.Subscriber("/robot5/odom", Odometry, newOdom5)
sub6 = rospy.Subscriber("/robot6/odom", Odometry, newOdom6)

pub1 = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=1)
pub2 = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=1)
pub3 = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=1)
pub4 = rospy.Publisher("/robot4/cmd_vel", Twist, queue_size=1)
pub5 = rospy.Publisher("/robot5/cmd_vel", Twist, queue_size=1)
pub6 = rospy.Publisher("/robot6/cmd_vel", Twist, queue_size=1)

speed1 = Twist()
speed2 = Twist()
speed3 = Twist()
speed4 = Twist()
speed5 = Twist()
speed6 = Twist()

goal = Point()
r = rospy.Rate(1)
goal.x = 5
goal.y = 5
t = 1.0

while not rospy.is_shutdown():
    xf1 = goal.x + 1
    yf1 = goal.y + 0 
    xf2 = goal.x + 0.5
    yf2 = goal.y + 1
    xf3 = goal.x - 0.5
    yf3 = goal.y + 1
    xf4 = goal.x - 1
    yf4 = goal.y - 0
    xf5 = goal.x - 0.5
    yf5 = goal.y - 1
    xf6 = goal.x + 0.5
    yf6 = goal.y - 1

    inc_x1 = xf1 - x1
    inc_y1 = yf1 - y1
    inc_x2 = xf2 - x2
    inc_y2 = yf2 - y2
    inc_x3 = xf3 - x3
    inc_y3 = yf3 - y3
    inc_x4 = xf4 - x4
    inc_y4 = yf4 - y4
    inc_x5 = xf5 - x5
    inc_y5 = yf5 - y5
    inc_x6 = xf6 - x6
    inc_y6 = yf6 - y6

    theta1_err = atan2(inc_y1, inc_x1)
    theta2_err = atan2(inc_y2, inc_x2)
    theta3_err = atan2(inc_y3, inc_x3)
    theta4_err = atan2(inc_y4, inc_x4)
    theta5_err = atan2(inc_y5, inc_x5)
    theta6_err = atan2(inc_y6, inc_x6)

    dist1 = abs(math.sqrt(inc_x1**2 + inc_y1**2))
    dist2 = abs(math.sqrt(inc_x2**2 + inc_y2**2))
    dist3 = abs(math.sqrt(inc_x3**2 + inc_y3**2))
    dist4 = abs(math.sqrt(inc_x4**2 + inc_y4**2))
    dist5 = abs(math.sqrt(inc_x5**2 + inc_y5**2))
    dist6 = abs(math.sqrt(inc_x6**2 + inc_y6**2))

    lin_speed = K1*dist1
    lin_speed1 = K2*dist2
    lin_speed2 = K3*dist3
    lin_speed3 = K4*dist4
    lin_speed4 = K5*dist5
    lin_speed5 = K6*dist6

    des_ang_goal = theta1_err - theta1
    des_ang_goal1 = theta2_err - theta2
    des_ang_goal2 = theta3_err - theta3
    des_ang_goal3 = theta4_err - theta4
    des_ang_goal4 = theta5_err - theta5
    des_ang_goal5 = theta6_err - theta6

    angular_speed = K_t*(des_ang_goal)
    angular_speed1 = K_t*(des_ang_goal1)
    angular_speed2 = K_t*(des_ang_goal2)
    angular_speed3 = K_t*(des_ang_goal3)
    angular_speed4 = K_t*(des_ang_goal4)
    angular_speed5 = K_t*(des_ang_goal5)

    speed1.linear.x = lin_speed
    speed1.angular.z = angular_speed

    speed2.linear.x = lin_speed1
    speed2.angular.z = angular_speed1

    speed3.linear.x = lin_speed2
    speed3.angular.z = angular_speed2

    speed4.linear.x = lin_speed3
    speed4.angular.z = angular_speed3

    speed5.linear.x = lin_speed4
    speed5.angular.z = angular_speed4

    speed6.linear.x = lin_speed5
    speed6.angular.z = angular_speed5

    pub1.publish(speed1)
    pub2.publish(speed2)
    pub3.publish(speed3)
    pub4.publish(speed4)
    pub5.publish(speed5)
    pub6.publish(speed6)

    print('x1= ', x1, 'y1= ', y1, 'distance to goal for agent 1=  ', dist1)
    print('x2= ', x2, 'y2= ', y2, 'distance to goal for agent 2=  ', dist2)
    print('x3= ', x3, 'y3= ', y3, 'distance to goal for agent 3=  ', dist3)
    print('x4= ', x4, 'y4= ', y4, 'distance to goal for agent 4=  ', dist4)
    print('x5= ', x5, 'y5= ', y5, 'distance to goal for agent 5=  ', dist5)
    print('x6= ', x6, 'y6= ', y6, 'distance to goal for agent 6=  ', dist6)

    if dist1 < 0.1:
        speed1.linear.x = 0.0
        speed1.angular.z = 0.0
        speed2.linear.x = 0.0
        speed2.angular.z = 0.0
        speed3.linear.x = 0.0
        speed3.angular.z = 0.0
        speed4.linear.x = 0.0
        speed4.angular.z = 0.0
        speed5.linear.x = 0.0
        speed5.angular.z = 0.0
        speed6.linear.x = 0.0
        speed6.angular.z = 0.0

        pub1.publish(speed1)
        pub2.publish(speed2)
        pub3.publish(speed3)
        pub4.publish(speed4)
        pub5.publish(speed5)
        pub6.publish(speed6)
        break

    r.sleep()