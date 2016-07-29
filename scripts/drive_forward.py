#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
rospy.init_node('meter test')
rate = rospy.Rate(10)

twist = Twist()
twist.linear.x = 0.01
twist.linear.y = 0.0
twist.linear.z = 0.0

i = 0

while not rospy.is_shutdown() and i < 100:
    pub.publish(twist)
    rate.sleep()

    i += 1
