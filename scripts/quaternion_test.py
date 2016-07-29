#!/usr/bin/python

import sys
import rospy
import tf

print tf.transformations.euler_from_quaternion([0.0, 0.0, 0.0, 0.0])
