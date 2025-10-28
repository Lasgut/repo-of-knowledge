#!/usr/bin/env python3

# example 3-5

import rospy
from costum_msg.msg import complex  # importing the costum message
from random import random

rospy.init_node('message_publisher')

pub = rospy.Publisher('complex', complex)

rate = rospy.Rate(2)
while not rospy.is_shutdown():
    msg = complex()
    msg.real = random()
    msg.imaginary = random()
    pub.publish(msg)
    rate.sleep()