#!/usr/bin/env python3

# example 3-6

import rospy
from costum_msg.msg import complex

def callback(msg):
    print('Real:', msg.real)
    print('Imaginary:', msg.imaginary)

rospy.init_node('message_subscriber')

sub = rospy.Subscriber('complex', complex, callback)

rospy.spin()