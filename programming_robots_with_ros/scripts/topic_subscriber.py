#!/usr/bin/env python3

# example 3-1

import rospy
from std_msgs.msg import Int32

def callback(msg):
    print(msg.data)

rospy.init_node('topic_subscriber') # naming the node

sub = rospy.Subscriber('counter', Int32, callback)

rospy.spin()