#!/usr/bin/env python3

# example 3-7

# Publishes data only when new data comming into the subscriber
# try: rostopic pub /number, while echoing doubled

import rospy
from std_msgs.msg import Int32

rospy.init_node('doubler')

def callback(msg):
    doubled = Int32()
    doubled.data = msg.data * 2
    pub.publish(doubled)

sub = rospy.Subscriber('number', Int32, callback)
pub = rospy.Publisher('doubled', Int32)
rospy.spin()