#!/usr/bin/env python3

# example 3-1

import rospy
from std_msgs.msg import Int32

rospy.init_node('topic_publisher') # initializing the node with name 'topic_publisher'

pub = rospy.Publisher('counter', Int32) # initializing the publisher (name, type)
                                        # latched topic --> (---,---,latched=True), when it is not published often such that a subscriber can fetch the last published object   
rate = rospy.Rate(2) # publishing rate in Hz

count = 0
while not rospy.is_shutdown(): # rospy.is_shutdown = True, if node is ready to be shut down False otherwise
    pub.publish(count) # publishing the count      
    count += 1
    rate.sleep() # sleep such that the desired rate (2 Hz) is maintained