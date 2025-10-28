#!/usr/bin/env python3

# example 3-2

import rospy
from std_msgs.msg import Int32

def callback(msg):  # Callback function used in the subscriber.
    print(msg.data) # .data referes to the data type data in the std_msgs/Int32

rospy.init_node('topic_subscriber') # naming the node

sub = rospy.Subscriber('counter', Int32, callback) # initializing the subscriber (topic name, type, callback function)

rospy.spin() # will only return when the node is ready to shut down.
             # shortcut to avoid top-level while loop