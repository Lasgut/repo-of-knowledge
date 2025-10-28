#!/usr/bin/env python3

# example 4-4

import rospy
from costum_msg.srv import WordCount
import sys

rospy.init_node('service_client')

rospy.wait_for_service('word_count') # wait for the service to be advertised

word_counter = rospy.ServiceProxy('word_count', WordCount) # initialize the service proxy (name, type)

words = ' '.join(sys.argv[1:]) # arguments in terminal: rosrun 'package' script 'argument'

word_count = word_counter(words)

print(words, '->', word_count.count)