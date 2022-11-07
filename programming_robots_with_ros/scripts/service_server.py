#!/usr/bin/env python3

# example 4-3

import rospy
from costum_msg.srv import WordCount,WordCountResponse

def count_words(request):
    return WordCountResponse(len(request.words.split())) # request.words.split (variable in def).(name of the service input value).(function)

rospy.init_node('service_server') # initializing the node with name 'service_server'

service = rospy.Service('word_count', WordCount, count_words) # initializing the service (name, type, callback function)

rospy.spin() # will only return when the node is ready to shut down.
             # shortcut to avoid top-level while loop