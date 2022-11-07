#!/usr/bin/env python3
 
from __future__ import print_function
 
from costum_msg.srv import add_two_ints,add_two_intsResponse
import rospy
 
def handle_add_two_ints(req): # the function takes all the input
    print("Returning [%s + %s = %s]"%(req.a, req.a, (req.a + req.b)))
    return add_two_intsResponse(req.a + req.b) # and returns the output
 
def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', add_two_ints, handle_add_two_ints) # (name, type, function)
    print("Ready to add two ints.")
    rospy.spin()
 
if __name__ == "__main__":
    add_two_ints_server()
