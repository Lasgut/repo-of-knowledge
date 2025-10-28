#!/usr/bin/env python3
 
from __future__ import print_function
 
import sys
import rospy
from costum_msg.srv import * # import all
 
def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints') # blocks until service is available
    try:
        AddTwoInts = rospy.ServiceProxy('add_two_ints', add_two_ints) # initializing the service (name, type)
        resp1 = AddTwoInts(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
 
def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
