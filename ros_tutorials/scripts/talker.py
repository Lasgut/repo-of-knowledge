#!/usr/bin/env python3

# Writing a simple publisher and subscriber

import rospy
from std_msgs.msg import String

def talker():

    pub = rospy.Publisher('chatter', String, queue_size=10) # (name, type, queue size) queue size --> limits the amount of queued  
                                                            # messages if any subscriber is not reciving them fast enough
                                                        
    rospy.init_node('talker', anonymous=True)               # anonymous=True --> ensure unique name by adding random numbers               
    rate = rospy.Rate(10)                                   # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass