#!/usr/bin/python3

import numpy as np

import rospy

import tf_conversions

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')

    tf_broadcaster = tf2_ros.TransformBroadcaster() # The broadcaster object used to publish the tf
    transform = geometry_msgs.msg.TransformStamped() # The tf

    start = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        time_since_start = rospy.Time.now().to_sec() - start

        theta = time_since_start / 2.0

        transform.header.stamp = rospy.Time.now()

        # TODO: populate the transform.header.frame_id and transform.child_frame_id according to spec
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        # TODO: populate the transform.transform.translation. fields (x, y, z)
        transform.transform.translation.x = np.cos(theta)
        transform.transform.translation.y = np.sin(theta)
        transform.transform.translation.z = 0

        # TODO: use the tf_conversions euler to quaternion function to generate a quaternion according to spec
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)

        # TODO: populate the transform.transform.rotation fields (x, y, z, w) with the quaternion from last steptransform.transform.rotation.x = 0
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        # TODO: publish transform
        tf_broadcaster.sendTransform(transform)

        rospy.sleep(0.1)

