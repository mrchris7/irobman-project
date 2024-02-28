#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped

# run: roslaunch irobman_project cam2eef_transform.launch robot_ip:=192.168.1.55

if __name__ == '__main__':
    frame_from = "panda_hand"
    frame_to = "world"

    rospy.init_node('point_transformer')

    listener = tf.TransformListener()

    # Create a PointStamped message with the point in the panda_link8 frame
    point = PointStamped()
    point.header.frame_id = frame_from
    point.point.x = 0.0  # Replace with point coordinates
    point.point.y = 0.0
    point.point.z = 0.0

    rate = rospy.Rate(10.0)
    counter = 0

    while not rospy.is_shutdown() and counter <= 5:
        try:
            # Transform the point to the panda_link0 frame
            point_transformed = listener.transformPoint(frame_to, point)
            
            # Print the transformed point coordinates
            rospy.loginfo("Point given in " + frame_from + ": (%f, %f, %f)",
                          point.point.x,
                          point.point.y,
                          point.point.z)

            rospy.loginfo("Point transformed to " + frame_to + ": (%f, %f, %f)",
                          point_transformed.point.x,
                          point_transformed.point.y,
                          point_transformed.point.z)


            (trans, rot) = listener.lookupTransform(frame_from, frame_to, rospy.Time(0))
            rospy.loginfo("Translation: %s", trans)
            rospy.loginfo("Rotation: %s", rot)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("repeat:", e)
        counter += 1
            
        rate.sleep()