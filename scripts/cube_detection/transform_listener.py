import rospy
import tf

def transform_callback(msg):
    # This callback function will be called whenever a new transform is received
    print("Transform received:")
    print("Translation:", msg.translation)
    print("Rotation:", msg.rotation)

if __name__ == "__main__":
    rospy.init_node("transform_listener")

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        try:
            # Get the transform from "panda_hand" to "zed2_left_camera_frame"
            (trans, rot) = listener.lookupTransform("panda_hand", "zed2_left_camera_frame", rospy.Time(0))
            
            # Create a message to store the transform
            transform_msg = tf.TransformerROS().fromTranslationRotation(trans, rot)
            
            # Call the callback function with the received transform message
            transform_callback(transform_msg)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()
