import rospy
import tf
import time


def get_transform(part_frame):
    rospy.loginfo("listening ...")
    listener = tf.TransformListener()  # instantiate a listener
    broadcaster = tf.TransformBroadcaster()
    time.sleep(3)  # wait for a while
    rate = rospy.Rate(10)  # set sleep rate

    # lookup transform betweem two frames
    while not rospy.is_shutdown():
        try:
            # compute transformation world-part frame
            (translation, rotation) = listener.lookupTransform('/world',
                                                               part_frame,
                                                               rospy.Time(0))
            
            #  convert Quaternion to Euler angles
            euler = tf.transformations.euler_from_quaternion(rotation)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            rospy.loginfo("Pose of part in the /world frame [{}][{}, {}, {}]"
                          .format(translation, roll, pitch, yaw))
            print("{}".format(translation))
            print("{}".format(rotation))
            broadcaster.sendTransform(translation, rotation, rospy.Time.now(), 
                                      '/logical_camera_bins0_assembly_battery_blue_5_frame',
                                      '/world')

        except(tf.LookupException,
               tf.ConnectivityException,
               tf.ExtrapolationException):
            continue
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('python_listener', anonymous=False)
    get_transform('/logical_camera_bins0_assembly_battery_blue_5_frame')
