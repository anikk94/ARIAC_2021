#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

if __name__ == '__main__':
    rospy.init_node("counter_pub",anonymous=True)
    rate = rospy.Rate(5)  # ROS Rate at 5Hz
    pub = rospy.Publisher("/counter", Int32, queue_size=10)
    counter = 0

    while not rospy.is_shutdown():
        counter += 1
        msg = Int32()
        msg.data = counter
        rospy.loginfo("Hello from main thread")
        # rospy.loginfo(counter)
        pub.publish(counter)
        rate.sleep()
