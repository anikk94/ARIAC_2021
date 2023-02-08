#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def callback(data):
    bol = data == String("done")
    rospy.loginfo(bol)
    rospy.loginfo(data)
     

rospy.init_node('TEST_comp_stat')
rospy.loginfo("----------- position of AGVs -----------")
rospy.Subscriber("/ariac/competition_state", String, callback)
rospy.spin()