#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(f"{data._connection_header['topic']} >> {data.data}")
     

rospy.init_node('agv_pos')
rospy.loginfo("----------- position of AGVs -----------")
rospy.Subscriber("/ariac/agv1/station", String, callback)
rospy.Subscriber("/ariac/agv2/station", String, callback)
rospy.Subscriber("/ariac/agv3/station", String, callback)
rospy.Subscriber("/ariac/agv4/station", String, callback)
rospy.spin()