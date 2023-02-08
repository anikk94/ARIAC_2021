#!/usr/bin/env python3
import rospy
from nist_gear.msg import Order



def callback(data):
    # this message needs to be stored TO-DO
    # rospy.loginfo(f"{data._connection_header['topic']} >> {data.data}")
    rospy.loginfo(data.order_id)



rospy.init_node('sensors_node')
rospy.loginfo('sensors node started')
rospy.Subscriber('/ariac/orders', Order, callback)
rospy.spin()