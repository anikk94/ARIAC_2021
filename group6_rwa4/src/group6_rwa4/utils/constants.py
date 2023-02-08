#!/usr/bin/env python

from utils.utility import print_partition
from tf.transformations import quaternion_from_euler
import rospy
import tf2_ros
import traceback
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose

class Const:
    """Program-wise Constants used throughout the scope of the program
    """
    # pylint: disable=too-few-public-methods

    INITIAL_DATA_RECEIVER_TIMEOUT_COUTNER = 200
    INITIAL_VERIFY_TASK_COUNTER = 135
    SLEEP_TIMER = 5

    tfBuffer = None
    listener = None
    tf_broadcaster = None

    INITIALIZE_DONE = False

    @staticmethod
    def initialize():
        if not Const.INITIALIZE_DONE:
            Const.INITIALIZE_DONE = True
            print_partition()
            # traceback.print_stack()
            rospy.loginfo("Const iniitialize")
            print_partition()
            Const.tfBuffer = tf2_ros.Buffer()
            Const.listener = tf2_ros.TransformListener(Const.tfBuffer)
            Const.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
