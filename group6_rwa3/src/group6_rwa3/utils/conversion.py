from geometry_msgs.msg import Quaternion
import numpy as np
from tf.transformations import quaternion_from_euler
import rospy
import tf2_ros
from utility import print_partition
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import copy
import random


def euler_to_quaternion(roll, pitch, yaw):
    """Conversion from euler to quaternion angles

    Args:
        roll (_type_): x
        pitch (_type_): y
        yaw (_type_): z
    """
    q = quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])


def get_target_world_pose(target, frame):
    """Pose of the part frame to world frame
    """

    world_target = None
    MAX_ATTEMPT = 5
    for i in range(MAX_ATTEMPT):
        tf_buffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tf_buffer)
        tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        h_frame = ''

        if frame == 'agv1':
            h_frame = 'kit_tray_1'
        elif frame == 'agv2':
            h_frame = 'kit_tray_2'
        elif frame == 'agv3':
            h_frame = 'kit_tray_3'
        elif frame == 'agv4':
            h_frame = 'kit_tray_4'
        else:
            h_frame = frame

        target_frame = 'target_frame' + str(random.randint(34563, 40000))

        # h_frame_str = ''.join(e for e in h_frame if e.isalpha())
        # if h_frame_str !=  "logicalcamerabins":
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = ''
        if h_frame:
            tf_msg.header.frame_id = h_frame

        tf_msg.header.stamp = rospy.Time()
        tf_msg.child_frame_id = target_frame
        tf_msg.transform.translation = target.pose.position
        # print(tf_msg.transform.translation)
        tf_msg.transform.rotation = target.pose.orientation
        # print(tf_msg.transform.rotation)

        # Broadcast the frame target_frame as a child of h_frame
        for _ in range(5):
            tf_broadcaster.sendTransform(tf_msg)

        world_target_tf = TransformStamped()
        # Get the transform between world and target_frame

        for _ in range(20):
            try:
                world_target_tf = tf_buffer.lookup_transform(
                    'world', target_frame, rospy.Time(), rospy.Duration(0.5))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass

        world_target = copy.deepcopy(target)
        world_target.pose.position = world_target_tf.transform.translation
        world_target.pose.orientation = world_target_tf.transform.rotation

        if (world_target.pose.position.x == 0 and
            world_target.pose.position.y == 0 and
            world_target.pose.position.z == 0):
            continue
        else:
            break

    if (world_target.pose.position.x == 0 and
            world_target.pose.position.y == 0 and
            world_target.pose.position.z == 0):
            print_partition()
            rospy.loginfo("ERROR:  UNABLE to get_target_world_pose()")
            print_partition

    return world_target
