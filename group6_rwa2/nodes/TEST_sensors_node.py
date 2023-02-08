#!/usr/bin/env python3
import rospy
from nist_gear.msg import LogicalCameraImage, Proximity
from sensor_msgs.msg import PointCloud, LaserScan, Range


def sensor_lc_qcs1_cb(data):
    if data.models:
        rospy.loginfo("Callback triggered for Topic /ariac/quality_control_sensor_1")
def sensor_lc_qcs2_cb(data):
    if data.models:
        rospy.loginfo("Callback triggered for Topic /ariac/quality_control_sensor_2")
def sensor_lc_qcs3_cb(data):
    if data.models:
        rospy.loginfo("Callback triggered for Topic /ariac/quality_control_sensor_3")
def sensor_lc_qcs4_cb(data):
    if data.models:
        rospy.loginfo("Callback triggered for Topic /ariac/quality_control_sensor_4")
def sensor_lc_b0_cb(data):
    if True:
        rospy.loginfo("Callback triggered for Topic /ariac/logical_camera_bins0")
def sensor_lc_s2_cb(data):
    if data.models:
        rospy.loginfo("Callback triggered for Topic /ariac/logical_camera_station2")
def sensor_dc_b1_cb(data):
    if True:
        rospy.loginfo("Callback triggered for Topic /ariac/deapth_camera_bins1")
def sensor_lp0_cb(data):
    if True:
        rospy.loginfo("Callback triggered for Topic /ariac/laser_profiler_0")
def sensor_ps0_cb(data):
    if data.max_range != data.range:
        rospy.loginfo("Callback triggered for Topic /ariac/proximity_sensor_0")
def sensor_bb0_cb(data):
    if data.object_detected:
        rospy.loginfo("Callback triggered for Topic /ariac/breakbeam_0")




# Main

# Init Node
rospy.init_node("sensors_node")


# Not working
rospy.Subscriber('/ariac/laser_profiler_0', LaserScan, sensor_lp0_cb)

# Working
rospy.Subscriber('/ariac/breakbeam_0', Proximity, sensor_bb0_cb)
rospy.Subscriber('/ariac/proximity_sensor_0', Range, sensor_ps0_cb)
rospy.Subscriber('/ariac/deapth_camera_bins1', PointCloud, sensor_dc_b1_cb)
rospy.Subscriber('/ariac/logical_camera_station2', LogicalCameraImage, sensor_lc_s2_cb)
rospy.Subscriber('/ariac/logical_camera_bins0', LogicalCameraImage, sensor_lc_b0_cb)
rospy.Subscriber('/ariac/quality_control_sensor_1', LogicalCameraImage, sensor_lc_qcs1_cb)
rospy.Subscriber('/ariac/quality_control_sensor_2', LogicalCameraImage, sensor_lc_qcs2_cb)
rospy.Subscriber('/ariac/quality_control_sensor_3', LogicalCameraImage, sensor_lc_qcs3_cb)
rospy.Subscriber('/ariac/quality_control_sensor_4', LogicalCameraImage, sensor_lc_qcs4_cb)


rospy.spin()



# logical_camera_bins0:
# type: logical_camera

# logical_camera_station2:
# type: logical_camera

# depth_camera_bins1:
# type: depth_camera

# laser_profiler_0:
# type: laser_profiler

# proximity_sensor_0:
# type: proximity_sensor

# breakbeam_0:
# type: break_beam

# -------------------------

# logical_camera_quality_control_sensor_1:
# type: logical_camera
# logical_camera_quality_control_sensor_2:
# type: logical_camera
# logical_camera_quality_control_sensor_3:
# type: logical_camera
# logical_camera_quality_control_sensor_4:
# type: logical_camera