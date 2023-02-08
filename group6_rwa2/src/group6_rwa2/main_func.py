import rospy
import tf
from constants import Const
from group6_rwa2.tasks.submit_kitting_shipment import SubmitKittingShipmentTask
from group6_rwa2.tasks.submit_assembly_shipment import SubmitAssemblyShipmentTask
from group6_rwa2.tasks.wait import WaitTask
from group6_rwa2.plan import Plan
from static_brain import StaticBrain

def initialize():
    """Initializes our brain which services and world environment
    """
    rospy.loginfo("In Initialize")
    StaticBrain.instance()

def make_plan_for_order():
    """makes a plan for the order received
    """
    if StaticBrain.instance().orders:
        order_d = StaticBrain.instance().orders.pop(0).order
        obj_avg = None

        plan = Plan(order_d)
        if order_d.kitting_shipments:
            kit_shipment = order_d.kitting_shipments[0]
            obj_avg = StaticBrain.instance().obj_world_model.get_agv_robot(kit_shipment.agv_id)
            kitting_task = SubmitKittingShipmentTask(obj_avg, kit_shipment.station_id, 
            kit_shipment.shipment_type)
            plan.add_task(WaitTask(15, order_d.order_id)) 
            plan.add_task(kitting_task)
            plan.add_task(WaitTask(15, order_d.order_id))
            
        if order_d.assembly_shipments:
            assm_shipment = order_d.assembly_shipments[0]
            submit_assembly = SubmitAssemblyShipmentTask(obj_avg, assm_shipment.station_id, 
            assm_shipment.shipment_type)
            plan.add_task(submit_assembly)

    StaticBrain.instance().add_plan(plan)

# CALL BACKS

# Clock
def clock_cb(msg):
    #rospy.loginfo(' >> triggered /clock: %s')
    pass

# Orders
def order_cb(data):
    """Order callback

    Args:
        data (Order): Order data
    """
    rospy.loginfo(' >> triggered /ariac/orders/: %s', data.order_id)
    StaticBrain.instance().add_order(data)
    make_plan_for_order()

# Sensors
def sensor_lc_qcs1_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    if data.models:
        StaticBrain.instance().obj_world_model.fsnapshot["lc_qcs1"] = data.models
        agv_parts = data.models.pop(0)
        if agv_parts.type != "":
            rospy.loginfo_throttle(100,"""
            =================================================================
            Faulty Product /ariac/quality_control_sensor_1
            =================================================================""")

def sensor_lc_qcs2_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    if data.models:
        StaticBrain.instance().obj_world_model.fsnapshot["lc_qcs2"] = data.models
        agv_parts = data.models.pop(0)
        if agv_parts.type != "":
            rospy.loginfo_throttle(100,"""
            =================================================================
            Faulty Product /ariac/quality_control_sensor_2
            =================================================================""")

def sensor_lc_qcs3_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    if data.models:
        StaticBrain.instance().obj_world_model.fsnapshot["lc_qcs3"] = data.models
        agv_parts = data.models.pop(0)
        if agv_parts.type != "":
            rospy.loginfo_throttle(100,"""
            =================================================================
            Faulty Product /ariac/quality_control_sensor_3
            =================================================================""")

def sensor_lc_qcs4_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    if data.models:
        StaticBrain.instance().obj_world_model.fsnapshot["lc_qcs4"] = data.models
        agv_parts = data.models.pop(0)
        if agv_parts.type != "":
            rospy.loginfo_throttle(100,"""
            =================================================================
            Faulty Product /ariac/quality_control_sensor_4
            =================================================================""")

def sensor_lc_b0_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    if data.models:
        for id in range(len(data.models)):
            part = data.models[id]
            lookup_part_name = "logical_camera_bins0_" + part.type + '_' + str(id + 1) + '_'+ "frame"
            try:
                part.pose.position, part.pose.orientation = \
                (StaticBrain.instance().obj_world_model.tf_listner.lookupTransform('/world', lookup_part_name, rospy.Time(0)))
                data.models[id] = part
            
            except(tf.LookupException,
                   tf.ConnectivityException,
                   tf.ExtrapolationException):
                continue

        StaticBrain.instance().obj_world_model.snapshot["lc_b0"] = data.models

def sensor_lc_b1_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    if data.models:
        for id in range(len(data.models)):
            part = data.models[id]
            lookup_part_name = "logical_camera_bins1_" + part.type + '_' + str(id + 1) + '_'+ "frame"
            try:
                part.pose.position, part.pose.orientation = \
                (StaticBrain.instance().obj_world_model.tf_listner.lookupTransform('/world', lookup_part_name, rospy.Time(0)))
                data.models[id] = part
           
            except(tf.LookupException,
                   tf.ConnectivityException,
                   tf.ExtrapolationException):
                continue

        StaticBrain.instance().obj_world_model.snapshot["lc_b1"] = data.models


def sensor_lc_s2_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    if data.models:
        StaticBrain.instance().obj_world_model.snapshot["lc_s2"] = data.models

def sensor_dc_b1_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

def sensor_lp0_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

def sensor_ps0_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

def sensor_bb0_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

def agv1_cam_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

def agv2_cam_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

def agv3_cam_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

def agv4_cam_cb(data):
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

# AGVs
def agv1_station_cb(data):
    pass

def agv2_station_cb(data):
    pass

def agv3_station_cb(data):
    pass

def agv4_station_cb(data):
    pass

def agv1_state_cb(data):
    pass

def agv2_state_cb(data):
    pass

def agv3_state_cb(data):
    pass

def agv4_state_cb(data):
    pass


# Competition state
def competition_state_cb(data):
    """competition state callback

    Args:
        data (String): competition state
    """
    # rospy.loginfo_throttle(1, data)
    StaticBrain.instance().obj_world_model.set_competition_state(data)