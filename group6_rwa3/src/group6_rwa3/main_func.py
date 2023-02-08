from scheduler.part import Part
from utils.utility import print_partition, sort_to_bin
from utils.conversion import get_target_world_pose
from tasks.move_part_task import MovePartTask
import rospy
from std_srvs.srv import Trigger
from tasks.submit_kitting_shipment import SubmitKittingShipmentTask
from tasks.submit_assembly_shipment import SubmitAssemblyShipmentTask
from tasks.wait import WaitTask
from scheduler.plan import Plan
from scheduler.static_brain import StaticBrain
from utils.constants import Const
from robots.agv_robot import AGVRobot
from tasks.kitting_sanity_check_task import KittingSanityCheckTask

def initialize():
    """Initializer for the whole ARIAC Environment
    """
    rospy.loginfo("Initialize...")
    StaticBrain.instance().obj_world_model.gantry_robot.go_home()


def start_competition():
    """Starts the competition service
    """
    rospy.loginfo("Starting Competition...")
    comp_srv = '/ariac/start_competition'
    rospy.wait_for_service(comp_srv)
    comp = rospy.ServiceProxy(comp_srv, Trigger)

    try:
        comp()
    except rospy.ServiceException as exc:
        rospy.loginfo('failed with error' + str(exc) + '\n')


def end_competition():
    """Ends the competition service
    """
    rospy.loginfo("Ending Competition...")
    end_comp_srvc = '/ariac/end_competition'
    rospy.wait_for_service(end_comp_srvc)
    end_comp = rospy.ServiceProxy(end_comp_srvc, Trigger)

    try:
        end_comp()
    except rospy.ServiceException as exc:
        rospy.loginfo('failed with error' + str(exc) + '\n')


def get_brain():
    """Calling the static brain instance
    """
    return StaticBrain.instance()


def make_plan_for_order():
    """makes a plan for the order received
    """

    if StaticBrain.instance().orders:
        order_d = StaticBrain.instance().orders.pop(0).order
        obj_agv = None

        print_partition()
        rospy.loginfo("Making Plan for order received:")
        rospy.loginfo(order_d)
        print_partition()

        plan = Plan(order_d, StaticBrain.instance())
        if order_d.kitting_shipments:
            for i in range(len(order_d.kitting_shipments)):
                kit_shipment = order_d.kitting_shipments[i]
                obj_agv = (
                    StaticBrain.instance().obj_world_model.
                    get_agv_robot(kit_shipment.agv_id))

                kitting_robot=(
                        StaticBrain.instance().obj_world_model.
                        kitting_robot)

                # PICK-PLACE PLANNING - MOVE PART TASK
                for product in kit_shipment.products:
                    plan.add_task(MovePartTask(
                        dest_part=product,
                        agv_id=obj_agv.agv_id,
                        robot=kitting_robot
                        ))

                # PERFORM SANITY CHECK
                plan.add_task(WaitTask(Const.SLEEP_TIMER, order_d.order_id))
                plan.add_task(KittingSanityCheckTask(kitting_robot, obj_agv.agv_id))


                # SUBMIT KITTING SHIPMENT
                plan.add_task(WaitTask(Const.SLEEP_TIMER, order_d.order_id))
                kitting_task = SubmitKittingShipmentTask(
                    obj_agv, kit_shipment.station_id,
                    kit_shipment.shipment_type)
                plan.add_task(kitting_task)

                # WAIT TASK FOR SWITCHING ORDERS
                plan.add_task(WaitTask(Const.SLEEP_TIMER, order_d.order_id))

                # if order_d.assembly_shipments:
                #     assm_shipment = order_d.assembly_shipments[0]
                #     submit_assembly = (SubmitAssemblyShipmentTask(obj_agv,
                #                     assm_shipment.station_id,
                #                     assm_shipment.shipment_type))
                #     plan.add_task(submit_assembly)

        StaticBrain.instance().add_plan(plan)


# CALL BACKS

# Clock
def clock_cb(msg):
    """Clock initialize
    """
    # rospy.loginfo(' >> triggered /clock: %s')
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

# AGV Logical Cameras
def sensor_lc_agv1_cb(data):
    """Logical camera associated with AGV1 data
    """
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    obj_agv = (StaticBrain.instance().obj_world_model.get_agv_robot("agv1"))
    
    if data.models:
        obj_agv.parts=[]
        for id in range(len(data.models)):
            part = data.models[id]
            obj_part = Part(part)
            obj_part.location_type = obj_agv.agv_id
            obj_agv.add_agv_parts(obj_part) 

def sensor_lc_agv2_cb(data):
    """Logical camera associated with AGV2 data
    """
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    obj_agv = (StaticBrain.instance().obj_world_model.get_agv_robot("agv2"))
    
    if data.models:
        obj_agv.parts=[]
        for id in range(len(data.models)):
            part = data.models[id]
            obj_part = Part(part)
            obj_part.location_type = obj_agv.agv_id
            obj_agv.add_agv_parts(obj_part)

def sensor_lc_agv3_cb(data):
    """Logical camera associated with AGV3 data
    """
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    obj_agv = (StaticBrain.instance().obj_world_model.get_agv_robot("agv3"))

    if data.models:
        obj_agv.parts=[]
        for id in range(len(data.models)):
            part = data.models[id]
            obj_part = Part(part)
            obj_part.location_type = obj_agv.agv_id
            obj_agv.add_agv_parts(obj_part)          

def sensor_lc_agv4_cb(data):
    """Logical camera associated with AGV4 data
    """
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    obj_agv = (StaticBrain.instance().obj_world_model.get_agv_robot("agv4"))
    
    if data.models:
        obj_agv.parts=[]
        for id in range(len(data.models)):
            part = data.models[id]
            obj_part = Part(part)
            obj_part.location_type = obj_agv.agv_id
            obj_agv.add_agv_parts(obj_part)

# Sensors
def sensor_lc_qcs1_cb(data):
    """Quality control sensor 1 data
    """
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

    agv_type = "agv1"
    prev_data = StaticBrain.instance().obj_world_model.fsnapshot.get(agv_type, None)
    len_prev_data = 0
    if prev_data is not None:
        len_prev_data = len(prev_data)    

    if len(data.models) != len_prev_data:    

        if data.models:
            for id in range(len(data.models)):
                part = data.models[id]
                part_world_pose = get_target_world_pose(part, "quality_control_sensor_1_frame")
                obj_part = Part(part_world_pose)
                obj_part.location_type = agv_type
                data.models[id] = obj_part

        print_partition()
        rospy.loginfo("Faulty Product /ariac/quality_control_sensor_1 main_func py")
        rospy.loginfo(data)
        print_partition()
        StaticBrain.instance().obj_world_model.fsnapshot[agv_type] = data.models


def sensor_lc_qcs2_cb(data):
    """Quality control sensor 2 data
    """
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

    agv_type = "agv2"
    prev_data = StaticBrain.instance().obj_world_model.fsnapshot.get(agv_type, None)
    len_prev_data = 0
    if prev_data is not None:
        len_prev_data = len(prev_data)

    if len(data.models) != len_prev_data:
        if data.models:
            for id in range(len(data.models)):
                part = data.models[id]
                part_world_pose = get_target_world_pose(part, "quality_control_sensor_2_frame")
                obj_part = Part(part_world_pose)
                obj_part.location_type = agv_type
                data.models[id] = obj_part

        print_partition()
        rospy.loginfo("Faulty Product /ariac/quality_control_sensor_2 main_func py")
        rospy.loginfo(data)
        print_partition()
        StaticBrain.instance().obj_world_model.fsnapshot[agv_type] = data.models


def sensor_lc_qcs3_cb(data):
    """Quality control sensor 3 data
    """
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

    agv_type = "agv3"
    prev_data = (StaticBrain.instance().
                 obj_world_model.fsnapshot.get(agv_type, None))
    len_prev_data = 0
    if prev_data is not None:
        len_prev_data = len(prev_data)

    if len(data.models) != len_prev_data:
        if data.models:
            for id in range(len(data.models)):
                part = data.models[id]
                part_world_pose = get_target_world_pose(part, "quality_control_sensor_3_frame")
                obj_part = Part(part_world_pose)
                obj_part.location_type = agv_type
                data.models[id] = obj_part

        print_partition()
        rospy.loginfo("Faulty Product /ariac/quality_control_sensor_3 main_func py")
        rospy.loginfo(data)
        print_partition()
        StaticBrain.instance().obj_world_model.fsnapshot[agv_type] = data.models


def sensor_lc_qcs4_cb(data):
    """Quality control sensor 4 data
    """
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

    agv_type = "agv4"
    prev_data = StaticBrain.instance().obj_world_model.fsnapshot.get(agv_type, None)
    len_prev_data = 0
    if prev_data is not None:
        len_prev_data = len(prev_data)

    if len(data.models) != len_prev_data:
        if data.models:
            for id in range(len(data.models)):
                part = data.models[id]
                part_world_pose = get_target_world_pose(part, "quality_control_sensor_4_frame")
                obj_part = Part(part_world_pose)
                obj_part.location_type = agv_type
                data.models[id] = obj_part

        print_partition()
        rospy.loginfo("Faulty Product /ariac/quality_control_sensor_4 main_func py")
        rospy.loginfo(data)
        print_partition()
        StaticBrain.instance().obj_world_model.fsnapshot[agv_type] = data.models


def sensor_lc_b0_cb(data):
    """Logical camera associated with bin 0 data
    """
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

    logical_camera_type = "logical_camera_bins0_frame"
    prev_data = (StaticBrain.instance().obj_world_model.
                 snapshot.get(logical_camera_type, None))
    len_prev_data = 0
    if prev_data is not None:
        len_prev_data = len(prev_data)

    if len(data.models) != len_prev_data:
        if data.models:
            for id in range(len(data.models)):
                part = data.models[id]
                part_world_pose = get_target_world_pose(part, logical_camera_type)
                obj_part = Part(part_world_pose)
                obj_part.location_type = sort_to_bin(part_world_pose)
                data.models[id] = obj_part
        print_partition()
        rospy.loginfo(logical_camera_type + " sensor data")
        rospy.loginfo(data)
        print_partition()
        StaticBrain.instance().obj_world_model.snapshot[logical_camera_type] = data.models


def sensor_lc_b1_cb(data):
    """Logical camera associated with bin 1 data
    """
    StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()

    logical_camera_type = "logical_camera_bins1_frame"
    prev_data = (StaticBrain.instance().obj_world_model.
                 snapshot.get(logical_camera_type, None))
    len_prev_data = 0
    if prev_data is not None:
        len_prev_data = len(prev_data)

    if len(data.models) != len_prev_data:
        if data.models:
            for id in range(len(data.models)):
                part = data.models[id]
                part_world_pose = get_target_world_pose(part, logical_camera_type)
                obj_part = Part(part_world_pose)
                obj_part.location_type = sort_to_bin(part_world_pose)
                data.models[id] = obj_part
        print_partition()
        rospy.loginfo(logical_camera_type + " sensor data")
        rospy.loginfo(data)
        print_partition()
        StaticBrain.instance().obj_world_model.snapshot[logical_camera_type] = data.models


def sensor_lc_s2_cb(data):
    """Logical camera associated with bin 2 data
    """
    # StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    if data.models:
        StaticBrain.instance().obj_world_model.snapshot["lc_s2"] = data.models


def sensor_ps0_cb(data):
    """Callback proximity sensor
    """
    # StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    pass


def sensor_bb0_cb(data):
    """Call back break beam sensor
    """
    # StaticBrain.instance().obj_world_model.reset_data_receiver_timeout_counter()
    pass


def competition_state_cb(data):
    """Call back for the sate of competiotion"""
    StaticBrain.instance().obj_world_model.set_competition_state(data)