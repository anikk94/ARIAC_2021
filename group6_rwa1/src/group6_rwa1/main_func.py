from constants import Const
from brain import StaticBrain
from plan import Plan
from std_srvs.srv import Trigger
from group6_rwa1.tasks.submit_kitting_shipment import SubmitKittingShipment
from group6_rwa1.tasks.submit_assembly_shipment import SubmitAssemblyShipment
from group6_rwa1.tasks.wait import WaitTask
from rospy import loginfo
import rospy


def initialize():
    """Initialize the world environment, with movable and immovable objects. This is our knowledge representation of the system.
    """
    rospy.loginfo("In Initialize")
    StaticBrain.instance()

def poll_plans():
    """When order is received, a plan is made. So, it polls for the plan and executes the plan
    """
    StaticBrain.instance().execute_plans()

def make_plan_for_order():
    """For each order we receive, in this method we formulate a plan
    """
    order_d = StaticBrain.instance().orders.pop(0).order
    plan = Plan(order_d.order_id)
    kit_shipment = order_d.kitting_shipments.pop(0)
    obj_agv = StaticBrain.instance().obj_world_model.get_agv_robot(kit_shipment.agv_id)
    kitting_task = SubmitKittingShipment(obj_agv, kit_shipment.station_id,
                                         kit_shipment.shipment_type)
    plan.add_task(kitting_task)
    wait_task = WaitTask(60)
    plan.add_task(wait_task) # wait for 1 minute
    assm_shipment = order_d.assembly_shipments.pop(0)
    submit_assembly = SubmitAssemblyShipment(obj_agv, assm_shipment.station_id,
                                            assm_shipment.shipment_type)
    plan.add_task(submit_assembly)

    StaticBrain.instance().add_plan(plan)

def start_competition():
    """Starts the competition
    """
    rospy.loginfo("Starting Competition...")
    comp_srv = '/ariac/start_competition'
    rospy.wait_for_service(comp_srv)
    comp = rospy.ServiceProxy(comp_srv, Trigger)

    try:
        comp()
    except rospy.ServiceException as exc:
        print('%s failed with some error'.format(comp_srv))

def end_competition():
    """ends the competition
    """
    rospy.loginfo("Ending Competition...")
    end_comp_srvc = '/ariac/end_competition'
    rospy.wait_for_service(end_comp_srvc)
    end_comp = rospy.ServiceProxy(end_comp_srvc, Trigger)

    try:
        end_comp()
    except rospy.ServiceException as exc:
        print('%s failed with some error'.format(end_comp_srvc))


#-------------------------------- CALL BACKS --------------------------------------------------------
# Orders
def order_cb(data):
    """Order CallBack

    Args:
        data (_type_): Receives the data
    """
    rospy.loginfo(' >> triggered /ariac/orders/: %s', data.order_id)
    StaticBrain.instance().add_order(data)
    make_plan_for_order()

# Sensors
def sensor_lc_qcs1_cb(data):
    if data.models:
        rospy.loginfo_throttle(10, "Callback triggered for Topic /ariac/quality_control_sensor_1")

def sensor_lc_qcs2_cb(data):
    if data.models:
        rospy.loginfo_throttle(10, "Callback triggered for Topic /ariac/quality_control_sensor_2")

def sensor_lc_qcs3_cb(data):
    if data.models:
        rospy.loginfo_throttle(10, "Callback triggered for Topic /ariac/quality_control_sensor_3")

def sensor_lc_qcs4_cb(data):
    if data.models:
        rospy.loginfo_throttle(10, "Callback triggered for Topic /ariac/quality_control_sensor_4")

def sensor_lc_b0_cb(data):
    if data.models:
        rospy.loginfo_throttle(10, "Callback triggered for Topic /ariac/logical_camera_bins0")

def sensor_lc_s2_cb(data):
    if data.models:
        rospy.loginfo_throttle(10, "Callback triggered for Topic /ariac/logical_camera_station2")

def sensor_dc_b1_cb(data):
    if True:
        rospy.loginfo("Callback triggered for Topic /ariac/deapth_camera_bins1")

def sensor_lp0_cb(data):
    if True:
        rospy.loginfo_throttle(10, "Callback triggered for Topic /ariac/laser_profiler_0")

def sensor_ps0_cb(data):
    if data.max_range != data.range:
        rospy.loginfo("Callback triggered for Topic /ariac/proximity_sensor_0")

def sensor_bb0_cb(data):
    if data.object_detected:
        rospy.loginfo("Callback triggered for Topic /ariac/breakbeam_0")

# AGVs
def agv1_station_cb(data):
    obj_agv = StaticBrain.instance().obj_world_model.agv_robots["agv1"]
    obj_agv.station = data
    rospy.loginfo_throttle(10, data)

def agv2_station_cb(data):
    obj_agv = StaticBrain.instance().obj_world_model.agv_robots["agv2"]
    obj_agv.station = data
    rospy.loginfo_throttle(10, data)

def agv3_station_cb(data):
    obj_agv = StaticBrain.instance().obj_world_model.agv_robots["agv3"]
    obj_agv.station = data
    rospy.loginfo_throttle(10, data)

def agv4_station_cb(data):
    obj_agv = StaticBrain.instance().obj_world_model.agv_robots["agv4"]
    obj_agv.station = data
    rospy.loginfo_throttle(10, data)

def agv1_state_cb(data):
    obj_agv = StaticBrain.instance().obj_world_model.agv_robots["agv1"]
    obj_agv.state = data
    rospy.loginfo_throttle(10, data)

def agv2_state_cb(data):
    obj_agv = StaticBrain.instance().obj_world_model.agv_robots["agv2"]
    obj_agv.state = data
    rospy.loginfo_throttle(10, data)

def agv3_state_cb(data):
    obj_agv = StaticBrain.instance().obj_world_model.agv_robots["agv3"]
    obj_agv.state = data
    rospy.loginfo_throttle(10, data)

def agv4_state_cb(data):
    obj_agv = StaticBrain.instance().obj_world_model.agv_robots["agv4"]
    obj_agv.state = data
    rospy.loginfo_throttle(10, data)

# Competition state
def competition_state_cb(data):
    StaticBrain.instance().obj_world_model.competition_state = data