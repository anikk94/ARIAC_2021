import rospy
from world_model import WorldModel
from order import OrderReceived
from plan_thread import PlanThread
from utils.utility import print_partition


class Brain:
    """Class to handle the orders and execute the shipments in an order.

    Attributes:
        obj_world_model: Representation of components in world coordinates
        orders: Data Structure to save orders
        plan: Data structure to store the
              sequence of events to execute an order
    """

    def __init__(self):
        print("Initializing Brain...")
        self.obj_world_model = WorldModel()
        self.orders = []
        self.plans = []
        self.normal_order_thread = None
        self.priority_order_thread = None

    def add_order(self, data):
        """Adds the orders received to a data structure
        Args:
        data (Order): Order to be added.
        """
        order_received = OrderReceived(data)
        self.orders.append(order_received)

    def add_plan(self, obj_plan):
        """Adds the sequence of steps to be executed
        Args:
            obj_plan (Plan): Plan to be added
        """
        print("Adding Plans...")
        self.plans.append(obj_plan)

    def process_priority_order(self):
        """processes the priority order
        """
        if self.plans:
            if not self.is_priority_order_being_processed():  # if no priority order being processed
                print_partition()
                rospy.loginfo("Processing High Priority order...")
                print_partition()
                obj_plan = self.plans.pop(0)
                self.priority_order_thread = PlanThread(obj_plan)
                self.priority_order_thread.start()

    def process_normal_order(self):
        """processes normal order
        """
        if self.plans:
            if not self.is_priority_order_being_processed():  # if no priority order being processed
                if self.normal_order_thread is None:  # if no normal order being processed
                    print_partition()
                    rospy.loginfo("Processing Normal order...")
                    print_partition()
                    obj_plan = self.plans.pop(0)
                    self.normal_order_thread = PlanThread(obj_plan)
                    self.normal_order_thread.start()

    def free_threads(self):
        """free threads, if they are finished processing and no longer alive
        """
        if not self.is_normal_order_being_processed():
            if self.normal_order_thread:
                self.normal_order_thread = None
                rospy.loginfo("Normal Order Thread deleted")

        if not self.is_priority_order_being_processed():
            if self.priority_order_thread:
                self.priority_order_thread = None
                rospy.loginfo("Priority Order Thread deleted")

    def is_priority_order_being_processed(self):
        """Checks if the priority order is being processes

        Returns:
            bool: returns True, if the priority order thread is alive
            and being processesed
        """
        flag = False

        if self.priority_order_thread is not None:
            if self.priority_order_thread.is_alive():
                flag = True

        return flag

    def is_normal_order_being_processed(self):
        """Checks if the normal order is being processed

        Returns:
            bool: return True, if the normal order thread
            is alive and being processed
        """
        flag = False

        if self.normal_order_thread is not None:
            if self.normal_order_thread.is_alive():
                flag = True

        return flag

    # only handle two orders simultaneously (normal order or a priority order)
    def process_order_threads(self):
        """Handles the processing of orders simultaneously (normal
        order or a priority order)
        """
        if self.plans:  # we have plans to process
            if (self.is_normal_order_being_processed() and
                not self.normal_order_thread.plan.plan_active):  # if normal order is processed but not active
                self.process_priority_order()  # process the new order as priority
            else:
                if self.is_normal_order_being_processed():
                    if not self.is_priority_order_being_processed():
                        # we have plans and normal order is being processed,
                        # then new order is priority order, therfore,
                        # pause the current normal order
                        self.normal_order_thread.plan.plan_active = False
                        rospy.loginfo("Pausing Normal Order...")
                else:
                    self.process_normal_order()  # process the new order
        else:
            if not self.is_priority_order_being_processed():
                if self.is_normal_order_being_processed():
                    if not self.normal_order_thread.plan.plan_active:
                        rospy.loginfo("Resuming Normal Order...")
                        self.normal_order_thread.plan.plan_active = True

    def poll_plans(self):
        """polls plan continuosly to create threads for the orders or free them
        """
        self.free_threads()
        self.process_order_threads()
