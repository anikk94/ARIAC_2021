import time
import rospy

from constants import Const
from static_brain import StaticBrain
from group6_rwa2.tasks.task import Task
from utility import print_partition

class Plan:
    """Class to represent the atrributes and methods of Plan
    """

    # pylint: disable=line-too-long

    def __init__(self, order):
        self.tasks = []
        self.order = order
        self.plan_active = True
        self.flag_only_once = True

    def add_task(self, task):
        """Adds the task to be executed
        Args:
            task (Task): Task to be executed
        """
        self.tasks.append(task)

    def verify_order(self):
        """Verifies the order for this particular plan

        Returns:
            bool: if the order can be complelted, and all parts required,
            for the order are present in the workcell
        """

        def verify_part(product):
            """internal function to verify the part

            Args:
                product (Part): Checks if this particular part is in the product

            Returns:
                bool: returns True if the part is in the workcell
            """
            is_part_found, part_found = StaticBrain.instance().obj_world_model.search_part_for(product.type)
            if self.flag_only_once:
                if is_part_found:
                    print_partition()
                    rospy.loginfo("WORLD Pose for part required: (" + self.order.order_id + ", " + part_found.type + ")")
                    rospy.loginfo(part_found.pose)
                    print_partition()
                else:
                    print_partition()
                    rospy.loginfo("Part NOT FOUND:(" + self.order.order_id + ", " + product.type + ")")
                    print_partition()

            return is_part_found

        flag = True
        if self.order.kitting_shipments:
            kit_shipment = self.order.kitting_shipments[0]
            for product in kit_shipment.products:
                flag = flag and verify_part(product)

        if self.order.assembly_shipments:
            assm_shipment = self.order.assembly_shipments[0]
            for product in assm_shipment.products:
                flag = flag and verify_part(product)

        self.flag_only_once = False
        return flag

    def execute_plan(self):
        """Execute the sequence of steps for kitting or assembly
        """

        def submit_plan():
            while self.tasks:
                if self.plan_active:
                    task_to_do = self.tasks.pop(0)
                    if isinstance(task_to_do, Task):
                        rospy.loginfo(task_to_do)
                        task_to_do.task_done()
                    # give a bit of sleep, so as to avoid over-processing
                    time.sleep(0.01)

        for i in range(Const.INITIAL_VERIFY_TASK_COUNTER):
            if self.verify_order():
                submit_plan()
                return # if we are finished processing tasks
            time.sleep(1) # sleep 1 second befor verifying again

        print_partition()
        rospy.loginfo("Triggered after " + str(Const.INITIAL_VERIFY_TASK_COUNTER)+"s, Insufficient parts to complete " + self.order.order_id)
        submit_plan()
        print_partition()
