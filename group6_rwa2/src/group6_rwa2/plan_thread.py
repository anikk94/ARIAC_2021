"""Thread class for priority order processing
"""
#!/usr/bin/python

import threading
import rospy


class PlanThread (threading.Thread):
    """This is thread which deals with executing the plan

    Args:
       plan : Takes the plan as input to execut the plan in the thread
    """

    def __init__(self, plan):
        threading.Thread.__init__(self)
        self.plan = plan

    def run(self):
        rospy.loginfo("Starting Plan for " + self.plan.order.order_id)
        self.plan.execute_plan()
        rospy.loginfo(self.plan.order.order_id + " Plan Exectued")
        