import time
import rospy
from group6_rwa2.tasks.task import Task

class WaitTask(Task):
    def __init__(self, delayInSecs, order_id):
       self._delay = delayInSecs
       self._order_id = order_id

    def task_done(self):
        """Prints to indicate that the task as complete
        """
        rospy.loginfo("Start delay of " + str(self._delay) + "s for " + self._order_id)
        time.sleep(self._delay)
        rospy.loginfo("Delay ended for " + self._order_id)
