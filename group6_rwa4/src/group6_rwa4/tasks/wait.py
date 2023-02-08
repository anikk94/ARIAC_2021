import time
import rospy
from tasks.task import Task


class WaitTask(Task):
    """Provides delay in seconds
    """
    def __init__(self, delay_in_secs, order_id=""):
        self._delay = delay_in_secs
        self._order_id = order_id

    def task_done(self):
        """Prints to indicate that the task as complete
        """
        rospy.loginfo("Start delay of " + str(self._delay)
                      + "s for " + self._order_id)
        time.sleep(self._delay)
        rospy.loginfo("Delay ended for " + self._order_id)
