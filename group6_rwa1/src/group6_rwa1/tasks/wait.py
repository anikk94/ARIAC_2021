import time
import rospy
from group6_rwa1.tasks.task import Task

class WaitTask(Task):
    def __init__(self, delayInSecs):
       self.delay = delayInSecs

    def task_done(self):
        """Prints to indicate that the task as complete
        """
        rospy.loginfo("Start delay for " + str(self.delay) + "s")
        time.sleep(self.delay)
        rospy.loginfo("Delay Ended")
