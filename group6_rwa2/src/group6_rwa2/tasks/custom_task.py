import imp
import rospkg
from group6_rwa2.tasks.task import Task
import rospy

class CustomTask(Task):
    def __init__(self, callback):
       self._callback = callback

    def task_done(self):
        """Prints to indicate that the task as complete
        """
        rospy.loginfo("Callback called")
        self._callback()
