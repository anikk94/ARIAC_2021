from tasks.task import Task
import rospy


class CustomTask(Task):
    """Callback for the task
    """
    def __init__(self, callback):
        self._callback = callback

    def task_done(self):
        """Prints to indicate that the task as complete
        """
        rospy.loginfo("Callback called")
        self._callback()
