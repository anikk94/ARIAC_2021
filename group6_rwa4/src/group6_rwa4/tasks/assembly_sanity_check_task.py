from task import Task


class AssemblySanityCheckTask(Task):
    """Sanity check for kitting
    """
    def __init__(self, robot, agv_id):
        self._robot = robot

    def task_done(self):
        """Completion of task
        """
        self._robot.sanity_check()
