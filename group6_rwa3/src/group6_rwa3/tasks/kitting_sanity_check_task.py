from task import Task


class KittingSanityCheckTask(Task):
    """Sanity check for kitting
    """
    def __init__(self, robot, agv_id):
        self._robot = robot
        self._agv_id = agv_id

    def task_done(self):
        """Completion of task
        """
        self._robot.sanity_check(self._agv_id)
