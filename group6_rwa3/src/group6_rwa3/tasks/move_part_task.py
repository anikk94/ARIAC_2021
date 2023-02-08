from task import Task


class MovePartTask(Task):
    """Information for moving the part
    """
    def __init__(self, robot, dest_part, agv_id):
        self._robot = robot
        self._agv_id = agv_id
        self._dest_part = dest_part

    def task_done(self):
        """Task being completed
        """
        self._robot.move_part(self._dest_part, self._agv_id)
