from task import Task


class MovePartAssembly(Task):
    """Information for moving the part
    """
    def __init__(self, robot, dest_part):
        self._robot = robot
        self._dest_part = dest_part

    def task_done(self):
        """Task being completed
        """
        self._robot.move_part_assembly(self._dest_part)