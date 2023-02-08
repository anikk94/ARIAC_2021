from tasks.task import Task


class SubmitKittingShipmentTask(Task):
    """Submitting the kitting shipment

    Args:
        Task (_type_): _description_
    """
    def __init__(self, obj_agv_robot, station, shipment_type):
        self._obj_robot = obj_agv_robot
        self._station = station
        self._shiptment_type = shipment_type

    def task_done(self):
        """Completion of the task
        """
        self._obj_robot.move(self._station, self._shiptment_type)
