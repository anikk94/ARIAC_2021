from task import Task


class SubmitAssemblyShipmentTask(Task):
    """This is the task for submitting the Assembly
    """
    def __init__(self, obj_agv_robot, station, shipment_type):
        self._obj_robot = obj_agv_robot
        self._station = station
        self._shiptment_type = shipment_type

    def task_done(self):
        """Completion of the task
        """
        self._obj_robot.submit_assembly(self._station, self._shiptment_type)
