from group6_rwa2.agv_robot import AGVRobot
from group6_rwa2.tasks.task import Task

class SubmitAssemblyShipmentTask(Task):
    """This is the task for submitting the Assembly

    Args:
        : _description_
    """
    def __init__(self, obj_agv_robot, station, shipment_type):
        self._obj_robot = obj_agv_robot
        self._station = station
        self._shiptment_type = shipment_type

    def task_done(self):
        if  isinstance(self._obj_robot, AGVRobot):
            self._obj_robot.submit_assembly(self._station, self._shiptment_type)