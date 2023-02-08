from task import Task
from nist_gear.srv import AssemblyStationSubmitShipment
import rospy 

class SubmitAssemblyShipmentTask(Task):
    """This is the task for submitting the Assembly
    """
    def __init__(self, station, shipment_type):
        self._station_id = station
        self._shipment_type = shipment_type
        self.srv_name= '/ariac/' + str(self._station_id) + '/submit_shipment'
        self.ship_briefcase=None

    def task_done(self):
        """Completion of the task
        """
        self.ship()

    def ship(self):
        """ It moves the avg to a particular station

        Args:
            station (String): It's the station to which agv gonna move to.
        """
        rospy.wait_for_service(self.srv_name)
        self.ship_briefcase = rospy.ServiceProxy(self.srv_name, AssemblyStationSubmitShipment)

        rospy.loginfo(
                    "Assembly completed at" + str(self._station_id))

        try:
            self.ship_briefcase(self._shipment_type)
        except rospy.ServiceException as exc:
            rospy.loginfo('failed with error' + str(exc) + '\n')
