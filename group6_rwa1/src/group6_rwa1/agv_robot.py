import rospy
from nist_gear.srv import AGVToAssemblyStation
from nist_gear.srv import AssemblyStationSubmitShipment


class AGVRobot:
    """Class to represent the atrributes and methods of the AGV Robots

    Attributes:
        agv_id: AGV ID
        station: location of AGV
    Methods:
        move(self, station, submit_shipment):
            Move AGV to destaination
    """
    def __init__(self, avg_id):
        self.avg_id = avg_id
        self.station = ""
        self.state = ""

        self.srv_name = '/ariac/' + str(self.avg_id) + '/submit_shipment'
        rospy.wait_for_service(self.srv_name)
        self.move_agv = rospy.ServiceProxy(self.srv_name, AGVToAssemblyStation)

    def move(self, station, submit_shipment):
        """ It moves the avg to a particular station

        Args:
            station (_type_): It's the station to which agv gonna move to.
        """

        self.station = station
        rospy.loginfo("AGVRobot" + str(self.avg_id) + " moved to station " + str(self.station))

        shipment_type = submit_shipment

        try:
            self.move_agv(self.station, shipment_type)
        except rospy.ServiceException as exc:
            rospy.loginfo('failed with error' + str(exc) + '\n')


    def submit_assembly(self, station, submit_shipment):
        """ It indicates that assembly at the station is complete and submits it.

        Args:
            station (_type_): It's the station at which assembly has been completed.
        """

        self.station = station
        rospy.loginfo("Assembly station " + str(self.avg_id) + " completed ")

        assm_srv_name = 'ariac/' + self.station + '/submit_shipment'
        rospy.wait_for_service(assm_srv_name)
        assm_submit = rospy.ServiceProxy(assm_srv_name, AssemblyStationSubmitShipment)

        shipment_type = submit_shipment

        try:
            assm_submit(shipment_type)
        except rospy.ServiceException as exc:
            rospy.loginfo('failed with error' + str(exc) + '\n')
