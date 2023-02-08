"""agv robot file containing the AGV Robot class
"""

from utils.utility import print_partition
import rospy
from utils.utility import print_partition
from utils.conversion import get_target_world_pose
from nist_gear.srv import AGVToAssemblyStation
from nist_gear.srv import AssemblyStationSubmitShipment


class AGVRobot:
    """Class to represent the atrributes and methods of the AGV Robots

    Attributes:
        agv_id: AGV ID
        station: location of AGV
    """
    def __init__(self, agv_id):
        self.agv_id = agv_id
        self.station = ""
        self.state = ""
        self.parts = []

        self.srv_name = '/ariac/' + str(self.agv_id) + '/submit_shipment'
        self.move_agv = None

    def move(self, station, submit_shipment):
        """ It moves the avg to a particular station

        Args:
            station (String): It's the station to which agv gonna move to.
        """
        rospy.wait_for_service(self.srv_name)
        self.move_agv = rospy.ServiceProxy(self.srv_name, AGVToAssemblyStation)

        self.station = station
        rospy.loginfo(
                    "AGVRobot" + str(self.agv_id) +
                    " moved to station " + str(self.station))

        shipment_type = submit_shipment

        try:
            self.move_agv(self.station, shipment_type)
        except rospy.ServiceException as exc:
            rospy.loginfo('failed with error' + str(exc) + '\n')

    def add_agv_parts(self, part):
        """ add parts which in in the tray of agv
        """
        self.parts.append(part)

    def submit_assembly(self, station, submit_shipment):
        """ It indicates that assembly at the station is
        complete and submits it.

        Args:
            station (String): It's the station at which assembly
            has been completed.
        """

        self.station = station
        rospy.loginfo("Assembly station " + str(self.agv_id) + " completed ")

        assm_srv_name = 'ariac/' + self.station + '/submit_shipment'
        rospy.wait_for_service(assm_srv_name)
        assm_submit = rospy.ServiceProxy(assm_srv_name,
                                         AssemblyStationSubmitShipment)

        shipment_type = submit_shipment

        try:
            assm_submit(shipment_type)
        except rospy.ServiceException as exc:
            rospy.loginfo('failed with error' + str(exc) + '\n')

    def find_part_on_tray(self, part_to_find):
        rospy.loginfo("in rwa4 agvrobot")
        tolerance_limit = 1 # 1 percent
        tolerance_min = (100 - tolerance_limit) * 0.01
        tolerance_max = (100 + tolerance_limit) * 0.01
        
        part_pos = part_to_find.pose.position

        part_pos_x_minima = min(part_pos.x * tolerance_min, part_pos.x * tolerance_max)
        part_pos_x_maxima = max(part_pos.x * tolerance_min, part_pos.x * tolerance_max)

        part_pos_y_minima = min(part_pos.y * tolerance_min, part_pos.y * tolerance_max)
        part_pos_y_maxima = max(part_pos.y * tolerance_min, part_pos.y * tolerance_max)

        part_pos_z_minima = min(part_pos.z * tolerance_min, part_pos.z * tolerance_max)
        part_pos_z_maxima = max(part_pos.z * tolerance_min, part_pos.z * tolerance_max)

        print_partition()
        rospy.loginfo("Faulty Part Min and Max:")
        rospy.loginfo(str(part_pos_x_minima) + " ," + str(part_pos_x_maxima))
        rospy.loginfo(str(part_pos_y_minima) + " ," + str(part_pos_y_maxima))
        rospy.loginfo(str(part_pos_z_minima) + " ," + str(part_pos_z_maxima))
        print_partition()

        def is_pos_same_with_part_to_find(pos):
            they_equal = False
            rospy.loginfo("Pos:")
            rospy.loginfo(pos)

            rospy.loginfo("is_pos_same_with_part_to_find Faulty Part Min and Max:")
            rospy.loginfo(str(part_pos_x_minima) + " ," + str(part_pos_x_maxima))
            rospy.loginfo(str(part_pos_y_minima) + " ," + str(part_pos_y_maxima))
            rospy.loginfo(str(part_pos_z_minima) + " ," + str(part_pos_z_maxima))
            if (pos.x > part_pos_x_minima and pos.x < part_pos_x_maxima):
                if (pos.y > part_pos_y_minima and pos.y < part_pos_y_maxima): 
                    #  if (pos.z > part_pos_z_minima and pos.z < part_pos_z_maxima): 
                        they_equal = True

            return they_equal

        similar_part_found_on_agv = None
        
        print_partition()
        rospy.loginfo("find_part_on_tray PARTS in " + self.agv_id)
        for part in self.parts:
            logical_camera_type = "logical_camera_"+ self.agv_id + "_frame"
            part = get_target_world_pose(part, logical_camera_type)
            rospy.loginfo(part.type)
            rospy.loginfo(part.pose)
        print_partition()

        for part in self.parts:
            logical_camera_type = "logical_camera_"+ self.agv_id + "_frame"
            part = get_target_world_pose(part, logical_camera_type)
            part_check_pose = part.pose.position
            if is_pos_same_with_part_to_find(part_check_pose):
                similar_part_found_on_agv = part
                print_partition()
                rospy.loginfo("Faulty Part on AGV " + self.agv_id)
                rospy.loginfo(similar_part_found_on_agv.type)
                print_partition()
                break
        
        print_partition()
        return similar_part_found_on_agv