import imp
from std_msgs.msg import String
import rospy
import tf
import time
from threading import Thread
from constants import Const
from utility import print_partition

from part import Part
from agv_robot import AGVRobot

class WorldModel:
    """Representation of the world model
    """

    # pylint: disable=bare-except
    # pylint: disable=line-too-long

    _sensor_blackout_flag = False
    _data_receiver_timeout_counter = Const.INITIAL_DATA_RECEIVER_TIMEOUT_COUTNER

    _competition_state = ""

    def __init__(self):
        self._agv_robots = {}
        self.apr = 1
        self.abb = 1
        self._agv_robots["agv1"] = AGVRobot("agv1")
        self._agv_robots["agv2"] = AGVRobot("agv2")
        self._agv_robots["agv3"] = AGVRobot("agv3")
        self._agv_robots["agv4"] = AGVRobot("agv4")

        self.tf_listner = tf.TransformListener()

        # normal parts
        self.snapshot = {}
        self.parts = []
        # faulty parts
        self.fsnapshot = {}
        self.faulty_parts = []

        try:
            thread_decrease_data_receiver_timeout_counter = Thread(target = WorldModel.decrease_data_receiver_timeout_counter)
            thread_decrease_data_receiver_timeout_counter.start()

            thread_create_snapshot_parts = Thread(target = self._create_snapshot_parts)
            thread_create_snapshot_parts.start()

            thread_create_fsnapshot_parts = Thread(target = self._create_fsnapshot_parts)
            thread_create_fsnapshot_parts.start()
        except:
            print("Error: unable to start thread")

    def get_agv_robot(self, id):
        """returns the agv robot object

        Args:
           id (String): the id of the agv robot to return that particular object

        Returns:
           AGVRobot: object with the agv_id requested
        """
        return self._agv_robots[id]

    def add_part(self, part):
        """Adds the part to the world model
        Args:
           part: Part to be added
        """
        '''
        Listener code to directly check for the part frames. Modify it accordingly.
        Parts however are not being added to the parts list.
        '''
        self.parts.append(Part(part))


    def _create_snapshot_parts(self):
        """The thread to create the snapshot of what all the cameras see
        """
        while True:
            # snapshot -> values = model.data
            self.parts = []
            for model_data in self.snapshot.values():
                for part in model_data:
                    self.add_part(part)
            
            # rospy.loginfo(self.parts)
            time.sleep(1)

    def _create_fsnapshot_parts(self):
        """The thread to create the snapshot of all the faulty parts we see by the quality sensor cameras
        """
        while True:
            # snapshot -> values = model.data
            self.faulty_parts = []
            for model_data in self.fsnapshot.values():
                for part in model_data:
                    self.add_part(part)
            
            # rospy.loginfo(self.parts)
            time.sleep(1)

    def search_part_for(self, part_type):
        """searches the part type inside parts

        Args:
            part_type (String): The part type we are searching for

        Returns:
            flag: True, if the part type is found in the parts
            part_found: The Part() object which we found
        """
        flag = False
        part_found = None
        for part in self.parts:
            if part.type == part_type and not part.is_being_used:
                flag = True
                part_found = part
                part.is_being_used = True
                break

        return (flag, part_found)
            

    @staticmethod
    def decrease_data_receiver_timeout_counter():
        """The thread to decrease the sensor blackout flag timeout
        """
        while True:
            if WorldModel._competition_state == String("go"):
                if WorldModel._data_receiver_timeout_counter == 0:
                    WorldModel.set_sensor_blackout_status(True)
                else:
                    # rospy.loginfo("decrease_data_receiver_timeout_counter:" + str(WorldModel._data_receiver_timeout_counter))
                    WorldModel._data_receiver_timeout_counter -= 1

            time.sleep(0.01)

    @staticmethod
    def reset_data_receiver_timeout_counter():
        """resets the sensor blackout timeout if we are receiving something from the sensors
        """
        WorldModel._data_receiver_timeout_counter = Const.INITIAL_DATA_RECEIVER_TIMEOUT_COUTNER
        WorldModel.set_sensor_blackout_status(False)

    @staticmethod
    def get_sensor_blackout_status():
        """Gets the sensor blackout state

        Returns:
            Bool: returns the flag value of the sensor black out
        """
        return WorldModel._sensor_blackout_flag

    @staticmethod
    def set_sensor_blackout_status(flag):
        """Sets the sensor blackout state

        Args:
            flag (Bool): the flag which needed to be set
        """
        if flag != WorldModel._sensor_blackout_flag:
            WorldModel._sensor_blackout_flag = flag
            print_partition()
            rospy.loginfo("Sensor Blackout State:" + str(WorldModel._sensor_blackout_flag))
            print_partition()

    @staticmethod
    def set_competition_state(state):
        """Sets the competition state

        Args:
            state (String): sets the state of the competition
        """
        WorldModel._competition_state = state

    @staticmethod
    def get_competition_state():
        """Gets competition state

        Returns:
            String: Gives the state of the competition
        """
        return WorldModel._competition_state
