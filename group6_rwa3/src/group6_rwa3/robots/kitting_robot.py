from time import sleep
from utils.constants import Const
from moveit_runner import MoveitRunner
import rospy
import copy
from utils.utility import print_partition
from utils.conversion import get_target_world_pose
from utils.utility import get_part_location_in_bins


class KittingRobot:
    """Class defining functions for kitting robot
    """
    kitting_group_names = ['kitting_arm']

    def __init__(self, world_model):
        self._world_model = world_model
        self.moveit_runner_kitting = None

    def discard_part(self, faulty_dest_part):
        """Start discarding unnecessary parts
        """
        print_partition()
        rospy.loginfo("Discarding Faulty Part.." + faulty_dest_part.type)
        print_partition()
        self.moveit_runner_kitting = MoveitRunner(KittingRobot.
                                                  kitting_group_names,
                                                  ns='/ariac/kitting')
        faulty_dest_part_trash = copy.deepcopy(faulty_dest_part)
        faulty_dest_part_trash.pose.position.x = -2.212158
        faulty_dest_part_trash.pose.position.y = -0.014119
        faulty_dest_part_trash.pose.position.z = 0.828004
        # self.moveit_runner_kitting.pick_part(faulty_dest_part)
        # self.moveit_runner_kitting.throw_part(faulty_dest_part_trash)
        self.moveit_runner_kitting.move_part(faulty_dest_part.location_type,
                                             faulty_dest_part,
                                             faulty_dest_part_trash,
                                             "trash_bin")

    def move_part(self, dest_part, agv_id, dest_part_in_world_pose_flag = False):
        """Reading the data and moving the part
        """
        print_partition()
        rospy.loginfo("MOVING PART for.." + dest_part.type)
        rospy.loginfo(dest_part.pose)
        rospy.loginfo(agv_id)
        rospy.loginfo("dest_part_in_world_pose_flag: " + str(dest_part_in_world_pose_flag))
        print_partition()

        if not dest_part_in_world_pose_flag:
            dest_part = get_target_world_pose(dest_part, agv_id)


        is_part_found = False
        part_found = None
        is_part_found, part_found = self._world_model.search_part_for(dest_part.type)

        if is_part_found:
            if (part_found.pose.position.x == 0 and
                part_found.pose.position.y == 0 and
                part_found.pose.position.z == 0):
                print_partition()
                rospy.loginfo("Error: MOVING PART for.." + dest_part.type + " FAILED!")
                print_partition
                return

            part_dest_pos = copy.deepcopy(dest_part)
            bin_type = part_found.location_type
            self.moveit_runner_kitting = MoveitRunner(KittingRobot.
                                                        kitting_group_names,
                                                        ns='/ariac/kitting')
            self.moveit_runner_kitting.move_part(bin_type, part_found,
                                                    part_dest_pos, agv_id)
        else:
            print_partition()
            rospy.loginfo("NO RELEVANT PART FOUND:" + dest_part.type)
            print_partition()
            return

        print_partition()
        rospy.loginfo("Verifying the Moved Part..")
        print_partition()
        sleep(Const.SLEEP_TIMER)
        self.sanity_check(agv_id)
 

    def sanity_check(self, agv_id):
        """Checking the wokring for faulty part
        """

        agv_robot = self._world_model.get_agv_robot(agv_id)
        flag, faulty_part = self._world_model.search_faulty_part_in(agv_id)

        if flag:
            print_partition()
            rospy.loginfo("Faulty Part Identified by Quality Sensor")
            rospy.loginfo(faulty_part.pose)
            print_partition()

            part_on_agv = copy.deepcopy(agv_robot.find_part_on_tray(faulty_part))
            # orig_part_pos = copy.deepcopy(faulty_part)
            if part_on_agv is not None:
                print_partition()
                rospy.loginfo("Identified FAULTY PART on " + agv_id + ":" + part_on_agv.type)
                rospy.loginfo(part_on_agv.pose)
                print_partition()
                self.discard_part(part_on_agv)

                self.move_part(part_on_agv, agv_id, True)
        else:
            return
