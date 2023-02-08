from moveit_runner import MoveitRunner
from utils.utility import print_partition
from utils.conversion import get_target_world_pose,get_target_world_pose_assembly
from utils.utility import get_part_location_in_bins
from scheduler.part import Part
import rospy
import copy
from time import sleep
from utils.constants import Const
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from math import pi


class Robot:
    """Class to represent the atrributes and methods for kitting and gantry robot
    """
    # pylint: disable=too-few-public-methods
    gantry_group_names = ['gantry_full', 'gantry_arm', 'gantry_torso']
    kitting_group_names = ['kitting_arm']
    PI_VALUE = round(pi, 3)
    FLIPPED_PART_DETECTED = False
    PART_NEED_TO_BE_FLIPPED = None
    KITTING_ROBOT_MOVE_IT_RUNNER = None

    def __init__(self, world_model):
        self._world_model = world_model
        self.moveit_runner_kitting = None
        self.moveit_runner_gantry = None

    @staticmethod
    def go_home_gantry():
        """Function for returning gantry
        to original position
        """
        moveit_runner_gantry = MoveitRunner(
                                            Robot.gantry_group_names,
                                            ns='/ariac/gantry')
        moveit_runner_gantry.gantry_status_publisher.publish("init")
        moveit_runner_gantry.go_home()

    @staticmethod
    def go_home_kitting():
        """Function for returning kitting
        to original position
        """
        rospy.loginfo(Robot.kitting_group_names)
        sleep(Const.SLEEP_TIMER)
        if Robot.KITTING_ROBOT_MOVE_IT_RUNNER is None:
            Robot.KITTING_ROBOT_MOVE_IT_RUNNER = MoveitRunner(Robot.kitting_group_names,ns='/ariac/kitting')
        
        Robot.KITTING_ROBOT_MOVE_IT_RUNNER.go_home()
        sleep(Const.SLEEP_TIMER)
 
    def discard_part_kitting(self, faulty_dest_part):
        """Start discarding unnecessary parts
        """
        print_partition()
        rospy.loginfo("Discarding Faulty Part.." + faulty_dest_part.type)
        print_partition()
        self.moveit_runner_kitting = MoveitRunner(Robot.
                                                  kitting_group_names,
                                                  ns='/ariac/kitting')
        faulty_dest_part_trash = copy.deepcopy(faulty_dest_part)
        faulty_dest_part_trash.pose.position.x = -2.212158
        faulty_dest_part_trash.pose.position.y = -0.014119
        faulty_dest_part_trash.pose.position.z = 0.828004
        # self.moveit_runner_gantry.pick_part(faulty_dest_part)
        # self.moveit_runner_gantry.throw_part(faulty_dest_part_trash)
        self.moveit_runner_kitting.move_part(faulty_dest_part.location_type,
                                             faulty_dest_part,
                                             faulty_dest_part_trash,
                                             "trash_bin") 


    def stock_parts_from_conveyor_belt(self, part_type):
        # rospy.loginfo("stock_parts_from_conveyor_belt")
        # rospy.loginfo(self._world_model.conveyor_parts)
        if self._world_model.conveyor_parts:
            flag, part_found = self._world_model.search_part_in_conveyor(part_type)
            if flag:
                part_found = get_target_world_pose(part_found, "logical_camera_conveyor_frame")
                part_found.pose.position.y = part_found.pose.position.y - 2.2
                self.move_part_from_conveyor_belt(part_found)

    def move_part_from_conveyor_belt(self, part):
        self.moveit_runner_kitting = MoveitRunner(Robot.
                                                  kitting_group_names,
                                                  ns='/ariac/kitting')      
        if self._world_model.empty_bins:
            for bin_id in self._world_model.empty_bins:
                break
            self.moveit_runner_kitting.move_part_conveyor_belt(part, bin_id)


    def flip_part(self):
        self.moveit_runner_kitting = MoveitRunner(Robot.
                                            kitting_group_names,
                                            ns='/ariac/kitting')
        
        # initial part
        part_init = copy.deepcopy(Robot.PART_NEED_TO_BE_FLIPPED)
        euler_values = euler_from_quaternion(
            [part_init.pose.orientation.x,
            part_init.pose.orientation.y,
            part_init.pose.orientation.z,
            part_init.pose.orientation.w])
        roll = 0
        pitch = euler_values[1]
        yaw = euler_values[2]
        rotation_zero = quaternion_from_euler(roll, pitch, yaw)
        part_init.pose.orientation.x = rotation_zero[0]
        part_init.pose.orientation.y = rotation_zero[1]
        part_init.pose.orientation.z = rotation_zero[2]
        part_init.pose.orientation.w = rotation_zero[3]

        # rotate by 90 degree
        roll = -Robot.PI_VALUE / 2
        part_dest = copy.deepcopy(Robot.PART_NEED_TO_BE_FLIPPED)
        rotation_piby2 = quaternion_from_euler(roll, pitch, yaw)
        part_dest.pose.orientation.x = rotation_piby2[0]
        part_dest.pose.orientation.y = rotation_piby2[1]
        part_dest.pose.orientation.z = rotation_piby2[2]
        part_dest.pose.orientation.w = rotation_piby2[3]
        self.moveit_runner_kitting.move_part(Robot.PART_NEED_TO_BE_FLIPPED.location_type, part_init,
                                                        part_dest, Robot.PART_NEED_TO_BE_FLIPPED.location_type)

        
        agv_robot = self._world_model.get_agv_robot(Robot.PART_NEED_TO_BE_FLIPPED.location_type)
        part_dest = copy.deepcopy(agv_robot.find_part_on_tray(part_dest))
        # rotate by 180 degree
        part_dest_end = copy.deepcopy(Robot.PART_NEED_TO_BE_FLIPPED)
        roll = -Robot.PI_VALUE
        rotation_pi = quaternion_from_euler(roll, pitch, yaw)
        part_dest_end.pose.orientation.x = rotation_pi[0]
        part_dest_end.pose.orientation.y = rotation_pi[1]
        part_dest_end.pose.orientation.z = rotation_pi[2]
        part_dest_end.pose.orientation.w = rotation_pi[3]
        self.moveit_runner_kitting.move_part(Robot.PART_NEED_TO_BE_FLIPPED.location_type, part_dest,
                                                        part_dest_end, Robot.PART_NEED_TO_BE_FLIPPED.location_type)

        # reposition for correct pose
        agv_robot = self._world_model.get_agv_robot(Robot.PART_NEED_TO_BE_FLIPPED.location_type)
        part_tray = copy.deepcopy(agv_robot.find_part_on_tray(part_dest_end))
        part_dest_end.pose.position.z = part_tray.pose.position.z
        self.moveit_runner_kitting.move_part(Robot.PART_NEED_TO_BE_FLIPPED.location_type, part_tray,
                                                        part_dest_end, Robot.PART_NEED_TO_BE_FLIPPED.location_type)

        Robot.FLIPPED_PART_DETECTED = False

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

        
        euler_values = euler_from_quaternion(
            [dest_part.pose.orientation.x,
            dest_part.pose.orientation.y,
            dest_part.pose.orientation.z,
            dest_part.pose.orientation.w])

        roll = round(euler_values[0], 3)
        pitch = euler_values[1]
        yaw = euler_values[2]

        rospy.loginfo(euler_values)

        if round(abs(roll),3) == round(Robot.PI_VALUE, 3):
            print_partition()
            rospy.loginfo("FLIPPED PART CHALLENGE DETECTED")
            Robot.FLIPPED_PART_DETECTED = True
            Robot.PART_NEED_TO_BE_FLIPPED = Part(dest_part)
            Robot.PART_NEED_TO_BE_FLIPPED.location_type = agv_id
            print_partition()

        roll = 0.0

        rotation_without_roll = quaternion_from_euler(roll, pitch, yaw)
        dest_part.pose.orientation.x = rotation_without_roll[0]
        dest_part.pose.orientation.y = rotation_without_roll[1]
        dest_part.pose.orientation.z = rotation_without_roll[2]
        dest_part.pose.orientation.w = rotation_without_roll[3]

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
            if bin_type in ("bin1","bin2","bin6","bin5"):
                self.moveit_runner_kitting = MoveitRunner(Robot.
                                            kitting_group_names,
                                            ns='/ariac/kitting')
                self.moveit_runner_kitting.move_part(bin_type, part_found,
                                                        part_dest_pos, agv_id)
            else:
                self.moveit_runner_gantry = MoveitRunner(Robot.
                                                            gantry_group_names,
                                                            ns='/ariac/gantry')
                self.moveit_runner_gantry.move_part_gantry(bin_type, part_found,
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

    def move_part_assembly(self, dest_part, dest_part_in_world_pose_flag = False):
        """Reading the data and moving the part
        """

        #agv2 = self._world_model.get_agv_robot("agv2")
        print('before conversion',dest_part)

        #Find part for AGV assembly parts and obtain parts 
        is_part_found = False
        part_found = None
        is_part_found, part_found, agv = copy.deepcopy(self._world_model.find_assembly_parts(dest_part))
        print('part in assembly',dest_part,is_part_found,agv)
        # if agv is not None:
        #     if not dest_part_in_world_pose_flag:
        #         print('Getting world pose')
        #         dest_part = get_target_world_pose(dest_part, agv)
        #         print(dest_part)

        if is_part_found:
            station_id = part_found.location_type
            if not dest_part_in_world_pose_flag:
                print('Getting world pose2')
                dest_part = get_target_world_pose_assembly(dest_part, station_id)
                print(dest_part)

            if (part_found.pose.position.x == 0 and
                part_found.pose.position.y == 0 and
                part_found.pose.position.z == 0):
                print_partition()
                rospy.loginfo("Error: MOVING PART for.." + dest_part.type + " FAILED!")
                print_partition
                return

            part_dest_pos = copy.deepcopy(dest_part)
            self.moveit_runner_gantry = MoveitRunner(Robot.
                                                        gantry_group_names,
                                                        ns='/ariac/gantry')
            self.moveit_runner_gantry.move_part_assembly(agv, part_found,
                                                    part_dest_pos, station_id)
        else:
            print_partition()
            rospy.loginfo("NO RELEVANT PART FOUND:" + dest_part.type)
            print_partition()
            return

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
                self.discard_part_kitting(part_on_agv)
                self.move_part(part_on_agv, agv_id, True)
        else:
            if Robot.FLIPPED_PART_DETECTED and Robot.PART_NEED_TO_BE_FLIPPED is not None:
                rospy.loginfo("Flip the part here")
                # write to see which part needs to be flipped
                rospy.loginfo(Robot.PART_NEED_TO_BE_FLIPPED)
                rospy.loginfo(Robot.PART_NEED_TO_BE_FLIPPED.type)
                rospy.loginfo(Robot.PART_NEED_TO_BE_FLIPPED.pose)
                self.flip_part()
                
            # self.go_home_kitting()
            return