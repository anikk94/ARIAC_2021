#!/usr/bin/env python

from time import sleep
import rospy
import os

# custom modules
from utils.conversion import euler_to_quaternion
from utils.utility import print_partition
from robots.gripper_manager import GripperManager
# nist
from nist_gear.srv import GetMaterialLocations
from nist_gear.msg import Model, LogicalCameraImage
# ros
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf.transformations import quaternion_multiply, quaternion_from_euler
# moveit
import moveit_commander as mc
# python
import copy
import yaml


class MoveitRunner():
    TOLERANCE_PART_HEIGHT = 0.1

    def __init__(self, group_names, node_name='ariac_moveit_example', ns='',
                 robot_description='robot_description'):

        rospy.loginfo("Initiailizing move it RUNNNNNER")
        self.gantry_status_publisher = rospy.Publisher(
            '/my/gantry_status', String, queue_size=10)
        self.gantry_status_subscriber = rospy.Subscriber(
            "/my/gantry_status", String, self.gantry_status_callback)
        self.gantry_status = ""

        self.robot = mc.RobotCommander(ns + '/' + robot_description, ns)
        print('Robot', self.robot)
        self.scene = mc.PlanningSceneInterface(ns)
        self.groups = {}
        for group_name in group_names:
            group = mc.MoveGroupCommander(
                group_name,
                robot_description=ns + '/' + robot_description,
                ns=ns
            )
            group.set_goal_tolerance(0.05)
            self.groups[group_name] = group

        self.set_ariac_specs()

    def gantry_status_callback(self, msg):
        self.gantry_status = msg.data

    def go_home(self):
        rospy.loginfo("Going Home...")
        for key in self.groups:

            if 'gantry' in key:
                # print("GROUPS", self.groups)
                self.goto_preset_location('home', robot_type="gantry_robot")

                # self.goto_preset_location('pre_gk_agv1', robot_type="gantry_robot")
                # self.goto_preset_location('agv1', robot_type="gantry_robot")
                # self.goto_preset_location('pre_gk_agv1', robot_type="gantry_robot")
                # self.goto_preset_location('home', robot_type="gantry_robot")
                # self.goto_preset_location('pre_gk_agv2', robot_type="gantry_robot")
                # self.goto_preset_location('agv2', robot_type="gantry_robot")
                # self.goto_preset_location('pre_gk_agv2', robot_type="gantry_robot")
                # self.goto_preset_location('home', robot_type="gantry_robot")
                # self.goto_preset_location('pre_gk_agv3', robot_type="gantry_robot")
                # self.goto_preset_location('agv3', robot_type="gantry_robot")
                # self.goto_preset_location('pre_gk_agv3', robot_type="gantry_robot")
                # self.goto_preset_location('home', robot_type="gantry_robot")
                # self.goto_preset_location('pre_gk_agv4', robot_type="gantry_robot")
                # self.goto_preset_location('agv4', robot_type="gantry_robot")
                # self.goto_preset_location('pre_gk_agv4', robot_type="gantry_robot")
                # self.goto_preset_location('home', robot_type="gantry_robot")


            elif 'kitting' in key:
                # print("GROUPS", self.groups)
                self.goto_preset_location('home', robot_type="kitting_robot")

    def load_preset_locations(self):
        """
        Return the path of the file containing specs for
        pick-and-place
        """
        parent = lambda path_name: os.path.dirname(path_name)
        print(parent)
        # i.e. /path/to/dir/kitting_commander_node.py
        current_file = os.path.abspath(__file__)
        test_competitor_dir = parent(parent(parent(parent(current_file))))
        print(test_competitor_dir)
        specs_path = "config/robot_workcell_specs.yaml"
        print(os.path.join(test_competitor_dir, specs_path))
        return os.path.join(test_competitor_dir, specs_path)

    def set_ariac_specs(self):
        '''
        Static file with preset locations for easy navigation.
        This file also contains specs for bins that may be useful
        '''

        ariac_specs_file = self.load_preset_locations()
        # Read YAML file
        rospy.loginfo(ariac_specs_file)
        with open(ariac_specs_file, 'r') as stream:
            data_loaded = yaml.safe_load(stream)

        locations = {}
        part_heights = {}
        bin_height = None
        agv_height = None

        for key, value in data_loaded.items():
            if key in "preset_locations":
                for loc, group in value.items():
                    kitting_arm = group['kitting_arm']
                    gantry_full = group['gantry_full']
                    gantry_torso = group['gantry_torso']
                    gantry_arm = group['gantry_arm']
                    locations[loc] = (
                        kitting_arm, gantry_full, gantry_torso, gantry_arm)
            if key in "bins":
                bin_height = value["height"]
            if key in "agvs":
                agv_height = value["height"]
            if key in "parts":
                for part, part_h in value.items():
                    part_name = part
                    part_height = part_h["height"]
                    part_heights[part_name] = part_height
                    # print(part_name, part_height)

        self.locations = locations
        self.part_heights = part_heights
        self.agv_height = agv_height
        self.bin_height = bin_height

    def goto_preset_location(self, location_name, robot_type="kitting_robot"):
        group = None
        if robot_type == 'kitting_robot':
            group = self.groups['kitting_arm']
        elif robot_type == 'gantry_robot':
            group = self.groups['gantry_full']

        kitting_arm, gantry_full, gantry_torso, gantry_arm = self.locations[location_name]
        location_pose = group.get_current_joint_values()
        rospy.loginfo(robot_type + " going to " + location_name)

        if robot_type == 'kitting_robot':
            location_pose[:] = kitting_arm
        elif robot_type == 'gantry_robot':
            location_pose[:] = gantry_full
            location_pose[:3] = gantry_torso
            location_pose[3:] = gantry_arm

        # If the robot controller reports a path tolerance violation,
        # this will automatically re-attempt the motion
        MAX_ATTEMPTS = 25
        attempts = 0
        while not group.go(location_pose, wait=True):
            attempts += 1
            rospy.sleep(1)
            assert(attempts < MAX_ATTEMPTS)


    def throw_part(self, part_target_pose):
        self.goto_preset_location("trash_bin")

        group = self.groups['kitting_arm']
        gm = GripperManager(ns='/ariac/kitting/arm/gripper/')

        flat_orientation = euler_to_quaternion(0, 1.57, 0)
        ee_pose = group.get_current_pose().pose
        ee_pose.orientation = flat_orientation

        q_current = quaternion_from_euler(0, 0, 0)
        q_current[0] = ee_pose.orientation.x
        q_current[1] = ee_pose.orientation.y
        q_current[2] = ee_pose.orientation.z
        q_current[3] = ee_pose.orientation.w

        # pose to place the part
        place_pose = copy.deepcopy(part_target_pose.pose)
        place_pose.orientation.x = q_current[0]
        place_pose.orientation.y = q_current[1]
        place_pose.orientation.z = q_current[2]
        place_pose.orientation.w = q_current[3]
        place_pose.position.z += 0.2

        path = [place_pose]
        self.cartesian_move(group, path)

        gm.deactivate_gripper()

    
    def get_pose_for_bin(self, bin_id):
        pose = Pose()
        dx = 0.20
        if bin_id == "bin1":
            pose.position.x = -1.898993
            pose.position.y = 3.379920
            pose.position.z = 0
        elif bin_id == "bin2":
            pose.position.x = -1.898993
            pose.position.y = 2.565006
            pose.position.z = 0
        elif bin_id == "bin3":
            pose.position.x = -2.651690 + dx
            pose.position.y = 2.565006
            pose.position.z = 0
        elif bin_id == "bin4":
            pose.position.x = -2.651690 + dx
            pose.position.y = 3.379920
            pose.position.z = 0
        elif bin_id == "bin5":
            pose.position.x = -1.898993
            pose.position.y = -3.379920
            pose.position.z = 0
        elif bin_id == "bin6":
            pose.position.x = -1.898993
            pose.position.y = -2.565006
            pose.position.z = 0
        elif bin_id == "bin7":
            pose.position.x = -2.651690 + dx
            pose.position.y = -2.565006
            pose.position.z = 0
        elif bin_id == "bin8":
            pose.position.x = -2.651690 + dx
            pose.position.y = -3.379920
            pose.position.z = 0

        return pose.position


    def move_part_conveyor_belt(self,
                  part_init_pose,
                  bin_id):
        """
        Move a part from conveyor belt to empty bin

        Args:
            part_type (str): Type of the part (e.g., assembly_sensor_red)
            part_init_pose: Pose of the part in bin (world frame)
            part_target_pose: Pose of the part on AGV (world frame)
            agv (str): AGV to place the part

        Returns:
            bool: True if the part was successfully moved
        """

        part_init_vessel = "belt_cam"

        part_target_pose = copy.deepcopy(part_init_pose)
        part_target_pose.pose.position = self.get_pose_for_bin(bin_id)
        part_target_pose.pose.position.z = part_init_pose.pose.position.z
        

        print_partition()
        rospy.loginfo("Called moveit_runner.py move_part_conveyor_belt with parameters:")
        rospy.loginfo(part_init_vessel)
        rospy.loginfo(part_init_pose.pose)
        rospy.loginfo(part_target_pose.pose)
        rospy.loginfo(bin_id)
        print_partition()

        BELT_POS_CORRECTION = 0.12

        z_pos = self.part_heights[part_init_pose.type] + BELT_POS_CORRECTION

        # This example uses the kitting robot only
        group = self.groups['kitting_arm']
        gm = GripperManager(ns='/ariac/kitting/arm/gripper/')

        # compute the relative rotation between part pose in bin and tray
        rel_rot_q = self.compute_relative_rotation(
            part_init_pose.pose, part_target_pose.pose)


        # Make sure the end effector is vertical
        # otherwise it will have a hard time attaching a part
        vert_orientation = euler_to_quaternion(0, 3.14159, -1.57)
        ee_pose = group.get_current_pose().pose
        ee_pose.orientation = vert_orientation
        
       
        # pre-grasp pose
        near_pick_pose = copy.deepcopy(part_init_pose.pose)
        near_pick_pose.orientation = ee_pose.orientation
        near_pick_pose.position.z = z_pos + MoveitRunner.TOLERANCE_PART_HEIGHT
        # grasp pose
        pick_pose = copy.deepcopy(part_init_pose.pose)
        pick_pose.orientation = ee_pose.orientation
        pick_pose.position.z = z_pos

        # Activate the gripper and make sure it is activated
        gripper_status = gm.activate_gripper()
        if not gripper_status:
            assert(gm.activate_gripper()), "Could not activate gripper"

        # Move the arm next to a bin
        self.goto_preset_location(part_init_vessel)
        # velocity_scaling_factor = 1.0  # default is 1.0
        path = [near_pick_pose, pick_pose]
        (plan, _) = group.compute_cartesian_path(path, 0.01, 0.0)
        group.execute(plan, wait=True)

        while not gm.is_object_attached():
            gripper_status = gm.activate_gripper()
            if not gripper_status:
                assert(gm.activate_gripper()), "Could not activate gripper"
            rospy.sleep(0.5)

        # Move to preset locations
        self.goto_preset_location(part_init_vessel)
        self.goto_preset_location(bin_id)

        q_current = quaternion_from_euler(0, 0, 0)
        q_current[0] = ee_pose.orientation.x
        q_current[1] = ee_pose.orientation.y
        q_current[2] = ee_pose.orientation.z
        q_current[3] = ee_pose.orientation.w

        ee_pose_q = quaternion_multiply(rel_rot_q, q_current)

        # pose to place the part
        place_pose = copy.deepcopy(part_target_pose.pose)
        place_pose.orientation.x = ee_pose_q[0]
        place_pose.orientation.y = ee_pose_q[1]
        place_pose.orientation.z = ee_pose_q[2]
        place_pose.orientation.w = ee_pose_q[3]

        place_pose.position.z -= 0.005

        path = [place_pose]
        self.cartesian_move(group, path)

        gm.deactivate_gripper()

        sleep(4)

        self.goto_preset_location(bin_id)

        return True

    def move_part(self,
                  part_init_vessel,
                  part_init_pose,
                  part_target_pose,
                  agv):
        """
        Move a part from a bin (part_init_vessel) to an AGV (agv)

        Args:
            part_type (str): Type of the part (e.g., assembly_sensor_red)
            part_init_vessel (str): location unit (e.g., bin1)
            part_init_pose: Pose of the part in bin (world frame)
            part_target_pose: Pose of the part on AGV (world frame)
            agv (str): AGV to place the part

        Returns:
            bool: True if the part was successfully moved
        """

        print_partition()
        rospy.loginfo("Called moveit_runner.py move_part with parameters:")
        rospy.loginfo(part_init_vessel)
        rospy.loginfo(part_init_pose.pose)
        rospy.loginfo(part_target_pose.pose)
        rospy.loginfo(agv)
        print_partition()

        # rospy.sleep(0.5)

        AGV_Z_POS_CORRECTION = 0.0001

        if part_init_vessel[0:3] == "agv":
            AGV_Z_POS_CORRECTION = 0.04

        z_pos = self.part_heights[part_init_pose.type] + AGV_Z_POS_CORRECTION

        # This example uses the kitting robot only
        group = self.groups['kitting_arm']
        gm = GripperManager(ns='/ariac/kitting/arm/gripper/')

        # compute the relative rotation between part pose in bin and tray
        rel_rot_q = self.compute_relative_rotation(
            part_init_pose.pose, part_target_pose.pose)


        # Make sure the end effector is flat
        # otherwise it will have a hard time attaching a part
        flat_orientation = euler_to_quaternion(0, 1.57, 0)
        ee_pose = group.get_current_pose().pose
        ee_pose.orientation = flat_orientation
        
        # PICK PART
        # pre-grasp pose
        near_pick_pose = copy.deepcopy(part_init_pose.pose)
        near_pick_pose.orientation = ee_pose.orientation
        near_pick_pose.position.z = z_pos + MoveitRunner.TOLERANCE_PART_HEIGHT
        # grasp pose
        pick_pose = copy.deepcopy(part_init_pose.pose)
        pick_pose.orientation = ee_pose.orientation
        pick_pose.position.z = z_pos

        # Activate the gripper and make sure it is activated
        gripper_status = gm.activate_gripper()
        if not gripper_status:
            assert(gm.activate_gripper()), "Could not activate gripper"

        # rospy.sleep(0.5)
        # Move the arm next to a bin
        self.goto_preset_location(part_init_vessel)

        path = [near_pick_pose, pick_pose]
        (plan, _) = group.compute_cartesian_path(path, 0.001, 0.0)
        group.execute(plan, wait=True)

        # rospy.sleep(0.5)

        while not gm.is_object_attached():
            gripper_status = gm.activate_gripper() 
            if not gripper_status:
                assert(gm.activate_gripper()), "Could not activate gripper"
            pick_pose.position.z -= 0.001
            plan, _ = group.compute_cartesian_path(
                [pick_pose], 0.001, 0.0)
            group.execute(plan, wait=True)
            rospy.sleep(0.5)

        # Once the part is attached
        # Lift the arm
        plan, _ = group.compute_cartesian_path(
            [group.get_current_pose().pose, near_pick_pose], 0.001, 0.0)
        group.execute(plan, wait=True)

        # PLACE PART
        # Move to preset locations
        # rospy.sleep(0.5)
        self.goto_preset_location(part_init_vessel)
        # rospy.sleep(0.5)
        self.goto_preset_location(agv)

        q_current = quaternion_from_euler(0, 0, 0)
        q_current[0] = ee_pose.orientation.x
        q_current[1] = ee_pose.orientation.y
        q_current[2] = ee_pose.orientation.z
        q_current[3] = ee_pose.orientation.w

        ee_pose_q = quaternion_multiply(rel_rot_q, q_current)

        # pose to place the part
        place_pose = copy.deepcopy(part_target_pose.pose)
        place_pose.orientation.x = ee_pose_q[0]
        place_pose.orientation.y = ee_pose_q[1]
        place_pose.orientation.z = ee_pose_q[2]
        place_pose.orientation.w = ee_pose_q[3]
        place_pose.position.z += 0.15

        path = [place_pose]
        self.cartesian_move(group, path)

        gm.deactivate_gripper()

        self.goto_preset_location(agv)

        return True

    def move_part_gantry(self,
                  part_init_vessel,
                  part_init_pose,
                  part_target_pose,
                  agv):
        """
        Move a part from a bin (part_init_vessel) to an AGV (agv)

        Args:
            part_type (str): Type of the part (e.g., assembly_sensor_red)
            part_init_vessel (str): location unit (e.g., bin1)
            part_init_pose: Pose of the part in bin (world frame)
            part_target_pose: Pose of the part on AGV (world frame)
            agv (str): AGV to place the part

        Returns:
            bool: True if the part was successfully moved
        """

        print_partition()
        rospy.loginfo("Called moveit_runner.py move_part_gantry with parameters:")
        rospy.loginfo(part_init_vessel)
        rospy.loginfo(part_init_pose.pose)
        rospy.loginfo(part_target_pose.pose)
        rospy.loginfo(agv)
        print_partition()

        AGV_Z_POS_CORRECTION = 0.0

        if part_init_vessel[0:3] == "agv":
            AGV_Z_POS_CORRECTION = 0.09

        z_pos = self.part_heights[part_init_pose.type] + AGV_Z_POS_CORRECTION

        # This example uses the kitting robot only
        group = self.groups['gantry_arm']
        gm = GripperManager(ns='/ariac/gantry/arm/gripper/')

        # compute the relative rotation between part pose in bin and tray
        rel_rot_q = self.compute_relative_rotation(
            part_init_pose.pose, part_target_pose.pose)

        flat_orientation = euler_to_quaternion(0, 1.57, 0)
        ee_pose = group.get_current_pose().pose
        ee_pose.orientation = flat_orientation
        
        # far_grasp pose
        far_pick_pose = copy.deepcopy(part_init_pose.pose)
        far_pick_pose.orientation = ee_pose.orientation
        far_pick_pose.position.z = z_pos + 0.2

        # pre-grasp pose
        near_pick_pose = copy.deepcopy(part_init_pose.pose)
        near_pick_pose.position.z = z_pos + MoveitRunner.TOLERANCE_PART_HEIGHT
        near_pick_pose.orientation = ee_pose.orientation
        
        # grasp pose
        pick_pose = copy.deepcopy(part_init_pose.pose)
        pick_pose.orientation = ee_pose.orientation
        pick_pose.position.z = z_pos

        # Activate the gripper and make sure it is activated
        gripper_status = gm.activate_gripper()
        if not gripper_status:
            assert(gm.activate_gripper()), "Could not activate gripper"


        # Move the arm next to a bin
        self.goto_preset_location(part_init_vessel,robot_type="gantry_robot")
        # velocity_scaling_factor = 1.0  # default is 1.0
        path = [far_pick_pose]
        (plan, _) = group.compute_cartesian_path(path, 1, 1)
        group.execute(plan, wait=True)
        path = [near_pick_pose, pick_pose]
        (plan, _) = group.compute_cartesian_path(path, 0.1, 0.01)
        group.execute(plan, wait=True)

        while not gm.is_object_attached():
            gripper_status = gm.activate_gripper()
            if not gripper_status:
                assert(gm.activate_gripper()), "Could not activate gripper"
            # pick_pose.position.z -= 0.001
            pick_pose.position.z -= 0.01
            plan, _ = group.compute_cartesian_path(
                [pick_pose], 0.001, 0.0)
            group.execute(plan, wait=True)
            rospy.sleep(0.1)


        # Once the part is attached
        # Lift the arm
        plan, _ = group.compute_cartesian_path(
            [near_pick_pose, far_pick_pose], 1, 0)
            # [group.get_current_pose().pose, near_pick_pose, far_pick_pose], 0.1, 0.001)
            # [group.get_current_pose().pose, near_pick_pose, far_pick_pose], 0.01, 0.001)
            # [group.get_current_pose().pose, near_pick_pose], 0.01, 0.0)
        group.execute(plan, wait=True)

        # Move to preset locations
        rospy.sleep(1) 
        sleep(1)
        self.goto_preset_location(part_init_vessel, 'gantry_robot')
        rospy.sleep(1) 
        sleep(1)
        self.goto_preset_location('home', 'gantry_robot')
        rospy.sleep(1) 
        sleep(1)
        self.goto_preset_location("pre_gk_" + agv, 'gantry_robot')
        rospy.sleep(1) 
        sleep(1)
        self.goto_preset_location(agv, 'gantry_robot')

        q_current = quaternion_from_euler(0, 0, 0)
        q_current[0] = ee_pose.orientation.x
        q_current[1] = ee_pose.orientation.y
        q_current[2] = ee_pose.orientation.z
        q_current[3] = ee_pose.orientation.w

        ee_pose_q = quaternion_multiply(rel_rot_q, q_current)

        # pose to place the part
        place_pose = copy.deepcopy(part_target_pose.pose)
        place_pose.orientation.x = ee_pose_q[0] 
        place_pose.orientation.y = ee_pose_q[1]
        place_pose.orientation.z = ee_pose_q[2]
        place_pose.orientation.w = ee_pose_q[3]
        place_pose.position.z += 0.2

        # far place pose
        far_place_pose = copy.deepcopy(place_pose)
        far_place_pose.position.z += 0.4
        # path = [far_place_pose]
        # self.cartesian_move(group, path)
        rospy.sleep(1) 
        sleep(1)

        path = [far_place_pose,  place_pose]
        self.cartesian_move(group, path)

        rospy.sleep(1) 
        sleep(1)

        gm.deactivate_gripper()

        # retreat to far place pose
        path = [far_place_pose]
        self.cartesian_move(group, path)

        self.goto_preset_location(agv, 'gantry_robot')
        rospy.sleep(1) 
        sleep(1)
        self.goto_preset_location("pre_gk_" + agv, 'gantry_robot')
        # self.goto_preset_location('home', 'gantry_robot')
        rospy.sleep(1) 
        sleep(1)
        return True

    def move_part_assembly(self,
                  part_init_vessel, # agv1, agv2, ...
                  part_init_pose, # position[], orientation[]
                  part_target_pose, # position[], orientation[]
                  station): # as1, as2, ...
        """
        Move a part from a bin (part_init_vessel) to an AGV (agv)

        Args:
            part_type (str): Type of the part (e.g., assembly_sensor_red)
            part_init_vessel (str): location unit (e.g., bin1, agv1) (from specs)
            part_init_pose: Pose of the part in bin (world frame)
            part_target_pose: Pose of the part on AGV (world frame)
            agv (str): AGV to place the part

        Returns:
            bool: True if the part was successfully moved
        """

        print_partition()
        rospy.loginfo("Called moveit_runner.py move_part_assembly with parameters:")
        rospy.loginfo(part_init_vessel)
        rospy.loginfo(part_init_pose.pose)
        rospy.loginfo(part_target_pose.pose)
        rospy.loginfo(station)
        print_partition()

        if str(station) == "as1" or str(station) == "as3":
            home = "home1"
        else: 
            home = "home2"

        AGV_Z_POS_CORRECTION = 0.0

        if part_init_vessel[0:3] == "agv":
            AGV_Z_POS_CORRECTION = 0.02

        z_pos = self.part_heights[part_init_pose.type] + AGV_Z_POS_CORRECTION

        # This example uses the kitting robot only
        group = self.groups['gantry_arm']
        gm = GripperManager(ns='/ariac/gantry/arm/gripper/')

        # compute the relative rotation between part pose in bin and tray
        rel_rot_q = self.compute_relative_rotation(
            part_init_pose.pose, part_target_pose.pose)

        # Make sure the end effector is flat 
        # otherwise it will have a hard time attaching a part
        flat_orientation = euler_to_quaternion(0, 1.57, 0)
        ee_pose = group.get_current_pose().pose
        ee_pose.orientation = flat_orientation

        # PICK
        # far_grasp pose
        far_pick_pose = copy.deepcopy(part_init_pose.pose)
        far_pick_pose.orientation = ee_pose.orientation
        far_pick_pose.position.z = z_pos + 0.2

        # pre-grasp pose
        near_pick_pose = copy.deepcopy(part_init_pose.pose)
        near_pick_pose.orientation = ee_pose.orientation
        near_pick_pose.position.z = z_pos + MoveitRunner.TOLERANCE_PART_HEIGHT
        
        # grasp pose
        pick_pose = copy.deepcopy(part_init_pose.pose)
        pick_pose.orientation = ee_pose.orientation
        pick_pose.position.z = z_pos

        # Activate the gripper and make sure it is activated
        gripper_status = gm.activate_gripper()
        if not gripper_status:
            print('could not activate gripper')
            assert(gm.activate_gripper()), "Could not activate gripper"
        
        # Move the arm next to a bin
        # PREVENT HOME MOVE FROM RECURRING FOR EVERY PART ON AGV IF POSSIBLE
        self.goto_preset_location(home, "gantry_robot")
        self.goto_preset_location("before_" + str(station), "gantry_robot")
        self.goto_preset_location(str(part_init_vessel) + "_" + str(station), "gantry_robot")
        rospy.sleep(0.5)
        
        path = [far_pick_pose, near_pick_pose, pick_pose]
        (plan, _) = group.compute_cartesian_path(path, 0.1, 0.0)
        group.execute(plan, wait=True)

        # GRASP
        while not gm.is_object_attached():
            print('Not attached')
            gripper_status = gm.activate_gripper()
            if not gripper_status:
                print('no gripper status')
                assert(gm.activate_gripper()), "Could not activate gripper"
            pick_pose.position.z -= 0.001
            plan, _ = group.compute_cartesian_path(
                [pick_pose], 0.01, 0.0)
            group.execute(plan, wait=True)
            rospy.sleep(0.1)


        # Once the part is attached lift the arm
        plan, _ = group.compute_cartesian_path(
            [near_pick_pose, far_pick_pose], 0.1, 0.0)
        group.execute(plan, wait=True)

        # PLACE ASSEMBLY PART
        # retreat from pick location and go to relay point
        self.goto_preset_location(str(part_init_vessel) + "_" + str(station), robot_type="gantry_robot")
        self.goto_preset_location("before_" + str(station), robot_type="gantry_robot")
        self.goto_preset_location(station,robot_type="gantry_robot")
        rospy.sleep(0.5)   
        
        q_current = quaternion_from_euler(0, 0, 0)
        q_current[0] = ee_pose.orientation.x
        q_current[1] = ee_pose.orientation.y
        q_current[2] = ee_pose.orientation.z
        q_current[3] = ee_pose.orientation.w

        ee_pose_q = quaternion_multiply(rel_rot_q, q_current)

        # pose to place the part
        place_pose = copy.deepcopy(part_target_pose.pose)
        place_pose.orientation.x = ee_pose_q[0] 
        place_pose.orientation.y = ee_pose_q[1]
        place_pose.orientation.z = ee_pose_q[2]
        place_pose.orientation.w = ee_pose_q[3]

        # far place pose
        far_place_pose = copy.deepcopy(place_pose)
        far_place_pose.position.z += 0.2

        # make the approach to place the part
        path = [far_place_pose, place_pose]
        self.cartesian_move(group, path)

        # release grasp
        gm.deactivate_gripper()

        # start of retreat moves after placing part
        path = [far_place_pose]
        self.cartesian_move(group, path)

        # retreat safely using backward sequence of preset locations
        self.goto_preset_location(station, robot_type="gantry_robot")
        rospy.sleep(0.1)
        self.goto_preset_location("before_" + station,robot_type="gantry_robot")
        rospy.sleep(0.1)
        # PREVENT THIS IF THERE ARE MORE PARTS ON AGV AND GANTRY CAN BE RETAINED
        self.goto_preset_location(home, robot_type="gantry_robot")
        rospy.sleep(0.1)        
        return True

    def cartesian_move(self, group, waypoints):
        """Compute the cartesian coordinated for motion
        """
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
        group.execute(plan, wait=True)

    def compute_relative_rotation(self, init_pose, target_pose):
        """
        Compute the relative rotation between two poses.
        This relative rotation will be applied to the current end effector
        orientation.

        Args:
            init_pose (geometry_msgs.Pose): Pose of the part in the bin
            target_pose (geometry_msgs.Pose): Pose of the part in the tray
        """

        quat_init_inv = [init_pose.orientation.x,
                         init_pose.orientation.y,
                         init_pose.orientation.z,
                         -init_pose.orientation.w]  # Negate for inverse

        quat_target = [target_pose.orientation.x,
                       target_pose.orientation.y,
                       target_pose.orientation.z,
                       target_pose.orientation.w]

        q_relative_rotation = quaternion_multiply(quat_target, quat_init_inv)
        # print(q_relative_rotation, type(q_relative_rotation))

        return q_relative_rotation
