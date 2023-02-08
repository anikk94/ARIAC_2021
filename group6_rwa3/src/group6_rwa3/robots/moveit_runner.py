#!/usr/bin/env python

from turtle import home
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
from std_msgs.msg import String
from tf.transformations import quaternion_multiply, quaternion_from_euler
# moveit
import moveit_commander as mc
# python
import copy
import yaml


class MoveitRunner():
    TOLERANCE_PART_HEIGHT = 0.05

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
            rospy.loginfo(group)
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
            elif 'kitting' in key:
                # print("GROUPS", self.groups)
                self.goto_preset_location('home', robot_type="kitting_robot")

    def load_preset_locations(self):
        """
        Return the path of the file containing specs for
        pick-and-place
        """
        parent = lambda path_name: os.path.dirname(path_name)

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

        rospy.loginfo("goto_preset_location: " + location_name)
        group = None
        if robot_type == 'kitting_robot':
            group = self.groups['kitting_arm']
        elif robot_type == 'gantry_robot':
            group = self.groups['gantry_full']

        kitting_arm, gantry_full, gantry_torso, gantry_arm = self.locations[location_name]
        location_pose = group.get_current_joint_values()
        # print("location_pose", location_pose)

        if robot_type == 'kitting_robot':
            location_pose[:] = kitting_arm
        elif robot_type == 'gantry_robot':
            location_pose[:] = gantry_full
            location_pose[:3] = gantry_torso
            location_pose[3:] = gantry_arm

        # If the robot controller reports a path tolerance violation,
        # this will automatically re-attempt the motion
        MAX_ATTEMPTS = 5
        attempts = 0
        while not group.go(location_pose, wait=True):
            attempts += 1
            assert(attempts < MAX_ATTEMPTS)

    def pick_part(self, part_init_pose):
        print_partition()
        rospy.loginfo("Called moveit_runner.py pick_part with parameters:")
        rospy.loginfo(part_init_pose)
        rospy.loginfo(part_init_pose.pose)
        rospy.loginfo(part_init_pose.type)
        rospy.loginfo(part_init_pose.location_type)
        print_partition()

        self.goto_preset_location(part_init_pose.location_type)

        part_height = self.part_heights[part_init_pose.type]

        # This example uses the kitting robot only
        group = self.groups['kitting_arm']
        gm = GripperManager(ns='/ariac/kitting/arm/gripper/')

        # Make sure the end effector is flat
        # otherwise it will have a hard time attaching a part
        flat_orientation = euler_to_quaternion(0, 1.57, 0)
        ee_pose = part_init_pose.pose
        ee_pose.orientation = flat_orientation

        # Activate the gripper and make sure it is activated
        gripper_status = gm.activate_gripper()
        if not gripper_status:
            assert(gm.activate_gripper()), "Could not activate gripper"

        pre_pose = copy.deepcopy(ee_pose)

        ee_pose.position.z = part_height + MoveitRunner.TOLERANCE_PART_HEIGHT
        plan, _ = group.compute_cartesian_path(
            [ee_pose], 0.01, 0.0)
        group.execute(plan, wait=True)

        while not gm.is_object_attached():
            gripper_status = gm.activate_gripper()
            if not gripper_status:
                assert(gm.activate_gripper()), "Could not activate gripper"
            ee_pose.position.z -= 0.001
            plan, _ = group.compute_cartesian_path(
                [ee_pose], 0.001, 0.0)
            group.execute(plan, wait=True)
            rospy.sleep(0.5)

        # Once the part is attached
        # Lift the arm
        plan, _ = group.compute_cartesian_path(
            [pre_pose], 0.01, 0.0)
        group.execute(plan, wait=True)

        self.goto_preset_location(part_init_pose.location_type)


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

    def place_part(self):
        gm = GripperManager(ns='/ariac/kitting/arm/gripper/')
        gm.deactivate_gripper()

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

        AGV_Z_POS_CORRECTION = 0.0

        if part_init_vessel[0:3] == "agv":
            AGV_Z_POS_CORRECTION = 0.04

        z_pos = self.part_heights[part_init_pose.type] + AGV_Z_POS_CORRECTION

        # This example uses the kitting robot only
        group = self.groups['kitting_arm']
        gm = GripperManager(ns='/ariac/kitting/arm/gripper/')

        # compute the relative rotation between part pose in bin and tray
        rel_rot_q = self.compute_relative_rotation(
            part_init_pose.pose, part_target_pose.pose)

        # group.set_goal_orientation_tolerance = 0.02
        # group.set_goal_position_tolerance = 0.02

        # Make sure the end effector is flat
        # otherwise it will have a hard time attaching a part
        flat_orientation = euler_to_quaternion(0, 1.57, 0)
        ee_pose = group.get_current_pose().pose
        ee_pose.orientation = flat_orientation
        
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
        (plan, _) = group.compute_cartesian_path(path, 0.001, 0.0)
        group.execute(plan, wait=True)

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

        # Move to preset locations
        self.goto_preset_location(part_init_vessel)
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
