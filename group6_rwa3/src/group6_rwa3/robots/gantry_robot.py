from moveit_runner import MoveitRunner
from utils.utility import print_partition
from utils.conversion import get_target_world_pose
from utils.utility import get_part_location_in_bins


class GantryRobot:
    """Class to represent the atrributes and methods for gantry robot
    """
    # pylint: disable=too-few-public-methods
    gantry_group_names = ['gantry_full', 'gantry_arm', 'gantry_torso']

    def __init__(self, world_model):
        self._world_model = world_model

    @staticmethod
    def go_home():
        """Function for returning gantry
        to original position
        """
        moveit_runner_gantry = MoveitRunner(
                                            GantryRobot.gantry_group_names,
                                            ns='/ariac/gantry')
        moveit_runner_gantry.gantry_status_publisher.publish("init")
        moveit_runner_gantry.go_home()
