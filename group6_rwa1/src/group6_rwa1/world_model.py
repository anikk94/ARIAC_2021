from agv_robot import AGVRobot

class WorldModel:
    """Representation of the world model
    """
    competition_state = ""

    def __init__(self):
        self.agv_robots = {}

        self.agv_robots["agv1"] = AGVRobot("agv1")
        self.agv_robots["agv2"] = AGVRobot("agv2")
        self.agv_robots["agv3"] = AGVRobot("agv3")
        self.agv_robots["agv4"] = AGVRobot("agv4")

    def get_agv_robot(self, id):
        """returns the agv robot object

        Args:
           id (_type_): the id of the agv robot to return that particular object

        Returns:
           _type_: AGBRobot()
        """
        return self.agv_robots[id]