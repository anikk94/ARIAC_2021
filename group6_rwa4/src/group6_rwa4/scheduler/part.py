import rospy


class Part:
    """Class to represent the atrributes and methods of Part
    Attributes:
        tasks: Data Structure to save the set of parts in an order
    Methods:
        add_[part](self, type): adding order details to data structure
    """
    def __init__(self, part):
        self.type = part.type
        self.pose = part.pose
        self.is_being_used = False
        self.is_being_verified = False
        self.location_type = ""
