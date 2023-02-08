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
        # this will only work if part id < 10 (single digit)
        #self.name = part.type[part.type.index("assembly_pump_red"):-2]
        # self.name = part.type
        # print("PART CLASS")
        # print(self.type)
        # print(self.name)
