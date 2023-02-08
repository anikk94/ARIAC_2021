from group6_rwa1.plan import Plan
from group6_rwa1.world_model import WorldModel
from order import OrderReceived

class Brain:

    """Class to handle the orders and execute the shipments in an order.

    Attributes:
        obj_world_model: Representation of components in world coordinates
        orders: Data Structure to save orders
        plan: Data structure to store the sequence of events to execute an order
    
    Methods:
        add_order(self, data): adding order details to data structure
        add_plan(self, obj_plan): adding plan details to data structure
        execute_plans(self): execute the sequence of events of an order
    """
    def __init__(self):
        self.obj_world_model = WorldModel()
        self.orders = []
        self.plans = []

    def add_order(self, data):
        """Adds the orders received to a data structure
        Args:
        data (_type_): Order to be added.
        """
        order_received = OrderReceived(data)
        self.orders.append(order_received)

    def add_plan(self, obj_plan):
        """Adds the sequence of steps to be executed
        Args:
            obj_plan (_type_): Plan to be added
        """
        self.plans.append(obj_plan)

    def execute_plans(self):
        """Executes the sequence of tasks in a plan
        """
        while self.plans:
            obj_plan = self.plans.pop(0)
            if isinstance(obj_plan, Plan):
                obj_plan.execute_plan()

class StaticBrain:
    """This utilizes Singleton Pattern to instantiate only one instance of Brain

    Returns:
        _type_: static instance of the object brain
    """
    _brain = None

    @staticmethod
    def instance():
        if not isinstance(StaticBrain._brain, Brain):
            StaticBrain._brain = Brain()

        return StaticBrain._brain
