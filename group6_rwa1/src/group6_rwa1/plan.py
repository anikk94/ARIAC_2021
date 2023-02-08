from group6_rwa1.tasks.task import Task

class Plan:
    
    """Class to represent the atrributes and methods of Plan
    Attributes:
        tasks: Data Structure to save the set of tasks in a plan    
    Methods:
        add_task(self, task): adding order details to data structure
        execute_plan(self): execute the tasks
    """

    def __init__(self, id):
        self.tasks = []
        self.order_id = id

    def add_task(self, task):
        """Adds the task to be executed
        Args:
            task (_type_): Task to be executed
        """
        self.tasks.append(task)

    def execute_plan(self):
        """Execute the sequence of steps for kitting or assembly
        """
        while self.tasks:
            task_to_do = self.tasks.pop(0)
            if isinstance(task_to_do, Task):
                task_to_do.task_done()
