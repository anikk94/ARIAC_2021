#!/usr/bin/env python

from brain import Brain

class StaticBrain:
    """This utilizes Singleton Pattern to instantiate only one instance of Brain

    Returns:
        Brain(): static instance of the object brain
    """
    # pylint: disable=too-few-public-methods
    _brain = Brain()

    @staticmethod
    def instance():
        """Returns one single instance of the brian
        
        If the instance of the brain is not initialized,
        it creates an instance then returns it,
        Otherwise, the already present instance.

        Returns:
            Brain(): _description_
        """
        return StaticBrain._brain