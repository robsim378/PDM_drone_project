import numpy as np

from src.environment.Environment import Environment

def GlobalPlanner():
    """ Class that generates global trajectories for the drone to follow. """

    def __init__(self, environment):
        """ Initialize a GlobalPlanner. 

        Parameters
        ----------
        Environment environment :
            The environment this planner operates in.

        """

        self.environment = environment
        raise NotImplementedError("This hasn't been implemented yet.")

    def computePath(self, start, end):
        """ Compute a path from start to end. 

        Parameters
        ----------
        ndarray(3,) start :
            The starting position of the path
        ndarray(3,) end :
            The ending position of the path
        """

        # NOTE: Might need to bump up start and end to shape (6,) for position, or probably at least (4,) for yaw
        raise NotImplementedError("This hasn't been implemented yet.")

