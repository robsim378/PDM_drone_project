
import numpy as np
# import cvxpy as cp
import pyomo.environ as pyo
from src.environment.Shape import Shape

class Cylinder(Shape):
    """ Class representing a cylinder of infinite height. Implementation of the Shape abstract class."""

    def __init__(self, radius=10.0):
        """ Initialize a cylinder.

        Parameters
        ----------
        float radius :
            The radius of the cylinder.
        """
        self.radius = radius

    def getCollisionConstraints(self, relative_position, padding_amount, binary_vars):
        """ For more details, see the docstring in Shape for this function. """

        squared_norm = pyo.quicksum(relative_position[i]**2 for i in range(2))
        min_squared_distance = (self.radius + padding_amount)**2

        constraint = squared_norm >= min_squared_distance

        return([constraint])

    def getInverseDistance(self, relative_position):
        """ For more details, see the docstring in Shape for this function. """

        squared_norm = pyo.quicksum(relative_position[i]**2 for i in range(2))
        min_squared_distance = (self.radius)**2

        return 1/(squared_norm - min_squared_distance)


    def getURDF(self):
        """ For more details, see the docstring in Shape for this function. """

        raise NotImplementedError("This hasn't been implemented yet.")
