import numpy as np
# import cvxpy as cp
import pyomo.environ as pyo
from src.environment.Shape import Shape

class Sphere(Shape):
    """ Class representing a sphere. Implementation of the Shape abstract class."""

    def __init__(self, radius=10.0):
        """ Initialize a Sphere.

        Parameters
        ----------
        float radius :
            The radius of the sphere.
        """
        self.radius = radius

    def getCollisionConstraints(self, relative_position, padding_amount, binary_vars):
        """ For more details, see the docstring in Shape for this function. """

        squared_norm = pyo.quicksum(relative_position[i]**2 for i in range(3))
        min_squared_distance = (self.radius + padding_amount)**2

        constraint = squared_norm >= min_squared_distance

        return([constraint])

    def getInverseDistance(self, relative_position):
        """ For more details, see the docstring in Shape for this function. """
        # return 1.0 / cp.norm(relative_position, 2)
        # return cp.square(cp.norm(relative_position, 2) - self.radius)
        # return cp.norm(relative_position, 2) - self.radius
        squared_norm = pyo.quicksum(relative_position[i]**2 for i in range(3))
        min_squared_distance = (self.radius)**2

        return 1/(squared_norm - min_squared_distance)


    def getURDF(self):
        """ For more details, see the docstring in Shape for this function. """

        raise NotImplementedError("This hasn't been implemented yet.")
