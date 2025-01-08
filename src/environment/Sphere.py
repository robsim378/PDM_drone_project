import numpy as np
import cvxpy as cp
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

    def getCollisionConstraints(self, relative_position, padding_amount):
        """ For more details, see the docstring in Shape for this function. """

        return [cp.norm(relative_position, 2) >= (self.radius + padding_amount)]

    def getURDF(self):
        """ For more details, see the docstring in Shape for this function. """

        raise NotImplementedError("This hasn't been implemented yet.")
