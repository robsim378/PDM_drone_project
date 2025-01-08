import numpy as np
from src.environment.Shape import Shape

class Sphere(Shape):
    """ Class representing a sphere. Implementation of the Shape abstract class."""

    def __init__(self, radius=10):
        """ Initialize a Sphere.

        Parameters
        ----------
        float radius :
            The radius of the sphere.
        """
        self.radius = radius

    def getCollisionConstraints(self, relative_pos, paddingAmount):
        """ For more details, see the docstring in Shape for this function. """
        
        x = cp.Variable()
        y = cp.Variable()
        z = cp.Variable()

        min_x = self.radius + paddingAmount
        min_y = self.radius + paddingAmount
        min_z = self.radius + paddingAmount

        constraints = [x >= min_x,
                        y >= min_y,
                        z >= min_z]

        # raise NotImplementedError("This hasn't been implemented yet.")


    def getURDF(self):
        """ For more details, see the docstring in Shape for this function. """

        raise NotImplementedError("This hasn't been implemented yet.")
