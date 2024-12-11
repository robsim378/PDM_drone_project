import numpy as np
from environment.Shape import Shape

class Sphere(Shape):
    """ Class representing a sphere. Implementation of the Shape abstract class."""

    def __init__(self, radius):
        """ Initialize a Sphere.

        Parameters
        ----------
        float radius :
            The radius of the sphere.
        """
        self.radius = radius

    def getCollisionConstraints(self, relative_pos, inflationAmount):
        """ For more details, see the docstring in Shape for this function. """

        raise NotImplementedError("This hasn't been implemented yet.")


    def getURDF(self):
        """ For more details, see the docstring in Shape for this function. """

        raise NotImplementedError("This hasn't been implemented yet.")
