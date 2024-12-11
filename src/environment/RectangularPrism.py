import numpy as np
from environment.Shape import Shape

class RectangularPrism(Shape):
    """ Class representing a rectangular prism. Implementation of the Shape abstract class."""

    def __init__(self, length, width, height):
        """ Initialize a RectangularPrism.

        Parameters
        ----------
        float length, width, height:
            The dimensions of the rectangular prism
        """
        self.length = length
        self.width = width
        self.height = height

    def checkCollision(self, relative_pos):
        """ For more details, see the docstring in Shape for this function. """

        raise NotImplementedError("This hasn't been implemented yet.")

    def getURDF(self):
        """ For more details, see the docstring in Shape for this function. """

        raise NotImplementedError("This hasn't been implemented yet.")


