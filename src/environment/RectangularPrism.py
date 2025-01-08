import numpy as np
import cvxpy as cp
from src.environment.Shape import Shape

class RectangularPrism(Shape):
    """ Class representing a rectangular prism. Implementation of the Shape abstract class."""

    def __init__(self, length, width, height, position):
        """ Initialize a RectangularPrism.

        Parameters
        ----------
        float length, width, height:
            The dimensions of the rectangular prism
        """
        self.length = length
        self.width = width
        self.height = height


    def getCollisionConstraints(self, relative_pos, paddingAmount):

        """ For more details, see the docstring in Shape for this function. """


        x = cp.Variable()
        y = cp.Variable()
        z = cp.Variable()

        min_x = paddingAmount + relative_pos[0] + self.length/2

        min_y = paddingAmount + relative_pos[1] + self.width/2

        min_z = paddingAmount + relative_pos[2] + self.height/2

        constraints = [x >= min_x,
                        y >= min_y,
                        z >= min_z]
        
        return constraints

        # raise NotImplementedError("This hasn't been implemented yet.")



    def getURDF(self):

        """ For more details, see the docstring in Shape for this function. """


        raise NotImplementedError("This hasn't been implemented yet.")



