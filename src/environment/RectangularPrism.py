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


    def getCollisionConstraints(self, relative_position, padding_amount):
    # def getCollisionConstraints(self, drone_global_position, obstacle_global_position, padding_amount):
        """ For more details, see the docstring in Shape for this function. """

        min_x = padding_amount + self.length/2
        min_y = padding_amount + self.width/2
        min_z = padding_amount + self.height/2

        # constraints = [cp.abs(drone_global_position[0]) >= min_x + obstacle_global_position[0],
        #                cp.abs(drone_global_position[1]) >= min_y + obstacle_global_position[1],
        #                cp.abs(drone_global_position[2]) >= min_z + obstacle_global_position[2]]
        return [cp.norm(relative_position[0]) >= min_x,
                       cp.norm(relative_position[1]) >= min_y,
                       cp.norm(relative_position[2]) >= min_z]


    def getURDF(self):

        """ For more details, see the docstring in Shape for this function. """


        raise NotImplementedError("This hasn't been implemented yet.")



