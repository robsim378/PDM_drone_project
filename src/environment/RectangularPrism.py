import numpy as np
import cvxpy as cp
import pyomo.environ as pyo
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


    def getCollisionConstraints(self, relative_position, padding_amount, binary_vars):
        """ For more details, see the docstring in Shape for this function. """

        # calculate the minimum distance in each direction the drone must be
        min_x = padding_amount + self.length/2
        min_y = padding_amount + self.width/2
        min_z = padding_amount + self.height/2


        print(binary_vars)
        # Define the constraints for each direction
        x_constraint = abs(relative_position[0])>=min_x*binary_vars[0]
        y_constraint = abs(relative_position[1])>=min_y*binary_vars[1]
        z_constraint = abs(relative_position[2])>=min_z*binary_vars[2]

        binary_constraint = pyo.quicksum(binary_vars[0], binary_vars[1], binary_vars[2])>=1
        binary_constraint = pyo.quicksum(binary_vars) >= 1

        return [x_constraint, y_constraint, z_constraint, binary_constraint]


    def getInverseDistance(self, relative_position):
        """ For more details, see the docstring in Shape for this function. """
        # return -cp.log(cp.norm(relative_position, 2))
        return 1.0 / cp.norm(relative_position, 2)


    def getURDF(self):

        """ For more details, see the docstring in Shape for this function. """


        raise NotImplementedError("This hasn't been implemented yet.")



