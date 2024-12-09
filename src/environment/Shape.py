import numpy as np
from abc import ABC, abstractmethod

class Shape(ABC):
    """ Abstract class representing a 3D shape. """

    @abstractmethod
    def getCollisionConstraints(self, relative_pos):
        """ Computes and returns the cvxpy constraints for a Shape 

        Parameters
        ----------
        ndarray(3,) relative_pos :
            The position to check for a collision in, relative to the centre of the Shape.

        Returns
        -------
        list of cvxpy Constraints : 
            The list of constraints defining collision with this shape.
        """
        pass
