import numpy as np
from abc import ABC, abstractmethod

class Shape(ABC):
    """ Abstract class representing a 3D shape. """

    @abstractmethod
    def getCollisionConstraints(self, relative_pos, paddingAmount):
        """ Computes and returns the cvxpy constraints for a Shape 

        Parameters
        ----------
        ndarray(3,) relative_pos :
            The position to check for a collision in, relative to the centre of the Shape.
        float inflationAmount :
            The amount to inflate the object by in all directions. Creates a safety buffer.

        Returns
        -------
        list of cvxpy Constraints : 
            The list of constraints defining collision with this shape.
        """
        pass

    @abstractmethod
    def getURDF(self):
        """ Returns the URDF file associated with this obstacle

        Returns
        -------
        String : 
            The filepath of the URDF file
        """
        pass
