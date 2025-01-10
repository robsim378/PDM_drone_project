import numpy as np
from abc import ABC, abstractmethod

class Shape(ABC):
    """ Abstract class representing a 3D shape. """

    @abstractmethod
    def getCollisionConstraints(self, relative_position, padding_amount):
        """ Computes and returns the cvxpy constraints for a Shape 

        Parameters
        ----------
        pyomo.Expression[3] relative_pos :
            The position to check for a collision in, relative to the centre of the Shape.
        float inflationAmount :
            The amount to inflate the object by in all directions. Creates a safety buffer.

        Returns
        -------
        list of pyomo.Expression : 
            The list of constraints defining collision with this shape.
        """
        pass

    @abstractmethod
    def getInverseDistance(self, relative_position):
        """ Returns a cvxpy expression that evaluates to the inverse of the distance to the object. 

        Parameters
        ----------
        pyomo.Expression[3] relative_position :
            The position to compute the inverse distance to, relative to the centre of the Shape.

        Returns
        -------
        pyomo.Expression :
            The inverse of the distance from the surface of the object
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
