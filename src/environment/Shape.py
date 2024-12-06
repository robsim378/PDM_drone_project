import numpy as np
from abc import ABC, abstractmethod

class Shape(ABC):
    """ Abstract class representing a 3D shape. """

    @abstractmethod
    def checkCollision(self, position):
        """ Checks if a certain point is within the bounds of this shape.

        Parameters
        ----------
        ndarray(3,) position :
            The position to check for a collision in

        Returns
        -------
        bool : True if the space is occupied, false if it is free.
        """
        pass
