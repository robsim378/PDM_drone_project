import numpy as np

class DynamicalModel:
    """ Class representing the state-space model of a drone. """

    def __init__(self, A, B):
        """ Initialize a DynamicalModel.

        Parameters
        ----------
        ndarray A, B:
            The A and B matrices of the state-space model
        """
        self.A = A
        self.B = B
