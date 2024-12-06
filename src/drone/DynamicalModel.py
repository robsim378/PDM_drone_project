import numpy as np

class DynamicalModel:
    """ Class representing the state-space model of a drone. """

    def __init__(self, A, B, C, D):
        """ Initialize a DynamicalModel.

        Parameters
        ----------
        ndarray A, B, C, D:
            The A, B, C, and D matrices of the state-space model.
        """
        self.A = A
        self.B = B
        self.C = C
        self.D = D
