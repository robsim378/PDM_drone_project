import numpy as np
from src.environment.Shape import Shape

class Obstacle():
    """ Class defining an obstacle in an environment. """
    
    def __init__(self, pose, shape, trajectory=None):
        """ Initialize an Obstacle.
        
        Parameters
        ----------
        ndarray(6,) pose :
            The position of the obstacle. For dynamic obstacles, this is their initial pose.
        Shape shape : 
            The shape of the obstacle.
        function trajectory :
            The trajectory of the obstacle as a function of time. For a static obstacle, 
            leave this unset.
        """
        self.pose = pose
        self.shape = shape

        # If the obstacle is static, the trajectory should always be 0.
        if self.trajectory is None:
            self.trajectory = lambda x: 0
        else:
            self.trajectory = trajectory

    def checkCollision(self, position, time=0):
        """ Checks if the obstacle will occupy a certain position at the specified time.

        Parameters
        ----------
        ndarray(3,) position :
            The position to check for a collision in
        float time :
            The time to check for a collision at. For static obstacles, leave this unset.

        Returns
        -------
        bool : True if the space is occupied, false if it is free.
        """

        # NOTE: This function will have to transform the point 'position' from whatever 
        # reference frame it is in to the frame of the obstacle's current position. It 
        # can then call the shape's checkCollision() function with the transformed point
        # as the position.

        raise NotImplementedError("This hasn't been implemented yet.")
