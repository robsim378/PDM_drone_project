import numpy as np
import pybullet as p

from src.environment.Obstacle import Obstacle

class Environment():
    """ Class representing the environment a drone operates in. """

    def __init__(self):
        """ Initialize an environment. 

        Throws
        ------
        RuntimeError :
            If initialized while there is no running PyBullet instance.
        """
        # Connect to a running pybullet instance (gym_pybullet_drones starts one, we want to connect to it)
        self.pyb_client_id = p.connect(p.SHARED_MEMORY)

        if self.pyb_client_id == -1:
            raise RuntimeError("Tried to initialize an Environment, but there was no running PyBullet instance.")

        self.obstacles = []


    def addObstacle(self, urdf, position, rotation):
        """ Add an obstacle to the environment.

        Parameters
        ----------
        string urdf :
            The filepath of the URDF file defining the obstacle
        float[3] position :
            The x, y, z coordinates to place the obstacle
        float[3] rotation :
            The euler angles defining the orientation of the obstacle
        """

        # TODO: Figure out if position and rotation have to be lists or if ndarrays are acceptable as well
        p.loadURDF(urdf, position, rotation, physicsClientId=self.pyb_client_id)

        # TODO: Add to the internal list of Obstacles


    def checkCollision(self, position):
        """ Checks if the requested space is occupied.

        Parameters
        ----------
        ndarray(3,) position :
            The position to check for a collision

        Returns
        -------
        bool : True if the space is occupied, false if it is free.
        """

        # Loop through all obstacles and check if there is a collision with any of them.
        raise NotImplementedError("This hasn't been implemented yet.")
