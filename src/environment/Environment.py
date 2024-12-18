import numpy as np
import cvxpy
import pybullet as p

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.BaseAviary import BaseAviary

from src.environment.Obstacle import Obstacle

class Environment():
    """ Class representing the environment a drone operates in. """

    def __init__(self, aviary_env: CtrlAviary):
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
        #Array of objects in the environment represented by their pybullet id's
        self.objects = []
        # self.drone_id = -1
        # self.env_id = p_id
        self.aviary_env = aviary_env

    def addObstacle(self, urdf, position, rotation, scale):
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
        # NOTE: Maybe just take a Shape object and include the URDF file as part of that? Would probably be easier than determining shape from URDF

        # TODO: Figure out if position and rotation have to be lists or if ndarrays are acceptable as well
        obj_id = p.loadURDF(urdf, position, rotation, globalScaling=1 ,physicsClientId=self.pyb_client_id)

        #pybullet predetermined shapes
        # obj_id = p.createCollisionShape(p.GEOM_SPHERE, physicsClientId=self.pyb_client_id)
        pose = np.zeros([6])
        pose[:3] = position
        pose[3] = rotation
        obstacle = Obstacle(pose, obj_id)
        # TODO: Add to the internal list of Obstacles
        self.obstacles += [obstacle]
        # Add id to objects
        self.objects += [obj_id]

    def checkCollision(self, position, inflationAmount):
        """ Checks if the requested space is occupied.

        Parameters
        ----------
        ndarray(3,) position :
            The position to check for a collision
        float inflationAmount :
            The amount to inflate the object by in all directions. Creates a safety buffer.

        Returns
        -------
        list of cvxpy constraints :
            Inequality constraints for collisions with all obstacles.
        """

        # Loop through all obstacles and check if there is a collision with any of them.
        # raise NotImplementedError("This hasn't been implemented yet.")
        constraints = []
        for obstacle in self.obstacles:
            constraints += obstacle.checkCollision(position, inflationAmount)
        return constraints
