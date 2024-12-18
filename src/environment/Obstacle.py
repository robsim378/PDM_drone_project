import numpy as np
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from src.environment.Shape import Shape

class Obstacle():
    """ Class defining an obstacle in an environment. """
    
    def __init__(self, pose, pyb_obj_id, shape=None, trajectory=None):
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
        # NOTE: if we use the built-in predefined pybullet shapes we can use their 
        # identifiers (e.g. p.GEOM_SPHERE) to determine which of our shape class we need
        self.pyb_obj_id = pyb_obj_id
        self.pyb_client_id = p.connect(p.SHARED_MEMORY)
        self.environment = BaseAviary()

        # If the obstacle is static, the trajectory should always be 0.
        # NOTE: We need to make sure that the trajectory is defined entirely in terms of
        # cvxpy functions so that it can be used in constraints in the solver.
        if self.trajectory is None:
            self.trajectory = lambda x: 0
        else:
            self.trajectory = trajectory

    def checkCollision(self, global_pos, inflationAmount, time=0):
        """ Checks if the obstacle will occupy a certain position at the specified time.

        Parameters
        ----------
        ndarray(3,) global_position :
            The position to check for a collision in, relative to the environment
        float inflationAmount :
            The amount to inflate the object by in all directions. Creates a safety buffer.
        float time :
            The time to check for a collision at. For static obstacles, leave this unset.

        Returns
        -------
        list of cvxpy Constraints :
            The list of constraints defining collision with an obstacle in this environment.
        """

        # NOTE: This function will have to transform the point position from whatever 
        # reference frame it is in to the frame of the obstacle's current position. It 
        # can then call the Shape's checkCollision() function with the transformed point
        # as the position.

        #The point position of the obstacle w.r.t the world
        global_pos_obs = self.pose[:3]
        #The point to be checked for collision relative to this obstacle's center point
        relative_pos = global_pos_obs - global_pos
        #TODO: something with the orientation? not relevant for spheres but yes for squares
        return self.shape.getCollisionConstraints(relative_pos, inflationAmount)

    def getDroneState(self, aviary_env: CtrlAviary):
        """ Returns the current state of the Drone

        Returns
        -------
        DroneState :
            The current state of the drone
        """
        # raise NotImplementedError("This hasn't been implemented yet.")
        state_vec_pyb = aviary_env._getDroneStateVector(self.pyb_client_id) 
        return [state_vec_pyb[0], state_vec_pyb[2], state_vec_pyb[3], state_vec_pyb[4]] #xyz position, roll, pitch, yaw
        

    def updateDroneState(self, aviary_env: CtrlAviary, RPMs):
        """ Updates the state of the drone in PyBullet.

        Parameters
        ----------
        ndarray(4,) RPMs :
            The RPM of each rotor, going clockwise starting from the rear right rotor.
        """
        aviary_env._physics(RPMs, 0) #0 because we only have one drone. TODO: if we have more, change to find the relevant drone.

