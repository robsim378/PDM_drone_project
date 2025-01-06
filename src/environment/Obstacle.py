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
        # self.pyb_obj_id = pyb_obj_id
        # self.pyb_client_id = p.connect(p.SHARED_MEMORY)
        # self.environment = CtrlAviary()

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
        # global_pose_obs = self.pose[:3]

        # Obstacle pose w.r.t world at time t
        global_pose_obs = self.pose + self.trajectory(time) 
        #NOTE: Not sure if this is how we use the trajectory function so double check. 
        # We said trajectory would be offsets of obstacle's position, 
        # so add it to the obstacle's initial pose to get pose at time t?



        relative_pose = np.zeros_like(self.pose)
        relative_pose[:3] = global_pos - global_pose_obs[:3]    #The point to be checked for collision relative to this obstacle's center point
        relative_pose[3:] = -global_pose_obs[3:]                #orientation of point relative to obstacle
        return self.shape.getCollisionConstraints(relative_pose, inflationAmount)

    def getDroneState(self, aviary_env: CtrlAviary):
        """ Returns the current state of the Drone

        Returns numpy array of length 6. x,y,z,rool,pitch,yaw
        -------
        DroneState :
            The current state of the drone from the environment.
        """
        state_vec_pyb = aviary_env._getDroneStateVector(nth_drone=0)
        #NOTE: nth_drone=0 because we only have one drone. 
        #TODO: if we have more drones, change to find the relevant drone.
        xyz_position = state_vec_pyb[0]
        rpy_orientation = [state_vec_pyb[2], state_vec_pyb[3], state_vec_pyb[4]]
        # return [state_vec_pyb[0], state_vec_pyb[2], state_vec_pyb[3], state_vec_pyb[4]] #xyz position, roll, pitch, yaw
        return np.array(xyz_position + rpy_orientation)
        

    def updateDroneState(self, aviary_env: CtrlAviary, rpm_list):
        """ Updates the state of the drone in PyBullet.

        Parameters
        ----------
        ndarray(4,) rpm_list :
            The RPM of each rotor, going clockwise starting from the rear right rotor.
        """
        aviary_env._physics(rpm_list, nth_drone=0) 
        #NOTE: nth_drone=0 because we only have one drone. 
        #TODO: if we have more drones, change to find the relevant drone.

        #TODO: change to just update the position
        

