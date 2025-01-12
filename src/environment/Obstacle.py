import numpy as np
import pyomo as pyo
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary

from src.environment.Shape import Shape
from src.environment.Cylinder import Cylinder

class Obstacle():
    """ Class defining an obstacle in an environment. """
    
    def __init__(self, position, pyb_obj_id, shape, environment, trajectory=None):
        """ Initialize an Obstacle.
        
        Parameters
        ----------
        ndarray(3,) position :
            The position of the obstacle. For dynamic obstacles, this is their initial pose.
        int pyb_obj_id :
            The ID of the pybullet object for this obstacle
        Shape shape : 
            The Shape of the obstacle.
        Environment environment :
            The environment this obstacle resides in
        function trajectory :
            The trajectory of the obstacle as a function of time. For a static obstacle, 
            leave this unset.
        """
        self.position = position
        self.shape = shape
        self.environment = environment
        self.pyb_obj_id = pyb_obj_id
        # NOTE: if we use the built-in predefined pybullet shapes we can use their 
        # identifiers (e.g. p.GEOM_SPHERE) to determine which of our shape class we need
        # self.pyb_obj_id = pyb_obj_id
        # self.pyb_client_id = p.connect(p.SHARED_MEMORY)
        # self.environment = CtrlAviary()

        # If the obstacle is static, the trajectory should always be 0.
        if trajectory is None:
            self.trajectory = lambda x: np.array([0, 0, 0])
        else:
            self.trajectory = trajectory

    def getCollisionConstraints(self, drone_global_position, padding_amount, binary_vars, timestep_index, obstacle_index):
        """ Checks if the obstacle will occupy a certain position at the specified time.

        Parameters
        ----------
        pyomo.Expression[3] position :
            The cvxpy variable representing the position to check for a collision, relative to
            the world origin.
        float padding_amount :
            The amount to inflate the object by in all directions. Creates a safety buffer.
        pyomo.Var[] binary_vars :
            Binary variables used for mixed-integer constraints where needed. Currently unused,
            since mixed-integer nonconvex programming is incredibly slow, but left in place just in case.
        int timestep_index :
            How many timesteps into the future to consider the position of this object at.

        Returns
        -------
        list of pyomo.Expressions :
            The list of constraints defining collision with an obstacle in this environment.
        """

        # Obstacle pose w.r.t world at time t
        obstacle_global_position = self.position + self.trajectory(self.environment.time + timestep_index * self.environment.dt) 

        # pyomo indexing/slicing is weird, so we have to construct the relative position like this
        drone_relative_position = []
        for i, drone_global_position_component in enumerate(drone_global_position):
            drone_relative_position.append(drone_global_position_component - obstacle_global_position[i])
            if i >= 2:
                break

        if isinstance(self.shape, Cylinder):
            drone_relative_position[-1] = 0

        binary_vars_list = []
        for binary_var in binary_vars[:, timestep_index, obstacle_index]:
            binary_vars_list.append(binary_var)

        constraints = self.shape.getCollisionConstraints(drone_relative_position, padding_amount, binary_vars_list) 
        return constraints

    def getInverseDistance(self, drone_global_position, timestep_index):
        """ Gets the inverse of the distance from a position as a cvxpy expression

        Parameters
        ----------
        cvxpy.Variable[3] drone_global_position :
            The position to check for a collision in, relative to the world origin.
        int timestep_index :
            How many timesteps into the future to consider the position of this object at.

        Returns
        -------
        list of cvxpy Constraints :
            The list of constraints defining collision with an obstacle in this environment.
        """

        # Obstacle pose w.r.t world at time t
        obstacle_global_position = self.position + self.trajectory(self.environment.time + timestep_index * self.environment.dt) 
        
        # pyomo indexing/slicing is weird, so we have to construct the relative position like this
        drone_relative_position = []
        for i, drone_global_position_component in enumerate(drone_global_position):
            drone_relative_position.append(drone_global_position_component - obstacle_global_position[i])
            if i >= 2:
                break

        if isinstance(self.shape, Cylinder):
            drone_relative_position[-1] = 0

        return self.shape.getInverseDistance(drone_relative_position)
