import numpy as np
import pyomo.environ as pyo
import pybullet as p

from src.drone.DroneState import DroneState
from src.environment.RectangularPrism import RectangularPrism
from src.environment.Obstacle import Obstacle
from src.environment.Sphere import Sphere
from src.environment.Cylinder import Cylinder

class Environment():
    """ Class representing the environment a drone operates in. """

    def __init__(self, env):
        """ Initialize an environment. 

        Parameters
        ----------
        BaseAviary env :
            The gym-pybullet-drones environment. Must be a subclass of BaseAviary

        Throws
        ------
        RuntimeError :
            If initialized while there is no running PyBullet instance.
        """
        # Connect to a running pybullet instance (gym_pybullet_drones starts one, we want to connect to it)
        p.connect(p.SHARED_MEMORY)
        self.pyb_client_id = env.getPyBulletClient()

        self.env = env
        self.dt = 1 / env.CTRL_FREQ
        self.time = 0

        self.drones = {}
        self.obstacles = []
        self.obstacle_distances = []
        self.ghost_tail = []
        self.target_id = -1
        self.tracker_id = -1

    def getDroneState(self, drone_id):
        """ Get the current state of the drone from pybullet 

        Returns
        -------
        DroneState :
            The current state of the drone

        """
        pose = self.env._getDroneStateVector(drone_id)[[0, 1, 2, 8]]
        velocity = self.env._getDroneStateVector(drone_id)[[10, 11, 12, 15]]
        state = DroneState(pose, velocity, None)
        return state


    def addDrone(self, drone):
        """ Add a Drone to the list of drones in this environment. 

        Parameters
        ----------
        Drone drone :
            The Drone to add to the list.
        """
        self.drones[drone.id] = drone


    def advanceSimulation(self):
        """ Advance the simulation, executing the queued actions for all drones
        and moving dynamic obstacles. """

        self.time += self.dt

        # Initialize empty actions for all drones
        actions = np.zeros((len(self.drones), 4))

        # Get queued actions for all Drones and reset them
        for id in self.drones.keys():
            actions[id, :] = self.drones[id].action
            self.drones[id].action = None

        # Step the simulation
        obs, reward, terminated, truncated, info = self.env.step(actions)

        # Move dynamic obstacles
        for obstacle in self.obstacles:
            p.resetBasePositionAndOrientation(obstacle.pyb_obj_id, obstacle.position + obstacle.trajectory(self.time), [1, 0, 0, 0], physicsClientId=self.pyb_client_id)

        # Re-sort list of obstacles by distance
        self.sortObstacles(self.drones[0].getState().pose[:3])

        return obs


    def addCylinder(self, position, radius=0.5, trajectory=None):
        """ Add a cylindrical obstacle to the environment with infinite height.

        Parameters
        ----------
        float radius :
            The radius of the cylinder
        float[3] position :
            The x, y, z coordinates to place the obstacle
        lambda trajectory :
            The trajectory of the obstacle as a function of time. Must take time as its
            only argument and return an np.array(3,) defining the offset from the base 
            position at the given time.
        """

        # Create the pybullet object
        cylinder = p.createCollisionShape(p.GEOM_CYLINDER, radius = radius, height = 5)
        cylinder_pyb_id = p.createMultiBody(0, cylinder, basePosition=position, physicsClientId=self.pyb_client_id)

        # Create the Obstacle
        obstacle = Obstacle(position, cylinder_pyb_id, Cylinder(radius), self, trajectory=trajectory)
        self.obstacles += [obstacle]


    def addSphere(self, position, radius=0.5, trajectory=None):
        """ Add a spherical obstacle to the environment.

        Parameters
        ----------
        float radius :
            The radius of the sphere
        float[3] position :
            The x, y, z coordinates to place the obstacle
        lambda trajectory :
            The trajectory of the obstacle as a function of time. Must take time as its
            only argument and return an np.array(3,) defining the offset from the base 
            position at the given time.
        """

        # Create the pybullet object
        sphere = p.createCollisionShape(p.GEOM_SPHERE, radius = radius)
        sphere_pyb_id = p.createMultiBody(0, sphere, basePosition=position, physicsClientId=self.pyb_client_id)

        # Create the Obstacle
        obstacle = Obstacle(position, sphere_pyb_id, Sphere(radius), self, trajectory=trajectory)
        self.obstacles += [obstacle]


    def addBox(self, position, length=1, width=1, height=1):
        """ Add a box obstacle to the environment.

        Parameters
        ----------
        float length, width, height :
            The dimensions of the box
        float[3] position :
            The x, y, z coordinates to place the obstacle
        lambda trajectory :
            The trajectory of the obstacle as a function of time. Must take time as its
            only argument and return an np.array(3,) defining the offset from the base 
            position at the given time.
        """

        box = p.createCollisionShape(p.GEOM_BOX, halfExtents = [length/2, width/2, height/2])
        box_pyb_id = p.createMultiBody(0, box, basePosition=position, physicsClientId=self.pyb_client_id)

        obstacle = Obstacle(position, box_pyb_id, RectangularPrism(length, width, height, position), self)
        self.obstacles += [obstacle]


    def generate_points(self, num_points, min_distance, bounds):
        points = []
        attempts = 0
        max_attempts = 10000  # Prevent infinite loops
        
        while len(points) < num_points and attempts < max_attempts:
            attempts += 1
            # Generate a random point within the bounds
            point = np.random.uniform(bounds[:, 0], bounds[:, 1])
            
            # Check the distance from all existing points
            if all(np.linalg.norm(point - np.array(p)) >= min_distance for p in points):
                points.append(point)
        
        if len(points) < num_points:
            raise ValueError("Could not generate the required number of points within the maximum attempts.")
    
        return np.array(points)

    def addStaticObstacles(self, num_obstacles, bounds, seed=None):
        """ Add a randomized set of static pillar obstacles to the environment. 
    
        Parameters
        ----------
        int num_obstacles :
            The number of obstacles to add
        np.array(2, 2) bounds :
            Two points defining the corners of the rectangle in which to place pillars
        int seed :
            The seed used to generate the positions of the pillars
        """

        if seed is not None:
            np.random.seed(seed)

        # Determine where to place pillars, leaving a minimum space between them
        positions = self.generate_points(num_obstacles, 1, bounds)

        # Add the pillars to the environment
        for i in range(num_obstacles):
            self.addCylinder([positions[i, 0], positions[i, 1], 2.5], radius=0.3, trajectory=None)


    def addDynamicObstacles(self, num_obstacles, bounds, speed_factor, seed=None):
        """ Add a randomized set of moving spherical obstacles to the environment. 
    
        Parameters
        ----------
        int num_obstacles :
            The number of obstacles to add
        np.array(2, 3) bounds :
            Two points defining the corners of the rectangle in which to place pillars
        float speed_factor :
            A multiplier for the obstacle's speed. Doesn't correspond to any real-world value.
        int seed :
            The seed used to generate the positions and trajectories of the obstacles.
        """

        if seed is not None:
            np.random.seed(seed)

        trajectories = [
            lambda time: np.sin(time / speed_factor),
            lambda time: -np.sin(time / speed_factor),
            lambda time: np.cos(time / speed_factor),
            lambda time: -np.cos(time / speed_factor)
        ]

        # Determine where to place pillars, leaving a minimum space between them
        positions = self.generate_points(num_obstacles, 1, bounds)

        # Add the pillars to the environment
        for i in range(num_obstacles):
            trajectory_elements = np.random.randint(3, size=3)
            trajectory = (lambda elements: 
                    lambda time: np.array([
                        trajectories[elements[0]](time),
                        trajectories[elements[1]](time),
                        trajectories[elements[2]](time),
                    ])
                )(trajectory_elements)
            self.addSphere([positions[i, 0], positions[i, 1], positions[i, 2]], radius=0.2, trajectory=trajectory)


    def wind(self, wind_vector):
        """ Apply a force in the specified direction to simulate wind. 

        Parameters
        ----------
        np.array(3,) wind_vector :
            The vector defining the direction and magnitude of the wind force.
        """
        pass


    def initMPCTail(self, tail_len):
        """ Initialize the ghost tail for MPC. 

        Parameters
        ----------
        int tail_len :
            The length of the tail
        """

        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.02, rgbaColor=[0.2, 1, 1, 0.5])
        for _ in range(tail_len):
            obj_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=[0, 0, 0])
            self.ghost_tail.append(obj_id)


    def drawMPCTail(self, tail):
        """ Render the MPC tail as a series of ghostly orbs. 

        Parameters
        ----------
        DroneState[] tail :
            A list of DroneStates to display.
        """
        
        for i, state in enumerate(tail):  # `mpc_tail_positions` is a list of new positions
            p.resetBasePositionAndOrientation(
                self.ghost_tail[i], 
                state.pose[:3], 
                p.getQuaternionFromEuler(np.array([0, 0, state.pose[3]]))
            )
            
    def initTracker(self):
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 0.5])
        obj_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=[0, 0, 0])
        self.tracker_id = obj_id

    def drawTracker(self, tracker_state):
        """ Renders the tracker state in the simulation

        Parameters
        ----------
        DroneState tracker_state:
            The tracker state to be displayed
        """
        p.resetBasePositionAndOrientation(self.tracker_id, 
                                        tracker_state.pose[:3], 
                                        p.getQuaternionFromEuler(np.array([0,0,tracker_state.pose[3]])))

    def initGlobalTarget(self):
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 0.5])
        obj_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=[0, 0, 0])
        self.global_target_id = obj_id

    def drawGlobalTarget(self, target_state):
        """ Renders the global target state in the simulation

        Parameters
        ----------
        DroneState target:
            The target state to be displayed
        """
        p.resetBasePositionAndOrientation(self.global_target_id, 
                                        target_state.pose[:3], 
                                        p.getQuaternionFromEuler(np.array([0,0,target_state.pose[3]])))

    def initLocalTarget(self):
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 1, 0, 0.5])
        obj_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=[0, 0, 0])
        self.local_target_id = obj_id

    def drawLocalTarget(self, target_state):
        """ Renders the local state in the simulation

        Parameters
        ----------
        DroneState target:
            The target state to be displayed
        """
        p.resetBasePositionAndOrientation(self.local_target_id, 
                                        target_state.pose[:3], 
                                        p.getQuaternionFromEuler(np.array([0,0,target_state.pose[3]])))

    def getNearbyObstacles(self, position, threshold, max_obstacles):
        """ Gets all obsacles within a certain distance of a target position. 
        
        Parameters
        ----------
        np.array(3) position :
            The position to check
        float threshold :
            The maximum distance to consider obstacles in.

        Returns
        -------
        Obstacle[] :
            All obstacles within the threshold distance
        """
        indices = np.where(self.obstacle_distances < threshold)[0]
        if len(indices) > max_obstacles:
            indices = indices[:max_obstacles]
        result = [self.obstacles[i] for i in indices]
        return result


    def sortObstacles(self, position):
        """ Sort the internal list of obstacles by distance

        Parameters
        ----------
        float[3] position :
            The position to base the sorting around
        """

        if len(self.obstacles) == 0:
            return

        # Calculate the current position of all obstacles
        obstacle_positions = []
        for obstacle in self.obstacles:
            obstacle_positions.append(obstacle.position + obstacle.trajectory(self.time))

            # Our cylinders are infinite, and therefore have no height
            if isinstance(obstacle.shape, Cylinder):
                obstacle_positions[-1][2] = 0
        obstacle_positions = np.stack(obstacle_positions)

        # Calculate the distance to all obstacles
        position = np.array(position)
        obstacle_distances = np.linalg.norm(obstacle_positions - position, axis=1)

        # Use argsort to get the indices of the obstacles, in order of distance
        sorted_indices = np.argsort(obstacle_distances)

        # Replace the list of obstacles with a sorted version
        sorted_obstacles = [self.obstacles[i] for i in sorted_indices]
        self.obstacles = sorted_obstacles

        sorted_distances = np.array([obstacle_distances[i] for i in sorted_indices])
        self.obstacle_distances = sorted_distances


    def getCollisionConstraints(self, position, initial_position, padding_amount, binary_vars, timestep_index, threshold, num_obstacles):
        """ Gets the collision constraints for the shape given some cvxpy expressions

        Parameters
        ----------
        pyomo.Expression[3] position :
            The cvxpy variable representing the position to check for a collision, relative to
            the world origin.
        float padding_amount :
            The amount to pad the object by in all directions. Creates a safety buffer.
        pyomo.Var[] binary_vars :
            Binary variables used for mixed-integer constraints where needed. Currently unused,
            since mixed-integer nonconvex programming is incredibly slow, but left in place just in case.
        int timestep_index :
            How many timesteps into the future to consider the position of this object at.
        int num_obstacles :
            How many of the nearest obstacles to consider.

        Returns
        -------
        list of pyomo.Expression : 
            The list of constraints defining collision with this obstacle.
        """

        # Loop through all obstacles and get their collision constraints.
        constraints = []
        # for i, obstacle in enumerate(self.obstacles[:num_obstacles]):
        for i, obstacle in enumerate(self.getNearbyObstacles(initial_position, threshold, num_obstacles)):
            constraints.append(obstacle.getCollisionConstraints(position, padding_amount, binary_vars, timestep_index, i))
        return constraints


    def getInverseDistances(self, position, initial_position, timestep_index, threshold, num_obstacles):
        """ Gets pyomo expressions for the inverse of the distance to each obstacle. 

        Parameters
        ----------
        pyomo.Expression[3] position :
            The position to check for a collision in, relative to the world origin.
        int timestep_index :
            How many timesteps into the future to consider the position of this object at.

        Returns
        -------
        list of pyomo.Expression :
            The list of expressions defining proximity to obstacles in this environment
        """


        inverse_distances = []
        for obstacle in self.getNearbyObstacles(initial_position, threshold, num_obstacles):
            inverse_distances.append(obstacle.getInverseDistance(position, timestep_index))
        return inverse_distances
