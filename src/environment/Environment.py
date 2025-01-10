import numpy as np
import cvxpy
import pybullet as p

from src.drone.DroneState import DroneState
from src.environment.Obstacle import Obstacle
from src.environment.Sphere import Sphere

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

        self.drones = {}


        # TODO: Figure out what's going on here

        # if self.pyb_client_id == -1:
        #     raise RuntimeError("Tried to initialize an Environment, but there was no running PyBullet instance.")

        self.obstacles = []
        #Array of objects (obst + drone etc) in the environment represented by their pybullet id's
        self.objects = []
        # self.drone_id = -1
        # self.env_id = p_id

        self.ghost_tail = []
        self.target_id = -1

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
        """ Advance the simulation, executing the queued actions for all drones. """

        # Initialize empty actions for all drones
        actions = np.zeros((len(self.drones), 4))

        # Get queued actions for all Drones and reset them
        for id in self.drones.keys():
            actions[id, :] = self.drones[id].action
            self.drones[id].action = None

        obs, reward, terminated, truncated, info = self.env.step(actions)
        return obs


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
        # NOTE: Maybe just take a Shape object and include the URDF file as part of that? Would probably be easier than determining shape from URDF

        # TODO: Figure out if position and rotation have to be lists or if ndarrays are acceptable as well
        obj_id = p.loadURDF(urdf, position, rotation, globalScaling=1 ,physicsClientId=self.pyb_client_id)

        #pybullet predetermined shapes
        # obj_id = p.createCollisionShape(p.GEOM_SPHERE, physicsClientId=self.pyb_client_id)

        # Add id to objects
        self.objects += [obj_id]

        #NOTE: this part doesnt seem to be doing anything, but it doesn't break anything either
        # TODO: Add to the internal list of Obstacles
        obstacle = Obstacle(position, obj_id, shape=Sphere)
        # print(f"obstacle {obj_id} added at {position}")
        self.obstacles += [obstacle]

    def initializeWarehouse(self):
        """ Place obstacles to create a warehouse environment for the demo. """

        # TODO: Once addObstacle is implemented, write this function to add all the necessary obstacles for the warehouse environment.
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
        """ Render the MPC tail as a series of ghost objects. 

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
            
    def initTarget(self):
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 0.5])
        obj_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=[0, 0, 0])
        self.target_id = obj_id

    def drawTarget(self, target_state):
        """ Renders the target state in the simulation

        Parameters
        ----------
        DroneState target:
            The target state to be displayed
        """
        p.resetBasePositionAndOrientation(self.target_id, 
                                        target_state.pose[:3], 
                                        p.getQuaternionFromEuler(np.array([0,0,target_state.pose[3]])))


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
