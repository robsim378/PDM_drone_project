import numpy as np
import cvxpy
import pybullet as p

from src.drone.DroneState import DroneState
from src.environment.RectangularPrism import RectangularPrism
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
        self.time = 0

        self.drones = {}
        self.obstacles = []
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
        """ Advance the simulation, executing the queued actions for all drones
        and moving dynamic obstacles. """

        # Initialize empty actions for all drones
        actions = np.zeros((len(self.drones), 4))

        # Get queued actions for all Drones and reset them
        for id in self.drones.keys():
            actions[id, :] = self.drones[id].action
            self.drones[id].action = None

        # Step the simulation
        obs, reward, terminated, truncated, info = self.env.step(actions)

        # Move dynamic obstacles
        self.time += self.dt
        for obstacle in self.obstacles:
            p.resetBasePositionAndOrientation(obstacle.pyb_obj_id, obstacle.position + obstacle.trajectory(self.time), [1, 0, 0, 0], physicsClientId=self.pyb_client_id)

        return obs


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


    def initializeWarehouse(self):
        """ Place obstacles to create a warehouse environment for the demo. """

        # TODO: Define the warehouse here
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



    def getCollisionConstraints(self, position, padding_amount, binary_vars, timestep_index):
        """ Gets the collision constraints for the shape given some cvxpy expressions

        Parameters
        ----------
        pyomo.Expression[3] position :
            The cvxpy variable representing the position to check for a collision, relative to
            the center of the obstacle.
        float padding_amount :
            The amount to pad the object by in all directions. Creates a safety buffer.

        Returns
        -------
        list of pyomo.Expression : 
            The list of constraints defining collision with this obstacle.
        """

        # Loop through all obstacles and get their collision constraints.
        constraints = []
        for i, obstacle in enumerate(self.obstacles):
            constraints.append(obstacle.getCollisionConstraints(position, padding_amount, binary_vars, timestep_index, i))
        return constraints

    def getInverseDistances(self, position):
        """ Gets pyomo expressions for the inverse of the distance to each obstacle. 

        pyomo.Expression[3] position :
            The position to check for a collision in, relative to the world origin.
        """
        costs = 0.
        for obstacle in self.obstacles:
            costs += obstacle.getInverseDistance(position)
        return costs
