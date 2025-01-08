import numpy as np

from src.drone.DynamicalModel import DynamicalModel
from src.drone.DroneState import DroneState
from src.drone.Mixer import Mixer


class Drone():
    """ Class representing a quadrotor drone. """

    def __init__(self, mixer, environment, drone_id=0):
        """ Initialize a Drone.

        Parameters
        ----------
        DynamicalModel model :
            The dynamical model of the drone.
        Mixer mixer :
            The mixer used to convert from thrusts and torques into motor RPMs
        DroneState initialState :
            The initial state of the drone.
        Environment environment :
            The environment the drone is in.
        int drone_id :
            The ID of the drone in the environment. 
        """
        self.mixer = mixer
        self.environment = environment
        self.id = drone_id
        self.state = self.getState()
        

        # Drone physical properties. These are all placeholders, and should ideally be extracted from the URDF file.
        # On that note, the URDF file should probably be part of the constructor, to make it easy to switch drones if we want.
        self.mass = 0.027
        self.min_RPM = 0
        self.max_RPM = 20000
        self.min_thrust = 0.1 # possibly derived from min_RPM
        self.max_thrust = 1 # possibly derived from max_RPM
        self.min_tilt_torque = -1 # Min torque for roll and pitch
        self.max_tilt_torque = 1 # Max torque for roll and pitch
        self.min_yaw_torque = -1 # Min torque for yaw
        self.max_yaw_torque = 1 # Max torque for yaw
        self.max_RPM_change_rate = 20 # TODO: Not sure if this should be part of the dynamical model or not, that needs to be figured out still.
        self.I_x = 1.4e-5
        self.I_y = 1.4e-5
        self.I_z = 2.17e-5

        # Initialize the dynamical model
        A_c = np.eye(8, k=4)
        A = np.eye(8) + A_c * environment.dt
        
        B_c = np.zeros((8,4))
        # B_c[6,0] = 1 / self.mass
        B_c[4,0] = environment.dt
        B_c[5,1] = environment.dt
        B_c[6,2] = environment.dt
        B_c[7,3] = environment.dt
        B = B_c * environment.dt

        self.model = DynamicalModel(A, B)

        # RPMs of all motors, clockwise from rear right.
        self.action = None

    def getState(self):
        """ Get the current state of the drone. 

        Returns
        -------
        DroneState :
            The current state of the drone
        """

        # pose = self.environment._getDroneStateVector(self.id)[[0, 1, 2, 9]]
        # velocity = self.environment._getDroneStateVector(self.id)[[10, 11, 12, 15]]
        # self.state = DroneState(pose, velocity, None)
        return self.environment.getDroneState(self.id)

        # Uncomment this line to treat action as [thrust, torque_roll, torque_pitch, torque_yaw]
        # action = self.mixer.input_to_RPM(input[0], input[1:4])
        # self.environment.env.step(action.reshape((1, 4)))

