import numpy as np

from src.drone.DynamicalModel import DynamicalModel
from src.drone.DroneState import DroneState
from src.drone.Mixer import Mixer


class Drone():
    """ Class representing a quadrotor drone. """

    def __init__(self, model, mixer, environment, drone_id=0):
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
        self.model = model
        self.mixer = mixer
        self.environment = environment
        self.id = drone_id
        self.state = self.getState()

        # Drone physical properties
        self.min_thrust = 0.1 # Placeholder value
        self.max_thrust = 1 # Placeholder value
        self.max_RPM_change_rate = 20 # TODO: Not sure if this should be part of the dynamical model or not, that needs to be figured out still.

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

    def updateState(self, input):
        """ Update the state of the drone based on input. 

        Parameters
        ----------
        ndarray(4,) :
            The control input to the system for this timestep. 
            Format: [thrust, torque_roll, torque_pitch, torque_yaw]
        """

        action = self.mixer.input_to_RPM(input[0], input[1:4])
        self.environment.env.step(action.reshape((1, 4)))

