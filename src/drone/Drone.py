import numpy as np

from src.drone.DynamicalModel import DynamicalModel
from src.drone.DroneState import DroneState
from src.drone.Mixer import Mixer


class Drone():
    """ Class representing a quadrotor drone. """

    def __init__(self, model, mixer, initialState, environment):
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
        """
        self.model = model
        self.mixer = mixer
        self.state = initialState
        self.environment = environment

        # Drone physical properties
        self.min_thrust = 0 # Placeholder value
        self.max_thrust = 1 # Placeholder value
        self.max_RPM_change_rate = 20 # TODO: Not sure if this should be part of the dynamical model or not, that needs to be figured out still.

    def getState(self):
        """ Get the current state of the drone. 

        Returns
        -------
        DroneState :
            The current state of the drone
        """

        # NOTE: Get the state from self.environment, which will itself get it from pybullet.
        raise NotImplementedError("This hasn't been implemented yet.")

    def updateState(self, input):
        """ Update the state of the drone based on input. 

        Parameters
        ----------
        ndarray(4,) :
            The control input to the system for this timestep. 
            Format: [thrust, torque_roll, torque_pitch, torque_yaw]
        """

        # NOTE: Don't use the dynamical model to caluclate the state, but
        # rather send the input to pybullet and let it handle that.
        raise NotImplementedError("This hasn't been implemented yet.")

