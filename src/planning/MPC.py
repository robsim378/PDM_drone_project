import numpy as np
import cvxpy

from src.drone.DynamicalModel import DynamicalModel

class MPC():
    """ Class representing a Model Predictive Control controller used to control a Drone. """

    def __init__(self, model, environment, dt, horizon=10):
        """ Initialize an MPC controller. 

        Parameters
        ----------
        DynamicalModel model :
            The state-space model to use for this controller.
        Environment environment :
            The environment to use for this controller.
        float dt :
            The timestep duration.
        int horizon :
            The number of timesteps into the future to predict.
        """
        self.model = model
        self.environment = environment
        self.dt = dt
        self.horizion = horizon

        # TODO: Initialize constraints.
        # - RPM constraints from Drone
        # - Position constraints (e.g. room boundaries) from Environment
        #   - These could also just be collision constraints with some walls
        # - Collision constraints from Environment


    def getOutput(self, currentState, targetState):
        """ Get the output of MPC for the given current and target states.

        Parameters
        ----------
        DroneState currentState :
            The current state of the drone.
        DroneState targetState :
            The target state of the drone.

        Returns
        -------
        ndarray(4,) :
            The control input to the system for this timestep. 
            Format: [thrust, torque_roll, torque_pitch, torque_yaw]
        """
        pass
