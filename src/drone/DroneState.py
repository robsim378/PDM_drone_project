import numpy as np

class DroneState():
    """ Class representing the current state of a drone. """

    def __init__(self, pose, velocity, RPMs):
        """ Initialize a DroneState.

        Parameters
        ----------
        ndarray(4,) pose :
            The current position and orientation of the drone, 
            in the format [x, y, z, yaw]
        ndarray(4,) velocity :
            The current velocity, both translational and rotational, of the drone,
            in the format [v_x, v_y, v_z, v_yaw]
        ndarray(4,) RPMs :
            The current RPM of each motor, going clockwise from the rear right motor.
        """

        # NOTE: Exact format of everything is not finalized yet

        # NOTE: We could use the Frame class from RDC. I (Robert) have a refactored version
        # with cleaner code I made that we could use.

        self.pose = pose
        self.velocity = velocity
        self.RPMs = RPMs

    def computeDistance(self, target):
        """ Computes the difference between two DroneStates. 

        Parameters
        ----------
        DroneState target :
            The state to compute the distance from.

        Returns
        -------
        (float, float) :
            Positional and rotational errors
        """
        pass

    def inputToRPMs(self, input):
        """ Converts system input into RPMs for use in PyBullet

        Parameters
        ----------
        ndarray(4,) input :
            The control input to the system for this timestep. 
            Format: [thrust, torque_roll, torque_pitch, torque_yaw]

        Returns
        -------
        ndarray(4,) :
            The RPM of each rotor, going clockwise starting from the rear right rotor.
        """
        pass

