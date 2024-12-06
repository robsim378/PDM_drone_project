import numpy as np

class DroneState():
    """ Class representing the current state of a drone. """

    def __init__(self, pose, velocity, RPMs):
        """ Initialize a DroneState.

        Parameters
        ----------
        ndarray(6,) pose :
            The current position and orientation of the drone, 
            in the format [x, y, z, r, p, y]
        ndarray(6,) velocity :
            The current velocity, both translational and rotational, of the drone,
            in the format [v_x, v_y, v_z, v_rot_x, v_rot_y, v_rot_z]
        ndarray(4,) RPMs :
            The current RPM of each motor, going clockwise from the rear right motor.
        """

        # NOTE: Exact format of everything is not finalized yet
        self.pose = pose
        self.velocity = velocity
        self.RPMs = RPMs
