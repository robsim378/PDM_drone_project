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

        # NOTE: We could use the Frame class from RDC. I (Robert) have a refactored version
        # with cleaner code I made that we could use.
        self.pose = pose
        self.velocity = velocity
        # NOTE: RPMs probably shouldn't be included here, but it needs more consideration before removing them.
        self.RPMs = RPMs

    def computeDistance(self, target):
        """ Computes the difference between two DroneStates. 

        Parameters
        ----------
        DroneState target :
            The state to compute the distance from.

        Returns
        -------
        (ndarray(4,), ndarray(4,))
            Positional and rotational errors for pose, positional and rotational errors for velocity
        """
        #depending on the fps of the camera other rotational velocity errors  might be handy 
        #the orientation error doesnt matter much until you arrive there
        pose_error = self.pose - target.pose
        velocity_error = self.velocity - target.velocity
        return pose_error, velocity_error 
