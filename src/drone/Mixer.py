import numpy as np

class Mixer():
    """ Class representing a quadrotor mixer. Adapted from gym-pybullet-drone's DSLPIDControl class."""

    def __init__(self, mixer_matrix, KF, PWM_to_RPM_scale, PWM_to_RPM_const, min_PWM, max_PWM):
        """ Initialize a Mixer.

        Parameters
        ----------
        ndarray mixer_matrix :
            Matrix defining the relation between drone thrust and torques and motor thrusts
        float KF :
            Coefficient for converting RPMs to thrust
        float PWM_to_RPM_scale :
            Used for converting from PWMs to RPMs
        float PWM_to_RPM_const : 
            Used for converting from PWMs to RPMs
        float min_PWM, max_PWM :
            The minimum and maximum PWM values for the drone.
        """
        self.mixer_matrix = mixer_matrix
        self.KF = KF
        self.PWM_to_RPM_scale = PWM_to_RPM_scale
        self.PWM_to_RPM_const = PWM_to_RPM_const
        self.min_PWM = min_PWM
        self.max_PWM = max_PWM

    def input_to_RPM(self, thrust, torques):
        """ Convert from a thrust and RPY torques to RPMs for each motor. 

        Parameters
        ----------
        float thrust :
            The thrust to apply, I believe in Kg
        ndarray(3,) torques :
            the torques to apply along the roll, pitch, and yaw axes.

        Returns
        -------
        ndarray(4,) :
            The RPM of each motor, clockwise from the rear right.
        """

        thrust = (np.sqrt(thrust / (4*self.KF)) - self.PWM_to_RPM_const) / self.PWM_to_RPM_scale
        torques = np.clip(torques, -3200, 3200)
        pwm = thrust + np.dot(self.mixer_matrix, torques)
        pwm = np.clip(pwm, self.min_PWM, self.max_PWM)
        return self.PWM_to_RPM_scale * pwm + self.PWM_to_RPM_const
