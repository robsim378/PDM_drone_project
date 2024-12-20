import numpy as np
import cvxpy as cp

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
        self.horizon = horizon

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

    def getDemoConstraints(self):
        """ This is just used for the first prototype, and will be removed soon. """
        # Hovering dynamics
        mass = 0.027
        g = 9.8

        pass


    def getDemoOutput(self, initial_state, target_state, prev_input):
        """ This is just used for the first prototype, and will be removed soon. 
        
        State in this demo is [height, velocity_upwards]
        Input is [thrust upwards]
        """

        weight_input = 1.0*np.eye(1)    # Weight on the input
        weight_position = 1.0*np.eye(1) # Weight on the position
        weight_velocity = 0.1 * np.eye(1) # Weight on the velocity

        max_thrust_rate = 1

        cost = 0.
        constraints = []

        # State-space model for 1D example
        A_c = np.array([
            [0, 1],
            [0, 0]
        ])
        A = np.eye(2) + A_c * self.dt
        
        B_c = np.array([
            [0],
            [1 / 0.027]
        ])
        B = B_c * self.dt

        # Gravity term for dynamics
        g_term = np.array([0., -9.8 * self.dt])

        # Create the optimization variables
        x = cp.Variable((2, self.horizon + 1)) # cp.Variable((dim_1, dim_2))
        u = cp.Variable((1, self.horizon))

        
        # Initial guesses for x and u, just the current state and no input
        x.value = np.tile(initial_state, (self.horizon + 1, 1)).T
        u.value = np.tile(prev_input, (self.horizon, 1)).T
        # u.value = np.zeros((self.horizon, 1)).T

        # HINTS: 
        # -----------------------------
        # - To add a constraint use
        #   constraints += [<constraint>] 
        #   i.e., to specify x <= 0, we would use 'constraints += [x <= 0]'
        # - To add to the cost, you can simply use
        #   'cost += <value>'
        # - Use @ to multiply matrices and vectors (i.e., 'A@x' if A is a matrix and x is a vector)
        # - A useful function to know is cp.quad_form(x, M) which implements x^T M x (do not use it for scalar x!)
        # - Use x[:, k] to retrieve x_k

        # For each stage in k = 0, ..., N-1
        print(f"Position cost: {weight_position * np.square(x.value[0, 0] - target_state[0])}")
        print(f"Velocity cost: {weight_velocity * np.square(x.value[1, 0] - target_state[1])}")
        print(f"Input cost: {weight_input * np.square(u.value[0, 0])}")
        for k in range(self.horizon):

            cost += weight_position * cp.square(x[0, k] - target_state[0])    # Position penalty
            cost += weight_velocity * cp.square(x[1, k] - target_state[1])    # Velocity penalty
            cost += weight_input * cp.square(u[0, k])    # Input penalty
            # cost += cp.quad_form(x[:, k] - target_state, weight_tracking)
            # cost += cp.quad_form(u[:, k], weight_input)

            constraints += [x[0, k] >= 0] # Min position
            constraints += [x[1, k] >= -100] # Min velocity

            constraints += [x[0, k] <= 5] # Max position
            constraints += [x[1, k] <= 100] # Max velocity

            constraints += [u[:, k] >= 0.01]    # Min thrust
            constraints += [u[:, k] <= 1]    # Max thrust
                
            if k < self.horizon - 1:
                constraints += [cp.abs(u[:, k+1] - u[:, k]) <= max_thrust_rate]

            constraints += [x[:, k+1] == (A @ x[:, k] + B @ u[:, k] + g_term)] # Dynamics



        # EXERCISE: Implement the cost components and/or constraints that need to be added once, here
        # YOUR CODE HERE
        constraints += [x[:, 0] == initial_state]    # initial position

        # Solves the problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP, verbose=False)

        # We return the MPC input and the next state
        return u[:, 0].value, x[:, 1].value, x.value, 0











