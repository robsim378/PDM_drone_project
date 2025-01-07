import numpy as np
import cvxpy as cp

from src.drone.DynamicalModel import DynamicalModel

class MPC():
    """ Class representing a Model Predictive Control controller used to control a Drone. """

    def __init__(self, drone, model, environment, dt, horizon=10):
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
        self.drone = drone
        self.model = model
        self.environment = environment
        self.dt = dt
        self.horizon = horizon

        # Initialize the x and u variables
        self.x = cp.Variable((self.model.A.shape[1], self.horizon + 1)) # cp.Variable((dim_1, dim_2))
        self.u = cp.Variable((self.model.B.shape[1], self.horizon))

        # Weights for the input, position, and velocity.
        # self.weight_input = 0.01 * np.eye(4) # Weight on the input
        self.weight_position = 1.0*np.eye(4) # Weight on the position
        # self.weight_velocity = 0.01 * np.eye(4) # Weight on the velocity


    def getConstraints(self, initial_state):
        """ Get the constraints for MPC given the initial state.

        Parameters
        ----------
        DroneState initial_state :
            The current state of the drone.

        Returns
        -------
        list of cvxpy constraints : 
            The constraints for the system.
        """

        constraints = []

        # This should probably go in DynamicalModel
        g_term = np.zeros(self.model.B.shape[0])
        g_term[6] = -9.8 * self.dt

        # This is a placeholder value
        max_thrust_rate = 1

        for k in range(self.horizon):
            constraints += [self.x[2, k] >= 0] # Min z position
            constraints += [self.x[6, k] >= -100] # Min z velocity

            constraints += [self.x[2, k] <= 5] # Max z position
            constraints += [self.x[6, k] <= 100] # Max z velocity

            if k < self.horizon:
                constraints += [cp.abs(self.x[:3, k+1] - self.x[:3, k]) <= 0.3] # Max movement per timestep

                # TODO: make this next line actually correct (this is not a robust rotational error calculation)
                constraints += [cp.abs(self.x[3, k+1] - self.x[3, k]) <= 0.1] # Max rotation per timestep
                
                constraints += [self.x[:4, k+1] == self.u[:, k]] # System "dynamics" (magical moving point mass)


            # constraints += [self.u[0, k] >= 0.01]    # Min thrust
            # constraints += [self.u[0, k] <= 1]    # Max thrust
            #
            # constraints += [self.u[1:4, k] >= np.array([-1] * 3)]  # Min torques
            # constraints += [self.u[1:4, k] <= np.array([1] * 3)]   # Max torques
                
            # if k < self.horizon - 1:
            #     constraints += [cp.abs(self.u[0, k+1] - self.u[0, k]) <= max_thrust_rate]

            # dynamics
            # constraints += [self.x[:, k+1] == (
            #     self.model.A @ self.x[:, k] + 
            #     self.model.B @ self.u[:, k] + 
            #     g_term
            # )] 

        constraints += [self.x[:, 0] == initial_state]    # Initial position
        

        return constraints


    def getCost(self, target_state):
        """ Get the cost function for MPC given the target state.

        Parameters
        ----------
        DroneState target_state :
            The target state of the drone.

        Returns
        -------
        cvxpy cost function : 
            The cost function.
        """

        cost = 0.

        for k in range(self.horizon):
            cost += cp.quad_form(self.x[0:4, k] - target_state[0:4], self.weight_position)
            # cost += cp.quad_form(self.x[4:8, k] - target_state[4:8], self.weight_velocity)
            # cost += cp.quad_form(self.u[:, k], self.weight_input)

        return cost


    def getOutput(self, initial_state, target_state):
        """ Get the output of MPC for the given current and target states.

        Parameters
        ----------
        DroneState initial_state :
            The current state of the drone.
        DroneState target_state :
            The target state of the drone.

        Returns
        -------
        ndarray(4,) :
            The control input to the system for this timestep. 
            Format: [thrust, torque_roll, torque_pitch, torque_yaw]
        ndarray(8,) :
            The predicted next state of the system
        ndarray(horizon, 8) :
            The MPC tail
        """

        # Convert the DroneStates into numpy arrays
        x_init = np.hstack((initial_state.pose, initial_state.velocity))
        x_target = np.hstack((target_state.pose, target_state.velocity))

        # Initialize the cost and constraints
        constraints = self.getConstraints(x_init)
        cost = self.getCost(x_target)

        # Solve the optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP, verbose=False)

        # Logging
        print(f"Position cost: {cp.quad_form(self.x[0:4, 1] - x_target[0:4], self.weight_position).value}")
        # print(f"Velocity cost: {cp.quad_form(self.x[4:8, 1] - x_target[4:8], self.weight_velocity).value}")
        # print(f"Input cost: {cp.quad_form(self.u[:, 1], self.weight_input).value}")

        # Return the next input, predicted next state, and tail
        return self.u[:, 0].value, self.x[:, 1].value, self.x.value

