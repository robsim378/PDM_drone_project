import numpy as np
import pyomo.environ as pyo

from src.drone.DynamicalModel import DynamicalModel

class MPC():
    """ Class representing a Model Predictive Control controller used to control a Drone. """

    def __init__(self, drone, dynamical_model, environment, dt, horizon=10, num_obstacles=3):
        """ Initialize an MPC controller. 

        Parameters
        ----------
        DynamicalModel dynamical_model :
            The state-space model to use for this controller.
        Environment environment :
            The environment to use for this controller.
        float dt :
            The timestep duration.
        int horizon :
            The number of timesteps into the future to predict.
        int num_obstacles :
            The maximum number of obstacles to consider for avoidance. Will take the nearest ones.
        """
        self.drone = drone
        self.dynamical_model = dynamical_model
        self.environment = environment
        self.dt = dt
        self.horizon = horizon
        self.num_obstacles = num_obstacles

        # Initialize the optimization variables
        self.model = pyo.ConcreteModel()

        # Define dimensions
        self.dim_x = self.dynamical_model.A.shape[1]  # State dimension
        self.dim_u = self.dynamical_model.B.shape[1]  # Control input dimension

        # Create variables in Pyomo
        self.model.x = pyo.Var(range(self.dim_x), range(self.horizon + 1))  # (dim_x, horizon + 1)
        self.model.u = pyo.Var(range(self.dim_u), range(self.horizon))     # (dim_u, horizon)
        self.model.binary_vars = pyo.Var(range(3), (range(self.horizon + 1)), range(self.num_obstacles), domain=pyo.Binary)

        # Initialize saved values used for warm-starting optimization variables
        self.saved_values = None
        self.initialize_saved_vars()

        # Weights for the input, position, and velocity.
        self.weight_input = 0.01 * np.eye(4) # Weight on the input
        self.weight_position = 1.0*np.eye(4) # Weight on the position
        self.weight_velocity = 0.01 * np.eye(4) # Weight on the velocity
        self.weight_obstacle_proximity =0.5
        
    def initialize_saved_vars(self):
        """ Initialize values of optimization variables for warm-starting. """
        self.saved_values = {}
        
        # Initialize for each variable (x, u, binary_vars)
        for var_name in ['x', 'u', 'binary_vars']:
            var = getattr(self.model, var_name)
            
            if isinstance(var, pyo.Var):
                self.saved_values[var_name] = {}
                for idx in var:
                    self.saved_values[var_name][idx] = 0  # Initialize to zero
    
    def save_pyomo_vars(self):
        """ Save current values of optimization variables for warm-starting. """
        self.saved_values = {}
        for var_name in ['x', 'u', 'binary_vars']:
            var = getattr(self.model, var_name)
            self.saved_values[var_name] = {
                idx: pyo.value(var[idx]) for idx in var if var[idx].value is not None
            }
        return self.saved_values

    def load_pyomo_vars(self):
        """ Load previous values of optimization variables for warm-starting. """
        for var_name, values in self.saved_values.items():
            var = getattr(self.model, var_name)
            for idx, value in values.items():
                var[idx].value = value



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

        self.model.constraints = pyo.ConstraintList()

        # This should probably go in DynamicalModel
        g_term = np.zeros(self.dynamical_model.B.shape[0])
        g_term[6] = -9.8 * self.dt

        # This is a placeholder value
        max_thrust_rate = 1

        for k in range(self.horizon):

            # Min height
            self.model.constraints.add(self.model.x[2, k] >= 0.2)

            if k < self.horizon:
                for i in range(3):  # For each position coordinate (x, y, z)
                    # Max movement per timestep
                    self.model.constraints.add(self.model.x[i, k + 1] - self.model.x[i, k] <= 0.2)
                    self.model.constraints.add(self.model.x[i, k + 1] - self.model.x[i, k] >= -0.2)

                # Max rotation per timestep
                self.model.constraints.add(self.model.x[3, k + 1] - self.model.x[3, k] <= 0.1)
                self.model.constraints.add(self.model.x[3, k + 1] - self.model.x[3, k] >= -0.1)

                # System dynamics (magical moving point mass)
                for i in range(4):  # Assume the first 4 states are directly controlled
                    self.model.constraints.add(self.model.x[i, k + 1] == self.model.u[i, k])


            # Collision constraints. First timestep is ignored, since small amounts of noise when
            # near an obstacle can push the drone within the safety bound and make the problem
            # infeasible.
            if (k > 0):
                # collision_constraints = self.environment.getCollisionConstraints(self.model.x[:, k], 0.1, self.model.binary_vars, k, self.num_obstacles)
                collision_constraints = self.environment.getCollisionConstraints(self.model.x[:, k], initial_state[:3], 0.1, self.model.binary_vars, k, self.num_obstacles)
                
                for obstacle_constraints in collision_constraints:
                    for constraint in obstacle_constraints:
                        self.model.constraints.add(constraint)


            # dynamics (outdated)
            # constraints += [self.x[:, k+1] == (
            #     self.dynamical_model.A @ self.x[:, k] + 
            #     self.dynamical_model.B @ self.u[:, k] + 
            #     g_term
            # )] 

        self.model.initial_state = pyo.Constraint(range(self.dim_x), rule=lambda model, i: model.x[i, 0] == initial_state[i])       


    def getCost(self, initial_state, target_state):
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

        def objective_rule(model):
            cost = 0

            for k in range(self.horizon):
                # Position cost
                cost += sum(
                    self.weight_position[i, j] *
                    (model.x[i, k] - target_state[i]) *
                    (model.x[j, k] - target_state[j])
                    for i in range(4) for j in range(4)
                )

                # Velocity cost
                cost += sum(
                    self.weight_velocity[i, j] *
                    (model.x[i + 4, k] - target_state[i + 4]) *
                    (model.x[j + 4, k] - target_state[j + 4])
                    for i in range(4) for j in range(4)
                )

                # Uncomment below for input cost (if needed)
                # cost += sum(
                #     model.weight_input[i, j] *
                #     model.u[i, k] *
                #     model.u[j, k]
                #     for i in range(dim_u) for j in range(dim_u)
                # )

                # Proximity to obstacles cost
                inverse_distances = self.environment.getInverseDistances(self.model.x[:, k], initial_state[:3], k, self.num_obstacles)
                cost += self.weight_obstacle_proximity * sum(inverse_distances)

            return cost

        # Add objective to model
        self.model.objective = pyo.Objective(rule=objective_rule, sense=pyo.minimize)



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

        # Before reassigning 'constraints'
        if hasattr(self.model, 'constraints'):
            self.model.del_component(self.model.constraints)

        # Before reassigning 'initial_state'
        if hasattr(self.model, 'initial_state'):
            self.model.del_component(self.model.initial_state)

        # Before reassigning 'objective'
        if hasattr(self.model, 'objective'):
            self.model.del_component(self.model.objective)
        
        # Load warm-start values
        self.load_pyomo_vars()
    
        # Convert the DroneStates into numpy arrays
        x_init = np.hstack((initial_state.pose, initial_state.velocity))
        x_target = np.hstack((target_state.pose, target_state.velocity))

        # Initialize the cost and constraints
        self.getConstraints(x_init)
        self.getCost(x_init, x_target)

        # Solve the optimization problem
        solver = pyo.SolverFactory('ipopt')  # Use a suitable solver, e.g., IPOPT
        # solver = pyo.SolverFactory('bonmin')
        result = solver.solve(self.model, tee=False)  # Set tee=True for verbose output

        # Check solver status
        if result.solver.termination_condition == pyo.TerminationCondition.optimal:
            print("Solver found an optimal solution.")
        else:
            print(f"Solver terminated with condition: {result.solver.termination_condition}")

        # Save values for warm-starting
        self.save_pyomo_vars()

        # Return the next input, predicted next state, and tail
        # Extract the control input at time step 0
        u_0 = np.array([self.model.u[i, 0].value for i in range(self.dim_u)])

        # Extract the state at time step 1
        x_1 = np.array([self.model.x[i, 1].value for i in range(self.dim_x)])

        # Extract the entire state trajectory
        x_all = np.array([[self.model.x[i, k].value for k in range(self.horizon + 1)] for i in range(self.dim_x)])

        # Return the extracted values
        return u_0, x_1, x_all        

