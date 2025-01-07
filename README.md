# RO47005: Planning and Decision Making - Project
## Setup instructions
1. Make sure you have Conda up and running on your system. If not, follow the instructions found [here](https://docs.anaconda.com/miniconda/)
2. In a separate directory from this project, clone [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) and follow the installation instructions there.
3. Clone this repository

In the end, your directory structure should look something like this:
```
.
├── demos
│   ├── test2.py
│   └── test.py
├── README.md
├── requirements.txt
├── setup.sh
└── src
    ├── drone
    │   ├── Drone.py
    │   ├── DroneState.py
    │   ├── DynamicalModel.py
    │   └── __init__.py
    ├── environment
    │   ├── Environment.py
    │   ├── __init__.py
    │   ├── Obstacle.py
    │   ├── RectangularPrism.py
    │   ├── Shape.py
    │   └── Sphere.py
    ├── __init__.py
    ├── planning
    │   ├── CartesianGraph.py
    │   ├── GlobalPlanner.py
    │   ├── Graph.py
    │   ├── __init__.py
    │   ├── MPC.py
    │   ├── Node.py
```

Every time you work on this project in a new shell, you must run the command `source setup.sh` to configure your environment for working on it.

If successful, you should be able to run the following commands:
```bash
cd 
python3 demos/mpc_test.py
```
This should bring up a simulator with a single drone hovering up and down



# Modules
The structure of this project consists of three main modules that work together to control a drone. 

## Drone
This module contains a representation of the drone. This consists of four classes, two of which are currently unused:

### Class Drone
Each instance of this class corresponds to a single drone in the simulator. It contains data on the drone model, such as its RPM limits and geometry, as well as a reference to the environment in which the drone lives. It also contains a function for getting the current state of the drone from the simulator.

### Class DroneState
This class stores the state of a Drone. This is mostly just for convenience, allowing us to write drone.state.pose instead of drone.state[:3], and makes it more robust to changes in the structure of the state vector during development.

### Class DynamicalModel
This class bundles together the A and B state-space matrices for convenience, but we're currently not using the dynamical model so it is unused.

### Class Mixer
This class is the mixer, which converts from torques/thrusts to motor RPMs for sending to the simulator. We are not currently computing torques and thrusts, so this is unused.


## Environment
This module contains a representation of the environment in which the drone operates. It handles interfacing with the simulator, 

### Class Environment
This class handles retrieving information from the simulation and advancing it. It also keeps track of all drones in the environment (currently we only use one), as well as adding obstacles to the environent and getting collision constraints for all obstacles in the environment.

### Class Obstacle
This class encompasses all obstacles that can be collided with in the environment. It stores their position, geometry data, and handles checking for collisions with a single obstacle.

### Class Shape
This abstract class contains the interface that all 3D geometric objects must follow. Subclasses must implement collision constraint calculation based on their specific geometry.

### Classes Sphere, RectangularPrism
These classes implement the Shape abstract class, and handle calculating collision constraints based on their specific geometry.


## Planning
This module handles motion planning. The system uses MPC for local planning, and currently uses nothing for global planning, as GlobalPlanner is not implemented. The remaining classes in the module are placeholders for A*, but we probably won't end up using them so they'll probably be removed soon

### Class MPC
This class contains our implementation of a model predictive controller. Once initialized, it takes a starting DroneState and a target DroneState and calculates control inputs to move the drone to the desired state. In its current implementation, the control inputs calculated are just a series of points moving towards the target destination, since we were not able to get full drone dynamics working. These points are then fed into a PID controller provided by the environment to move the drone to them.

### Class GlobalPlanner
This class is currently empty, but will contain a global motion planner. Most likely, this will just be a straight line from the current state to the target state, but if we have the time we can use a basic A* or RRT implementation.

### Classes Graph, Node, CartesianGraph
These classes could be used to set up A* if we have time, but it's a low priority so for now they just sit there.





# Report on what I did
## Drone
### Implemented
- Conversion from [thrust, torque_x, torque_y, torque_z] into [rpm_1, rpm_2, rpm_3, rpm_4] is done in the Mixer class
    - The code that does this is pretty much just ripped straight from the CtrlAviary class in gym-pybullet-drones
- DynamicalModel class doesn't really need any more work, there's not much to it.
- DroneState setup is complete, but parts of the implementation are missing (more detail further down)
- Drone class constructor is set up, but is missing most of the drone's physical properties (mass, etc) are hardcoded.
    - Drone constructor also sets up the state-space model, so when that gets implemented fully that's where it'll be.
- Getting the current state from the environment is done
- Updating the state and stepping the pybullet simulation is done
### TODO
- Include the URDF file in the drone
- Extract the drone's physical properties (mass, moments of inertia, etc) from the URDF file in the Drone constructor
- Proper calculation of error/distance between two drone states
- Full implementation of the drone dynamics
    - If this proves too hard (I spent a full day trying and wasn't able to, but I also have no background in physics), then the input vector in MPC should be modified to simplify the problem
        - Specifically, make it output desired velocities in the world frame, and find some way to feed those directly into pybullet. This means you'll have to change Drone.updateState() accordingly.

## Environment
### Implemented
- Constructor is mostly set up, but the pyb_client_id is still -1 despite things working. For now, I have commented out the check for this, but this may be worth investigating if it's ever needed.
- Getting the current drone state from pybullet is done
### TODO
- Add obstacles
- Get collision constraints for obstacles
- Implement all obstacle-related classes

## Planning
### Implemented
- A first prototype of MPC is set up. This uses a simplified version of the system dynamics (i.e. state-space matrices) and only considers altitude.
- Constraints for min/max position, velocity, thrust, and torques are implemented (position/velocity currently only for z-axis)
- Dynamics constraints are implemented
### TODO
- Implement the full system dynamics. The state-space matrices are set up in the constructor of the Drone class.
    - More details in the Drone TODO section
- Add collision constraints

## Main
- The script to run is `mpc_test.py`. This sets up the pybullet environment, and then sets up our system.
- Our system is set up starting from the line with the comment saying "Model Predictive Control setup" (line 101 at the time of writing)
- It initializes instances of all the classes I've implemented at the moment
- There are two nested for loops. You can ignore the loops themselves, those won't change. The code inside is where MPC is run. The target state is generated, MPC is used to get the control output, and the output is given to drone.updateState(). This all happens once every iteration of the loop. drone.updateState handles advancing the simulation.
- There's nothing after the loop that we'll have to worry about.


















# Everything below this point is some notes I took while investigating the environment and setting up the project structure.




# TODO
## Drone
- Implement full state-space model
- Compute difference between two DroneStates
- Extract properties from URDF file (currently hardcoded, mostly placeholder values)
    - min thrust
    - max thrust
    - max RPM change rate
    - mass

## Environment
- Add obstacles to pybullet environment
- Check for collision from Environment
- Check for collision from Obstacle
- Check for collision from Shape
- Display ghost for target position, in red
- Display ghost for MPC tail, in blue

## Planning
- Generate constraints
- Implement solver


# Notes on architecture
- I don't know if DynamicalModel should actually have a nextState function, there may not be any use for it at all
- Instead of an np array for pose, we could use the Frame class from dynamics and control
- Current plan for collision detection works on points, but if we can add collision detection between two shapes that would be even better
- Probably should change Sphere to Ellipsoid, or at least add Ellipsoid as a class.

# Notes on environment
- BaseAviary is the base environment, the PID demo (and our test.py) uses its subclass CtrlAviary.
- The environments contain functions to apply all the forces and torques to each drone, and to advance the simulation to the next timestep.
- CtrlAviary is a subclass that implements a few mandatory functions that are not implemented in BaseAviary, this is what we will use
- We can connect to the same pybullet instance by doing `p.connect(p.SHARED_MEMORY)`, allowing us to directly interface with pybullet (needed to add custom obstacles)

# Notes on BaseAviary
- Handles calculating all kinds of constants and interfacing with pybullet

## Useful Functions
- getDroneIds() gets the IDs of all the drones in the simulation
- \_getDroneStateVector(id) gets the state vector of the specified drone
    - The format is an array of shape (20,), with the following contents:
        - position, quaternion, roll, pitch, yaw, velocity, angular velocity, last_clipped_action (idk what that last one is)
    - Exact format of each of these requires more investigation
        - position: x, y, z values
        - quaternion: 4 values defining the quaternion
        - rpy: roll, pitch, yaw (why are both this and quaternion provided?)
        - velocity: x, y, z values
        - angular velocity: 3 values
        - last_clipped_action: Current theory is RPM for each rotor. This would be 4 values, which lines up with the shape of the array
- \_physics(rpm, drone_ID) sets the RPM of the specified drone's motors
    - rpm is an array of shape (4,) containing the RPM values of each motor (need to figure out which value corresponds to what)
    - \_dynamics(rpm, drone_ID) seems to do the same thing? requires more investigation
- getPyBulletClient() returns the pybullet client ID. This will be useful for interacting directly with the environment, but we should generally prefer to use BaseAviary.
- \_addObstacles() adds a few predefined obstacles. 
    - To add custom ones, we will probably need to interface with pybullet directly.
- \_normalizedActionToRPM(action) takes an array of shape (4,) in the range [-1, 1] and de-normalizes it to the range [0, MAX_RPM].

# Notes on existing control strategy
- Uses PID control to control drone motor RPMs
- Specifically, uses two PID controllers. One to determine thrust and orientation required to achieve it, and the second to determine control input to get to that orientation
- First calculates target thrust based on position error, velocity error (i.e. derivative term), also taking into account effect of gravity
- Then computes target orientation with z-axis pointing in direction of thrust
- returns thrust scalar, target orientation, and position error
- Passes those to the second controller, which uses a PID controller to compute target torques to reach the desired orientations
- Takes the torques and converts them to RPMs using the mixer matrix (idk what that is, probably dynamics stuff)
