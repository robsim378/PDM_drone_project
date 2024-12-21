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

## What's left to do (specific)

## What's left to do (higher-level)



















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
