# RO47005: Planning and Decision Making - Project
## Setup instructions
1. Make sure you have Conda up and running on your system. If not, follow the instructions found [here](https://docs.anaconda.com/miniconda/)
2. In a separate directory from this project, clone [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) and follow the installation instructions there.
3. Clone this repository

In the end, your directory structure should look something like this:
```
.
├── gym-pybullet-drones
│   ├── build_project.sh
│   ├── CITATION.cff
│   ├── gym_pybullet_drones
│   ├── LICENSE
│   ├── pypi_description.md
│   ├── pyproject.toml
│   ├── README.md
│   └── tests
├── PDM_drone_project
│   ├── README.md
│   └── src
│       ├── CartesianGraph.py
│       ├── Graph.py
│       ├── __init__.py
│       ├── Node.py
│       ├── __pycache__
│       ├── results
│       └── test.py
```

If successful, you should be able to run the following commands:
```bash
cd src
python3 test.py
```
This should bring up a simulator with a single drone flying between three waypoints.



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
