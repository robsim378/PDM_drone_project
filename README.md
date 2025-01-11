# RO47005: Planning and Decision Making - Project
This repository contains our group's implementation of the Planning and Decision Making final project. The goal of this project is to implement motion planning for a quadrotor robot in 3D space. Our implementation focuses on local planning with model predictive control (MPC), and does not have any real global path planning. The drone tries to fly in the direction of the target position and uses MPC to avoid local obstacles.
![til](./assets/drone_flying_demo.gif)

## Setup instructions
1. Make sure you have Conda up and running on your system. If not, follow the instructions found [here](https://docs.anaconda.com/miniconda/)
2. In a separate directory from this project, clone [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) and follow the installation instructions there.
3. Clone this repository
4. Run the commmand `cd PDM_drone_project`
5. Run the command `source setup.sh`
6. Run the command `conda install -c conda-forge ipopt`

In the end, your directory structure should look something like this:
```
.
├── demos
│   ├── collision_avoidance_test.py
│   ├── drone_mixer_test.py
│   ├── mpc_test.py
│   ├── test2.py
│   ├── test.py
│   └── warehousetest.py
├── README.md
├── requirements.txt
├── setup.bat
├── setup.sh
└── src
    ├── drone
    │   ├── Drone.py
    │   ├── DroneState.py
    │   ├── DynamicalModel.py
    │   ├── __init__.py
    │   ├── Mixer.py
    ├── environment
    │   ├── Environment.py
    │   ├── __init__.py
    │   ├── Obstacle.py
    │   ├── RectangularPrism.py
    │   ├── Shape.py
    │   └── Sphere.py
    ├── __init__.py
    └── planning
        ├── __init__.py
        └── MPC.py
```

Every time you work on this project in a new shell, you must run the command `source setup.sh` to configure your environment for working on it.

## Usage
To run a demo of the drone flying through a simulated environment with static and dynamic obstacles, run the following command:
```bash
python3 demos/collision_avoidance_test.py
```


## Modules
The structure of this project consists of three main modules that work together to control a drone. 

### Drone
This module contains a representation of the drone. This consists of four classes, two of which are currently unused:

#### Class Drone
Each instance of this class corresponds to a single drone in the simulator. It contains data on the drone model, such as its RPM limits and geometry, as well as a reference to the environment in which the drone lives. It also contains a function for getting the current state of the drone from the simulator.

#### Class DroneState
This class stores the state of a Drone. This is mostly just for convenience, allowing us to write drone.state.pose instead of drone.state[:3], and makes it more robust to changes in the structure of the state vector during development.

#### Class DynamicalModel
This class bundles together the A and B state-space matrices for convenience, but we're currently not using the dynamical model so it is unused.

#### Class Mixer
This class is the mixer, which converts from torques/thrusts to motor RPMs for sending to the simulator. We are not currently computing torques and thrusts, so this is unused.


### Environment
This module contains a representation of the environment in which the drone operates. It handles interfacing with the simulator, 

#### Class Environment
This class handles retrieving information from the simulation and advancing it. It also keeps track of all drones in the environment (currently we only use one), as well as adding obstacles to the environent and getting collision constraints for all obstacles in the environment.

#### Class Obstacle
This class encompasses all obstacles that can be collided with in the environment. It stores their position, geometry data, and handles checking for collisions with a single obstacle.

#### Class Shape
This abstract class contains the interface that all 3D geometric objects must follow. Subclasses must implement collision constraint calculation based on their specific geometry.

#### Classes Sphere, RectangularPrism
These classes implement the Shape abstract class, and handle calculating collision constraints based on their specific geometry.


### Planning
This module handles motion planning. The system uses MPC for local planning, and has no implementation of a global planner, meaning MPC always tries to go in a straight line towards the target.

#### Class MPC
This class contains our implementation of a model predictive controller. Once initialized, it takes a starting DroneState and a target DroneState and calculates control inputs to move the drone to the desired state. In its current implementation, the control inputs calculated are just a series of points moving towards the target destination, since we were not able to get full drone dynamics working. These points are then fed into a PID controller provided by the environment to move the drone to them.

