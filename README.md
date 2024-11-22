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
