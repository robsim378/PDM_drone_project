"""
This is a test script to test the Mixer class
"""

import os
import sys
import time
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# To run on Windows it needs this
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
sys.path.insert(0, parent_dir)

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

from src.drone.Mixer import Mixer
from src.drone.Drone import Drone
from src.drone.DroneState import DroneState
from src.planning.CartesianGraph import CartesianGraph
from src.planning.Node import Node, Connection

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 12
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

def run(
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VISION,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB
        ):
    #### Initialize the simulation #############################
    H = .1
    H_STEP = .05
    R = .3

    # Drone starting position
    INIT_XYZS = np.array([[0, 0, 1] for i in range(num_drones)])
    INIT_RPYS = np.array([[0, 0, 0] for i in range(num_drones)])

    #### Create the environment ################################
    env = CtrlAviary(drone_model=drone,
                        num_drones=num_drones,
                        initial_xyzs=INIT_XYZS,
                        initial_rpys=INIT_RPYS,
                        physics=physics,
                        neighbourhood_radius=10,
                        pyb_freq=simulation_freq_hz,
                        ctrl_freq=control_freq_hz,
                        gui=gui,
                        record=record_video,
                        obstacles=obstacles,
                        user_debug_gui=user_debug_gui
                        )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()
    p.connect(p.SHARED_MEMORY)

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    # Test adding obstacles
    p.loadURDF("sphere2.urdf",
                   [0, 2, .5],
                   p.getQuaternionFromEuler([0, 0, 0]),
                   physicsClientId=PYB_CLIENT
                   )

    # Initialize the Mixer
    mixer_matrix = np.array([
        [-.5, -.5, -1],
        [-.5, .5, 1],
        [.5, .5, -1],
        [.5, -.5, 1],
    ])
    mixer = Mixer(mixer_matrix, 3.16e-10, 0.2685, 4070.3, 20000, 65535)

    # Initialize the Drone
    drone = Drone(mixer, env)


    #### Run the simulation ####################################
    def test_drone_approaches_target(drone, target_pose):
        """
        Test if the drone approaches the target pose over time.

        Parameters
        ----------
        drone : Drone
            The drone object to be tested.
        target_pose : ndarray
            The target position and orientation of the drone [x, y, z, yaw].
        """
        target_state = DroneState(target_pose, np.zeros(4), None)
        x_positions = []
        y_positions = []
        z_positions = []

        k_p_pos = 1.0  # Proportional gain for position control
        k_d_pos = 0.1  # Derivative gain for velocity damping
        k_i_pos = 0.01  # Integral gain for eliminating steady-state error

        k_p_att = 1.0  # Proportional gain for attitude control (roll, pitch, yaw)
        k_d_att = 0.1  # Derivative gain for attitude damping
        k_i_att = 0.01  # Integral gain for attitude control

        # Initialize integral and derivative terms for position and attitude
        integral_pos = np.zeros(4)
        max_iterations = 10000
        iterations = 0

        current_state = drone.getState()
        while np.any(np.abs(target_pose-current_state.pose) > 0.1) and iterations < max_iterations:
            iterations += 1
            current_state = drone.getState()
            pose_error, velocity_error = current_state.computeDistance(target_state)
            integral_pos[:3] += pose_error[:3] * env.CTRL_TIMESTEP

            thrust = drone.mass * 9.81 + k_p_pos * pose_error[2] + k_d_pos * velocity_error[2] + k_i_pos * integral_pos[2]
            torques = -k_p_att * pose_error[:3] - k_d_att * velocity_error[:3] - k_i_att * integral_pos[:3]
            control_input = np.hstack((thrust, torques))
            drone.updateState(control_input)

            x_positions.append(current_state.pose[0])
            y_positions.append(current_state.pose[1])
            z_positions.append(current_state.pose[2])

        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')
        # Plot the drone's path
        ax.plot(x_positions, y_positions, z_positions, label='Drone Path')

        # Plot the target position
        target_position = target_pose[:3]  # Target position (x, y, z)
        ax.scatter(target_position[0], target_position[1], target_position[2], color='r', s=100, label='Target Position')

        # Label the axes
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title('Drone 3D Path Towards Target')

        # Show the legend
        ax.legend()

        plt.show()

    target_pose = np.array([10.0, 100.0, 5.0, 0.0])
    test_drone_approaches_target(drone, target_pose)
   
    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()
    # logger.save_as_csv("pid") # Optional CSV save

    #### Plot the simulation results ###########################
    if plot:
        logger.plot()

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=DEFAULT_DRONES,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones',         default=DEFAULT_NUM_DRONES,          type=int,           help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics',            default=DEFAULT_PHYSICS,      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VISION,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=DEFAULT_USER_DEBUG_GUI,      type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--obstacles',          default=DEFAULT_OBSTACLES,       type=str2bool,      help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))
