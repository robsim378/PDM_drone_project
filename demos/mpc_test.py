"""
This is a test script to test the core MPC implementation
"""

import os
import time
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

from src.drone.DroneState import DroneState
from src.drone.DynamicalModel import DynamicalModel
from src.drone.Mixer import Mixer
from src.drone.Drone import Drone
from src.environment.Environment import Environment
from src.planning.MPC import MPC
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
    test = p.connect(p.SHARED_MEMORY)

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    # Test adding obstacles
    sphere = p.loadURDF("sphere2.urdf",
                   [0, 2, .5],
                   p.getQuaternionFromEuler([0, 0, 0]),
                   physicsClientId=PYB_CLIENT
                   )

    ##### Model Predictive Control setup ##########################
    # Initialize the simulation and do a single timestep to ensure everything is initialized properly
    START = time.time()
    env.step(np.zeros((1, 4)))

    # This is now stored in drone.env, and is initialized from the same values when the Environment is initialized.
    # dt = 1 / control_freq_hz

    # Initialize the Mixer
    mixer_matrix = np.array([
        [-.5, -.5, -1],
        [-.5, .5, 1],
        [.5, .5, -1],
        [.5, -.5, 1],
    ])
    mixer = Mixer(mixer_matrix, 3.16e-10, 0.2685, 4070.3, 20000, 65535)

    # Initialize the environment
    environment = Environment(env)

    # Initialize the Drone 
    drone = Drone(mixer, environment)
    environment.addDrone(drone)

    # Initialize the controller
    horizon = 10
    mpc_controller = MPC(drone, drone.model, environment, environment.dt, horizon)

    pid_controller = DSLPIDControl(drone_model=DroneModel.CF2X)

    control_input = np.zeros((1, 4))
    drone.action = control_input

    # Initialize the ghost tail for MPC
    environment.initMPCTail(horizon+1)




    p.setGravity(0, 0, 0)

    link_index = -1  # Use -1 for the base object or specify a link index
    sphere_mass = p.getDynamicsInfo(sphere, link_index)[0]  # Get the object's mass




    #### Run the simulation ####################################
    for i in range(0, int(duration_sec*env.CTRL_FREQ)):

        # Loop for multiple drones. We don't have multiple drones, but removing this is a hassle with no real benefit.
        for j in range(num_drones):
            # obs, reward, terminated, truncated, info = env.step(control_input)
            obs = environment.advanceSimulation()


            # Determine the trajectory. For now just hover up and down
            # target_z = 0.5 * np.sin(i/10) + 1
            target_z = 1.5
            # target_x = -1.0
            # target_y = -1.0
            target_x = 0
            target_y = 0
            target_yaw = 0
            target_state = DroneState(np.array([target_x, target_y, target_z, target_yaw]), np.array([0, 0, 0, 0]), None)

            # Get the drone's current state
            current_state = drone.getState()





            pos, orn = p.getBasePositionAndOrientation(sphere)
            linear_vel, angular_vel = p.getBaseVelocity(sphere)
            sphere_state = DroneState(np.append(np.array(pos), p.getEulerFromQuaternion(orn)[2]), np.append(np.array(linear_vel), angular_vel[2]), None)








            # Get the output from MPC. In the current state of our system, this is just a position and yaw.
            # next_input, next_state, tail = mpc_controller.getOutput(current_state, target_state)
            next_input, next_state, tail = mpc_controller.getOutput(sphere_state, target_state)

            state_tail = []
            for state in tail.T:
                state_tail.append(DroneState(state[:4], np.array([0, 0, 0, 0]), None))

            environment.drawMPCTail(state_tail)


            # Calculate force
            acceleration = next_input[:3]
            force = [a * sphere_mass for a in acceleration]

            # Apply the force
            p.applyExternalForce(
                objectUniqueId=sphere,
                linkIndex=link_index,
                forceObj=force,
                posObj=[0, 0, 0],  # Apply at the center of mass
                flags=p.WORLD_FRAME  # Use world or local frame as needed
            )


            # Compute inputs to the PID controller based on the output from MPC
            # target_pos = tail[2, :3]
            # target_rpy = INIT_RPYS[j, :]
            # target_rpy[2] += tail[2, 3]

            # Send the control inputs to MPC
            # drone.action = pid_controller.computeControlFromState(
            #     control_timestep=env.CTRL_TIMESTEP,
            #     state=obs[j],
            #     target_pos=target_pos,
            #     target_rpy=target_rpy
            # )[0]

        # Logging
        # print(f"Predicted next state: {next_state}")
            # print(f"Calculated control input: {control_input}")

            # Update the state in simulation. This also advances the simulation.
            # drone.updateState(control_input)


        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

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
