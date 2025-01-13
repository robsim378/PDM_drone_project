"""
This is a test script to test the collision avoidance implementation
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
DEFAULT_COLAB = False,
DEFAULT_STATIC_OBSTACLES = True,
DEFAULT_DYNAMIC_OBSTACLES = True

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
        colab=DEFAULT_COLAB,
        static_obstacles=DEFAULT_STATIC_OBSTACLES,
        dynamic_obstacles=DEFAULT_DYNAMIC_OBSTACLES,
        ):
    
    print("sys.argv:", sys.argv) 
    #### Initialize the simulation #############################
    H = .1
    H_STEP = .05
    R = .3

    # Drone starting position
    INIT_XYZS = np.array([[0, 0, 2] for i in range(num_drones)])
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
    dt_factor = 3
    environment.dt = environment.dt * dt_factor

    
    # Create static obstacles
    pillar_bounds = np.array([
        [-4, -1],
        [-3, 3]
    ])
    if static_obstacles:
        environment.addStaticObstacles(15, pillar_bounds, seed=None)

    # Create dynamic obstacles
    sphere_bounds = np.array([
        [-4, -1],
        [-3, 3],
        [0, 4]
    ])
    if dynamic_obstacles:
        environment.addDynamicObstacles(40, sphere_bounds, dt_factor, seed=None)



    # Create box obstacle (NOT WORKING)
    # environment.addBox([-1.0, 0, 0.5], 0.5, 1, 1)

    # Initialize the Drone 
    drone = Drone(mixer, environment)
    environment.addDrone(drone)

    # Initialize the controller
    mpc_controller = MPC(drone, drone.model, environment, environment.dt, 
                         horizon=10, 
                         num_obstacles=10,
                         obstacle_threshold=2,
                         obstacle_padding=0.035,
                         weight_position=2.0,
                         weight_obstacle_proximity=0.5,
                         )

    pid_controller = DSLPIDControl(drone_model=DroneModel.CF2X)

    acceleration_gain = 0.013
    global_planner_distance = 2
    success_threshold = 0.1
    target_reached = False
    solver_failed = False

    control_input = np.zeros((1, 4))
    drone.action = control_input

    global_target_z = 2.0
    global_target_x = -6
    global_target_y = 0.0
    global_target_yaw = 0
    global_target_state = DroneState(np.array([global_target_x, global_target_y, global_target_z, global_target_yaw]), np.array([0, 0, 0, 0]), None)

    # Initialize the ghost tail for MPC
    environment.initMPCTail(mpc_controller.horizon+1)
    environment.initLocalTarget()
    environment.initGlobalTarget()
    environment.initTracker()

    previous_position = np.array([0, 0, 2])
    total_distance = 0.0

    straight_line_distance = np.linalg.norm(np.array([0, 0, 2]) - global_target_state.pose[:3])

    progress = np.full(int(duration_sec*env.CTRL_FREQ), -1.0)
    nearest_obstacle_distance = np.full(int(duration_sec*env.CTRL_FREQ), -1.0)

    #### Run the simulation ####################################
    for i in range(0, int(duration_sec*env.CTRL_FREQ)):

        # Loop for multiple drones. We don't have multiple drones, but removing this is a hassle with no real benefit.
        for j in range(num_drones):
            if i > 0:
                previous_position = current_state.pose[:3]

            # Advance the simulation
            obs = environment.advanceSimulation()

            # Get the drone's current state
            current_state = drone.getState()

            total_distance += np.linalg.norm(current_state.pose[:3] - previous_position)

            # Get a point in the direction of the target that isn't too far away (global planner)
            positional_error = global_target_state.pose - current_state.pose
            positional_error_scalar = np.linalg.norm(positional_error)
            corrected_magnitude = min(positional_error_scalar, global_planner_distance)
            local_target = current_state.pose + positional_error/positional_error_scalar * corrected_magnitude
            local_target_state = DroneState(local_target, np.array([0, 0, 0, 0]), None)


            # Move the camera to the drone's new position
            camera_distance = 0.5
            camera_pitch = -30
            camera_yaw = 90
            p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, current_state.pose[:3])

            # Draw the target position as a green sphere
            environment.drawLocalTarget(local_target_state)
            environment.drawGlobalTarget(global_target_state)


            # Gather metrics
            progress[i] = np.linalg.norm(current_state.pose[:3] - global_target_state.pose[:3])
            if len(environment.obstacles) > 0:
                nearest_obstacle_distance[i] = environment.obstacle_distances[0]

            if progress[i] < success_threshold:
                target_reached = True
                break

            # Get the output from MPC
            next_input, next_state, tail, solver_failed = mpc_controller.getOutput(current_state, local_target_state)

            if solver_failed:
                break

            # Create a position for the PID controller to track based on the acceleration direction
            next_waypoint = current_state.pose + acceleration_gain * next_input

            # Draw the PID controller's target as a red sphere
            environment.drawTracker(DroneState(next_waypoint[:4], np.array([0, 0, 0, 0]), None))

            # Draw the tail as a series of blue spheres
            state_tail = []
            for state in tail.T:
                state_tail.append(DroneState(state[:4], np.array([0, 0, 0, 0]), None))
            environment.drawMPCTail(state_tail)

            # Compute inputs to the PID controller based on the output from MPC
            target_pos = next_waypoint[:3]
            target_rpy = INIT_RPYS[j, :]
            target_rpy[2] += next_waypoint[3]

            # Send the control inputs to MPC
            drone.action = pid_controller.computeControlFromState(
                control_timestep=env.CTRL_TIMESTEP,
                state=obs[j],
                target_pos=target_pos,
                target_rpy=target_rpy
            )[0]

        if target_reached or solver_failed:
            break

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
    times = np.arange(len(progress)) * environment.dt

    if plot:
        # Plot the line graph
        plt.plot(times[:i + 1], progress[:i + 1], marker='o', label="Distance to target")
        if len(environment.obstacles) > 0:
            plt.plot(times[:i + 1], nearest_obstacle_distance[:i + 1], marker='o', label="Distance to nearest obstacle")

        # Add labels, title, and legend
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.title('Static and dynamic obstacles')
        plt.legend()

        # Show the graph
        plt.grid()
        plt.show()

    total_distance += np.linalg.norm(global_target_state.pose[:3] - previous_position)

    return target_reached, solver_failed, progress[:i + 1], nearest_obstacle_distance[:i + 1], straight_line_distance / total_distance 


def parse_arguments():
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
    parser.add_argument('--static_obstacles',          default=DEFAULT_STATIC_OBSTACLES,       type=str2bool,      help='Whether to add static obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--dynamic_obstacles',          default=DEFAULT_DYNAMIC_OBSTACLES,       type=str2bool,      help='Whether to add dynamic obstacles to the environment (default: True)', metavar='')
    return parser.parse_args()

if __name__ == "__main__":
    ARGS = parse_arguments()

    run(**vars(ARGS))
