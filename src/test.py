
"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ python pid.py

Notes
-----
The drones move, at different altitudes, along cicular trajectories 
in the X-Y plane, around point (0, -.3).

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

from Node import Node, Connection, a_star

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = True
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
    # INIT_XYZS = np.array([[R*np.cos((i/6)*2*np.pi+np.pi/2), R*np.sin((i/6)*2*np.pi+np.pi/2)-R, H+i*H_STEP] for i in range(num_drones)])
    INIT_XYZS = np.array([[0, 0, 1 + i] for i in range(num_drones)])
    INIT_RPYS = np.array([[0, 0,  i * (np.pi/2)/num_drones] for i in range(num_drones)])

    """ Initialize a graph with 1 node per 1 unit of space """
    # Nodes per unit distance
    graph_resolution = 1

    # Define graph boundaries
    graph_min_X = -10
    graph_min_Y = -10
    graph_min_Z = 0
    graph_max_X = 10
    graph_max_Y = 10
    graph_max_Z = 20

    # Calculate the dimensions of the graph
    X_nodes = round(abs(graph_min_X) + abs(graph_max_X) / graph_resolution)
    Y_nodes = round(abs(graph_min_Y) + abs(graph_max_Y) / graph_resolution)
    Z_nodes = round(abs(graph_min_Z) + abs(graph_max_Z) / graph_resolution)


    # Initialize graph
    graph = np.empty((X_nodes, Y_nodes, Z_nodes), dtype=object)

    # Initialize nodes
    for x in range(X_nodes):
        for y in range(Y_nodes):
            for z in range(Z_nodes):
                x_coordinate = graph_min_X + x * graph_resolution
                y_coordinate = graph_min_Y + y * graph_resolution
                z_coordinate = graph_min_Z + z * graph_resolution
                position = (x_coordinate, y_coordinate, z_coordinate)
                graph[x][y][z] = Node(data=position)

    # Initialize connections between nodes. This must be done after initializing all
    # nodes since a connection requires all nodes to exist
    # NOTE: This code is disgusting and I should write a Graph class to handle all this in a way cleaner way, but for now here it is
    for x in range(X_nodes):
        for y in range(Y_nodes):
            for z in range(Z_nodes):
                node = graph[x][y][z]

                # loop over the neighbouring nodes, respecting graph boundaries and ignoring the current node
                for local_x in range(max(x-1, 0), min(x+2, X_nodes)):
                    for local_y in range(max(y-1, 0), min(y+2, Y_nodes)):
                        for local_z in range(max(z-1, 0), min(z+2, Z_nodes)):
                            if (local_x == x and local_y == y and local_z == z):
                                # Ignore the current node
                                continue
                            # Get the length of the vector between the two points
                            distance = np.linalg.norm(np.array((local_x - x, local_y - y, local_z - z))) * graph_resolution
                            node.add_connection(graph[local_x][local_y][local_z], distance, bidirectional=True)

    def node_from_coordinates(x, y, z):
        """ Given a set of x, y, z coordinates, extract the node from the graph

        NOTE: Nesting functions like this is legitimately disgusting, and this needs to be seriously cleaned up.
            Seriously, this is horrible. I will write a Graph class to encapuslate all this behavior in a much
            better way at a later date.
            - Robert
        """

        # Check that coordinates are in valid region
        if x > graph_max_X or y > graph_max_Y or z > graph_max_Z \
            or x < graph_min_X or y < graph_min_Y or z < graph_min_Z:
            print("Coordinates outside bounds")
            return

        x_index = round((x - graph_min_X) / graph_resolution)
        y_index = round((y - graph_min_Y) / graph_resolution)
        z_index = round((z - graph_min_Z) / graph_resolution)

        return (graph[x_index][y_index][z_index])


    # NOTE FROM ROB: Each line here adds a node to the path. Make sure it lines up correctly with the end of the previous one.
    path_1 = a_star(graph.flatten().tolist(), node_from_coordinates(0, 0, 1).id, node_from_coordinates(1, 0, 1).id, dijkstra=True)
    path_2 = a_star(graph.flatten().tolist(), node_from_coordinates(1, 0, 1).id, node_from_coordinates(0, 1, 1).id, dijkstra=True)
    path_3 = a_star(graph.flatten().tolist(), node_from_coordinates(0, 1, 1).id, node_from_coordinates(0, 0, 1).id, dijkstra=True)

    # print(connection for connection in node_from_coordinates(0, 0, 1).connections.values())
    print(list(connection.node.data for connection in node_from_coordinates(0, 0, 1).connections.values()))


    test_path = path_1[0] + path_2[0][1:] + path_3[0][1:]
    test_path_length = path_1[1] + path_2[1] + path_3[1]

    # total_path = (path_1[0].append(path_2[0]).append(path_3[0]), path_1[1] + path_2[1] + path_3[1])


    # print(total_path)
    for node in test_path:
        print(node.data)
    print(test_path_length)


    """ TODO: Fix this horrible, horrible code I have written
    - Create a graph class to encapsulate nodes in 3D space, maybe CartesianGraph? Storing their coordinates in the 
        data field of a node feels messy.
        - Probably modify the Node class as well to have a position
    - Improve path generation from list of nodes, there are duplicate values when it switches from one node to another
    - Add ability to calculate A* heuristics for nodes dynamically based on current target node 
        - This should be part of CartesianGraph
    """



    #### Initialize a linear trajectory ######################
    PERIOD = 10
    NUM_WP = control_freq_hz*PERIOD
    # TARGET_POS = np.zeros((NUM_WP,3))
    TARGET_POS = np.empty((0,3))

    num_wp_per_edge = round(NUM_WP / len(test_path) - 1)
    # Generate all control points in the path
    for i in range(len(test_path) - 1):
        start = test_path[i].data
        end = test_path[i+1].data
        x_points = np.linspace(start[0], end[0], num_wp_per_edge)
        y_points = np.linspace(start[1], end[1], num_wp_per_edge)
        z_points = np.linspace(start[2], end[2], num_wp_per_edge)
        points = np.vstack((x_points, y_points, z_points)).T
        TARGET_POS = np.concatenate((TARGET_POS, points), axis=0)



    # for i in range(NUM_WP):
        # Original line, initializes a circular trajectory
        # TARGET_POS[i, :] = R*np.cos((i/NUM_WP)*(2*np.pi)+np.pi/2)+INIT_XYZS[0, 0], R*np.sin((i/NUM_WP)*(2*np.pi)+np.pi/2)-R+INIT_XYZS[0, 1], 0

        # Replaced with linear trajectory
        # TARGET_POS[i, :] = R*(i/NUM_WP)*5+INIT_XYZS[0, 0], R*(i/NUM_WP)*5+INIT_XYZS[0, 1], 0
    wp_counters = np.array([int((i*NUM_WP/6)%NUM_WP) for i in range(num_drones)])

    #### Debug trajectory ######################################
    #### Uncomment alt. target_pos in .computeControlFromState()
    # INIT_XYZS = np.array([[.3 * i, 0, .1] for i in range(num_drones)])
    # INIT_RPYS = np.array([[0, 0,  i * (np.pi/3)/num_drones] for i in range(num_drones)])
    # NUM_WP = control_freq_hz*15
    # TARGET_POS = np.zeros((NUM_WP,3))
    # for i in range(NUM_WP):
    #     if i < NUM_WP/6:
    #         TARGET_POS[i, :] = (i*6)/NUM_WP, 0, 0.5*(i*6)/NUM_WP
    #     elif i < 2 * NUM_WP/6:
    #         TARGET_POS[i, :] = 1 - ((i-NUM_WP/6)*6)/NUM_WP, 0, 0.5 - 0.5*((i-NUM_WP/6)*6)/NUM_WP
    #     elif i < 3 * NUM_WP/6:
    #         TARGET_POS[i, :] = 0, ((i-2*NUM_WP/6)*6)/NUM_WP, 0.5*((i-2*NUM_WP/6)*6)/NUM_WP
    #     elif i < 4 * NUM_WP/6:
    #         TARGET_POS[i, :] = 0, 1 - ((i-3*NUM_WP/6)*6)/NUM_WP, 0.5 - 0.5*((i-3*NUM_WP/6)*6)/NUM_WP
    #     elif i < 5 * NUM_WP/6:
    #         TARGET_POS[i, :] = ((i-4*NUM_WP/6)*6)/NUM_WP, ((i-4*NUM_WP/6)*6)/NUM_WP, 0.5*((i-4*NUM_WP/6)*6)/NUM_WP
    #     elif i < 6 * NUM_WP/6:
    #         TARGET_POS[i, :] = 1 - ((i-5*NUM_WP/6)*6)/NUM_WP, 1 - ((i-5*NUM_WP/6)*6)/NUM_WP, 0.5 - 0.5*((i-5*NUM_WP/6)*6)/NUM_WP
    # wp_counters = np.array([0 for i in range(num_drones)])

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

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]

    #### Run the simulation ####################################
    action = np.zeros((num_drones,4))
    START = time.time()
    for i in range(0, int(duration_sec*env.CTRL_FREQ)):

        #### Make it rain rubber ducks #############################
        # if i/env.SIM_FREQ>5 and i%10==0 and i/env.SIM_FREQ<10: p.loadURDF("duck_vhacd.urdf", [0+random.gauss(0, 0.3),-0.5+random.gauss(0, 0.3),3], p.getQuaternionFromEuler([random.randint(0,360),random.randint(0,360),random.randint(0,360)]), physicsClientId=PYB_CLIENT)

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        for j in range(num_drones):
            action[j, :], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                    state=obs[j],
                                                                    target_pos=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2]]),
                                                                    # target_pos=INIT_XYZS[j, :] + TARGET_POS[wp_counters[j], :],
                                                                    target_rpy=INIT_RPYS[j, :]
                                                                    )

        #### Go to the next way point and loop #####################
        for j in range(num_drones):
            wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WP-1) else 0

        #### Log the simulation ####################################
        for j in range(num_drones):
            logger.log(drone=j,
                       timestamp=i/env.CTRL_FREQ,
                       state=obs[j],
                       control=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2], INIT_RPYS[j, :], np.zeros(6)])
                       # control=np.hstack([INIT_XYZS[j, :]+TARGET_POS[wp_counters[j], :], INIT_RPYS[j, :], np.zeros(6)])
                       )

        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()
    logger.save_as_csv("pid") # Optional CSV save

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
