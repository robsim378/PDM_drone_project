from demos.collision_avoidance_demo import run, parse_arguments

import sys
import numpy as np

runs = 50

target_reached_list = []
solver_failed_list = []
progress_list = []
nearest_obstacle_distances_list = []
path_efficiency_list = []

for i in range(runs):
    # Simulate command-line arguments
    sys.argv = [
        "demos/collision_avoidance_test.py",
        "--gui", "False",
        "--plot", "False",
        "--static_obstacles", "True",
        "--dynamic_obstacles", "True"
    ]

    args = parse_arguments()   

    # Run the script and store the result
    target_reached, solver_failed, progress, nearest_obstacle_distances, path_efficiency = run(**vars(args))

    target_reached_list.append(target_reached)
    solver_failed_list.append(solver_failed)
    progress_list.append(progress)
    nearest_obstacle_distances_list.append(nearest_obstacle_distances)
    path_efficiency_list.append(path_efficiency)

# Calculate some metrics based on the results

# Success rate 
target_reached_list = np.array(target_reached_list)
success_rate = sum(target_reached_list) / len(target_reached_list)
# print(f"Successes: {target_reached_list}")
print(f"Success rate: {success_rate}")

# Average distance to nearest obstacle
average_distances_to_obstacle = [np.mean(arr) for arr in nearest_obstacle_distances_list]
average_distance_to_obstacle = np.mean(average_distances_to_obstacle)
# print(f"Average distance to nearest obstacle in each run: {average_distances_to_obstacle}")
print(f"Average distance to nearest obstacle in all runs: {average_distance_to_obstacle}")


# Average nearest pass to an obstacle
nearest_approaches = [np.min(arr) for arr in nearest_obstacle_distances_list]
average_nearest_approach = np.mean(nearest_approaches)
# print(f"Nearest approaches: {nearest_approaches}")
print(f"Average nearest approach: {average_nearest_approach}")

# Average optimality factor
average_path_efficiency = np.mean(np.array(path_efficiency_list)[target_reached_list])
# print(f"Path efficiencies: {path_efficiency_list}")
print(f"Average path efficiency: {average_path_efficiency}")


