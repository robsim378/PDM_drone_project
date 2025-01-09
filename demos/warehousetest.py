import os
import time
import numpy as np
import pybullet as p
import pybullet_data
import random
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync

# Default settings for simulation
DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_OBSTACLES = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 12
DEFAULT_OUTPUT_FOLDER = 'results'

# Half dimensions of the environment
halfx = 18
halfy = 18
halfz=5
shelf_t=1
wall_t=0.1
length_shelf=5
clearance_ceiling=1
humanoid_torso=0.25
humanoid_belly=0.15
humanoid_length=0.9
forklift_length=0.75
forklift_width=0.6
forklift_height=1.25

def setup_environment(env, num_humans=3):
    mass= 0 
    grey_color = [0.5, 0.5, 0.5, 0.5] #wall,ceiling,shelf color
    forklift_color = [1, 0, 0, 1]  # Red
    human_color = [0, 0, 1, 1]  # Blue
    human_ids = []
    human_location = []  

    PYB_CLIENT = env.getPyBulletClient()
    p.connect(p.SHARED_MEMORY)
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)

    p.loadURDF("plane.urdf", physicsClientId=PYB_CLIENT)
    wall_y = p.createCollisionShape(p.GEOM_BOX, halfExtents = [wall_t, halfy, halfz])
    wall_x = p.createCollisionShape(p.GEOM_BOX, halfExtents = [halfx, wall_t, halfz])
    ceiling = p.createCollisionShape(p.GEOM_BOX, halfExtents = [halfx, halfy, wall_t])
    shelf = p.createCollisionShape(p.GEOM_BOX, halfExtents = [shelf_t, length_shelf, halfz-clearance_ceiling])
    humanoid = p.createCollisionShape(p.GEOM_BOX, halfExtents = [humanoid_torso, humanoid_belly, humanoid_length])
    forklift = p.createCollisionShape(p.GEOM_BOX, halfExtents = [forklift_length, forklift_width, forklift_height])

    wall_ids = [
        p.createMultiBody(mass, wall_y, basePosition=[halfx, 0, 0], physicsClientId=PYB_CLIENT),
        p.createMultiBody(mass, wall_x, basePosition=[0, halfy, 0], physicsClientId=PYB_CLIENT),
        p.createMultiBody(mass, wall_y, basePosition=[-halfx, 0, 0], physicsClientId=PYB_CLIENT),
        p.createMultiBody(mass, wall_x, basePosition=[0, -halfy, 0], physicsClientId=PYB_CLIENT)
    ]

    # Create and color ceiling
    ceiling_id = p.createMultiBody(mass, ceiling, basePosition=[0, 0, halfz], physicsClientId=PYB_CLIENT)
    p.changeVisualShape(ceiling_id, -1, rgbaColor=grey_color, physicsClientId=PYB_CLIENT)

    # Color the walls
    for wall_id in wall_ids:
        p.changeVisualShape(wall_id, -1, rgbaColor=grey_color, physicsClientId=PYB_CLIENT)

    # Create shelves
    for i in np.arange(-halfx + shelf_t, halfx - shelf_t, (shelf_t * 2 + 2)):
        for j in [-(halfy + 1.5) / 2 - 1.5, (halfy + 1.5) / 2 + 1.5]:
            shelf_id = p.createMultiBody(mass, shelf, basePosition=[i, j, 0], physicsClientId=PYB_CLIENT)
            p.changeVisualShape(shelf_id, -1, rgbaColor=grey_color, physicsClientId=PYB_CLIENT)
            if i != -halfx + shelf_t and j < 0:
                human_location.append([i - shelf_t - 1, -halfy, 0])

    forklift_id = p.createMultiBody(mass, baseCollisionShapeIndex=forklift, basePosition=[-halfx, 0, 0], physicsClientId=PYB_CLIENT)
    p.changeVisualShape(forklift_id, -1, rgbaColor=forklift_color, physicsClientId=PYB_CLIENT)

    for _ in range(min(num_humans, len(human_location))):  
        chosen_human = random.choice(human_location)
        human_id = p.createMultiBody(mass, baseCollisionShapeIndex=humanoid, basePosition=chosen_human, physicsClientId=PYB_CLIENT)
        p.changeVisualShape(human_id, -1, rgbaColor=human_color, physicsClientId=PYB_CLIENT)
        human_ids.append(human_id)
        human_location.remove(chosen_human)  

    
    return forklift_id, human_ids

def run():
    # Initialize the simulation environment
    env = CtrlAviary(drone_model=DEFAULT_DRONES,
                     num_drones=DEFAULT_NUM_DRONES,
                     physics=DEFAULT_PHYSICS,
                     gui=DEFAULT_GUI,
                     obstacles=DEFAULT_OBSTACLES,
                     )

    # Create a Logger
    logger = Logger(logging_freq_hz=int(DEFAULT_CONTROL_FREQ_HZ),
                    num_drones=DEFAULT_NUM_DRONES
                    )

    # Start simulation
    forklift_id, human_ids= setup_environment(env)

    # Precompute paths for smooth movement
    forklift_path = np.concatenate([np.linspace(-halfx, halfx, 1000), np.linspace(halfx, -halfx, 1000)])
    human_paths = [random.choice([-1,1])*np.concatenate([np.linspace(-halfy, halfy, 1500), np.linspace(halfy, -halfy, 1500)]) for _ in human_ids]
    
    original_positions=[]
    for human_id in human_ids:
            original_position, _ = p.getBasePositionAndOrientation(human_id)
            original_positions.append(original_position[0])
    # Run simulation
    for step in range(int(DEFAULT_DURATION_SEC * DEFAULT_SIMULATION_FREQ_HZ)):
        # Calculate the index for forklift and human paths
        forklift_index = step % len(forklift_path)
        forklift_position = forklift_path[forklift_index]
        p.resetBasePositionAndOrientation(forklift_id, [forklift_position, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

        # Update human positions
        for idx, human_id in enumerate(human_ids):
            human_index = step % len(human_paths[idx])
            human_position = human_paths[idx][human_index]
            p.resetBasePositionAndOrientation(human_id, [original_positions[idx], human_position, 0], p.getQuaternionFromEuler([0, 0, 0]))
        p.stepSimulation()
        time.sleep(1 / DEFAULT_SIMULATION_FREQ_HZ)

    env.close()
    logger.save()
    if DEFAULT_GUI:
        logger.plot()

if __name__ == "__main__":
    run()