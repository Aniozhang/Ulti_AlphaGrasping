# src/environment.py

import os
import pybullet as p
import pybullet_data
from gym import Env, spaces
import numpy as np
from sceneGenerator import load_scene

class MoveObjectsEnv(Env):
    def __init__(self, dataset_dir="../dataset"):
        super(MoveObjectsEnv, self).__init__()
        self.dataset_dir = dataset_dir
        self.data_files = self._load_data_files()
        self.current_idx = 0
        
        # Connect to PyBullet and set environment
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -9.81)

        # Initialize action and observation spaces
        self.action_space = spaces.Discrete(4)  # Four directions for each object
        self.observation_space = spaces.Box(low=-5, high=5, shape=(2,), dtype=np.float32)

    def _load_data_files(self):
        """Loads all the initial and final state file paths from the dataset directory."""
        files = []
        for filename in sorted(os.listdir(self.dataset_dir)):
            if filename.startswith("initial_state_") and filename.endswith(".sdf"):
                idx = filename.split("_")[-1].replace(".sdf", "")
                final_filename = f"final_state_{idx}.sdf"
                if os.path.exists(os.path.join(self.dataset_dir, final_filename)):
                    files.append((filename, final_filename))
        return files

    def reset(self):
        """Loads the initial state from the dataset and resets the environment."""
        initial_file, final_file = self.data_files[self.current_idx]
        self.green_objects = load_scene(os.path.join(self.dataset_dir, initial_file), offset_x=-3)
        self.blue_targets = load_scene(os.path.join(self.dataset_dir, final_file), offset_x=3)
        
        # Move to the next dataset entry
        self.current_idx = (self.current_idx + 1) % len(self.data_files)
        
        # Return the initial state observation
        return self._get_state()

    def step(self, action):
        # Implement action handling here
        # For simplicity, this is left as is from the original example
        pass

    def _get_state(self):
        # Return current positions of green objects
        state = []
        for obj_id in self.green_objects:
            pos, _ = p.getBasePositionAndOrientation(obj_id)
            state.extend([pos[0], pos[1]])
        return np.array(state, dtype=np.float32)

    def _compute_reward(self):
        # Implement reward computation here
        pass

    def render(self, mode="human"):
        pass

    def close(self):
        p.disconnect()
