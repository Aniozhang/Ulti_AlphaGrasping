from stable_baselines3 import PPO
from environment import MoveObjectsEnv
import os

# Ensure model directory exists
model_dir = "../models"
os.makedirs(model_dir, exist_ok=True)

# Create the environment and agent
env = MoveObjectsEnv()
model = PPO("MlpPolicy", env, verbose=1)

# Training parameters
total_epochs = 1000  # Total number of epochs
save_interval = 100   # Save model every 50 epochs
timesteps_per_epoch = 200  # Number of timesteps per epoch

# Training loop
for epoch in range(1, total_epochs + 1):
    model.learn(total_timesteps=timesteps_per_epoch, reset_num_timesteps=False)
    
    # Save the model every `save_interval` epochs
    if epoch % save_interval == 0:
        model_path = os.path.join(model_dir, f"move_objects_agent_epoch_{epoch}.zip")
        model.save(model_path)
        print(f"Saved model at epoch {epoch} to {model_path}")

print("Training complete.")




 


