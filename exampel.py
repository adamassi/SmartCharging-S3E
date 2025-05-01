import sim_ur5.mujoco_env.milestones  # Ensure milestones are registered
import sim_ur5.mujoco_env as gymjoco
import gymnasium as gym

print(gym.envs.registry.keys())

# Create a simple environment with rendering
env = gymjoco.make('GymV21Environment-v0', render_mode='human')
obs, info = env.reset()

# Run a simple random agent
for _ in range(1000):
    action = env.action_space.sample()  # Random action
    obs, reward, terminated, truncated, info = env.step(action)
    
    if terminated or truncated:
        obs, info = env.reset()

env.close()