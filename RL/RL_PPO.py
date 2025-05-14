from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.vec_env import VecEnv
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import CF_folder_traj_data_sort as cf
import sup_learning as sl
from gym import spaces, Env

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)

class MonocopterEnv(Env):
    def __init__(self, pretrained_model=None):
        super(MonocopterEnv, self).__init__()
        self.action_space = spaces.Box(low=-1, high=1, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(5,), dtype=np.float32)
        self.pretrained_model = pretrained_model

    def reset(self):
        self.state = np.random.uniform(-1, 1, (5,))  # Random initial state
        return self.state

    def step(self, action):
        # Ensure action is a scalar
        action = action[0] if isinstance(action, np.ndarray) else action

        # Use the pre-trained model's output as a correction term
        if self.pretrained_model:
            self.pretrained_model.eval()  # Set model to evaluation mode
            with torch.no_grad():
                state_tensor = torch.FloatTensor(self.state).unsqueeze(0).to(device) 
                pre_action = self.pretrained_model(state_tensor).item()
            action = 0.8 * pre_action + 0.2 * action  # Hybrid policy

        # Calculate next state (your dynamics)
        self.state = self.state + 0.01 * action  # Simple example
        self.state = np.nan_to_num(self.state)  # Prevent NaN values

        # ✅ Minimize Only the First Value (XYZ error norm)
        reward = -abs(self.state[0])  # Penalize the absolute value of the first state value
        done = False
        return self.state, reward, done, {}


# Load the pre-trained model
pretrained_model = sl.SupervisedPolicy().to(device)
pretrained_model.load_state_dict(torch.load("short_wing_circle_pretrained_policy.pth", map_location=device))
pretrained_model.eval()  # Set to evaluation mode


#type: ignore
env = DummyVecEnv([lambda: MonocopterEnv(pretrained_model)]) #type: ignore


# Initialize PPO with the environment
ppo_model = PPO("MlpPolicy", env, verbose=1, device="cuda")
ppo_model.learn(total_timesteps=200_000)
ppo_model.save("short_wing_circle_ppo_rl_trained_policy")
