from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecEnv, VecNormalize
from stable_baselines3.common.env_checker import check_env
import torch
import numpy as np
import sup_learning as sl
import gymnasium as gym

# Proximal Policy Optimization (PPO)
# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# reason being:
""" You are trying to run PPO on the GPU, but it is primarily intended to run on the CPU when not using a 
CNN policy (you are using ActorCriticPolicy which should be a MlpPolicy). 
See https://github.com/DLR-RM/stable-baselines3/issues/1245 for more info. 
You can pass `device='cpu'` or `export CUDA_VISIBLE_DEVICES=` to force using the CPU.Note: 
The model will train, but the GPU utilization will be poor and the training might take longer than on 
CPU. """

device = torch.device("cpu")
print("Using device for PPO(Critic):", device)

class MonocopterEnv(gym.Env):
    def __init__(self, pretrained_model=None):
        super(MonocopterEnv, self).__init__()
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(1,), dtype=np.float32) # must be float32
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(5,), dtype=np.float32)
        self.pretrained_model = pretrained_model


    def reset(self, *, seed=None, options=None):
        # 1) Seed the RNG if requested
        super().reset(seed=seed)

        # 2) Initialize your state
        self.state = np.random.uniform(-1, 1, size=(5,))
        # no NaNs here, but cast anyway
        self.state = self.state.astype(np.float32)

        # 3) Return (observation, info)
        obs, info = self.state.astype(np.float32), {}
        assert np.all(np.isfinite(obs)), f"reset() produced invalid obs {obs}"
        return (obs, info)


    def step(self, action):
        # Ensure action is a scalar
        action = action[0] if isinstance(action, np.ndarray) else action


        # Gymnasium semantics: two boolean flags
        terminated = False
        truncated = False


        # Hybrid policy correction (if you have a pretrained_model)
        if self.pretrained_model:
            self.pretrained_model.eval()
            with torch.no_grad():
                # 1) Prepare input tensor
                state_tensor = torch.tensor(self.state, dtype=torch.float32, device=device).unsqueeze(0)
                # 2) Forward pass
                out = self.pretrained_model(state_tensor)               # shape [1,1] or [1]
                # 3) Clamp any NaN/Inf in the output
                out = torch.nan_to_num(out, nan=0.0, posinf=1.0, neginf=-1.0)
                # 4) Extract scalar
                pre_action = float(out.item())

            # 5) Blend with the PPO action
            action = 0.8 * pre_action + 0.2 * action

            # 6) Ensure action is a pure Python float
            action = float(np.clip(action, 0.0, +65530))


        # Dynamics update

        # 1) dynamics
        self.state = self.state + 0.01 * action
        self.state = np.clip(self.state, -200, 200)

        # 2) sanitize
        self.state = (np.nan_to_num(self.state, nan=0.0, posinf=10, neginf=-10)).astype(np.float32) # converts all funky NaN or positive negative extremes
        if not np.all(np.isfinite(self.state)):
            raise RuntimeError(f"Env produced invalid state: {self.state}")
        
        # 3) build Gymnasium return
        obs = self.state.astype(np.float32)
        assert np.all(np.isfinite(obs)), f"step() produced invalid obs {obs}"
        reward = float(-abs(obs[0]))
        reward = np.clip(reward, -1e2, 1e2)

        terminated, truncated = False, False
        info = {}
        return (self.state, reward, terminated, truncated, info)


# Load the pre-trained model
pretrained_model = sl.SupervisedPolicy().to(device) # moves all weights/buffers from GPU to CPU
pretrained_model.load_state_dict(torch.load("short_wing_circle_pretrained_policy.pth", map_location=device))
pretrained_model.eval() # Set to evaluation mode


#type: ignore
env = DummyVecEnv([lambda: MonocopterEnv(pretrained_model)]) #type: ignore
env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0, clip_reward=10.0)
check_env(MonocopterEnv(pretrained_model))
check_env(env.envs[0])


# Initialize PPO with the environment
ppo_model = PPO("MlpPolicy", env, learning_rate=3e-4, clip_range=0.1, verbose=0, device=device)
ppo_model.learn(total_timesteps=50_000)
ppo_model.save("short_wing_circle_ppo_rl_trained_policy")


""" ppo_model = PPO(
    "MlpPolicy", env,
    learning_rate=3e-5,      # smaller than the default 3e-4
    clip_range=0.1,          # tighter clip to slow down updates
    device="cpu",
    verbose=1
) """