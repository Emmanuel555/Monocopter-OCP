import numpy as np
from stable_baselines3 import PPO
import CF_folder_traj_data_sort as cf

# --- Configuration
DEVICE = "cpu"
N_TEST = 100   # number of test states you want to compare
STATE_DIM = 5  # dimensionality of your Monocopter state

# --- Load your pretrained supervised (actor) model
#pretrained_model = sl.SupervisedPolicy().to(DEVICE)
#pretrained_model.load_state_dict(
#    torch.load("short_wing_circle_pretrained_policy.pth", map_location=DEVICE)
#)
#pretrained_model.eval()

# --- Load your trained RL agent
ppo_model = PPO.load("short_wing_circle_ppo_rl_trained_policy.zip", device=DEVICE)

# --- Generate or load the same test states for both models
# Here we just sample uniformly in the valid observation space:
data_sorter = cf.sort_traj_data()
#long_wing = 'long_traj_data'
short_wing = 'short_traj_data'
selected_wing = short_wing
wing_circle = data_sorter.train_circle5(selected_wing)

# Extract data from wing_circle
traj_time = wing_circle[0]
rx = wing_circle[4]
ry = wing_circle[5]
rz = wing_circle[6]
xyz_error_norm = wing_circle[7]
ref_xyz_norm = wing_circle[8]
motor_cmd = wing_circle[10]
body_yaw = wing_circle[11]

# Prepare input features
inputs = np.zeros((len(traj_time), 5))
inputs[:, 0] = xyz_error_norm  # Position error norm
inputs[:, 1] = rx    # Reference x position 
inputs[:, 2] = ry    # Reference y position 
inputs[:, 3] = rz    # Reference z position 
inputs[:, 4] = body_yaw        # Body yaw angle in deg

# Convert to torch tensors
#test_states = torch.FloatTensor(inputs)
test_states = inputs

# Storage for outputs
ppo_actions = []
sup_actions = []

for state in test_states:

    # 1) Get RL policy output
    #    NOTE: ppo_model.predict expects a batch-shape (n_envs, obs_dim)
    #obs = state.cpu().numpy() 
    action_rl, _ = ppo_model.predict(state, deterministic=True)
    # action_rl will be array shape (1,) here
    ppo_actions.append(float(action_rl.flatten()[0]))
    

    # 2) Get supervised model output
    """ state_tensor = state.unsqueeze(0)  # shape [1, STATE_DIM]
    with torch.no_grad():
        out = pretrained_model(state_tensor)          # e.g. tensor([[…]])
        out = torch.nan_to_num(out, nan=0.0)          # clamp any NaN/Inf
        sup_actions.append(float(out.item())) """

# --- Convert to NumPy for analysis
ppo_actions = np.array(ppo_actions)
""" sup_actions = np.array(sup_actions)

# --- Simple metrics
diffs = ppo_actions - sup_actions
mae  = np.mean(np.abs(diffs))
rmse = np.sqrt(np.mean(diffs**2))

print(f"Compared on {N_TEST} states:")
print(f"  Mean Absolute Error  : {mae:.4f}")
print(f"  Root-Mean-Squared Err: {rmse:.4f}")

# --- (Optional) inspect a few pairs
for i in range(min(10, N_TEST)):
    print(f"State {i:2d}:  RL={ppo_actions[i]:+.3f},  SUP={sup_actions[i]:+.3f},  Δ={diffs[i]:+.3f}")
 """