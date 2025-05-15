import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import CF_folder_traj_data_sort as cf


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

# Prepare output target (motor command)
outputs = np.zeros((len(traj_time), 1))
outputs[:, 0] = motor_cmd  # Motor command as target output

# Convert to torch tensors
inputs = torch.FloatTensor(inputs)
outputs = torch.FloatTensor(outputs)

# Define a simple neural network
class SupervisedPolicy(nn.Module):
    def __init__(self):
        super(SupervisedPolicy, self).__init__()
        self.fc1 = nn.Linear(5, 64) # 5 inputs - 1st layer
        self.fc2 = nn.Linear(64, 32) # 2nd layer
        self.out = nn.Linear(32, 1)  # Single output - final layer

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.out(x)

## Initialize model, loss, and optimizer
# model = SupervisedPolicy()
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device for Sup_Learning(Actor):", device)
model = SupervisedPolicy().to(device)
inputs = inputs.to(device)
outputs = outputs.to(device)

#criterion = nn.MSELoss()
def log_mse_loss(pred, target):
    return torch.mean(torch.log((pred - target) ** 2 + 1))
criterion = log_mse_loss

#type: ignore
optimizer = optim.Adam(model.parameters(), lr=0.005) #type: ignore

# Train the model
for epoch in range(30000):  # training epochs
    optimizer.zero_grad()
    predicted_output = model(inputs)
    loss = criterion(predicted_output, outputs)
    loss.backward()
    optimizer.step()
    if epoch % 50 == 0:
        print(f"Epoch {epoch}, Loss: {loss.item()}")

# Save the pre-trained model
torch.save(model.state_dict(), "short_wing_circle_pretrained_policy.pth")

# print this to test if can use cuda:
# python3 -c "import torch; print(torch.cuda.is_available())"