import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import random
import json
import numpy as np
from collections import deque

class LidarCNN(nn.Module):
    def __init__(self, lidar_dim, output_dim=32):
        super(LidarCNN, self).__init__()
        self.act_fea_cv1 = nn.Conv1d(
            in_channels=1, out_channels=output_dim, kernel_size=5, stride=2, padding=6) 
        self.act_fea_cv2 = nn.Conv1d(
            in_channels=output_dim, out_channels=output_dim, kernel_size=3, stride=2, padding=1)

    def forward(self, x):
        x = torch.relu(self.act_fea_cv1(x))
        x = torch.relu(self.act_fea_cv2(x))
        return x
            

class Actor(nn.Module):
    def __init__(self, lidar_dim, non_lidar_dim, action_dim, device):
        super(Actor, self).__init__()
        self.lidar_cnn = LidarCNN(lidar_dim, output_dim=32)
        self.device = device
        # Calculate the output size of the LidarCNN
        with torch.no_grad():
            sample_input = torch.randn(1, 1, lidar_dim)
            sample_output = self.lidar_cnn(sample_input)
            conv_output_size = sample_output.view(1, -1).shape[1]

        self.fc1 = nn.Linear(conv_output_size, 64)
        self.fc2 = nn.Linear(64 + non_lidar_dim, 64)
        self.mu = nn.Linear(64, action_dim)
        self.sigma = nn.Linear(64, action_dim)

        torch.nn.init.xavier_uniform_(self.fc1.weight)
        torch.nn.init.xavier_uniform_(self.fc2.weight)
        torch.nn.init.xavier_uniform_(self.mu.weight)
        torch.nn.init.xavier_uniform_(self.sigma.weight)

    def forward(self, lidar, non_lidar):
        x = torch.relu(self.lidar_cnn(lidar))
        x = x.view(x.size(0), -1)  # Flatten the output for the fully connected layer
        x = torch.relu(self.fc1(x))
        non_lidar = non_lidar.view(non_lidar.size(0), -1)
        x = torch.cat((x, non_lidar), dim=-1)
        x = torch.relu(self.fc2(x))
        mu = self.mu(x)
        mu = torch.clamp(mu, -1.5, 1.5)
        sigma = torch.nn.functional.softplus(self.sigma(x)) + 1e-2
        return mu, sigma
    
class Critic(nn.Module):
    def __init__(self, lidar_dim, non_lidar_dim, device):
        super(Critic, self).__init__()
        self.lidar_cnn = LidarCNN(lidar_dim, output_dim=32)
        self.device = device

        with torch.no_grad():
            sample_input = torch.randn(1, 1, lidar_dim)
            sample_output = self.lidar_cnn(sample_input)
            conv_output_size = sample_output.view(1, -1).shape[1]

        self.fc1 = nn.Linear(conv_output_size, 64)
        self.fc2 = nn.Linear(64 + non_lidar_dim, 64) 
        self.fc3 = nn.Linear(64, 1)

        torch.nn.init.xavier_uniform_(self.fc1.weight)
        torch.nn.init.xavier_uniform_(self.fc2.weight)
        torch.nn.init.xavier_uniform_(self.fc3.weight)

    def forward(self, lidar, non_lidar):
        x = torch.relu(self.lidar_cnn(lidar))
        x = x.view(x.size(0), -1)  # Flatten the output for the fully connected layer
        x = torch.relu(self.fc1(x))
        non_lidar = non_lidar.view(non_lidar.size(0), -1)  # Ensure non-lidar is flattened
        x = torch.cat((x, non_lidar), dim=-1)  # Concatenate all inputs
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

class ActorCritic():
    def __init__(self, lidar_dim, non_lidar_dim, action_dim, device, lr_actor=1e-5, lr_critic=5e-4, gamma=0.99):
        super(ActorCritic, self).__init__()
        self.actor = Actor(lidar_dim, non_lidar_dim, action_dim, device).to(device)
        self.critic = Critic(lidar_dim, non_lidar_dim, device).to(device)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=lr_actor)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=lr_critic)
        self.device = device
        self.gamma = gamma
        self.transitions = []

    def get_actor_loss(self, lidar, non_lidar, action, advantage, done):
        print("Getting actor loss...")
        mu, sigma = self.actor.forward(lidar.to(self.device), non_lidar.to(self.device))
        m = torch.distributions.Normal(mu, sigma)
        log_prob = m.log_prob(action)
        print(f"Log prob: {log_prob}, Advantage: {advantage}")
        return (-log_prob * advantage).mean()

    def get_critic_loss(self, lidar, non_lidar, action, reward, next_lidar, next_non_lidar, done):
        print("Getting critic loss...")
        value = self.critic.forward(lidar.to(self.device), non_lidar.to(self.device))
        next_value = self.critic.forward(next_lidar.to(self.device), next_non_lidar.to(self.device))
        td_target = reward + self.gamma * next_value * (1 - done)
        td_error = td_target - value
        return F.mse_loss(value, td_target)

    def get_action(self, lidar, non_lidar):
        print("Getting action...")
        mu, sigma = self.actor.forward(lidar.to(self.device), non_lidar.to(self.device))
        m = torch.distributions.Normal(mu, sigma)
        action = m.sample().detach().cpu().numpy()
        return action

    def get_value(self, lidar, non_lidar):
        lidar = torch.tensor(lidar, dtype=torch.float32).to(self.device)
        non_lidar = torch.tensor(non_lidar, dtype=torch.float32).to(self.device)
        value = self.critic(lidar, non_lidar)
        return value.item()

    def update_transitions(self, lidar, non_lidar, action, reward, done):
        print("Updating transitions...")
        self.transitions.append((lidar, non_lidar, action, reward, done))
        
        # If we have at least two transitions, we can do an online update
        if len(self.transitions) >= 2:
            self.online_update()

    def online_update(self):
        print("Online update...")

        # Get the most recent transition
        current_lidar, current_non_lidar, action, reward, done = self.transitions[-1]
        
        # If this isn't the first transition, we can do an update
        if len(self.transitions) > 1:
            # Get the previous state
            prev_lidar, prev_non_lidar, prev_action, _, _ = self.transitions[-2]
            
            # Process for one-step online update
            self.learn_step(prev_lidar, prev_non_lidar, prev_action, reward, current_lidar, current_non_lidar, done)

    def learn_step(self, lidar, non_lidar, action, reward, next_lidar, next_non_lidar, done, I=1.0):
        """Update the model for a single step"""
        # Convert inputs to tensors
        lidar_tensor = torch.tensor(lidar, dtype=torch.float32).unsqueeze(0).to(self.device)
        non_lidar_tensor = torch.tensor(non_lidar, dtype=torch.float32).unsqueeze(0).to(self.device)
        action_tensor = torch.tensor(action, dtype=torch.float32).unsqueeze(0).to(self.device)
        next_lidar_tensor = torch.tensor(next_lidar, dtype=torch.float32).unsqueeze(0).to(self.device)
        next_non_lidar_tensor = torch.tensor(next_non_lidar, dtype=torch.float32).unsqueeze(0).to(self.device)
        
        print("Getting losses...")
        advantage = reward + self.gamma * self.get_value(next_lidar_tensor, next_non_lidar_tensor) - self.get_value(lidar_tensor, non_lidar_tensor)
        print(f"Advantage: {advantage}")
        actor_loss = self.get_actor_loss(lidar_tensor, non_lidar_tensor, action_tensor, advantage, done)
        critic_loss = self.get_critic_loss(lidar_tensor, non_lidar_tensor, action_tensor, reward, next_lidar_tensor, next_non_lidar_tensor, done)
        print(f"Actor loss: {actor_loss}, Critic loss: {critic_loss}")
        
        # Update actor
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
        
        # Update critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

if __name__ == "__main__":
    lidar_dim = 720
    non_lidar_dim = 4
    action_dim = 2
    actor = Actor(lidar_dim, non_lidar_dim, action_dim)
    critic = Critic(lidar_dim, non_lidar_dim, action_dim)  
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    actor = actor.to(device)
    critic = critic.to(device)

    # pass a random tensor through the model
    lidar_tensor = torch.randn(1, 1, lidar_dim).to(device)
    non_lidar_tensor = torch.randn(1, 1, non_lidar_dim).to(device)
    action = actor(lidar_tensor, non_lidar_tensor)
    print(action)

    # pass a random tensor through the critic
    value = critic(lidar_tensor, non_lidar_tensor, action)
    print(value)
    
