import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Categorical, Normal


class FormasyonSecimPPO(nn.Module):
    """
    PPO (Proximal Policy Optimization) ile Formasyon Seçim Ağı
    RL modelindeki yapı korunarak Actor-Critic'e dönüştürülmüştür
    """
    
    def __init__(self, input_dim=230, num_formations=20):
        super(FormasyonSecimPPO, self).__init__()
        
        # Ortak Katmanlar (RL modelinden)
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 64)

        # ACTOR HEADS - RL'deki head'ler aynı şekilde
        self.formation_id_head = nn.Linear(64, num_formations)  # Logits için
        self.spacing_head = nn.Linear(64, 1)
        self.yaw_head = nn.Linear(64, 1)
        self.leader_position_head = nn.Linear(64, 3)
        
        # CRITIC HEAD - PPO için eklenen value function
        self.value_head = nn.Linear(64, 1)
    
    def forward(self, x):
        """
        Forward pass - Actor ve Critic çıktıları
        RL'den farkı: value_head eklendi ve formation softmax yerine logits
        """
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))

        formation_id_logits = self.formation_id_head(x)
        spacing_pred = self.spacing_head(x)
        yaw_pred = self.yaw_head(x)
        leader_position_pred = self.leader_position_head(x)
        value = self.value_head(x)

        return formation_id_logits, spacing_pred, yaw_pred, leader_position_pred, value
    
    def get_action(self, x, deterministic=False):
        """PPO için action sampling"""
        formation_logits, spacing, yaw, position, value = self.forward(x)
        
        # Formation ID (categorical distribution)
        formation_probs = F.softmax(formation_logits, dim=-1)
        dist = Categorical(formation_probs)
        formation_action = dist.sample() if not deterministic else torch.argmax(formation_probs, dim=-1)
        log_prob = dist.log_prob(formation_action)
        
        return formation_action, spacing, yaw, position, log_prob, value
    
    def evaluate_actions(self, x, formation_actions):
        """PPO update için action evaluation"""
        formation_logits, spacing, yaw, position, value = self.forward(x)
        
        formation_probs = F.softmax(formation_logits, dim=-1)
        dist = Categorical(formation_probs)
        log_probs = dist.log_prob(formation_actions)
        entropy = dist.entropy()
        
        return log_probs, value.squeeze(-1), entropy
