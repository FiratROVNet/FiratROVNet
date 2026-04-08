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
    
    def __init__(
        self,
        input_dim=38,
        num_formations=21,
        map_grid_size=32,
        map_input_channels=2,
        map_feature_dim=64,
    ):
        super(FormasyonSecimPPO, self).__init__()
        self.base_input_dim = input_dim
        self.input_dim_with_group = input_dim + 1
        self.map_grid_size = int(map_grid_size)
        self.map_input_channels = int(map_input_channels)
        self.map_feature_dim = map_feature_dim
        self.cnn_out_channels = 16

        self.harita_cnn = nn.Sequential(
            nn.Conv2d(in_channels=self.map_input_channels, out_channels=16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Conv2d(in_channels=16, out_channels=self.cnn_out_channels, kernel_size=3, padding=1),
            nn.ReLU(),
        )

        self.map_flat_dim = self.cnn_out_channels * self.map_grid_size * self.map_grid_size
        self.map_fc = nn.Linear(self.map_flat_dim, self.map_feature_dim)
        self.map_norm = nn.LayerNorm(self.map_feature_dim)
        
        # Ortak Katmanlar (RL modelindeki güncel mimari)
        self.fc1 = nn.Linear(self.input_dim_with_group + self.map_feature_dim, 128)
        self.fc2 = nn.Linear(128, 64)

        # ACTOR HEADS - RL'deki head'ler aynı şekilde
        self.formation_id_head = nn.Linear(64, num_formations)  # Logits için
        self.spacing_head = nn.Linear(64, 1)
        self.yaw_head = nn.Linear(64, 1)
        self.leader_position_head = nn.Linear(64, 3)
        
        # Continuous action log_std heads (Normal distributions için)
        self.spacing_log_std_head = nn.Linear(64, 1)
        self.yaw_log_std_head = nn.Linear(64, 1)
        self.position_log_std_head = nn.Linear(64, 3)
        
        # CRITIC HEAD - PPO için eklenen value function
        self.value_head = nn.Linear(64, 1)
    
    def _hazirla_harita_ozellikleri(self, map_tensor, batch_size, device, dtype):
        if map_tensor is None:
            return torch.zeros((batch_size, self.map_feature_dim), device=device, dtype=dtype)

        if not torch.is_tensor(map_tensor):
            map_tensor = torch.tensor(map_tensor, device=device, dtype=dtype)
        else:
            map_tensor = map_tensor.to(device=device, dtype=dtype)

        if map_tensor.dim() == 2:
            map_tensor = map_tensor.unsqueeze(0).unsqueeze(0).repeat(1, self.map_input_channels, 1, 1)
        elif map_tensor.dim() == 3:
            if map_tensor.size(0) == self.map_input_channels:
                map_tensor = map_tensor.unsqueeze(0)
            else:
                map_tensor = map_tensor.unsqueeze(1).repeat(1, self.map_input_channels, 1, 1)
        elif map_tensor.dim() != 4:
            raise ValueError(
                f"map_tensor shape desteklenmiyor: {tuple(map_tensor.shape)} | beklenen: (H,W), (C,H,W) veya (B,C,H,W)"
            )

        if map_tensor.size(1) != self.map_input_channels:
            if map_tensor.size(1) == 1 and self.map_input_channels == 2:
                map_tensor = map_tensor.repeat(1, 2, 1, 1)
            else:
                raise ValueError(
                    f"map_tensor kanal sayısı hatalı: {map_tensor.size(1)} != {self.map_input_channels}"
                )

        if map_tensor.size(2) != self.map_grid_size or map_tensor.size(3) != self.map_grid_size:
            raise ValueError(
                f"map_tensor boyutu sabit olmalı: ({self.map_grid_size}, {self.map_grid_size}), gelen: ({map_tensor.size(2)}, {map_tensor.size(3)})"
            )

        if map_tensor.size(0) == 1 and batch_size > 1:
            map_tensor = map_tensor.expand(batch_size, -1, -1, -1)
        elif map_tensor.size(0) != batch_size:
            raise ValueError(
                f"map_tensor batch boyutu x ile uyumsuz: {map_tensor.size(0)} != {batch_size}"
            )

        map_tensor = torch.clamp(map_tensor, 0.0, 1.0)

        cnn_features = self.harita_cnn(map_tensor)
        pooled_flat = cnn_features.reshape(batch_size, -1)
        map_features = F.relu(self.map_fc(pooled_flat))
        map_features = self.map_norm(map_features)
        return map_features

    def _hazirla_group_id(self, x, group_id):
        if x.size(-1) == self.base_input_dim:
            if group_id is None:
                group_id = torch.zeros((x.size(0), 1), device=x.device, dtype=x.dtype)
            else:
                if not torch.is_tensor(group_id):
                    group_id = torch.tensor(group_id, device=x.device, dtype=x.dtype)
                else:
                    group_id = group_id.to(device=x.device, dtype=x.dtype)

                if group_id.dim() == 0:
                    group_id = group_id.view(1, 1).expand(x.size(0), 1)
                elif group_id.dim() == 1:
                    if group_id.size(0) == x.size(0):
                        group_id = group_id.unsqueeze(1)
                    elif group_id.size(0) == 1:
                        group_id = group_id.view(1, 1).expand(x.size(0), 1)
                    else:
                        raise ValueError(
                            f"group_id batch boyutu x ile uyumsuz: {group_id.size(0)} != {x.size(0)}"
                        )
                elif group_id.dim() == 2:
                    if group_id.size(1) != 1:
                        raise ValueError(
                            f"group_id şekli (batch, 1) olmalı, gelen: {tuple(group_id.shape)}"
                        )
                    if group_id.size(0) == 1 and x.size(0) > 1:
                        group_id = group_id.expand(x.size(0), 1)
                    elif group_id.size(0) != x.size(0):
                        raise ValueError(
                            f"group_id batch boyutu x ile uyumsuz: {group_id.size(0)} != {x.size(0)}"
                        )
                else:
                    raise ValueError(
                        f"group_id desteklenmeyen shape: {tuple(group_id.shape)}"
                    )

            x = torch.cat([x, group_id], dim=1)
        elif x.size(-1) != self.input_dim_with_group:
            raise ValueError(
                f"Beklenen giriş boyutu {self.base_input_dim} veya {self.input_dim_with_group}, gelen: {x.size(-1)}"
            )

        return x

    def forward(self, x, group_id=None, map_tensor=None):
        """
        Forward pass - Actor ve Critic çıktıları
        RL modelindeki group_id + map_tensor koşullaması korunur.
        
        Returns:
            formation_id_logits: (batch, num_formations)
            spacing_mean: (batch, 1)
            spacing_log_std: (batch, 1)
            yaw_mean: (batch, 1)
            yaw_log_std: (batch, 1)
            position_mean: (batch, 3)
            position_log_std: (batch, 3)
            value: (batch, 1)
        """
        if x.dim() == 1:
            x = x.unsqueeze(0)

        x = self._hazirla_group_id(x, group_id)

        map_features = self._hazirla_harita_ozellikleri(
            map_tensor=map_tensor,
            batch_size=x.size(0),
            device=x.device,
            dtype=x.dtype,
        )
        x = torch.cat([x, map_features], dim=1)

        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))

        formation_id_logits = self.formation_id_head(x)
        spacing_mean = self.spacing_head(x)
        spacing_log_std = self.spacing_log_std_head(x)
        yaw_mean = self.yaw_head(x)
        yaw_log_std = self.yaw_log_std_head(x)
        position_mean = self.leader_position_head(x)
        position_log_std = self.position_log_std_head(x)
        value = self.value_head(x)

        return formation_id_logits, spacing_mean, spacing_log_std, yaw_mean, yaw_log_std, position_mean, position_log_std, value
    
    def _clamp_log_std(self, log_std):
        """log_std'ı sayısal stabilite için sınırlandır"""
        return torch.clamp(log_std, min=-20.0, max=2.0)
    
    def get_action(self, x, deterministic=False, group_id=None, map_tensor=None):
        """
        PPO için action sampling - Seçenek 2: Continuous actions with Normal distributions
        
        Returns:
            formation_action: (batch,) - Discrete formation ID
            spacing_action: (batch, 1) - Sampled spacing (continuous)
            yaw_action: (batch, 1) - Sampled yaw (continuous)
            position_action: (batch, 3) - Sampled leader position (continuous)
            log_probs: dict with 'formation', 'spacing', 'yaw', 'position' keys
            value: (batch,) - Value estimate
        """
        (formation_logits, spacing_mean, spacing_log_std, 
         yaw_mean, yaw_log_std, position_mean, position_log_std, value) = self.forward(
            x, group_id=group_id, map_tensor=map_tensor
        )
        
        # Formation ID - Discrete (Categorical)
        formation_probs = F.softmax(formation_logits, dim=-1)
        formation_dist = Categorical(formation_probs)
        formation_action = formation_dist.sample() if not deterministic else torch.argmax(formation_probs, dim=-1)
        formation_log_prob = formation_dist.log_prob(formation_action)
        
        # Continuous actions - Normal distributions
        spacing_log_std_clamped = self._clamp_log_std(spacing_log_std)
        spacing_std = torch.exp(spacing_log_std_clamped)
        spacing_dist = Normal(spacing_mean, spacing_std)
        spacing_action = spacing_dist.mean if deterministic else spacing_dist.rsample()
        spacing_log_prob = spacing_dist.log_prob(spacing_action)
        
        yaw_log_std_clamped = self._clamp_log_std(yaw_log_std)
        yaw_std = torch.exp(yaw_log_std_clamped)
        yaw_dist = Normal(yaw_mean, yaw_std)
        yaw_action = yaw_dist.mean if deterministic else yaw_dist.rsample()
        yaw_log_prob = yaw_dist.log_prob(yaw_action)
        
        position_log_std_clamped = self._clamp_log_std(position_log_std)
        position_std = torch.exp(position_log_std_clamped)
        position_dist = Normal(position_mean, position_std)
        position_action = position_dist.mean if deterministic else position_dist.rsample()
        position_log_prob = position_dist.log_prob(position_action).sum(dim=-1, keepdim=True)
        
        log_probs = {
            'formation': formation_log_prob,
            'spacing': spacing_log_prob,
            'yaw': yaw_log_prob,
            'position': position_log_prob
        }
        
        return formation_action, spacing_action, yaw_action, position_action, log_probs, value.squeeze(-1)
    
    def evaluate_actions(self, x, actions_dict, group_id=None, map_tensor=None):
        """
        PPO update için action evaluation - Seçenek 2: Continuous and discrete action log_probs
        
        Args:
            actions_dict: dict with keys 'formation', 'spacing', 'yaw', 'position'
        
        Returns:
            log_probs: dict with log_probs for each action
            value: (batch,) - Value estimate
            entropy: dict with entropy for each action
        """
        (formation_logits, spacing_mean, spacing_log_std, 
         yaw_mean, yaw_log_std, position_mean, position_log_std, value) = self.forward(
            x, group_id=group_id, map_tensor=map_tensor
        )
        
        # Formation ID - Discrete
        formation_probs = F.softmax(formation_logits, dim=-1)
        formation_dist = Categorical(formation_probs)
        formation_log_prob = formation_dist.log_prob(actions_dict['formation'])
        formation_entropy = formation_dist.entropy()
        
        # Spacing - Continuous
        spacing_log_std_clamped = self._clamp_log_std(spacing_log_std)
        spacing_std = torch.exp(spacing_log_std_clamped)
        spacing_dist = Normal(spacing_mean, spacing_std)
        spacing_log_prob = spacing_dist.log_prob(actions_dict['spacing'])
        spacing_entropy = spacing_dist.entropy()
        
        # Yaw - Continuous
        yaw_log_std_clamped = self._clamp_log_std(yaw_log_std)
        yaw_std = torch.exp(yaw_log_std_clamped)
        yaw_dist = Normal(yaw_mean, yaw_std)
        yaw_log_prob = yaw_dist.log_prob(actions_dict['yaw'])
        yaw_entropy = yaw_dist.entropy()
        
        # Position - Continuous (3D vector)
        position_log_std_clamped = self._clamp_log_std(position_log_std)
        position_std = torch.exp(position_log_std_clamped)
        position_dist = Normal(position_mean, position_std)
        position_log_prob = position_dist.log_prob(actions_dict['position']).sum(dim=-1, keepdim=True)
        position_entropy = position_dist.entropy().sum(dim=-1, keepdim=True)
        
        log_probs = {
            'formation': formation_log_prob,
            'spacing': spacing_log_prob,
            'yaw': yaw_log_prob,
            'position': position_log_prob
        }
        
        entropy = {
            'formation': formation_entropy,
            'spacing': spacing_entropy,
            'yaw': yaw_entropy,
            'position': position_entropy
        }
        
        return log_probs, value.squeeze(-1), entropy
