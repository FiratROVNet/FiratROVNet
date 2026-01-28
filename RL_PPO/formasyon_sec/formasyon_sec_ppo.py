"""
Formation with Proximal Policy Optimization (PPO)
================================================

Bu modül, PPO algoritmasını kullanarak ROV filo formasyonlarını optimize eder.
- Actor: Formasyon tipi politikası
- Critic: Formasyon kalitesi değerlendirmesi
"""
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Categorical
from collections import deque
import math
from typing import List, Tuple, Optional


class FormasyonPPOActor(nn.Module):
    """PPO Actor - Formasyon tipi seçimi"""
    
    def __init__(self, state_size: int = 32, action_size: int = 20, hidden_size: int = 256):
        super(FormasyonPPOActor, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.policy_head = nn.Linear(hidden_size, action_size)
        
        self.relu = nn.ReLU()
        self.softmax = nn.Softmax(dim=-1)
    
    def forward(self, state):
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        action_probs = self.softmax(self.policy_head(x))
        return action_probs


class FormasyonPPOCritic(nn.Module):
    """PPO Critic - Formasyon kalitesi değerlendirmesi"""
    
    def __init__(self, state_size: int = 32, hidden_size: int = 256):
        super(FormasyonPPOCritic, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.value_head = nn.Linear(hidden_size, 1)
        
        self.relu = nn.ReLU()
    
    def forward(self, state):
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        value = self.value_head(x)
        return value


class FormasyonPPO:
    """PPO tabanlı formasyon seçimi ve optimizasyonu"""
    
    def __init__(self, num_rovs: int = 6, learning_rate: float = 0.0003,
                 gamma: float = 0.99, gae_lambda: float = 0.95,
                 clip_ratio: float = 0.2, entropy_coef: float = 0.01,
                 value_coef: float = 0.5, num_epochs: int = 3):
        """
        Args:
            num_rovs: ROV sayısı
            learning_rate: Öğrenme oranı
            gamma: Discount factor
            gae_lambda: GAE lambda
            clip_ratio: PPO clipping oranı
            entropy_coef: Entropy regularization
            value_coef: Value loss katsayısı
            num_epochs: Eğitim epochs
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.num_rovs = num_rovs
        
        # Actor-Critic Networks
        state_size = num_rovs * 3 + 6
        self.actor = FormasyonPPOActor(state_size=state_size, action_size=20).to(self.device)
        self.critic = FormasyonPPOCritic(state_size=state_size).to(self.device)
        
        # Optimizer
        self.optimizer = torch.optim.Adam([
            {'params': self.actor.parameters(), 'lr': learning_rate},
            {'params': self.critic.parameters(), 'lr': learning_rate}
        ])
        
        # Hiperparametreler
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_ratio = clip_ratio
        self.entropy_coef = entropy_coef
        self.value_coef = value_coef
        self.num_epochs = num_epochs
        self.mini_batch_size = 32
        
        # Memory
        self.memory = {
            'states': [],
            'actions': [],
            'rewards': [],
            'values': [],
            'log_probs': [],
            'dones': []
        }
        
        # Formasyon tipleri
        self.formasyon_types = [
            "LINE", "V_SHAPE", "DIAMOND", "TRIANGLE",
            "WEDGE", "X_SHAPE", "CIRCLE", "GRID_2x3",
            "GRID_3x2", "SPIRAL", "RANDOM_COMPACT", "RANDOM_LOOSE",
            "FORMATION_12", "FORMATION_13", "FORMATION_14",
            "FORMATION_15", "FORMATION_16", "FORMATION_17",
            "FORMATION_18", "FORMATION_19"
        ]
    
    def extract_state(self, rov_positions: List[Tuple[float, float, float]],
                     leader_id: int, target_position: Tuple[float, float, float]) -> np.ndarray:
        """
        State vektörünü oluştur
        
        Args:
            rov_positions: [[x, y, z], ...] - Tüm ROV pozisyonları
            leader_id: Lider ROV ID
            target_position: Hedef pozisyonu (x, y, z)
            
        Returns:
            State vektörü
        """
        state_list = []
        
        # Her ROV'un normalizlenmiş pozisyonunu ekle
        for pos in rov_positions:
            state_list.extend([pos[0] / 500.0, pos[1] / 500.0, pos[2] / 500.0])
        
        # Lider pozisyonu
        leader_pos = rov_positions[leader_id]
        state_list.extend([leader_pos[0] / 500.0, leader_pos[1] / 500.0, leader_pos[2] / 500.0])
        
        # Hedef pozisyonu
        state_list.extend([target_position[0] / 500.0, target_position[1] / 500.0])
        
        # Standart sapma (formasyon yoğunluğu)
        positions = np.array(rov_positions)
        stdev = np.std(positions)
        state_list.append(stdev / 100.0)
        
        # Lider-hedef mesafesi
        dist_to_target = np.sqrt(
            (leader_pos[0] - target_position[0])**2 +
            (leader_pos[1] - target_position[1])**2
        )
        state_list.append(dist_to_target / 500.0)
        
        state = np.array(state_list, dtype=np.float32)
        
        # Padding
        state_size = self.num_rovs * 3 + 6
        if len(state) < state_size:
            state = np.pad(state, (0, state_size - len(state)), mode='constant')
        
        return state[:state_size]
    
    def select_action(self, state: np.ndarray) -> Tuple[int, float, float]:
        """
        Aksiyon seçimi (formasyon tipi)
        
        Args:
            state: State vektörü
            
        Returns:
            (action, log_prob, value)
        """
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            action_probs = self.actor(state_tensor)
            value = self.critic(state_tensor)
        
        dist = Categorical(action_probs)
        action = dist.sample().item()
        log_prob = dist.log_prob(torch.tensor([action]).to(self.device)).item()
        value = value.item()
        
        return action, log_prob, value
    
    def calculate_gae(self, rewards: List[float], values: List[float], 
                     dones: List[bool]) -> Tuple:
        """
        Generalized Advantage Estimation hesapla
        
        Args:
            rewards: Reward listesi
            values: Value estimatları
            dones: Done flagları
            
        Returns:
            (advantages, returns)
        """
        advantages = []
        gae = 0
        next_value = 0
        
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value = 0
            else:
                next_value = values[t + 1]
            
            td_error = rewards[t] + self.gamma * next_value * (1 - dones[t]) - values[t]
            gae = td_error + self.gamma * self.gae_lambda * (1 - dones[t]) * gae
            advantages.insert(0, gae)
        
        advantages = np.array(advantages)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        returns = advantages + np.array(values)
        
        return advantages, returns
    
    def calculate_reward(self, rov_distances: np.ndarray, energy_efficiency: float,
                        goal_aligned: bool = False) -> float:
        """
        Reward hesaplama
        
        Args:
            rov_distances: ROV'lar arası mesafeler
            energy_efficiency: Enerji verimliliği
            goal_aligned: Hedef yönü uyumlu mu?
            
        Returns:
            Reward değeri
        """
        formation_consistency = 1.0 - (np.std(rov_distances) / (np.mean(rov_distances) + 1e-6))
        formation_reward = formation_consistency * 50.0
        
        energy_reward = energy_efficiency * 30.0
        goal_bonus = 20.0 if goal_aligned else 0.0
        collision_penalty = -100.0 if np.any(rov_distances < 10.0) else 0.0
        
        return formation_reward + energy_reward + goal_bonus + collision_penalty
    
    def remember(self, state: np.ndarray, action: int, reward: float,
                log_prob: float, value: float, done: bool):
        """Memory'ye ekle"""
        self.memory['states'].append(state)
        self.memory['actions'].append(action)
        self.memory['rewards'].append(reward)
        self.memory['log_probs'].append(log_prob)
        self.memory['values'].append(value)
        self.memory['dones'].append(done)
    
    def train(self):
        """PPO eğitim"""
        if len(self.memory['states']) < self.mini_batch_size:
            return 0.0
        
        advantages, returns = self.calculate_gae(
            self.memory['rewards'],
            self.memory['values'],
            self.memory['dones']
        )
        
        states = torch.FloatTensor(np.array(self.memory['states'])).to(self.device)
        actions = torch.LongTensor(np.array(self.memory['actions'])).to(self.device)
        old_log_probs = torch.FloatTensor(np.array(self.memory['log_probs'])).to(self.device)
        advantages = torch.FloatTensor(advantages).to(self.device)
        returns = torch.FloatTensor(returns).to(self.device)
        
        total_loss = 0.0
        
        for epoch in range(self.num_epochs):
            indices = np.arange(len(states))
            np.random.shuffle(indices)
            
            for i in range(0, len(states), self.mini_batch_size):
                batch_indices = indices[i:i + self.mini_batch_size]
                
                batch_states = states[batch_indices]
                batch_actions = actions[batch_indices]
                batch_old_log_probs = old_log_probs[batch_indices]
                batch_advantages = advantages[batch_indices]
                batch_returns = returns[batch_indices]
                
                # Forward pass
                action_probs = self.actor(batch_states)
                values = self.critic(batch_states).squeeze(1)
                
                # Yeni log probs
                dist = Categorical(action_probs)
                new_log_probs = dist.log_prob(batch_actions)
                
                # PPO loss
                ratio = torch.exp(new_log_probs - batch_old_log_probs)
                surr1 = ratio * batch_advantages
                surr2 = torch.clamp(ratio, 1 - self.clip_ratio, 1 + self.clip_ratio) * batch_advantages
                actor_loss = -torch.min(surr1, surr2).mean()
                
                critic_loss = nn.MSELoss()(values, batch_returns)
                entropy = dist.entropy().mean()
                
                loss = actor_loss + self.value_coef * critic_loss - self.entropy_coef * entropy
                
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(
                    list(self.actor.parameters()) + list(self.critic.parameters()),
                    0.5
                )
                self.optimizer.step()
                
                total_loss += loss.item()
        
        self.clear_memory()
        
        return total_loss / self.num_epochs
    
    def clear_memory(self):
        """Memory'yi temizle"""
        self.memory = {
            'states': [],
            'actions': [],
            'rewards': [],
            'values': [],
            'log_probs': [],
            'dones': []
        }
    
    def select_formation_with_ppo(self, rov_positions: List[Tuple[float, float, float]],
                                 leader_id: int, target_position: Tuple[float, float, float],
                                 filo_ref=None) -> Tuple[int, str]:
        """
        PPO kullanarak en uygun formasyonu seç (Orijinal formasyon_sec ile entegreli)
        
        Args:
            rov_positions: ROV pozisyonları
            leader_id: Lider ROV ID
            target_position: Hedef pozisyonu
            filo_ref: Filo referansı (orijinal formasyon_sec metodu çağırısı için)
            
        Returns:
            (formasyon_id, formasyon_tipi_adı)
        """
        state = self.extract_state(rov_positions, leader_id, target_position)
        
        self.actor.eval()
        self.critic.eval()
        with torch.no_grad():
            action, _, _ = self.select_action(state)
        
        formation_name = self.formasyon_types[action]
        
        # Eğer filo_ref varsa ve formasyon_sec metodu varsa, orijinal metodunu çağır
        if filo_ref and hasattr(filo_ref, 'formasyon_sec') and callable(filo_ref.formasyon_sec):
            try:
                # 50% ihtimalle orijinal formasyon_sec() metodunu çağır
                if np.random.random() < 0.5:
                    # Orijinal formasyon_sec metodunu çağır
                    filo_ref.formasyon_sec(
                        type=formation_name,
                        margin=30.0,
                        harita=False
                    )
            except Exception as e:
                print(f"⚠️ Orijinal formasyon_sec metodu başarısız: {e}")
        
        return action, formation_name
    
    def save_model(self, filepath: str):
        """Model'i kaydet"""
        torch.save({
            'actor': self.actor.state_dict(),
            'critic': self.critic.state_dict(),
            'optimizer': self.optimizer.state_dict(),
        }, filepath)
    
    def load_model(self, filepath: str):
        """Model'i yükle"""
        checkpoint = torch.load(filepath, map_location=self.device)
        self.actor.load_state_dict(checkpoint['actor'])
        self.critic.load_state_dict(checkpoint['critic'])
        self.optimizer.load_state_dict(checkpoint['optimizer'])
