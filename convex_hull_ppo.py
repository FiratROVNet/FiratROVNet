"""
Convex Hull with Proximal Policy Optimization (PPO)
================================================

Bu modül, PPO kullanarak güvenli işlem alanı (Convex Hull) yaratılmasını optimize eder.
- Actor: Hull parametreleri seçim politikası
- Critic: Hull kalitesi değerlendirmesi
"""

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Categorical
from collections import deque
import math
from typing import List, Tuple, Dict


class ConvexHullPPOActor(nn.Module):
    """PPO Actor - Hull parametreleri seçimi"""
    
    def __init__(self, state_size: int = 50, action_size: int = 10, hidden_size: int = 256):
        super(ConvexHullPPOActor, self).__init__()
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


class ConvexHullPPOCritic(nn.Module):
    """PPO Critic - Hull kalitesi değerlendirmesi"""
    
    def __init__(self, state_size: int = 50, hidden_size: int = 256):
        super(ConvexHullPPOCritic, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.value_head = nn.Linear(hidden_size, 1)
        
        self.relu = nn.ReLU()
    
    def forward(self, state):
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        value = self.value_head(x)
        return value


class ConvexHullPPO:
    """PPO tabanlı Convex Hull oluşturma ve optimizasyonu"""
    
    def __init__(self, learning_rate: float = 0.0003, gamma: float = 0.99,
                 gae_lambda: float = 0.95, clip_ratio: float = 0.2,
                 entropy_coef: float = 0.01, value_coef: float = 0.5,
                 num_epochs: int = 3):
        """
        Args:
            learning_rate: Öğrenme oranı
            gamma: Discount factor
            gae_lambda: GAE lambda
            clip_ratio: PPO clipping oranı
            entropy_coef: Entropy regularization
            value_coef: Value loss katsayısı
            num_epochs: Eğitim epochs
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Actor-Critic Networks
        self.actor = ConvexHullPPOActor(state_size=50, action_size=10).to(self.device)
        self.critic = ConvexHullPPOCritic(state_size=50).to(self.device)
        
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
        
        # Hull parametreleri
        self.hull_params = [
            (10.0, 1.5, 5.0),
            (15.0, 2.0, 10.0),
            (20.0, 2.5, 10.0),
            (25.0, 3.0, 15.0),
            (30.0, 2.0, 10.0),
            (35.0, 2.5, 15.0),
            (40.0, 3.0, 20.0),
            (45.0, 2.5, 15.0),
            (50.0, 3.0, 20.0),
            (55.0, 3.5, 25.0),
        ]
    
    def extract_state(self, obstacles: List[Tuple[float, float, float]],
                     rov_positions: List[Tuple[float, float, float]],
                     target_area: Tuple[float, float, float] = None) -> np.ndarray:
        """State vektörünü oluştur"""
        state_list = []
        
        # Engeller istatistikleri
        if obstacles:
            obs_array = np.array(obstacles)
            state_list.append(len(obstacles) / 100.0)
            state_list.append(np.mean(obs_array[:, 0]) / 500.0)
            state_list.append(np.mean(obs_array[:, 1]) / 500.0)
            state_list.append(np.mean(obs_array[:, 2]) / 500.0)
            state_list.append(np.std(obs_array[:, 0]) / 500.0)
            state_list.append(np.std(obs_array[:, 1]) / 500.0)
            state_list.append(np.std(obs_array[:, 2]) / 500.0)
        else:
            state_list.extend([0.0] * 7)
        
        # ROV pozisyonları istatistikleri
        if rov_positions:
            rov_array = np.array(rov_positions)
            state_list.append(len(rov_positions) / 100.0)
            state_list.append(np.mean(rov_array[:, 0]) / 500.0)
            state_list.append(np.mean(rov_array[:, 1]) / 500.0)
            state_list.append(np.mean(rov_array[:, 2]) / 500.0)
            state_list.append(np.std(rov_array[:, 0]) / 500.0)
            state_list.append(np.std(rov_array[:, 1]) / 500.0)
            state_list.append(np.std(rov_array[:, 2]) / 500.0)
        else:
            state_list.extend([0.0] * 7)
        
        # Hedef alan
        if target_area:
            state_list.extend([target_area[0] / 500.0, target_area[1] / 500.0, target_area[2] / 500.0])
        else:
            state_list.extend([0.0, 0.0, 0.0])
        
        # Minimum mesafe
        min_dist = 1000.0
        if obstacles and rov_positions:
            for obs in obstacles:
                for rov in rov_positions:
                    dist = math.sqrt((obs[0]-rov[0])**2 + (obs[1]-rov[1])**2 + (obs[2]-rov[2])**2)
                    min_dist = min(min_dist, dist)
        state_list.append(min_dist / 500.0)
        
        # Hull hacmi tahmini
        if obstacles:
            obs_array = np.array(obstacles)
            hull_volume_estimate = (np.max(obs_array[:, 0]) - np.min(obs_array[:, 0])) * \
                                  (np.max(obs_array[:, 1]) - np.min(obs_array[:, 1])) * \
                                  (np.max(obs_array[:, 2]) - np.min(obs_array[:, 2]))
            state_list.append(hull_volume_estimate / 1000000.0)
        else:
            state_list.append(0.0)
        
        state = np.array(state_list, dtype=np.float32)
        if len(state) < 50:
            state = np.pad(state, (0, 50 - len(state)), mode='constant')
        
        return state[:50]
    
    def select_action(self, state: np.ndarray) -> Tuple[int, float, float]:
        """Aksiyon seçimi"""
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
        """GAE hesapla"""
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
    
    def calculate_reward(self, hull_validity: bool, coverage: float,
                        safety_margin: float, computation_time: float) -> float:
        """Reward hesaplama"""
        validity_bonus = 50.0 if hull_validity else -50.0
        coverage_reward = coverage * 30.0
        safety_reward = min(safety_margin / 50.0, 1.0) * 20.0
        speed_penalty = -computation_time * 10.0
        
        return validity_bonus + coverage_reward + safety_reward + speed_penalty
    
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
    
    def select_hull_params_with_ppo(self, obstacles: List[Tuple[float, float, float]],
                                    rov_positions: List[Tuple[float, float, float]],
                                    hull_manager_ref=None) -> Dict[str, float]:
        """
        PPO kullanarak en uygun hull parametrelerini seç (Orijinal convex_hull_3d ile entegreli)
        
        Args:
            obstacles: Engel pozisyonları
            rov_positions: ROV pozisyonları
            hull_manager_ref: Hull manager referansı (orijinal convex_hull_3d için)
            
        Returns:
            Hull parametreleri: {'offset': float, 'alpha': float, 'buffer_radius': float}
        """
        state = self.extract_state(obstacles, rov_positions)
        
        self.actor.eval()
        self.critic.eval()
        with torch.no_grad():
            action, _, _ = self.select_action(state)
        
        offset, alpha, buffer_radius = self.hull_params[action]
        
        params = {
            'offset': offset,
            'alpha': alpha,
            'buffer_radius': buffer_radius,
            'channel_width': buffer_radius * 2.0
        }
        
        # Eğer hull_manager_ref varsa, orijinal convex_hull_3d metodunu çağır
        if hull_manager_ref and hasattr(hull_manager_ref, 'convex_hull_3d'):
            try:
                # Engelleri 3D noktaları olarak düzenle
                if obstacles:
                    # Testin ortası
                    test_point = tuple(np.mean(rov_positions, axis=0)) if rov_positions else (0, 0, 0)
                    
                    # Orijinal convex_hull_3d metodunu çağır
                    hull_result = hull_manager_ref.convex_hull_3d(
                        points=obstacles,
                        test_point=test_point,
                        margin=offset
                    )
                    
                    # Hull sonucunu params'e ekle
                    if hull_result and 'center' in hull_result:
                        params['hull_center'] = hull_result['center']
                        params['hull_valid'] = hull_result['inside']
            except Exception as e:
                print(f"⚠️ Orijinal convex_hull başarısız: {e}")
        
        return params
    
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
