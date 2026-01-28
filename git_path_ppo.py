"""
Git Path (Path Following) with Proximal Policy Optimization (PPO)
=============================================================

Bu modül, PPO kullanarak ROV'un hesapladığı yolu takip etmesini optimize eder.
- Actor: Hareket seçim politikası
- Critic: Yol takibi başarısı değerlendirmesi
"""

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Categorical
from collections import deque
import math
from typing import List, Tuple, Dict


class GitPathPPOActor(nn.Module):
    """PPO Actor - Yol takibi hareketi"""
    
    def __init__(self, state_size: int = 20, action_size: int = 8, hidden_size: int = 128):
        super(GitPathPPOActor, self).__init__()
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


class GitPathPPOCritic(nn.Module):
    """PPO Critic - Yol takibi başarısı değerlendirmesi"""
    
    def __init__(self, state_size: int = 20, hidden_size: int = 128):
        super(GitPathPPOCritic, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.value_head = nn.Linear(hidden_size, 1)
        
        self.relu = nn.ReLU()
    
    def forward(self, state):
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        value = self.value_head(x)
        return value


class GitPathPPO:
    """PPO tabanlı yol takibi ve hareketi"""
    
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
        self.actor = GitPathPPOActor(state_size=20, action_size=8).to(self.device)
        self.critic = GitPathPPOCritic(state_size=20).to(self.device)
        
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
        
        # Hareket aksiyonları
        self.action_map = {
            0: (1.0, 0.0, 0.0),      # İleri
            1: (-1.0, 0.0, 0.0),     # Geri
            2: (0.0, 1.0, 0.0),      # Sağa
            3: (0.0, -1.0, 0.0),     # Sola
            4: (0.0, 0.0, 1.0),      # Yukarı
            5: (0.0, 0.0, -1.0),     # Aşağı
            6: (0.5, 0.5, 0.0),      # İleri-Sağ
            7: (0.5, -0.5, 0.0),     # İleri-Sol
        }
    
    def extract_state(self, current_pos: Tuple[float, float, float],
                     path: List[Tuple[float, float, float]],
                     path_index: int,
                     battery: float = 100.0) -> np.ndarray:
        """State vektörünü oluştur"""
        state_list = []
        
        # Mevcut pozisyon
        state_list.extend([current_pos[0] / 500.0, current_pos[1] / 500.0, current_pos[2] / 500.0])
        
        # Sonraki hedef
        if path_index < len(path):
            next_target = path[path_index]
            state_list.extend([next_target[0] / 500.0, next_target[1] / 500.0, next_target[2] / 500.0])
            
            dist_to_next = math.sqrt(
                (current_pos[0] - next_target[0])**2 +
                (current_pos[1] - next_target[1])**2 +
                (current_pos[2] - next_target[2])**2
            )
            state_list.append(dist_to_next / 500.0)
        else:
            state_list.extend([0.0, 0.0, 0.0, 0.0])
        
        # Son hedef
        if len(path) > 0:
            final_target = path[-1]
            state_list.extend([final_target[0] / 500.0, final_target[1] / 500.0, final_target[2] / 500.0])
            
            dist_to_final = math.sqrt(
                (current_pos[0] - final_target[0])**2 +
                (current_pos[1] - final_target[1])**2 +
                (current_pos[2] - final_target[2])**2
            )
            state_list.append(dist_to_final / 500.0)
        else:
            state_list.extend([0.0, 0.0, 0.0, 0.0])
        
        # Yol ilerlemesi
        state_list.append((path_index / (len(path) + 1)) if len(path) > 0 else 0.0)
        
        # Batarya
        state_list.append(battery / 100.0)
        
        state = np.array(state_list, dtype=np.float32)
        
        if len(state) < 20:
            state = np.pad(state, (0, 20 - len(state)), mode='constant')
        
        return state[:20]
    
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
    
    def calculate_reward(self, distance_to_target: float, distance_to_final: float,
                        energy_used: float, collision: bool,
                        reached_waypoint: bool) -> float:
        """Reward hesaplama"""
        target_reward = (100.0 - distance_to_target) / 100.0 * 30.0
        final_reward = (500.0 - distance_to_final) / 500.0 * 20.0
        energy_penalty = -energy_used * 0.1
        collision_penalty = -100.0 if collision else 0.0
        waypoint_bonus = 50.0 if reached_waypoint else 0.0
        
        return target_reward + final_reward + energy_penalty + collision_penalty + waypoint_bonus
    
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
    
    def get_movement_with_ppo(self, current_pos: Tuple[float, float, float],
                             path: List[Tuple[float, float, float]],
                             path_index: int,
                             battery: float = 100.0,
                             rov_ref=None) -> Tuple[Tuple[float, float, float], float]:
        """
        PPO kullanarak yol takibi hareketi belirle (Orijinal git metodu ile entegreli)
        
        Args:
            current_pos: Mevcut pozisyon
            path: Takip edilecek yol
            path_index: Yoldaki indeks
            battery: Batarya seviyesi
            rov_ref: ROV referansı (orijinal git metodu çağırısı için)
            
        Returns:
            (hareket_vektörü, güç)
        """
        state = self.extract_state(current_pos, path, path_index, battery)
        
        self.actor.eval()
        self.critic.eval()
        with torch.no_grad():
            action, _, _ = self.select_action(state)
        
        movement = self.action_map[action]
        power = 0.8 + (battery / 100.0) * 0.2
        
        # Eğer ROV referansı varsa ve git metodu varsa, orijinal metodunu çağır
        if rov_ref and hasattr(rov_ref, 'git') and callable(rov_ref.git):
            try:
                # Sonraki hedef noktaya git
                if path_index < len(path):
                    next_target = path[path_index]
                    
                    # 60% ihtimalle orijinal git() metodunu çağır
                    if np.random.random() < 0.6:
                        rov_ref.git(next_target, power=power)
                        return next_target, power
            except Exception as e:
                print(f"⚠️ Orijinal git metodu başarısız: {e}")
        
        return movement, power
    
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
