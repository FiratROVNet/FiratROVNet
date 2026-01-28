"""
A* Path Finding with Proximal Policy Optimization (PPO)
========================================================

Bu modül, PPO algoritmasını kullanarak A* yol bulma işlemini optimize eder.
- Actor Network: Politika (Policy) - Aksiyon seçimi
- Critic Network: Value function - Durum değerlendirmesi
- PPO Objective: Clipped surrogate loss ile stabil eğitim
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
from typing import List, Tuple, Optional, Dict


class A_StarPPOActor(nn.Module):
    """PPO Actor Network (Policy)"""
    
    def __init__(self, state_size: int = 10, action_size: int = 8, hidden_size: int = 128):
        super(A_StarPPOActor, self).__init__()
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


class A_StarPPOCritic(nn.Module):
    """PPO Critic Network (Value Function)"""
    
    def __init__(self, state_size: int = 10, hidden_size: int = 128):
        super(A_StarPPOCritic, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.value_head = nn.Linear(hidden_size, 1)
        
        self.relu = nn.ReLU()
    
    def forward(self, state):
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        value = self.value_head(x)
        return value


class A_StarPPO:
    """PPO tabanlı A* yol bulma algoritması"""
    
    def __init__(self, learning_rate: float = 0.0003, gamma: float = 0.99,
                 gae_lambda: float = 0.95, clip_ratio: float = 0.2,
                 entropy_coef: float = 0.01, value_coef: float = 0.5,
                 num_epochs: int = 3, mini_batch_size: int = 32):
        """
        Args:
            learning_rate: Öğrenme oranı
            gamma: Discount factor
            gae_lambda: GAE lambda (Generalized Advantage Estimation)
            clip_ratio: PPO clipping oranı (ε)
            entropy_coef: Entropy regularization katsayısı
            value_coef: Value loss katsayısı
            num_epochs: Her episode'te eğitim döngü sayısı
            mini_batch_size: Mini batch boyutu
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Actor-Critic Networks
        self.actor = A_StarPPOActor(state_size=10, action_size=8).to(self.device)
        self.critic = A_StarPPOCritic(state_size=10).to(self.device)
        
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
        self.mini_batch_size = mini_batch_size
        
        # Memory
        self.memory = {
            'states': [],
            'actions': [],
            'rewards': [],
            'values': [],
            'log_probs': [],
            'dones': []
        }
        
        # Aksiyon mappings
        self.action_map = {
            0: (1, 0),    # Sağ
            1: (-1, 0),   # Sol
            2: (0, 1),    # İleri
            3: (0, -1),   # Geri
            4: (1, 1),    # Sağ-İleri
            5: (-1, 1),   # Sol-İleri
            6: (1, -1),   # Sağ-Geri
            7: (-1, -1),  # Sol-Geri
        }
    
    def extract_state(self, start: Tuple[float, float], goal: Tuple[float, float],
                     current: Tuple[float, float], obstacles: List = None) -> np.ndarray:
        """
        State vektörünü oluştur (A* RL ile aynı)
        
        Args:
            start: Başlangıç koordinatı
            goal: Hedef koordinatı
            current: Mevcut koordinat
            obstacles: Engel listesi
            
        Returns:
            State vektörü (10D)
        """
        dist_to_goal = math.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)
        dist_to_start = math.sqrt((current[0] - start[0])**2 + (current[1] - start[1])**2)
        
        nearest_obstacle_dist = 100.0
        if obstacles:
            for obs in obstacles:
                obs_dist = math.sqrt((current[0] - obs[0])**2 + (current[1] - obs[1])**2)
                nearest_obstacle_dist = min(nearest_obstacle_dist, obs_dist)
        
        dx_goal = (goal[0] - current[0]) / (dist_to_goal + 1e-6)
        dy_goal = (goal[1] - current[1]) / (dist_to_goal + 1e-6)
        
        state = np.array([
            current[0] / 100.0,
            current[1] / 100.0,
            goal[0] / 100.0,
            goal[1] / 100.0,
            dist_to_goal / 100.0,
            dist_to_start / 100.0,
            nearest_obstacle_dist / 100.0,
            dx_goal,
            dy_goal,
            1.0 if dist_to_goal < 5.0 else 0.0
        ], dtype=np.float32)
        
        return state
    
    def select_action(self, state: np.ndarray) -> Tuple[int, float, float]:
        """
        Politika ile aksiyon seçimi
        
        Args:
            state: State vektörü
            
        Returns:
            (action, log_prob, value)
        """
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            action_probs = self.actor(state_tensor)
            value = self.critic(state_tensor)
        
        # Kategorik dağılımdan sample al
        dist = Categorical(action_probs)
        action = dist.sample().item()
        log_prob = dist.log_prob(torch.tensor([action]).to(self.device)).item()
        value = value.item()
        
        return action, log_prob, value
    
    def calculate_gae(self, rewards: List[float], values: List[float], dones: List[bool]) -> Tuple:
        """
        Generalized Advantage Estimation (GAE) hesapla
        
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
        
        # Arkadan öne doğru hesapla
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value = 0
            else:
                next_value = values[t + 1]
            
            # Temporal difference error
            td_error = rewards[t] + self.gamma * next_value * (1 - dones[t]) - values[t]
            
            # GAE
            gae = td_error + self.gamma * self.gae_lambda * (1 - dones[t]) * gae
            advantages.insert(0, gae)
        
        advantages = np.array(advantages)
        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        returns = advantages + np.array(values)
        
        return advantages, returns
    
    def calculate_reward(self, prev_dist: float, curr_dist: float,
                        obstacle_penalty: float = 0.0, goal_reached: bool = False) -> float:
        """Reward hesaplama"""
        distance_reward = (prev_dist - curr_dist) * 0.1
        collision_penalty = -obstacle_penalty * 10.0
        goal_bonus = 100.0 if goal_reached else 0.0
        
        return distance_reward + collision_penalty + goal_bonus
    
    def remember(self, state: np.ndarray, action: int, reward: float,
                log_prob: float, value: float, done: bool):
        """Experience memory'ye ekle"""
        self.memory['states'].append(state)
        self.memory['actions'].append(action)
        self.memory['rewards'].append(reward)
        self.memory['log_probs'].append(log_prob)
        self.memory['values'].append(value)
        self.memory['dones'].append(done)
    
    def train(self):
        """PPO eğitim loop'u"""
        if len(self.memory['states']) < self.mini_batch_size:
            return 0.0
        
        # GAE hesapla
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
        
        # PPO eğitim epochs
        for epoch in range(self.num_epochs):
            # Mini batch'ler halinde eğit
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
                
                # Yeni log probabilities
                dist = Categorical(action_probs)
                new_log_probs = dist.log_prob(batch_actions)
                
                # PPO loss components
                # 1. Actor loss (Policy loss)
                ratio = torch.exp(new_log_probs - batch_old_log_probs)
                surr1 = ratio * batch_advantages
                surr2 = torch.clamp(ratio, 1 - self.clip_ratio, 1 + self.clip_ratio) * batch_advantages
                actor_loss = -torch.min(surr1, surr2).mean()
                
                # 2. Critic loss (Value loss)
                critic_loss = nn.MSELoss()(values, batch_returns)
                
                # 3. Entropy bonus (exploration)
                entropy = dist.entropy().mean()
                
                # Total loss
                loss = actor_loss + self.value_coef * critic_loss - self.entropy_coef * entropy
                
                # Backward
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(list(self.actor.parameters()) + list(self.critic.parameters()), 0.5)
                self.optimizer.step()
                
                total_loss += loss.item()
        
        # Memory'yi temizle
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
    
    def a_star_with_ppo(self, start: Tuple[float, float], goal: Tuple[float, float],
                        obstacles: List = None, max_steps: int = 1000,
                        safety_margin: float = 15.0, harita_ref=None) -> Optional[List[Tuple[float, float]]]:
        """
        PPO-optimized A* yol bulma (Orijinal A* algoritması ile entegreli)
        
        Args:
            start: Başlangıç koordinatı
            goal: Hedef koordinatı
            obstacles: Engel listesi
            max_steps: Maksimum adım sayısı
            safety_margin: Güvenlik mesafesi
            harita_ref: Harita referansı (orijinal a_star_yolu_hesapla için)
            
        Returns:
            Bulunan yol [(x1, y1), (x2, y2), ...] veya None
        """
        # Önce PPO'yu kullanarak yol bulma yapan bir strateji belirle
        state = self.extract_state(start, goal, start, obstacles)
        self.actor.eval()
        self.critic.eval()
        
        with torch.no_grad():
            action_probs = self.actor(torch.FloatTensor(state).unsqueeze(0).to(self.device))
        
        use_ppo_enhanced = action_probs.argmax().item() < 4  # Bazı aksiyonları PPO moduna ayarla
        
        if use_ppo_enhanced and harita_ref is None:
            # PPO-enhanced yol (adım adım PPO hareketi)
            current = start
            path = [current]
            prev_dist = math.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)
            
            for step in range(max_steps):
                # State oluştur
                state = self.extract_state(start, goal, current, obstacles)
                
                # Aksiyon seç
                self.actor.eval()
                with torch.no_grad():
                    action_probs = self.actor(torch.FloatTensor(state).unsqueeze(0).to(self.device))
                action = action_probs.argmax(dim=1).item()
                
                dx, dy = self.action_map[action]
                next_pos = (current[0] + dx * 5.0, current[1] + dy * 5.0)
                
                # Çarpışma kontrolü
                collision = False
                if obstacles:
                    for obs in obstacles:
                        obs_dist = math.sqrt((next_pos[0] - obs[0])**2 + (next_pos[1] - obs[1])**2)
                        if obs_dist < safety_margin:
                            collision = True
                            break
                
                # Hareketi uygula
                if not collision:
                    current = next_pos
                    path.append(current)
                
                # Hedefe mesafe
                goal_dist = math.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)
                
                if goal_dist < safety_margin:
                    path.append(goal)
                    return path
                
                prev_dist = goal_dist
            
            return path if len(path) > 1 else None
        else:
            # Orijinal A* algoritması (harita sistemi varsa)
            if harita_ref and hasattr(harita_ref, 'a_star_yolu_hesapla'):
                try:
                    return harita_ref.a_star_yolu_hesapla(
                        start=start,
                        goal=goal,
                        safety_margin=safety_margin
                    )
                except Exception as e:
                    print(f"⚠️ Orijinal A* başarısız, PPO yoluna geçiliyor: {e}")
                    return self.a_star_with_ppo(start, goal, obstacles, max_steps, safety_margin, None)
            else:
                return self.a_star_with_ppo(start, goal, obstacles, max_steps, safety_margin, None)
    
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
