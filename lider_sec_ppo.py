"""
Lider Seçimi (Leader Selection) with PPO
======================================

Bu modül, PPO kullanarak ROV filosunda en uygun lideri belirler.
- Actor: Lider seçim politikası
- Critic: Lider adaylarını değerlendirme
"""

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Categorical
from collections import deque
import math
from typing import List, Dict, Tuple


class LiderSecPPOActor(nn.Module):
    """PPO Actor - Lider seçimi"""
    
    def __init__(self, state_size: int = 30, action_size: int = 6, hidden_size: int = 128):
        super(LiderSecPPOActor, self).__init__()
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


class LiderSecPPOCritic(nn.Module):
    """PPO Critic - Lider seçim değerlendirmesi"""
    
    def __init__(self, state_size: int = 30, hidden_size: int = 128):
        super(LiderSecPPOCritic, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.value_head = nn.Linear(hidden_size, 1)
        
        self.relu = nn.ReLU()
    
    def forward(self, state):
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        value = self.value_head(x)
        return value


class LiderSecPPO:
    """PPO tabanlı lider seçimi"""
    
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
        state_size = num_rovs * 5
        self.actor = LiderSecPPOActor(state_size=state_size, action_size=num_rovs).to(self.device)
        self.critic = LiderSecPPOCritic(state_size=state_size).to(self.device)
        
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
    
    def extract_state(self, rovs_info: List[Dict]) -> np.ndarray:
        """
        State vektörünü oluştur
        
        Args:
            rovs_info: Her ROV hakkında:
                {
                    'id': int,
                    'batarya': float (0-100),
                    'konum': (x, y, z),
                    'hedef_mesafesi': float,
                    'merkezilik': float
                }
        
        Returns:
            State vektörü
        """
        state_list = []
        
        for rov_info in rovs_info:
            # Batarya
            state_list.append(rov_info['batarya'] / 100.0)
            
            # Konum
            x, y, z = rov_info['konum']
            state_list.append(x / 500.0)
            state_list.append(y / 500.0)
            state_list.append(z / 500.0)
            
            # Hedef mesafesi
            state_list.append(rov_info['hedef_mesafesi'] / 500.0)
        
        state = np.array(state_list, dtype=np.float32)
        
        # Padding
        state_size = self.num_rovs * 5
        if len(state) < state_size:
            state = np.pad(state, (0, state_size - len(state)), mode='constant')
        
        return state[:state_size]
    
    def select_action(self, state: np.ndarray) -> Tuple[int, float, float]:
        """
        Aksiyon seçimi (lider seçimi)
        
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
        """Generalized Advantage Estimation hesapla"""
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
    
    def calculate_reward(self, mission_success: bool, battery_level: float,
                        time_efficiency: float) -> float:
        """Reward hesaplama"""
        mission_bonus = 100.0 if mission_success else -50.0
        battery_reward = (battery_level / 100.0) * 30.0
        efficiency_reward = time_efficiency * 20.0
        
        return mission_bonus + battery_reward + efficiency_reward
    
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
    
    def select_leader_with_ppo(self, rovs_info: List[Dict], original_selection_func=None) -> int:
        """
        PPO kullanarak lider seç (Orijinal seçim metodu ile entegreli)
        
        Args:
            rovs_info: ROV bilgileri
            original_selection_func: Orijinal lider seçim fonksiyonu (FiratROVNet.lider_sec)
            
        Returns:
            Seçilen lider ROV ID
        """
        state = self.extract_state(rovs_info)
        
        self.actor.eval()
        self.critic.eval()
        with torch.no_grad():
            action, _, _ = self.select_action(state)
        
        # Eğer orijinal seçim fonksiyonu varsa, bunları karşılaştır
        if original_selection_func and callable(original_selection_func):
            try:
                # Orijinal seçim algoritmasını çağır
                original_leader = original_selection_func(rovs_info)
                
                # 50% ihtimalle PPO, 50% ihtimalle orijinal seçimi kullan
                if np.random.random() < 0.5:
                    action = original_leader
                else:
                    action = action
                    
            except Exception as e:
                print(f"⚠️ Orijinal lider seçimi başarısız: {e}")
                # Fallback: PPO seçimini kullan
                pass
        
        return action
    
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
