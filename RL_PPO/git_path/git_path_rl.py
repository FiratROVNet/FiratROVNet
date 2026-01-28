"""
Git Path (Path Following) with Reinforcement Learning (RL)
========================================================

Bu modül, RL kullanarak ROV'un hesapladığı yolu takip etmesini optimize eder.
- State: Mevcut pozisyon, hedef pozisyon, yol bilgisi
- Action: Hareket kararı (ileri, sağ, sol, dönüş vb.)
- Reward: Hedefe yaklaşma, enerji verimliliği, güvenlik
"""
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import numpy as np
import torch
import torch.nn as nn
from collections import deque
import math
from typing import List, Tuple, Optional, Dict


class GitPathRLNetwork(nn.Module):
    """Yol takibi için RL Network"""
    
    def __init__(self, state_size: int = 20, action_size: int = 8, hidden_size: int = 128):
        """
        Args:
            state_size: ROV durumu ve yol bilgisi
            action_size: 8 hareket yönü
            hidden_size: Gizli katman boyutu
        """
        super(GitPathRLNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, action_size)
        
        self.relu = nn.ReLU()
    
    def forward(self, state):
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        q_values = self.fc3(x)
        return q_values


class GitPathRL:
    """RL tabanlı yol takibi ve hareketi"""
    
    def __init__(self, learning_rate: float = 0.001, gamma: float = 0.99,
                 epsilon: float = 0.1):
        """
        Args:
            learning_rate: Öğrenme oranı
            gamma: Discount factor
            epsilon: Exploration rate
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Networks
        self.q_network = GitPathRLNetwork(state_size=20, action_size=8).to(self.device)
        self.target_network = GitPathRLNetwork(state_size=20, action_size=8).to(self.device)
        self.target_network.load_state_dict(self.q_network.state_dict())
        
        # Optimizer
        self.optimizer = torch.optim.Adam(self.q_network.parameters(), lr=learning_rate)
        self.criterion = nn.MSELoss()
        
        # Hiperparametreler
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = 0.995
        self.learning_rate = learning_rate
        
        # Memory
        self.memory = deque(maxlen=10000)
        self.batch_size = 32
        
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
        """
        State vektörünü oluştur
        
        Args:
            current_pos: Mevcut ROV pozisyonu (x, y, z)
            path: Takip edilecek yol
            path_index: Yoldaki mevcut indeks
            battery: Batarya seviyesi (0-100)
            
        Returns:
            State vektörü
        """
        state_list = []
        
        # Mevcut pozisyon (normalize)
        state_list.extend([current_pos[0] / 500.0, current_pos[1] / 500.0, current_pos[2] / 500.0])
        
        # Sonraki hedef nokta
        if path_index < len(path):
            next_target = path[path_index]
            state_list.extend([next_target[0] / 500.0, next_target[1] / 500.0, next_target[2] / 500.0])
            
            # Hedefe uzaklık
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
        
        # Padding
        if len(state) < 20:
            state = np.pad(state, (0, 20 - len(state)), mode='constant')
        
        return state[:20]
    
    def select_action(self, state: np.ndarray, training: bool = True) -> int:
        """Epsilon-greedy aksiyon seçimi"""
        if training and np.random.random() < self.epsilon:
            return np.random.randint(0, 8)
        
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.q_network(state_tensor)
        
        return q_values.argmax(dim=1).item()
    
    def calculate_reward(self, distance_to_target: float, distance_to_final: float,
                        energy_used: float, collision: bool,
                        reached_waypoint: bool) -> float:
        """
        Reward hesaplama
        
        Args:
            distance_to_target: Sonraki hedefe uzaklık
            distance_to_final: Son hedefe uzaklık
            energy_used: Harcanan enerji
            collision: Çarpışma var mı?
            reached_waypoint: Waypoint'e ulaşıldı mı?
            
        Returns:
            Reward değeri
        """
        # Sonraki hedefe yaklaşma
        target_reward = (100.0 - distance_to_target) / 100.0 * 30.0
        
        # Son hedefe yaklaşma
        final_reward = (500.0 - distance_to_final) / 500.0 * 20.0
        
        # Enerji verimliği
        energy_penalty = -energy_used * 0.1
        
        # Çarpışma cezası
        collision_penalty = -100.0 if collision else 0.0
        
        # Waypoint bonus
        waypoint_bonus = 50.0 if reached_waypoint else 0.0
        
        return target_reward + final_reward + energy_penalty + collision_penalty + waypoint_bonus
    
    def remember(self, state: np.ndarray, action: int, reward: float,
                next_state: np.ndarray, done: bool):
        """Memory'ye ekle"""
        self.memory.append((state, action, reward, next_state, done))
    
    def train(self):
        """DQN eğitim"""
        if len(self.memory) < self.batch_size:
            return 0.0
        
        batch = [self.memory[i] for i in np.random.choice(len(self.memory), self.batch_size, replace=False)]
        
        states = torch.FloatTensor(np.array([exp[0] for exp in batch])).to(self.device)
        actions = torch.LongTensor(np.array([exp[1] for exp in batch])).to(self.device)
        rewards = torch.FloatTensor(np.array([exp[2] for exp in batch])).to(self.device)
        next_states = torch.FloatTensor(np.array([exp[3] for exp in batch])).to(self.device)
        dones = torch.FloatTensor(np.array([exp[4] for exp in batch])).to(self.device)
        
        # Q-Learning
        q_values = self.q_network(states).gather(1, actions.unsqueeze(1)).squeeze(1)
        next_q_values = self.target_network(next_states).max(dim=1)[0]
        target_q_values = rewards + (1 - dones) * self.gamma * next_q_values
        
        loss = self.criterion(q_values, target_q_values)
        
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        self.epsilon *= self.epsilon_decay
        
        return loss.item()
    
    def update_target_network(self):
        """Target network'ü güncelle"""
        self.target_network.load_state_dict(self.q_network.state_dict())
    
    def get_movement_with_rl(self, current_pos: Tuple[float, float, float],
                            path: List[Tuple[float, float, float]],
                            path_index: int,
                            battery: float = 100.0,
                            rov_ref=None) -> Tuple[Tuple[float, float, float], float]:
        """
        RL kullanarak yol takibi hareketi belirle (Orijinal git metodu ile entegreli)
        
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
        action = self.select_action(state, training=False)
        
        movement = self.action_map[action]
        power = 0.8 + (battery / 100.0) * 0.2  # Bataryaya bağlı güç
        
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
            'q_network': self.q_network.state_dict(),
            'target_network': self.target_network.state_dict(),
            'optimizer': self.optimizer.state_dict(),
            'epsilon': self.epsilon,
        }, filepath)
    
    def load_model(self, filepath: str):
        """Model'i yükle"""
        checkpoint = torch.load(filepath, map_location=self.device)
        self.q_network.load_state_dict(checkpoint['q_network'])
        self.target_network.load_state_dict(checkpoint['target_network'])
        self.optimizer.load_state_dict(checkpoint['optimizer'])
        self.epsilon = checkpoint['epsilon']
