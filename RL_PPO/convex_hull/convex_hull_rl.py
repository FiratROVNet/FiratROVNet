"""
Convex Hull with Reinforcement Learning (RL)
===========================================

Bu modül, RL kullanarak güvenli işlem alanı (Convex Hull) yaratılmasını optimize eder.
- State: Engel pozisyonları, ROV pozisyonları, hedef alan
- Action: Hull parametreleri seçimi (offset, margin)
- Reward: Hull kalitesi, güvenlik, hesaplama hızı
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
from typing import List, Tuple, Dict, Optional


class ConvexHullRLNetwork(nn.Module):
    """Convex Hull parametreleri seçimi için RL Network"""
    
    def __init__(self, state_size: int = 50, action_size: int = 10, hidden_size: int = 256):
        """
        Args:
            state_size: Engel ve ROV bilgilerini içeren state boyutu
            action_size: Hull parametresi kombinasyonu sayısı
            hidden_size: Gizli katman boyutu
        """
        super(ConvexHullRLNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, action_size)
        
        self.relu = nn.ReLU()
    
    def forward(self, state):
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        q_values = self.fc3(x)
        return q_values


class ConvexHullRL:
    """RL tabanlı Convex Hull oluşturma ve optimizasyonu"""
    
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
        self.q_network = ConvexHullRLNetwork(state_size=50, action_size=10).to(self.device)
        self.target_network = ConvexHullRLNetwork(state_size=50, action_size=10).to(self.device)
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
        
        # Hull parametreleri (offset, alpha, buffer_radius)
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
        """
        State vektörünü oluştur
        
        Args:
            obstacles: Engel pozisyonları
            rov_positions: ROV pozisyonları
            target_area: Hedef alan merkezi
            
        Returns:
            State vektörü
        """
        state_list = []
        
        # Engellerin istatistikleri
        if obstacles:
            obs_array = np.array(obstacles)
            state_list.append(len(obstacles) / 100.0)  # Engel sayısı
            state_list.append(np.mean(obs_array[:, 0]) / 500.0)  # Engellerin orta X
            state_list.append(np.mean(obs_array[:, 1]) / 500.0)  # Engellerin orta Y
            state_list.append(np.mean(obs_array[:, 2]) / 500.0)  # Engellerin orta Z
            state_list.append(np.std(obs_array[:, 0]) / 500.0)   # X standart sapması
            state_list.append(np.std(obs_array[:, 1]) / 500.0)   # Y standart sapması
            state_list.append(np.std(obs_array[:, 2]) / 500.0)   # Z standart sapması
        else:
            state_list.extend([0.0] * 7)
        
        # ROV pozisyonlarının istatistikleri
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
        
        # Hedef alan bilgisi
        if target_area:
            state_list.extend([target_area[0] / 500.0, target_area[1] / 500.0, target_area[2] / 500.0])
        else:
            state_list.extend([0.0, 0.0, 0.0])
        
        # Engeller ile ROV'lar arasındaki minimum mesafe
        min_dist = 1000.0
        if obstacles and rov_positions:
            for obs in obstacles:
                for rov in rov_positions:
                    dist = math.sqrt((obs[0]-rov[0])**2 + (obs[1]-rov[1])**2 + (obs[2]-rov[2])**2)
                    min_dist = min(min_dist, dist)
        state_list.append(min_dist / 500.0)
        
        # Engel yoğunluğu
        if obstacles:
            hull_volume_estimate = (max(obs[:, 0]) - min(obs[:, 0])) * \
                                  (max(obs[:, 1]) - min(obs[:, 1])) * \
                                  (max(obs[:, 2]) - min(obs[:, 2]))
            state_list.append(hull_volume_estimate / 1000000.0)
        else:
            state_list.append(0.0)
        
        # Padding
        state = np.array(state_list, dtype=np.float32)
        if len(state) < 50:
            state = np.pad(state, (0, 50 - len(state)), mode='constant')
        
        return state[:50]
    
    def select_action(self, state: np.ndarray, training: bool = True) -> int:
        """Epsilon-greedy aksiyon seçimi"""
        if training and np.random.random() < self.epsilon:
            return np.random.randint(0, 10)
        
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.q_network(state_tensor)
        
        return q_values.argmax(dim=1).item()
    
    def calculate_reward(self, hull_validity: bool, coverage: float,
                        safety_margin: float, computation_time: float) -> float:
        """
        Reward hesaplama
        
        Args:
            hull_validity: Hull geçerli mi?
            coverage: Kapsama alanı (0-1)
            safety_margin: Güvenlik marjı
            computation_time: Hesaplama süresi (saniye)
            
        Returns:
            Reward değeri
        """
        validity_bonus = 50.0 if hull_validity else -50.0
        coverage_reward = coverage * 30.0
        safety_reward = min(safety_margin / 50.0, 1.0) * 20.0
        speed_penalty = -computation_time * 10.0
        
        return validity_bonus + coverage_reward + safety_reward + speed_penalty
    
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
    
    def select_hull_params_with_rl(self, obstacles: List[Tuple[float, float, float]],
                                   rov_positions: List[Tuple[float, float, float]],
                                   hull_manager_ref=None) -> Dict[str, float]:
        """
        RL kullanarak en uygun hull parametrelerini seç (Orijinal convex_hull_3d ile entegreli)
        
        Args:
            obstacles: Engel pozisyonları
            rov_positions: ROV pozisyonları
            hull_manager_ref: Hull manager referansı (orijinal convex_hull_3d için)
            
        Returns:
            Hull parametreleri: {'offset': float, 'alpha': float, 'buffer_radius': float}
        """
        state = self.extract_state(obstacles, rov_positions)
        action = self.select_action(state, training=False)
        
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
