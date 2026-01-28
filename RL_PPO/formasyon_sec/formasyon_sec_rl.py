"""
Formation Selection with Reinforcement Learning (RL)
==================================================

Bu modül, RL kullanarak Convex Hull ve formasyon seçimini entegre eder.
- State: Hull bilgisi, ROV pozisyonları, hedef
- Action: Formasyon seçimi ve parametre optimizasyonu
- Reward: Formasyon uygunluğu ve güvenlik
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


class FormasyonSecRLNetwork(nn.Module):
    """Formasyon Sec seçimi için RL Network"""
    
    def __init__(self, state_size: int = 40, action_size: int = 20, hidden_size: int = 256):
        super(FormasyonSecRLNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, action_size)
        
        self.relu = nn.ReLU()
    
    def forward(self, state):
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        q_values = self.fc3(x)
        return q_values


class FormasyonSecRL:
    """RL tabanlı formasyon seçimi (Convex Hull ile entegreli)"""
    
    def __init__(self, num_rovs: int = 6, learning_rate: float = 0.001,
                 gamma: float = 0.99, epsilon: float = 0.1):
        """
        Args:
            num_rovs: ROV sayısı
            learning_rate: Öğrenme oranı
            gamma: Discount factor
            epsilon: Exploration rate
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.num_rovs = num_rovs
        
        # Networks
        state_size = num_rovs * 3 + 15  # ROV pozisyonları + hull bilgisi + hedef info
        self.q_network = FormasyonSecRLNetwork(state_size=state_size, action_size=20).to(self.device)
        self.target_network = FormasyonSecRLNetwork(state_size=state_size, action_size=20).to(self.device)
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
                     leader_id: int, target_position: Tuple[float, float, float],
                     hull_center: Tuple[float, float, float] = None,
                     hull_volume: float = 0.0) -> np.ndarray:
        """
        State vektörünü oluştur (Hull bilgisi ile)
        
        Args:
            rov_positions: ROV pozisyonları
            leader_id: Lider ROV ID
            target_position: Hedef pozisyonu
            hull_center: Convex hull merkezi
            hull_volume: Hull hacmi
            
        Returns:
            State vektörü
        """
        state_list = []
        
        # ROV pozisyonları
        for pos in rov_positions:
            state_list.extend([pos[0] / 500.0, pos[1] / 500.0, pos[2] / 500.0])
        
        # Lider pozisyonu
        leader_pos = rov_positions[leader_id]
        state_list.extend([leader_pos[0] / 500.0, leader_pos[1] / 500.0, leader_pos[2] / 500.0])
        
        # Hedef pozisyonu
        state_list.extend([target_position[0] / 500.0, target_position[1] / 500.0])
        
        # Hull merkezi (varsa)
        if hull_center:
            state_list.extend([hull_center[0] / 500.0, hull_center[1] / 500.0, hull_center[2] / 500.0])
        else:
            state_list.extend([0.0, 0.0, 0.0])
        
        # Hull hacmi
        state_list.append(hull_volume / 1000000.0)
        
        # Pozisyon standart sapması
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
        state_size = self.num_rovs * 3 + 15
        if len(state) < state_size:
            state = np.pad(state, (0, state_size - len(state)), mode='constant')
        
        return state[:state_size]
    
    def select_action(self, state: np.ndarray, training: bool = True) -> int:
        """Epsilon-greedy aksiyon seçimi"""
        if training and np.random.random() < self.epsilon:
            return np.random.randint(0, 20)
        
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.q_network(state_tensor)
        
        return q_values.argmax(dim=1).item()
    
    def calculate_reward(self, is_valid: bool, hull_fitness: float,
                        safety_margin: float, energy_cost: float) -> float:
        """
        Reward hesaplama
        
        Args:
            is_valid: Formasyon geçerli mi?
            hull_fitness: Hull uygunluk puanı (0-1)
            safety_margin: Güvenlik payı
            energy_cost: Enerji maliyeti
            
        Returns:
            Reward değeri
        """
        validity_bonus = 50.0 if is_valid else -50.0
        hull_reward = hull_fitness * 30.0
        safety_reward = min(safety_margin / 50.0, 1.0) * 20.0
        energy_penalty = -energy_cost * 10.0
        
        return validity_bonus + hull_reward + safety_reward + energy_penalty
    
    def remember(self, state: np.ndarray, action: int, reward: float,
                next_state: np.ndarray, done: bool):
        """Memory'ye ekle"""
        self.memory.append((state, action, reward, next_state, done))
    
    def train(self):
        """DQN eğitim"""
        if len(self.memory) < self.batch_size:
            return
        
        batch = [self.memory[i] for i in np.random.choice(len(self.memory), self.batch_size, replace=False)]
        
        states = torch.FloatTensor(np.array([exp[0] for exp in batch])).to(self.device)
        actions = torch.LongTensor(np.array([exp[1] for exp in batch])).to(self.device)
        rewards = torch.FloatTensor(np.array([exp[2] for exp in batch])).to(self.device)
        next_states = torch.FloatTensor(np.array([exp[3] for exp in batch])).to(self.device)
        dones = torch.FloatTensor(np.array([exp[4] for exp in batch])).to(self.device)
        
        # Q-Learning güncelleme
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
    
    def select_formation_with_hull_rl(self, rov_positions: List[Tuple[float, float, float]],
                                      leader_id: int, target_position: Tuple[float, float, float],
                                      hull_center: Tuple[float, float, float] = None,
                                      hull_volume: float = 0.0,
                                      filo_ref=None) -> Tuple[int, str, Dict]:
        """
        RL kullanarak hull ile uyumlu formasyonu seç (Orijinal formasyon_sec ile entegreli)
        
        Args:
            rov_positions: ROV pozisyonları
            leader_id: Lider ROV ID
            target_position: Hedef pozisyonu
            hull_center: Hull merkezi
            hull_volume: Hull hacmi
            filo_ref: Filo referansı (orijinal formasyon_sec metodu çağırısı için)
            
        Returns:
            (formasyon_id, formasyon_tipi_adı, ekstra_parametreler)
        """
        state = self.extract_state(rov_positions, leader_id, target_position, hull_center, hull_volume)
        action = self.select_action(state, training=False)
        
        # Ek parametreler
        extra_params = {
            'margin': 30.0,
            'is_3d': True if action > 9 else False,
            'offset': 20.0,
            'harita': False
        }
        
        # Eğer filo_ref varsa ve formasyon_sec metodu varsa, orijinal metodunu çağır
        if filo_ref and hasattr(filo_ref, 'formasyon_sec') and callable(filo_ref.formasyon_sec):
            try:
                # 50% ihtimalle orijinal formasyon_sec() metodunu çağır
                if np.random.random() < 0.5:
                    # Orijinal formasyon_sec metodunu çağır
                    hull_dict = {'center': hull_center, 'volume': hull_volume}
                    filo_ref.formasyon_sec(
                        type=self.formasyon_types[action],
                        margin=extra_params['margin'],
                        harita=extra_params['harita']
                    )
            except Exception as e:
                print(f"⚠️ Orijinal formasyon_sec metodu başarısız: {e}")
        
        return action, self.formasyon_types[action], extra_params
    
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
