"""
Formation with Reinforcement Learning (RL) - Enhanced
=====================================================

Bu modül, RL kullanarak ROV filo formasyonlarını optimize eder.
- State: Her ROV'un pozisyonu, lider pozisyonu, hedef formasyon
- Action: Formasyon türü seçimi (0-19 arası)
- Reward: Formasyon şekli tutarlılığı, enerji verimliliği
"""

import numpy as np
import torch
import torch.nn as nn
from collections import deque
import math
from typing import List, Tuple, Optional, Dict


class FormasyonRLNetwork(nn.Module):
    """Formasyon seçimi için RL Network (Q-Learning)"""
    
    def __init__(self, state_size: int = 32, action_size: int = 20, hidden_size: int = 256):
        """
        Args:
            state_size: State boyutu (ROV pozisyonları + lider + hedef)
            action_size: 20 formasyon tipi
            hidden_size: Gizli katman boyutu
        """
        super(FormasyonRLNetwork, self).__init__()
        self.state_size = state_size
        self.action_size = action_size
        
        # DQN Network
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, action_size)
        
        self.relu = nn.ReLU()
    
    def forward(self, state):
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        q_values = self.fc3(x)
        return q_values


class FormasyonRL_Enhanced:
    """RL tabanlı formasyon seçimi ve koordinasyonu (Enhanced)"""
    
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
        state_size = num_rovs * 3 + 6  # Her ROV 3D pozisyon + lider info + target info
        self.q_network = FormasyonRLNetwork(state_size=state_size, action_size=20).to(self.device)
        self.target_network = FormasyonRLNetwork(state_size=state_size, action_size=20).to(self.device)
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
        
        # Standart sapma hesapla (formasyon yoğunluğu)
        positions = np.array(rov_positions)
        stdev = np.std(positions)
        state_list.append(stdev / 100.0)
        
        # Lider ile hedef arası mesafe
        dist_to_target = np.sqrt(
            (leader_pos[0] - target_position[0])**2 +
            (leader_pos[1] - target_position[1])**2
        )
        state_list.append(dist_to_target / 500.0)
        
        state = np.array(state_list, dtype=np.float32)
        
        # Padding (tam boyuta getir)
        state_size = self.num_rovs * 3 + 6
        if len(state) < state_size:
            state = np.pad(state, (0, state_size - len(state)), mode='constant')
        
        return state[:state_size]
    
    def select_action(self, state: np.ndarray, training: bool = True) -> int:
        """
        Epsilon-greedy aksiyon seçimi
        
        Args:
            state: State vektörü
            training: Eğitim modunda mı?
            
        Returns:
            Formasyon tipi ID (0-19)
        """
        if training and np.random.random() < self.epsilon:
            return np.random.randint(0, 20)
        
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.q_network(state_tensor)
        
        return q_values.argmax(dim=1).item()
    
    def calculate_reward(self, formation_id: int, rov_distances: np.ndarray,
                        energy_efficiency: float, goal_aligned: bool = False) -> float:
        """
        Reward hesaplama
        
        Args:
            formation_id: Seçilen formasyon tipi
            rov_distances: ROV'lar arası mesafeler
            energy_efficiency: Enerji verimliliği (0-1)
            goal_aligned: Formasyon hedef yönünde mi?
            
        Returns:
            Reward değeri
        """
        # Formasyon tutarlılığı (düşük varyans = iyi)
        formation_consistency = 1.0 - (np.std(rov_distances) / (np.mean(rov_distances) + 1e-6))
        formation_reward = formation_consistency * 50.0
        
        # Enerji verimliliği
        energy_reward = energy_efficiency * 30.0
        
        # Hedef yönelim bonus
        goal_bonus = 20.0 if goal_aligned else 0.0
        
        # Collision penalty (formasyondaki ROV'lar çarpışmaması)
        collision_penalty = -100.0 if np.any(rov_distances < 10.0) else 0.0
        
        return formation_reward + energy_reward + goal_bonus + collision_penalty
    
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
    
    def select_formation_with_rl(self, rov_positions: List[Tuple[float, float, float]],
                                leader_id: int, target_position: Tuple[float, float, float],
                                filo_ref=None) -> Tuple[int, str]:
        """
        RL kullanarak en uygun formasyonu seç (Orijinal Formasyon ile entegreli)
        
        Args:
            rov_positions: ROV pozisyonları
            leader_id: Lider ROV ID
            target_position: Hedef pozisyonu
            filo_ref: Filo referansı (orijinal formasyon metodu çağırısı için)
            
        Returns:
            (formasyon_id, formasyon_tipi_adı)
        """
        state = self.extract_state(rov_positions, leader_id, target_position)
        action = self.select_action(state, training=False)
        
        formation_name = self.formasyon_types[action]
        
        # Eğer filo_ref varsa ve formasyon metodu varsa, orijinal metodunu çağır
        if filo_ref and hasattr(filo_ref, 'formasyon') and callable(filo_ref.formasyon):
            try:
                # 50% ihtimalle orijinal formasyon() metodunu çağır
                if np.random.random() < 0.5:
                    # Orijinal formasyon metodunu çağır
                    filo_ref.formasyon(
                        type=formation_name,
                        target_distance=10.0,
                        angle_offset=0.0
                    )
            except Exception as e:
                print(f"⚠️ Orijinal formasyon metodu başarısız: {e}")
        
        return action, formation_name
    
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
