"""
Lider Seçimi (Leader Selection) dengan RL
========================================

Bu modül, RL kullanarak ROV filosunda en uygun lideri belirler.
- State: Her ROV'un batarya, konum, hedef mesafesi, merkezilik
- Action: Lider adayı seçimi
- Reward: Başarılı lider seçimi ve görev tamamlama
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


class LiderSecRLNetwork(nn.Module):
    """Lider seçimi için RL Network"""
    
    def __init__(self, state_size: int = 30, action_size: int = 6, hidden_size: int = 128):
        """
        Args:
            state_size: Her ROV'un özelliklerini içeren state boyutu
            action_size: ROV sayısı (lider adayları)
            hidden_size: Gizli katman boyutu
        """
        super(LiderSecRLNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, action_size)
        
        self.relu = nn.ReLU()
    
    def forward(self, state):
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        q_values = self.fc3(x)
        return q_values


class LiderSecRL:
    """RL tabanlı lider seçimi"""
    
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
        state_size = num_rovs * 5  # Her ROV: batarya, x, y, z, görev_başarısı
        self.q_network = LiderSecRLNetwork(state_size=state_size, action_size=num_rovs).to(self.device)
        self.target_network = LiderSecRLNetwork(state_size=state_size, action_size=num_rovs).to(self.device)
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
        
        # Lider seçim kriteri
        self.criteria_weights = {
            'batarya': 1.0,
            'konum': 0.8,
            'hedef_mesafesi': 0.6,
            'merkezilik': 0.7
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
            # Batarya (normalize 0-1)
            state_list.append(rov_info['batarya'] / 100.0)
            
            # Konum (normalize)
            x, y, z = rov_info['konum']
            state_list.append(x / 500.0)
            state_list.append(y / 500.0)
            state_list.append(z / 500.0)
            
            # Hedef mesafesi (normalize)
            state_list.append(rov_info['hedef_mesafesi'] / 500.0)
        
        state = np.array(state_list, dtype=np.float32)
        
        # Padding
        state_size = self.num_rovs * 5
        if len(state) < state_size:
            state = np.pad(state, (0, state_size - len(state)), mode='constant')
        
        return state[:state_size]
    
    def select_action(self, state: np.ndarray, training: bool = True) -> int:
        """Epsilon-greedy aksiyon seçimi"""
        if training and np.random.random() < self.epsilon:
            return np.random.randint(0, self.num_rovs)
        
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.q_network(state_tensor)
        
        return q_values.argmax(dim=1).item()
    
    def calculate_reward(self, leader_id: int, mission_success: bool,
                        battery_level: float, time_efficiency: float) -> float:
        """
        Reward hesaplama
        
        Args:
            leader_id: Seçilen lider ID
            mission_success: Görev başarılı mı?
            battery_level: Lider batarya seviyesi (0-100)
            time_efficiency: Zaman verimliliği (0-1)
            
        Returns:
            Reward değeri
        """
        mission_bonus = 100.0 if mission_success else -50.0
        battery_reward = (battery_level / 100.0) * 30.0
        efficiency_reward = time_efficiency * 20.0
        
        return mission_bonus + battery_reward + efficiency_reward
    
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
    
    def select_leader_with_rl(self, rovs_info: List[Dict], original_selection_func=None) -> int:
        """
        RL kullanarak lider seç (Orijinal seçim metodu ile entegreli)
        
        Args:
            rovs_info: ROV bilgileri
            original_selection_func: Orijinal lider seçim fonksiyonu (FiratROVNet.lider_sec)
            
        Returns:
            Seçilen lider ROV ID
        """
        state = self.extract_state(rovs_info)
        leader_id = self.select_action(state, training=False)
        
        # Eğer orijinal seçim fonksiyonu varsa, bunları karşılaştır
        if original_selection_func and callable(original_selection_func):
            try:
                # Orijinal seçim algoritmasını çağır
                original_leader = original_selection_func(rovs_info)
                
                # 50% ihtimalle RL, 50% ihtimalle orijinal seçimi kullan
                if np.random.random() < 0.5:
                    leader_id = original_leader
                else:
                    leader_id = leader_id
                    
            except Exception as e:
                print(f"⚠️ Orijinal lider seçimi başarısız: {e}")
                # Fallback: RL seçimini kullan
                pass
        
        return leader_id
    
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
