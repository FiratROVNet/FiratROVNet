"""
A* Path Finding with Reinforcement Learning (RL) Enhancement
=====================================================

Bu modül, RL kullanarak A* algoritmasını optimize etmektedir.
- State: ROV konumu, hedef, engel bilgisi
- Action: Hareket yönü seçimi
- Reward: Hedefe yaklaşma mesafesi, çarpışma cezası
"""

import numpy as np
import torch
import torch.nn as nn
from collections import deque
import math
from typing import List, Tuple, Optional, Dict


class A_StarRLNetwork(nn.Module):
    """A* yol bulma için RL sinir ağı (Q-Learning/DQN)"""
    
    def __init__(self, state_size: int = 10, action_size: int = 8, hidden_size: int = 128):
        """
        Args:
            state_size: State vektörü boyutu (konum, hedef, engel vb.)
            action_size: Aksiyon sayısı (8 yön: ileri, geri, sağ, sol, vs.)
            hidden_size: Gizli katman boyutu
        """
        super(A_StarRLNetwork, self).__init__()
        self.state_size = state_size
        self.action_size = action_size
        
        # DQN Network
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, action_size)
        
        self.relu = nn.ReLU()
        
    def forward(self, state):
        """
        State'i alarak Q-values döndürür
        
        Args:
            state: (batch_size, state_size) tensor
            
        Returns:
            Q-values: (batch_size, action_size) tensor
        """
        x = self.relu(self.fc1(state))
        x = self.relu(self.fc2(x))
        q_values = self.fc3(x)
        return q_values


class A_StarRL:
    """RL tabanlı A* yol bulma algoritması"""
    
    def __init__(self, learning_rate: float = 0.001, gamma: float = 0.99, 
                 epsilon: float = 0.1, epsilon_decay: float = 0.995):
        """
        Args:
            learning_rate: Öğrenme oranı
            gamma: Discount factor (gelecek reward'ların ağırlığı)
            epsilon: Keşif oranı (exploration rate)
            epsilon_decay: Epsilon'un her episode'te azalma oranı
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Ağlar
        self.q_network = A_StarRLNetwork(state_size=10, action_size=8).to(self.device)
        self.target_network = A_StarRLNetwork(state_size=10, action_size=8).to(self.device)
        self.target_network.load_state_dict(self.q_network.state_dict())
        
        # Optimizer
        self.optimizer = torch.optim.Adam(self.q_network.parameters(), lr=learning_rate)
        self.criterion = nn.MSELoss()
        
        # Hiperparametreler
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.learning_rate = learning_rate
        
        # Experience Replay Buffer
        self.memory = deque(maxlen=10000)
        self.batch_size = 32
        
        # Aksiyon mappings (8 yön)
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
        Haritalanan state vektörünü oluştur
        
        Args:
            start: Başlangıç koordinatı (x, y)
            goal: Hedef koordinatı (x, y)
            current: Mevcut koordinat (x, y)
            obstacles: Engel listesi
            
        Returns:
            State vektörü (10D)
        """
        # Normalize mesafeler
        dist_to_goal = math.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)
        dist_to_start = math.sqrt((current[0] - start[0])**2 + (current[1] - start[1])**2)
        
        # Engel mesafesi (varsa)
        nearest_obstacle_dist = 100.0
        if obstacles:
            for obs in obstacles:
                obs_dist = math.sqrt((current[0] - obs[0])**2 + (current[1] - obs[1])**2)
                nearest_obstacle_dist = min(nearest_obstacle_dist, obs_dist)
        
        # Yön vektörleri (normalize)
        dx_goal = (goal[0] - current[0]) / (dist_to_goal + 1e-6)
        dy_goal = (goal[1] - current[1]) / (dist_to_goal + 1e-6)
        
        # State vektörü: [x, y, hedef_x, hedef_y, dist_goal, dist_start, obstacle_dist, dx_goal, dy_goal, goal_reached]
        state = np.array([
            current[0] / 100.0,           # 0: Mevcut x (normalize)
            current[1] / 100.0,           # 1: Mevcut y (normalize)
            goal[0] / 100.0,              # 2: Hedef x (normalize)
            goal[1] / 100.0,              # 3: Hedef y (normalize)
            dist_to_goal / 100.0,         # 4: Hedefe mesafe (normalize)
            dist_to_start / 100.0,        # 5: Başlangıca mesafe (normalize)
            nearest_obstacle_dist / 100.0,# 6: En yakın engel mesafesi (normalize)
            dx_goal,                      # 7: Hedef yön X
            dy_goal,                      # 8: Hedef yön Y
            1.0 if dist_to_goal < 5.0 else 0.0  # 9: Hedef yakınlığı flag
        ], dtype=np.float32)
        
        return state
    
    def select_action(self, state: np.ndarray, training: bool = True) -> int:
        """
        Epsilon-greedy stratejisi ile aksiyon seçimi
        
        Args:
            state: State vektörü
            training: Eğitim modunda mı?
            
        Returns:
            Aksiyon ID (0-7)
        """
        if training and np.random.random() < self.epsilon:
            return np.random.randint(0, 8)
        
        # Greedy aksiyon seçimi
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.q_network(state_tensor)
        
        return q_values.argmax(dim=1).item()
    
    def calculate_reward(self, prev_dist: float, curr_dist: float, 
                        obstacle_penalty: float = 0.0, goal_reached: bool = False) -> float:
        """
        Reward hesaplama
        
        Args:
            prev_dist: Önceki hedefe mesafe
            curr_dist: Mevcut hedefe mesafe
            obstacle_penalty: Çarpışma cezası
            goal_reached: Hedefe ulaşıldı mı?
            
        Returns:
            Reward değeri
        """
        # Hedefe yaklaşma reward
        distance_reward = (prev_dist - curr_dist) * 0.1
        
        # Çarpışma cezası
        collision_penalty = -obstacle_penalty * 10.0
        
        # Hedef ulaşma bonus
        goal_bonus = 100.0 if goal_reached else 0.0
        
        return distance_reward + collision_penalty + goal_bonus
    
    def remember(self, state: np.ndarray, action: int, reward: float, 
                next_state: np.ndarray, done: bool):
        """Experience memory'ye ekle"""
        self.memory.append((state, action, reward, next_state, done))
    
    def train(self):
        """Experience replay ile eğitim"""
        if len(self.memory) < self.batch_size:
            return
        
        # Random batch al
        batch = deque(np.random.choice(self.memory, self.batch_size, replace=False))
        
        states = torch.FloatTensor(np.array([exp[0] for exp in batch])).to(self.device)
        actions = torch.LongTensor(np.array([exp[1] for exp in batch])).to(self.device)
        rewards = torch.FloatTensor(np.array([exp[2] for exp in batch])).to(self.device)
        next_states = torch.FloatTensor(np.array([exp[3] for exp in batch])).to(self.device)
        dones = torch.FloatTensor(np.array([exp[4] for exp in batch])).to(self.device)
        
        # Q-Learning güncelleme
        q_values = self.q_network(states).gather(1, actions.unsqueeze(1)).squeeze(1)
        next_q_values = self.target_network(next_states).max(dim=1)[0]
        target_q_values = rewards + (1 - dones) * self.gamma * next_q_values
        
        # Loss hesapla ve backprop
        loss = self.criterion(q_values, target_q_values)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        # Epsilon decay
        self.epsilon *= self.epsilon_decay
        
        return loss.item()
    
    def update_target_network(self):
        """Target network'ü güncelle"""
        self.target_network.load_state_dict(self.q_network.state_dict())
    
    def a_star_with_rl(self, start: Tuple[float, float], goal: Tuple[float, float],
                       obstacles: List = None, max_steps: int = 1000,
                       safety_margin: float = 15.0, harita_ref=None) -> Optional[List[Tuple[float, float]]]:
        """
        RL-enhanced A* yol bulma (Orijinal A* algoritması ile entegreli)
        
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
        # Önce RL'yi kullanarak yol bulma yapan bir strateji belirle
        state = self.extract_state(start, goal, start, obstacles)
        use_rl_enhanced = self.select_action(state, training=False) < 4  # 50% şans RL yol
        
        if use_rl_enhanced and harita_ref is None:
            # RL-enhanced yol (adım adım RL hareketi)
            current = start
            path = [current]
            prev_dist = math.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)
            
            for step in range(max_steps):
                # State oluştur
                state = self.extract_state(start, goal, current, obstacles)
                
                # Aksiyon seç
                action = self.select_action(state, training=False)
                dx, dy = self.action_map[action]
                
                # Yeni pozisyon
                next_pos = (current[0] + dx * 5.0, current[1] + dy * 5.0)
                
                # Çarpışma kontrolü
                collision = False
                if obstacles:
                    for obs in obstacles:
                        obs_dist = math.sqrt((next_pos[0] - obs[0])**2 + (next_pos[1] - obs[1])**2)
                        if obs_dist < safety_margin:
                            collision = True
                            break
                
                # Çarpışmadıysa hareketi uygula
                if not collision:
                    current = next_pos
                    path.append(current)
                
                # Hedefe mesafe hesapla
                goal_dist = math.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)
                
                # Hedef kontrol
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
                    print(f"⚠️ Orijinal A* başarısız, RL yoluna geçiliyor: {e}")
                    # RL'ye geri dön
                    return self.a_star_with_rl(start, goal, obstacles, max_steps, safety_margin, None)
            else:
                # RL yolunu kullan
                return self.a_star_with_rl(start, goal, obstacles, max_steps, safety_margin, None)
    
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
