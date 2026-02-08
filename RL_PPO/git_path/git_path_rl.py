"""
Git Path with Reinforcement Learning (DQN)
==========================================

Bu modül, git_path() fonksiyonunu RL ile optimize eder.
A* yerine RL agent yol planlama yapar.

- State: ROV pozisyonu, hedef pozisyonu, engel bilgileri
- Action: 8 yön hareketi (N, S, E, W, NE, NW, SE, SW)
- Reward: Hedefe yaklaşma, engelden kaçınma, yol uzunluğu optimizasyonu
"""
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
import math
from collections import deque
import random
from typing import List, Tuple, Optional


# =============================================================================
# DQN NETWORK: Path Planning
# =============================================================================
class PathPlanningDQN(nn.Module):
    """
    Deep Q-Network for path planning
    State: ROV position + goal position + obstacle info
    Action: 8 directional movements
    """
    def __init__(self, state_size=20, action_size=8, hidden_size=128):
        super(PathPlanningDQN, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, hidden_size // 2)
        self.fc4 = nn.Linear(hidden_size // 2, action_size)
        
        self.dropout = nn.Dropout(0.2)
        self.layernorm1 = nn.LayerNorm(hidden_size)
        self.layernorm2 = nn.LayerNorm(hidden_size)
    
    def forward(self, state):
        """
        Args:
            state: (batch, state_size) - [rov_x, rov_y, rov_z, goal_x, goal_y, goal_z, 
                                          dist_to_goal, angle_to_goal, obstacles...]
        Returns:
            Q-values: (batch, action_size) - Q-values for 8 directions
        """
        x = F.relu(self.layernorm1(self.fc1(state)))
        x = self.dropout(x)
        x = F.relu(self.layernorm2(self.fc2(x)))
        x = self.dropout(x)
        x = F.relu(self.fc3(x))
        q_values = self.fc4(x)
        return q_values


# =============================================================================
# RL PATH PLANNER AGENT
# =============================================================================
class PathPlannerRL:
    """
    RL tabanlı yol planlayıcı
    A* algoritmasının yerini alır
    """
    
    # 8 yönlü hareket (grid based)
    ACTIONS = {
        0: (0, 1, 0),    # İleri (North)
        1: (0, -1, 0),   # Geri (South)
        2: (1, 0, 0),    # Sağ (East)
        3: (-1, 0, 0),   # Sol (West)
        4: (1, 1, 0),    # Sağ-İleri (NE)
        5: (-1, 1, 0),   # Sol-İleri (NW)
        6: (1, -1, 0),   # Sağ-Geri (SE)
        7: (-1, -1, 0)   # Sol-Geri (SW)
    }
    
    def __init__(self, grid_size=200, step_size=5.0, learning_rate=0.001, 
                 gamma=0.95, epsilon=0.3, epsilon_decay=0.995):
        """
        Args:
            grid_size: Grid boyutu (200x200 varsayılan)
            step_size: Her adımda hareket mesafesi (5.0m varsayılan)
            learning_rate: Öğrenme oranı
            gamma: Discount factor
            epsilon: Exploration rate
            epsilon_decay: Epsilon decay rate
        """
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"🔧 [PathPlanner-RL] Device: {self.device}")
        
        self.grid_size = grid_size
        self.step_size = step_size
        
        # DQN Networks
        self.q_network = PathPlanningDQN(state_size=20, action_size=8, hidden_size=128).to(self.device)
        self.target_network = PathPlanningDQN(state_size=20, action_size=8, hidden_size=128).to(self.device)
        self.target_network.load_state_dict(self.q_network.state_dict())
        
        # Optimizer
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=learning_rate)
        self.criterion = nn.MSELoss()
        
        # Hiperparametreler
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = 0.01
        
        # Experience Replay
        self.memory = deque(maxlen=50000)
        self.batch_size = 128
        
        # İstatistikler
        self.training_stats = {
            'episode': 0,
            'success_rate': 0,
            'avg_path_length': 0,
            'avg_reward': 0
        }
    
    def extract_state(self, current_pos: Tuple[float, float, float],
                     goal_pos: Tuple[float, float, float],
                     obstacles: List[Tuple[float, float, float]]) -> np.ndarray:
        """
        State vektörü oluştur
        
        Args:
            current_pos: Mevcut ROV pozisyonu (x, y, z)
            goal_pos: Hedef pozisyon (x, y, z)
            obstacles: Engel listesi [(x, y, z), ...]
            
        Returns:
            State vektörü (20D)
        """
        # Pozisyonlar (normalize edilmiş)
        state = [
            current_pos[0] / self.grid_size,  # 0: ROV X
            current_pos[1] / self.grid_size,  # 1: ROV Y
            current_pos[2] / 50.0,            # 2: ROV Z (depth)
            goal_pos[0] / self.grid_size,     # 3: Goal X
            goal_pos[1] / self.grid_size,     # 4: Goal Y
            goal_pos[2] / 50.0,               # 5: Goal Z
        ]
        
        # Hedefe mesafe ve açı
        dx = goal_pos[0] - current_pos[0]
        dy = goal_pos[1] - current_pos[1]
        dz = goal_pos[2] - current_pos[2]
        
        dist_to_goal = math.sqrt(dx**2 + dy**2 + dz**2)
        angle_to_goal = math.atan2(dy, dx)  # Radyan cinsinden
        
        state.extend([
            dist_to_goal / self.grid_size,    # 6: Mesafe (normalize)
            angle_to_goal / math.pi,          # 7: Açı (normalize)
        ])
        
        # En yakın engeller (8 yön için)
        obstacle_distances = []
        for action_id in range(8):
            dx_act, dy_act, _ = self.ACTIONS[action_id]
            min_dist = self.grid_size  # Maksimum mesafe
            
            if obstacles:
                for obs in obstacles:
                    # Engelin bu yöndeki mesafesi
                    obs_dx = obs[0] - current_pos[0]
                    obs_dy = obs[1] - current_pos[1]
                    
                    # Yön vektörü ile engel vektörü arasındaki açı
                    dot_product = dx_act * obs_dx + dy_act * obs_dy
                    
                    # Eğer engel bu yöndeyse
                    if dot_product > 0:
                        dist = math.sqrt(obs_dx**2 + obs_dy**2)
                        min_dist = min(min_dist, dist)
            
            obstacle_distances.append(min_dist / self.grid_size)  # Normalize
        
        state.extend(obstacle_distances)  # 8-15: Obstacle distances (8 yön)
        
        # Grid sınırlarına mesafe (4 yön: N, S, E, W)
        border_distances = [
            (self.grid_size/2 - current_pos[1]) / self.grid_size,  # 16: North
            (self.grid_size/2 + current_pos[1]) / self.grid_size,  # 17: South
            (self.grid_size/2 - current_pos[0]) / self.grid_size,  # 18: East
            (self.grid_size/2 + current_pos[0]) / self.grid_size,  # 19: West
        ]
        state.extend(border_distances)
        
        return np.array(state, dtype=np.float32)
    
    def select_action(self, state: np.ndarray, training: bool = True) -> int:
        """Epsilon-greedy action selection"""
        if training and np.random.random() < self.epsilon:
            return np.random.randint(0, 8)
        
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.q_network(state_tensor)
        
        return q_values.argmax(dim=1).item()
    
    def plan_path(self, start_pos: Tuple[float, float, float],
                 goal_pos: Tuple[float, float, float],
                 obstacles: List[Tuple[float, float, float]],
                 max_steps: int = 200) -> List[Tuple[float, float, float]]:
        """RL ile yol planlama (inference)"""
        path = [start_pos]
        current_pos = list(start_pos)
        
        for step in range(max_steps):
            state = self.extract_state(tuple(current_pos), goal_pos, obstacles)
            action = self.select_action(state, training=False)
            
            dx, dy, dz = self.ACTIONS[action]
            next_pos = [
                current_pos[0] + dx * self.step_size,
                current_pos[1] + dy * self.step_size,
                current_pos[2] + dz * self.step_size
            ]
            
            dist_to_goal = math.sqrt(
                (next_pos[0] - goal_pos[0])**2 + 
                (next_pos[1] - goal_pos[1])**2 + 
                (next_pos[2] - goal_pos[2])**2
            )
            
            if dist_to_goal < self.step_size:
                path.append(goal_pos)
                break
            
            path.append(tuple(next_pos))
            current_pos = next_pos
        
        return path
    
    def store_transition(self, state, action, reward, next_state, done):
        """Experience replay buffer'a geçiş ekle"""
        self.memory.append((state, action, reward, next_state, done))
    
    def sample_batch(self):
        """Batch örnekleme"""
        batch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        
        states = torch.FloatTensor(np.array(states)).to(self.device)
        actions = torch.LongTensor(actions).to(self.device)
        rewards = torch.FloatTensor(rewards).to(self.device)
        next_states = torch.FloatTensor(np.array(next_states)).to(self.device)
        dones = torch.FloatTensor(dones).to(self.device)
        
        return states, actions, rewards, next_states, dones
    
    def train_step(self):
        """Bir training step (batch update)"""
        if len(self.memory) < self.batch_size:
            return 0.0
        
        states, actions, rewards, next_states, dones = self.sample_batch()
        
        # Current Q values
        current_q = self.q_network(states).gather(1, actions.unsqueeze(1)).squeeze(1)
        
        # Target Q values (Double DQN)
        with torch.no_grad():
            # Q-network ile action seç
            next_actions = self.q_network(next_states).argmax(dim=1)
            # Target network ile Q-value hesapla
            next_q = self.target_network(next_states).gather(1, next_actions.unsqueeze(1)).squeeze(1)
            target_q = rewards + (1 - dones) * self.gamma * next_q
        
        # Loss hesapla ve backprop
        loss = self.criterion(current_q, target_q)
        
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.q_network.parameters(), max_norm=1.0)
        self.optimizer.step()
        
        return loss.item()
    
    def update_target_network(self):
        """Target network'ü güncelle"""
        self.target_network.load_state_dict(self.q_network.state_dict())
    
    def decay_epsilon(self):
        """Epsilon decay"""
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
    
    def calculate_reward(self, current_pos, next_pos, goal_pos, obstacles, done, collision):
        """
        Reward hesapla
        - Hedefe yaklaşma: +1
        - Hedefe uzaklaşma: -0.5
        - Hedefe ulaşma: +100
        - Çarpışma: -50
        - Step penalty: -0.1
        """
        current_dist = math.sqrt(
            (current_pos[0] - goal_pos[0])**2 + 
            (current_pos[1] - goal_pos[1])**2 + 
            (current_pos[2] - goal_pos[2])**2
        )
        
        next_dist = math.sqrt(
            (next_pos[0] - goal_pos[0])**2 + 
            (next_pos[1] - goal_pos[1])**2 + 
            (next_pos[2] - goal_pos[2])**2
        )
        
        # Step penalty
        reward = -0.1
        
        # Hedefe yaklaşma/uzaklaşma
        if next_dist < current_dist:
            reward += 1.0  # Yaklaşıyor
        else:
            reward -= 0.5  # Uzaklaşıyor
        
        # Terminal states
        if collision:
            reward = -50.0
        elif done:
            reward = 100.0
        
        # Engellere yakınlık penalty'si
        if obstacles:
            min_obstacle_dist = min([
                math.sqrt((obs[0]-next_pos[0])**2 + (obs[1]-next_pos[1])**2)
                for obs in obstacles
            ])
            if min_obstacle_dist < self.step_size * 2:
                reward -= 2.0
        
        return reward
    
    def train(self, num_episodes: int = 1000, target_update_freq: int = 10,
              save_path: str = "path_planner_rl.pth"):
        """
        RL Agent'ı eğit
        
        Args:
            num_episodes: Episode sayısı
            target_update_freq: Target network güncelleme frekansı
            save_path: Model kayıt yolu
        """
        print(f"\n{'='*80}")
        print(f"🚀 PATH PLANNER RL TRAINING BAŞLIYOR")
        print(f"{'='*80}")
        print(f"📊 Episodes: {num_episodes}")
        print(f"📊 Device: {self.device}")
        print(f"📊 Grid Size: {self.grid_size}x{self.grid_size}")
        print(f"📊 Step Size: {self.step_size}m")
        print(f"{'='*80}\n")
        
        episode_rewards = []
        episode_lengths = []
        success_count = 0
        
        for episode in range(num_episodes):
            # Random start ve goal pozisyonları
            start_pos = (
                np.random.uniform(-self.grid_size/4, self.grid_size/4),
                np.random.uniform(-self.grid_size/4, self.grid_size/4),
                np.random.uniform(0, 30)
            )
            
            goal_pos = (
                np.random.uniform(-self.grid_size/4, self.grid_size/4),
                np.random.uniform(-self.grid_size/4, self.grid_size/4),
                np.random.uniform(0, 30)
            )
            
            # Random obstacles
            num_obstacles = np.random.randint(5, 15)
            obstacles = [
                (np.random.uniform(-self.grid_size/3, self.grid_size/3),
                 np.random.uniform(-self.grid_size/3, self.grid_size/3),
                 np.random.uniform(0, 30))
                for _ in range(num_obstacles)
            ]
            
            current_pos = list(start_pos)
            episode_reward = 0
            path_length = 0
            
            for step in range(200):  # Max 200 step
                state = self.extract_state(tuple(current_pos), goal_pos, obstacles)
                action = self.select_action(state, training=True)
                
                dx, dy, dz = self.ACTIONS[action]
                next_pos = [
                    current_pos[0] + dx * self.step_size,
                    current_pos[1] + dy * self.step_size,
                    current_pos[2] + dz * self.step_size
                ]
                
                # Check goal reach
                dist_to_goal = math.sqrt(
                    (next_pos[0] - goal_pos[0])**2 + 
                    (next_pos[1] - goal_pos[1])**2 + 
                    (next_pos[2] - goal_pos[2])**2
                )
                done = dist_to_goal < self.step_size
                
                # Check collision
                collision = False
                for obs in obstacles:
                    obs_dist = math.sqrt(
                        (next_pos[0] - obs[0])**2 + 
                        (next_pos[1] - obs[1])**2
                    )
                    if obs_dist < self.step_size:
                        collision = True
                        break
                
                # Check bounds
                if (abs(next_pos[0]) > self.grid_size/2 or 
                    abs(next_pos[1]) > self.grid_size/2 or 
                    next_pos[2] < 0 or next_pos[2] > 50):
                    collision = True
                
                next_state = self.extract_state(tuple(next_pos), goal_pos, obstacles)
                reward = self.calculate_reward(current_pos, next_pos, goal_pos, 
                                               obstacles, done, collision)
                
                self.store_transition(state, action, reward, next_state, done or collision)
                
                episode_reward += reward
                path_length += 1
                
                if len(self.memory) >= self.batch_size:
                    loss = self.train_step()
                
                if done:
                    success_count += 1
                    break
                
                if collision:
                    break
                
                current_pos = next_pos
            
            episode_rewards.append(episode_reward)
            episode_lengths.append(path_length)
            
            # Target network update
            if episode % target_update_freq == 0:
                self.update_target_network()
            
            # Epsilon decay
            self.decay_epsilon()
            
            # Logging
            if episode % 50 == 0:
                avg_reward = np.mean(episode_rewards[-50:]) if len(episode_rewards) >= 50 else np.mean(episode_rewards)
                avg_length = np.mean(episode_lengths[-50:]) if len(episode_lengths) >= 50 else np.mean(episode_lengths)
                success_rate = success_count / (episode + 1) * 100
                
                print(f"📈 Episode {episode}/{num_episodes} | "
                      f"Avg Reward: {avg_reward:.2f} | "
                      f"Avg Length: {avg_length:.1f} | "
                      f"Success: {success_rate:.1f}% | "
                      f"Epsilon: {self.epsilon:.3f}")
                
                self.training_stats['episode'] = episode
                self.training_stats['avg_reward'] = float(avg_reward)
                self.training_stats['avg_path_length'] = float(avg_length)
                self.training_stats['success_rate'] = float(success_rate)
        
        # Model kaydet
        self.save_model(save_path)
        
        print(f"\n{'='*80}")
        print(f"✅ EĞİTİM TAMAMLANDI!")
        print(f"📊 Toplam Episode: {num_episodes}")
        print(f"📊 Başarı Oranı: {success_count/num_episodes*100:.1f}%")
        print(f"📊 Ortalama Reward: {np.mean(episode_rewards):.2f}")
        print(f"📊 Ortalama Yol Uzunluğu: {np.mean(episode_lengths):.1f}")
        print(f"{'='*80}\n")
    
    def save_model(self, filepath: str):
        """Model'i kaydet"""
        torch.save({
            'q_network': self.q_network.state_dict(),
            'target_network': self.target_network.state_dict(),
            'optimizer': self.optimizer.state_dict(),
            'epsilon': self.epsilon,
            'training_stats': self.training_stats
        }, filepath)
        print(f"✅ [PathPlanner-RL] Model kaydedildi: {filepath}")
    
    def load_model(self, filepath: str):
        """Model'i yükle"""
        checkpoint = torch.load(filepath, map_location=self.device)
        self.q_network.load_state_dict(checkpoint['q_network'])
        self.target_network.load_state_dict(checkpoint['target_network'])
        self.optimizer.load_state_dict(checkpoint['optimizer'])
        self.epsilon = checkpoint['epsilon']
        self.training_stats = checkpoint['training_stats']
        print(f"✅ [PathPlanner-RL] Model yüklendi: {filepath}")


# =============================================================================
# MAIN TRAINING LOOP
# =============================================================================
if __name__ == "__main__":
    print("\n" + "="*80)
    print("🤖 GIT_PATH RL TRAINING")
    print("="*80 + "\n")
    
    # Agent oluştur
    agent = PathPlannerRL(
        grid_size=200,
        step_size=5.0,
        learning_rate=0.001,
        gamma=0.95,
        epsilon=0.3,
        epsilon_decay=0.995
    )
    
    # Eğitim
    agent.train(
        num_episodes=1000,
        target_update_freq=10,
        save_path="path_planner_rl_model.pth"
    )
    
    # Test
    print("\n" + "="*80)
    print("🧪 TEST PHASE")
    print("="*80 + "\n")
    
    start = (0, 0, 10)
    goal = (80, 80, 10)
    obstacles = [(30, 30, 10), (50, 50, 10), (70, 40, 10)]
    
    path = agent.plan_path(start, goal, obstacles, max_steps=200)
    
    print(f"✅ Path planned: {len(path)} waypoints")
    print(f"   Start: {start}")
    print(f"   Goal: {goal}")
    print(f"   Path length: {len(path)}")
    
    print("\n" + "="*80)
    print("✅ TAMAMLANDI!")
    print("="*80 + "\n")
