"""
Formasyon Yönetimi with PPO (Proximal Policy Optimization)
==========================================================

Bu modül, orijinal formasyon algoritmasını PPO ile optimize eder.
- Orijinal A* tabanlı formasyon yönetimi temel alınır
- PPO ağı, formasyon tipini, mesafeleri ve adaptasyonu öğrenir
- State: Lider pozisyonu, takipçi pozisyonları, engel mesafeleri, formasyon tipi
- Action: Formasyon tipi seçimi (V_SHAPE, LINE, WEDGE, COLUMN), mesafe ayarı
- Reward: Formasyon kalitesi, çarpışma önleme, hedef takibi
"""
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Categorical
import numpy as np
import heapq
import math
from typing import List, Tuple, Optional, Dict
from collections import deque


# =============================================================================
# ORİJİNAL A* PLANLAYICI (formasyon.py'den)
# =============================================================================
class AStarPlanlayici:
    """
    Grid tabanlı A* yol planlama algoritması
    Engelleri dikkate alarak güvenli rota hesaplar
    """
    def __init__(self, width, height, safety_padding=15): 
        self.width = int(width)
        self.height = int(height)
        self.safety_padding = safety_padding
        self.grid = np.zeros((self.width, self.height))

    def harita_guncelle(self, engeller):
        """Engelleri A* gridine işler"""
        self.grid = np.zeros((self.width, self.height))
        
        for engel in engeller:
            ox = int(engel[0] + 100)  # Dünya koordinatı -> Grid
            oy = int(engel[1] + 100)
            
            x_min = max(0, ox - self.safety_padding)
            x_max = min(self.width, ox + self.safety_padding)
            y_min = max(0, oy - self.safety_padding)
            y_max = min(self.height, oy + self.safety_padding)
            self.grid[x_min:x_max, y_min:y_max] = 1

    def heuristic(self, a, b):
        """Euclidean mesafe"""
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def planla(self, start_pos, goal_pos):
        """A* algoritması ile yol planlama"""
        start = (int(start_pos[0] + 100), int(start_pos[1] + 100))
        goal = (int(goal_pos[0] + 100), int(goal_pos[1] + 100))
        
        start = (np.clip(start[0], 0, self.width-1), np.clip(start[1], 0, self.height-1))
        goal = (np.clip(goal[0], 0, self.width-1), np.clip(goal[1], 0, self.height-1))

        if self.grid[start[0]][start[1]] == 1: 
            return []

        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:self.heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))
        
        while oheap:
            current = heapq.heappop(oheap)[1]
            if self.heuristic(current, goal) < 5.0:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                
                path = []
                for p in data[::-1][::3]:
                    path.append((p[0]-100, p[1]-100))
                path.append((goal[0]-100, goal[1]-100))
                return path
            
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j 
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    if self.grid[neighbor[0]][neighbor[1]] == 1: 
                        continue
                else: 
                    continue
                
                tentative_g_score = gscore[current] + np.sqrt(i**2+j**2)
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0): 
                    continue
                
                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [k[1] for k in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        
        return []


# =============================================================================
# PPO NETWORK: ACTOR-CRITIC ARCHITECTURE
# =============================================================================
class ActorCriticNetwork(nn.Module):
    """
    PPO için Actor-Critic sinir ağı
    Actor: Policy (aksiyon dağılımı)
    Critic: Value function (state değeri)
    """
    
    def __init__(self, state_size: int = 25, action_size: int = 12, hidden_size: int = 256):
        """
        Args:
            state_size: State vektörü boyutu (ROV pozisyonları, engellar, formasyon bilgisi)
            action_size: Aksiyon sayısı (4 formasyon tipi × 3 mesafe ayarı = 12)
            hidden_size: Gizli katman boyutu
        """
        super(ActorCriticNetwork, self).__init__()
        self.state_size = state_size
        self.action_size = action_size
        
        # Shared feature extractor
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.layernorm1 = nn.LayerNorm(hidden_size)
        self.layernorm2 = nn.LayerNorm(hidden_size)
        
        # Actor head (policy)
        self.actor_fc = nn.Linear(hidden_size, hidden_size // 2)
        self.actor_out = nn.Linear(hidden_size // 2, action_size)
        
        # Critic head (value function)
        self.critic_fc = nn.Linear(hidden_size, hidden_size // 2)
        self.critic_out = nn.Linear(hidden_size // 2, 1)
        
        self.relu = nn.ReLU()
        self.dropout = nn.Dropout(0.2)
        
    def forward(self, state):
        """
        State'i alarak aksiyon olasılıkları ve state değeri döndürür
        
        Args:
            state: (batch_size, state_size) tensor
            
        Returns:
            action_probs: (batch_size, action_size) tensor - aksiyon olasılıkları
            state_value: (batch_size, 1) tensor - state değeri
        """
        # Shared layers
        x = self.relu(self.layernorm1(self.fc1(state)))
        x = self.dropout(x)
        x = self.relu(self.layernorm2(self.fc2(x)))
        x = self.dropout(x)
        
        # Actor (policy)
        actor_x = self.relu(self.actor_fc(x))
        action_logits = self.actor_out(actor_x)
        action_probs = F.softmax(action_logits, dim=-1)
        
        # Critic (value)
        critic_x = self.relu(self.critic_fc(x))
        state_value = self.critic_out(critic_x)
        
        return action_probs, state_value
    
    def get_action(self, state):
        """
        State'ten aksiyon seç (stochastic policy)
        
        Args:
            state: State tensor
            
        Returns:
            action: Seçilen aksiyon
            action_log_prob: Log probability
            state_value: State değeri
        """
        action_probs, state_value = self.forward(state)
        dist = Categorical(action_probs)
        action = dist.sample()
        action_log_prob = dist.log_prob(action)
        
        return action, action_log_prob, state_value
    
    def evaluate_actions(self, states, actions):
        """
        Batch state ve action'ları değerlendir
        
        Args:
            states: State batch
            actions: Action batch
            
        Returns:
            action_log_probs: Log probabilities
            state_values: State values
            entropy: Policy entropy
        """
        action_probs, state_values = self.forward(states)
        dist = Categorical(action_probs)
        action_log_probs = dist.log_prob(actions)
        entropy = dist.entropy()
        
        return action_log_probs, state_values, entropy


# =============================================================================
# PPO MEMORY BUFFER
# =============================================================================
class PPOMemory:
    """PPO için trajectory buffer"""
    
    def __init__(self):
        self.states = []
        self.actions = []
        self.log_probs = []
        self.rewards = []
        self.values = []
        self.dones = []
    
    def add(self, state, action, log_prob, reward, value, done):
        """Transition ekle"""
        self.states.append(state)
        self.actions.append(action)
        self.log_probs.append(log_prob)
        self.rewards.append(reward)
        self.values.append(value)
        self.dones.append(done)
    
    def clear(self):
        """Buffer'ı temizle"""
        self.states = []
        self.actions = []
        self.log_probs = []
        self.rewards = []
        self.values = []
        self.dones = []
    
    def get(self):
        """Buffer içeriğini döndür"""
        return (
            np.array(self.states, dtype=np.float32),
            np.array(self.actions, dtype=np.int64),
            np.array(self.log_probs, dtype=np.float32),
            np.array(self.rewards, dtype=np.float32),
            np.array(self.values, dtype=np.float32),
            np.array(self.dones, dtype=np.float32)
        )


# =============================================================================
# PPO-ENHANCED FORMASYON YÖNETİCİSİ
# =============================================================================
class FormasyonPPO:
    """
    PPO ile geliştirilmiş formasyon yöneticisi
    Orijinal formasyon algoritmasını temel alır ve PPO ile optimize eder
    """
    
    def __init__(self, n_rovs: int = 4, learning_rate: float = 3e-4, 
                 gamma: float = 0.99, gae_lambda: float = 0.95,
                 clip_epsilon: float = 0.2, value_coef: float = 0.5,
                 entropy_coef: float = 0.01):
        """
        Args:
            n_rovs: ROV sayısı
            learning_rate: Öğrenme oranı
            gamma: Discount factor
            gae_lambda: GAE lambda
            clip_epsilon: PPO clip parameter
            value_coef: Value loss coefficient
            entropy_coef: Entropy coefficient
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"🔧 [Formasyon-PPO] Device: {self.device}")
        
        self.n_rovs = n_rovs
        
        # Orijinal A* planlayıcı
        self.a_star_planner = AStarPlanlayici(200, 200, safety_padding=18)
        
        # PPO network
        self.policy = ActorCriticNetwork(state_size=25, action_size=12).to(self.device)
        
        # Optimizer
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=learning_rate)
        
        # Hiperparametreler
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_epsilon = clip_epsilon
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        self.learning_rate = learning_rate
        
        # PPO memory
        self.memory = PPOMemory()
        
        # PPO training parameters
        self.ppo_epochs = 4
        self.batch_size = 64
        self.max_grad_norm = 0.5
        
        # Formasyon tipleri (aksiyonlar)
        self.formation_types = {
            0: "V_SHAPE",      # V şekli
            1: "LINE",         # Düz çizgi
            2: "WEDGE",        # Kama şekli
            3: "COLUMN"        # Kolon formasyonu
        }
        
        # Mesafe çarpanları (her formasyon için 3 seçenek)
        self.distance_multipliers = {
            0: 0.7,   # Dar formasyon
            1: 1.0,   # Normal formasyon
            2: 1.5    # Geniş formasyon
        }
        
        # Aksiyon mapping: 4 formasyon × 3 mesafe = 12 aksiyon
        # Aksiyon 0-2: V_SHAPE (dar, normal, geniş)
        # Aksiyon 3-5: LINE (dar, normal, geniş)
        # Aksiyon 6-8: WEDGE (dar, normal, geniş)
        # Aksiyon 9-11: COLUMN (dar, normal, geniş)
        
        # Durum bilgileri
        self.current_path = []
        self.path_index = 0
        self.current_formation_type = "V_SHAPE"
        self.current_gap = 8.0
        
        # İstatistikler
        self.training_stats = {
            'episode': 0,
            'total_reward': 0,
            'formation_changes': 0,
            'collisions': 0,
            'success_rate': 0
        }
    
    def extract_state(self, lider_pos: Tuple[float, float, float],
                     takipci_pozisyonlari: List[Tuple[float, float, float]],
                     hedef_pos: Tuple[float, float, float],
                     engeller: List[Tuple[float, float, float]],
                     current_formation: str = "V_SHAPE") -> np.ndarray:
        """
        Formasyon karar verme için state vektörü oluşturur
        
        Args:
            lider_pos: Lider ROV pozisyonu (x, y, z)
            takipci_pozisyonlari: Takipçi ROV pozisyonları
            hedef_pos: Hedef pozisyon
            engeller: Engel listesi [(x, y, z), ...]
            current_formation: Mevcut formasyon tipi
            
        Returns:
            State vektörü (25D)
        """
        # Mesafe hesaplamaları
        dist_to_goal = math.sqrt(
            (lider_pos[0] - hedef_pos[0])**2 + 
            (lider_pos[1] - hedef_pos[1])**2
        )
        
        # En yakın engel mesafesi
        min_obstacle_dist = 100.0
        obstacle_count_close = 0
        obstacle_count_medium = 0
        
        if engeller:
            for engel in engeller:
                dist = math.sqrt(
                    (lider_pos[0] - engel[0])**2 + 
                    (lider_pos[1] - engel[1])**2
                )
                min_obstacle_dist = min(min_obstacle_dist, dist)
                if dist < 20.0:
                    obstacle_count_close += 1
                elif dist < 40.0:
                    obstacle_count_medium += 1
        
        # Takipçi formasyon kalitesi (dağılma miktarı)
        formation_quality = 0.0
        if len(takipci_pozisyonlari) > 0:
            # Takipçilerin birbirlerine mesafesi
            distances = []
            for i in range(len(takipci_pozisyonlari)):
                for j in range(i + 1, len(takipci_pozisyonlari)):
                    d = math.sqrt(
                        (takipci_pozisyonlari[i][0] - takipci_pozisyonlari[j][0])**2 +
                        (takipci_pozisyonlari[i][1] - takipci_pozisyonlari[j][1])**2
                    )
                    distances.append(d)
            
            if distances:
                # Standart sapma (düşük = iyi formasyon)
                formation_quality = np.std(distances) / 10.0
        
        # Formasyon tipi one-hot encoding
        formation_encoding = [0, 0, 0, 0]
        if current_formation == "V_SHAPE":
            formation_encoding[0] = 1
        elif current_formation == "LINE":
            formation_encoding[1] = 1
        elif current_formation == "WEDGE":
            formation_encoding[2] = 1
        elif current_formation == "COLUMN":
            formation_encoding[3] = 1
        
        # Yön vektörü
        dx_goal = (hedef_pos[0] - lider_pos[0]) / (dist_to_goal + 1e-6)
        dy_goal = (hedef_pos[1] - lider_pos[1]) / (dist_to_goal + 1e-6)
        
        # State vektörü (25D)
        state = np.array([
            lider_pos[0] / 100.0,              # 0: Lider x (normalize)
            lider_pos[1] / 100.0,              # 1: Lider y (normalize)
            lider_pos[2] / 20.0,               # 2: Lider z (normalize)
            hedef_pos[0] / 100.0,              # 3: Hedef x (normalize)
            hedef_pos[1] / 100.0,              # 4: Hedef y (normalize)
            dist_to_goal / 100.0,              # 5: Hedefe mesafe (normalize)
            min_obstacle_dist / 100.0,         # 6: En yakın engel (normalize)
            obstacle_count_close / 10.0,       # 7: Yakın engel sayısı
            obstacle_count_medium / 20.0,      # 8: Orta mesafe engel sayısı
            dx_goal,                           # 9: Hedef yön X
            dy_goal,                           # 10: Hedef yön Y
            formation_quality,                 # 11: Formasyon kalitesi
            formation_encoding[0],             # 12: V_SHAPE flag
            formation_encoding[1],             # 13: LINE flag
            formation_encoding[2],             # 14: WEDGE flag
            formation_encoding[3],             # 15: COLUMN flag
            len(takipci_pozisyonlari) / 10.0,  # 16: Takipçi sayısı
            self.current_gap / 15.0,           # 17: Mevcut mesafe (normalize)
            1.0 if dist_to_goal < 10.0 else 0.0, # 18: Hedefe yakın flag
            1.0 if min_obstacle_dist < 30.0 else 0.0, # 19: Engele yakın flag
            # Takipçi pozisyon özellikleri (ortalama)
            np.mean([t[0] for t in takipci_pozisyonlari]) / 100.0 if takipci_pozisyonlari else 0.0, # 20
            np.mean([t[1] for t in takipci_pozisyonlari]) / 100.0 if takipci_pozisyonlari else 0.0, # 21
            # Takipçi-lider mesafe ortalaması
            np.mean([math.sqrt((t[0]-lider_pos[0])**2 + (t[1]-lider_pos[1])**2) 
                    for t in takipci_pozisyonlari]) / 20.0 if takipci_pozisyonlari else 0.0, # 22
            # Path progress
            self.path_index / (len(self.current_path) + 1),  # 23: Yol ilerleme oranı
            1.0 if len(self.current_path) > 0 else 0.0       # 24: Path mevcut flag
        ], dtype=np.float32)
        
        return state
    
    def decode_action(self, action: int) -> Tuple[str, float]:
        """
        Aksiyon numarasını formasyon tipi ve mesafeye çevirir
        
        Args:
            action: Aksiyon ID (0-11)
            
        Returns:
            (formation_type, gap_distance)
        """
        formation_idx = action // 3  # 0-3: formasyon tipi
        distance_idx = action % 3     # 0-2: mesafe çarpanı
        
        formation_type = self.formation_types[formation_idx]
        base_gap = 8.0
        gap_distance = base_gap * self.distance_multipliers[distance_idx]
        
        return formation_type, gap_distance
    
    def select_action(self, state: np.ndarray, training: bool = True):
        """
        PPO policy kullanarak aksiyon seç
        
        Args:
            state: State vektörü
            training: Eğitim modunda mı?
            
        Returns:
            action: Aksiyon ID
            log_prob: Log probability
            value: State value
        """
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            action, log_prob, value = self.policy.get_action(state_tensor)
        
        return action.item(), log_prob.item(), value.item()
    
    def get_formation_offset(self, index: int, formation_type: str, gap: float) -> Tuple[float, float]:
        """
        Formasyon tipine göre ROV offset hesaplar
        
        Args:
            index: ROV indeksi (takipçi)
            formation_type: Formasyon tipi
            gap: Ara mesafe
            
        Returns:
            (dx, dy) offset
        """
        if formation_type == "V_SHAPE":
            row = (index + 1) // 2
            side = 1 if index % 2 != 0 else -1
            return (side * gap * row * 1.5, -gap * row)
        
        elif formation_type == "LINE":
            return (0, -gap * index)
        
        elif formation_type == "WEDGE":
            row = index
            return (0, -gap * row * 0.8)
        
        elif formation_type == "COLUMN":
            col = index % 2
            row = index // 2
            return (gap * (col - 0.5), -gap * row)
        
        return (0, -gap * index)
    
    def calculate_reward(self, prev_state_info: Dict, current_state_info: Dict,
                        formation_change: bool, collision: bool) -> float:
        """
        Reward hesaplama
        
        Args:
            prev_state_info: Önceki state bilgileri
            current_state_info: Mevcut state bilgileri
            formation_change: Formasyon değişti mi?
            collision: Çarpışma oldu mu?
            
        Returns:
            Reward değeri
        """
        reward = 0.0
        
        # 1. Hedefe yaklaşma reward
        dist_improvement = prev_state_info['dist_to_goal'] - current_state_info['dist_to_goal']
        reward += dist_improvement * 0.5
        
        # 2. Formasyon kalitesi reward (düşük dağılma = yüksek reward)
        formation_quality = current_state_info.get('formation_quality', 0.5)
        reward += (1.0 - formation_quality) * 2.0
        
        # 3. Engel önleme reward
        obstacle_dist = current_state_info.get('min_obstacle_dist', 100.0)
        if obstacle_dist < 15.0:
            reward -= 5.0 * (1.0 - obstacle_dist / 15.0)
        elif obstacle_dist > 30.0:
            reward += 1.0
        
        # 4. Çarpışma cezası
        if collision:
            reward -= 20.0
        
        # 5. Formasyon değişikliği cezası (gereksiz değişiklik önleme)
        if formation_change:
            reward -= 0.5
        
        # 6. Hedef ulaşma bonus
        if current_state_info['dist_to_goal'] < 5.0:
            reward += 50.0
        
        # 7. Takipçi-lider mesafe optimizasyonu
        avg_follower_dist = current_state_info.get('avg_follower_dist', 10.0)
        ideal_dist = self.current_gap * 1.5
        dist_diff = abs(avg_follower_dist - ideal_dist)
        reward += (1.0 - dist_diff / ideal_dist) * 1.0
        
        return reward
    
    def compute_gae(self, rewards, values, dones, next_value):
        """
        Generalized Advantage Estimation (GAE) hesapla
        
        Args:
            rewards: Reward dizisi
            values: Value dizisi
            dones: Done flags
            next_value: Son state'in value'su
            
        Returns:
            advantages: Advantage dizisi
            returns: Return dizisi
        """
        advantages = np.zeros_like(rewards)
        lastgaelam = 0
        
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                nextnonterminal = 1.0 - dones[t]
                nextvalue = next_value
            else:
                nextnonterminal = 1.0 - dones[t]
                nextvalue = values[t + 1]
            
            delta = rewards[t] + self.gamma * nextvalue * nextnonterminal - values[t]
            advantages[t] = lastgaelam = delta + self.gamma * self.gae_lambda * nextnonterminal * lastgaelam
        
        returns = advantages + values
        return advantages, returns
    
    def train(self):
        """PPO training step"""
        if len(self.memory.states) == 0:
            return None, None, None
        
        # Memory'den veri al
        states, actions, old_log_probs, rewards, values, dones = self.memory.get()
        
        # Son value'yu hesapla (bootstrap için)
        with torch.no_grad():
            last_state = torch.FloatTensor(states[-1]).unsqueeze(0).to(self.device)
            _, last_value = self.policy(last_state)
            last_value = last_value.item()
        
        # GAE hesapla
        advantages, returns = self.compute_gae(rewards, values, dones, last_value)
        
        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        # Tensörlere dönüştür
        states_tensor = torch.FloatTensor(states).to(self.device)
        actions_tensor = torch.LongTensor(actions).to(self.device)
        old_log_probs_tensor = torch.FloatTensor(old_log_probs).to(self.device)
        advantages_tensor = torch.FloatTensor(advantages).to(self.device)
        returns_tensor = torch.FloatTensor(returns).to(self.device)
        
        # PPO epochs
        total_policy_loss = 0
        total_value_loss = 0
        total_entropy = 0
        num_updates = 0
        
        for _ in range(self.ppo_epochs):
            # Mini-batch indices
            indices = np.arange(len(states))
            np.random.shuffle(indices)
            
            for start in range(0, len(states), self.batch_size):
                end = start + self.batch_size
                batch_indices = indices[start:end]
                
                # Batch data
                batch_states = states_tensor[batch_indices]
                batch_actions = actions_tensor[batch_indices]
                batch_old_log_probs = old_log_probs_tensor[batch_indices]
                batch_advantages = advantages_tensor[batch_indices]
                batch_returns = returns_tensor[batch_indices]
                
                # Evaluate actions
                new_log_probs, state_values, entropy = self.policy.evaluate_actions(
                    batch_states, batch_actions
                )
                
                # Ratio for PPO
                ratio = torch.exp(new_log_probs - batch_old_log_probs)
                
                # Clipped surrogate objective
                surr1 = ratio * batch_advantages
                surr2 = torch.clamp(ratio, 1.0 - self.clip_epsilon, 1.0 + self.clip_epsilon) * batch_advantages
                policy_loss = -torch.min(surr1, surr2).mean()
                
                # Value loss
                value_loss = F.mse_loss(state_values.squeeze(), batch_returns)
                
                # Entropy bonus
                entropy_loss = -entropy.mean()
                
                # Total loss
                loss = policy_loss + self.value_coef * value_loss + self.entropy_coef * entropy_loss
                
                # Optimize
                self.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
                self.optimizer.step()
                
                total_policy_loss += policy_loss.item()
                total_value_loss += value_loss.item()
                total_entropy += entropy.mean().item()
                num_updates += 1
        
        # Memory'yi temizle
        self.memory.clear()
        
        avg_policy_loss = total_policy_loss / num_updates
        avg_value_loss = total_value_loss / num_updates
        avg_entropy = total_entropy / num_updates
        
        return avg_policy_loss, avg_value_loss, avg_entropy
    
    def plan_path(self, start_pos: Tuple[float, float], goal_pos: Tuple[float, float],
                 engeller: List[Tuple[float, float, float]]):
        """
        A* ile yol planlama
        
        Args:
            start_pos: Başlangıç (x, y)
            goal_pos: Hedef (x, y)
            engeller: Engel listesi
        """
        # Engelleri güncelle
        self.a_star_planner.harita_guncelle(engeller)
        
        # Yol hesapla
        self.current_path = self.a_star_planner.planla(start_pos, goal_pos)
        self.path_index = 0
        
        return self.current_path
    
    def save_model(self, filepath: str):
        """Model'i kaydet"""
        torch.save({
            'policy': self.policy.state_dict(),
            'optimizer': self.optimizer.state_dict(),
            'training_stats': self.training_stats,
        }, filepath)
        print(f"✅ [Formasyon-PPO] Model kaydedildi: {filepath}")
    
    def load_model(self, filepath: str):
        """Model'i yükle"""
        if not os.path.exists(filepath):
            print(f"⚠️ [Formasyon-PPO] Model dosyası bulunamadı: {filepath}")
            return
        
        checkpoint = torch.load(filepath, map_location=self.device)
        self.policy.load_state_dict(checkpoint['policy'])
        self.optimizer.load_state_dict(checkpoint['optimizer'])
        self.training_stats = checkpoint.get('training_stats', self.training_stats)
        print(f"✅ [Formasyon-PPO] Model yüklendi: {filepath}")


# =============================================================================
# EĞİTİM FONKSİYONU
# =============================================================================
def train_formation_ppo(n_episodes: int = 1000, save_interval: int = 100):
    """
    Formasyon PPO modelini eğit
    
    Args:
        n_episodes: Episode sayısı
        save_interval: Model kaydetme aralığı
    """
    print("🚀 [Formasyon-PPO] Eğitim başlıyor...")
    
    # PPO agent'ı oluştur
    agent = FormasyonPPO(n_rovs=4, learning_rate=3e-4, gamma=0.99,
                        gae_lambda=0.95, clip_epsilon=0.2,
                        value_coef=0.5, entropy_coef=0.01)
    
    # Eğitim istatistikleri
    episode_rewards = []
    episode_policy_losses = []
    episode_value_losses = []
    
    for episode in range(n_episodes):
        # Simülasyon senaryosu oluştur
        n_rovs = np.random.randint(3, 7)
        n_obstacles = np.random.randint(10, 30)
        
        # Random başlangıç ve hedef
        start_pos = (np.random.uniform(-80, -40), np.random.uniform(-80, -40), 0)
        goal_pos = (np.random.uniform(40, 80), np.random.uniform(40, 80), 0)
        
        # Random engeller
        engeller = [(np.random.uniform(-90, 90), np.random.uniform(-90, 90), 0) 
                   for _ in range(n_obstacles)]
        
        # Yol planla
        agent.plan_path(start_pos[:2], goal_pos[:2], engeller)
        
        # Episode değişkenleri
        lider_pos = list(start_pos)
        takipci_pozisyonlari = [
            (lider_pos[0] + np.random.uniform(-5, 5), 
             lider_pos[1] - 5 * (i+1), 
             0) 
            for i in range(n_rovs - 1)
        ]
        
        episode_reward = 0
        step = 0
        max_steps = 500
        
        prev_formation = "V_SHAPE"
        
        while step < max_steps:
            # State oluştur
            state = agent.extract_state(tuple(lider_pos), takipci_pozisyonlari, 
                                       goal_pos, engeller, agent.current_formation_type)
            
            # Aksiyon seç (PPO policy)
            action, log_prob, value = agent.select_action(state, training=True)
            formation_type, gap = agent.decode_action(action)
            
            # Formasyon değişikliği kontrolü
            formation_changed = (formation_type != prev_formation)
            prev_formation = formation_type
            
            # Simülasyon adımı
            # Lider hareketi (A* yolu boyunca)
            if agent.path_index < len(agent.current_path):
                waypoint = agent.current_path[agent.path_index]
                dx = waypoint[0] - lider_pos[0]
                dy = waypoint[1] - lider_pos[1]
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist < 3.0:
                    agent.path_index += 1
                else:
                    step_size = min(2.0, dist)
                    lider_pos[0] += (dx / dist) * step_size
                    lider_pos[1] += (dy / dist) * step_size
            
            # Takipçi hareketi (formasyon)
            new_takipci_pozisyonlari = []
            for i in range(len(takipci_pozisyonlari)):
                offset = agent.get_formation_offset(i, formation_type, gap)
                hedef_x = lider_pos[0] + offset[0]
                hedef_y = lider_pos[1] + offset[1]
                
                # Takipçiyi hedefe doğru hareket ettir
                current = takipci_pozisyonlari[i]
                dx = hedef_x - current[0]
                dy = hedef_y - current[1]
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist > 0:
                    step_size = min(1.5, dist)
                    new_x = current[0] + (dx / dist) * step_size
                    new_y = current[1] + (dy / dist) * step_size
                    new_takipci_pozisyonlari.append((new_x, new_y, 0))
                else:
                    new_takipci_pozisyonlari.append(current)
            
            takipci_pozisyonlari = new_takipci_pozisyonlari
            
            # Çarpışma kontrolü
            collision = False
            for engel in engeller:
                dist = math.sqrt((lider_pos[0] - engel[0])**2 + (lider_pos[1] - engel[1])**2)
                if dist < 10.0:
                    collision = True
                    break
            
            # State info
            dist_to_goal = math.sqrt((lider_pos[0] - goal_pos[0])**2 + 
                                    (lider_pos[1] - goal_pos[1])**2)
            
            prev_state_info = {
                'dist_to_goal': dist_to_goal + 2.0,
                'formation_quality': 0.3,
                'min_obstacle_dist': 50.0
            }
            
            current_state_info = {
                'dist_to_goal': dist_to_goal,
                'formation_quality': 0.2,
                'min_obstacle_dist': min([math.sqrt((lider_pos[0] - e[0])**2 + 
                                                    (lider_pos[1] - e[1])**2) 
                                         for e in engeller]),
                'avg_follower_dist': np.mean([math.sqrt((t[0]-lider_pos[0])**2 + 
                                                        (t[1]-lider_pos[1])**2) 
                                             for t in takipci_pozisyonlari])
            }
            
            # Reward hesapla
            reward = agent.calculate_reward(prev_state_info, current_state_info, 
                                           formation_changed, collision)
            
            # Bitti mi?
            done = dist_to_goal < 5.0 or collision or step >= max_steps - 1
            
            # Memory'ye ekle
            agent.memory.add(state, action, log_prob, reward, value, done)
            
            episode_reward += reward
            step += 1
            
            agent.current_formation_type = formation_type
            agent.current_gap = gap
            
            if done:
                break
        
        # Episode sonunda PPO güncelleme
        policy_loss, value_loss, entropy = agent.train()
        
        # Episode bitişi
        episode_rewards.append(episode_reward)
        if policy_loss is not None:
            episode_policy_losses.append(policy_loss)
            episode_value_losses.append(value_loss)
        
        agent.training_stats['episode'] = episode + 1
        agent.training_stats['total_reward'] = episode_reward
        
        # Log
        if (episode + 1) % 10 == 0:
            avg_reward = np.mean(episode_rewards[-10:])
            avg_policy_loss = np.mean(episode_policy_losses[-10:]) if episode_policy_losses else 0
            avg_value_loss = np.mean(episode_value_losses[-10:]) if episode_value_losses else 0
            print(f"📊 Episode {episode+1}/{n_episodes} | "
                  f"Avg Reward: {avg_reward:.2f} | "
                  f"Policy Loss: {avg_policy_loss:.4f} | "
                  f"Value Loss: {avg_value_loss:.4f}")
        
        # Model kaydet
        if (episode + 1) % save_interval == 0:
            model_path = f"formasyon_ppo_model_ep{episode+1}.pth"
            agent.save_model(model_path)
    
    # Final model
    agent.save_model("formasyon_ppo_model_final.pth")
    print("✅ [Formasyon-PPO] Eğitim tamamlandı!")
    
    return agent


# =============================================================================
# MAIN
# =============================================================================
if __name__ == "__main__":
    print("🎯 Formasyon PPO Eğitim Sistemi")
    print("=" * 60)
    
    # Eğitimi başlat
    trained_agent = train_formation_ppo(n_episodes=1000, save_interval=100)
    
    print("\n✅ Tüm işlemler tamamlandı!")
