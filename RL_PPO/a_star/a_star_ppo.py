"""
PPO-A* (Proximal Policy Optimization Enhanced A-Star)

Bu modül, PyTorch kullanarak bir Actor-Critic sinir ağı eğitir.
Critic ağının öğrendiği 'Value' (Değer) fonksiyonu, A* algoritmasının
sezgisel (heuristic) fonksiyonu olarak kullanılır.
"""

import numpy as np
import math
import random
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Categorical
from heapq import heappush, heappop
from typing import List, Tuple, Optional, Set

# --- 1. SİNİR AĞI MODÜLLERİ (PPO BEYNİ) ---

class ActorCritic(nn.Module):
    """
    Hem aksiyon seçen (Actor) hem de durumu değerlendiren (Critic) ağ.
    Input: [x, y, hedef_x, hedef_y, mesafe]
    """
    def __init__(self, input_dim, action_dim):
        super(ActorCritic, self).__init__()
        
        # Ortak katmanlar (Haritayı anlama)
        self.feature_layer = nn.Sequential(
            nn.Linear(input_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh()
        )
        
        # Actor: Hangi yöne gitmeli? (Policy)
        self.actor = nn.Sequential(
            nn.Linear(64, action_dim),
            nn.Softmax(dim=-1)
        )
        
        # Critic: Burası hedefe ne kadar yakın/iyi? (Value)
        # Bu değer A* için Heuristic olacak.
        self.critic = nn.Linear(64, 1)
        
    def forward(self, x):
        features = self.feature_layer(x)
        return self.actor(features), self.critic(features)

    def get_value(self, x):
        features = self.feature_layer(x)
        return self.critic(features)


# --- 2. PPO AGENT ---

class PPOAgent:
    def __init__(self, input_dim, action_dim, lr=0.002, gamma=0.99, eps_clip=0.2, k_epochs=4):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.k_epochs = k_epochs
        
        self.policy = ActorCritic(input_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.policy_old = ActorCritic(input_dim, action_dim)
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        self.MseLoss = nn.MSELoss()
    
    def select_action(self, state):
        with torch.no_grad():
            state = torch.FloatTensor(state)
            probs, val = self.policy_old(state)
            m = Categorical(probs)
            action = m.sample()
            return action.item(), m.log_prob(action), val.item()

    def update(self, memory):
        # Hafızadaki verileri tensörlere çevir
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(memory.rewards), reversed(memory.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
            
        rewards = torch.tensor(rewards, dtype=torch.float32)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-5) # Normalize et
        
        old_states = torch.tensor(np.array(memory.states), dtype=torch.float32)
        old_actions = torch.tensor(memory.actions, dtype=torch.float32)
        old_logprobs = torch.tensor(memory.logprobs, dtype=torch.float32)
        
        # K Epochs boyunca ağı güncelle
        for _ in range(self.k_epochs):
            probs, state_values = self.policy(old_states)
            dist = Categorical(probs)
            
            # Yeni log_probs
            logprobs = dist.log_prob(old_actions)
            dist_entropy = dist.entropy()
            state_values = torch.squeeze(state_values)
            
            # Ratio (r_t)
            ratios = torch.exp(logprobs - old_logprobs)
            
            # Surrogate Loss
            advantages = rewards - state_values.detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, rewards) - 0.01 * dist_entropy
            
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
            
        self.policy_old.load_state_dict(self.policy.state_dict())

class Memory:
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []
    
    def clear(self):
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]


# --- 3. PPO İLE GÜÇLENDİRİLMİŞ A* PLANLAYICI ---

class AStarNode:
    """Orijinal düğüm yapısı"""
    def __init__(self, x: int, y: int, g_cost: float = 0, h_cost: float = 0, parent=None):
        self.x = x
        self.y = y
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost


class PPOAStarPlanner:
    def __init__(self, grid_size: float = 1.0):
        self.grid_size = grid_size
        # State: [rel_x, rel_y, dist_norm, is_obstacle_around]
        self.input_dim = 4 
        self.actions = [ # 8 Yön
            (-1, -1), (0, -1), (1, -1),
            (-1, 0),           (1, 0),
            (-1, 1),  (0, 1),  (1, 1)
        ]
        self.agent = PPOAgent(self.input_dim, len(self.actions))
        
    def _get_state(self, x, y, goal_x, goal_y, obstacle_map, bounds):
        """Sinir ağı için durumu vektöre çevirir"""
        # Normalizasyon faktörü (harita büyüklüğü tahmini)
        max_dist = max(bounds[1], bounds[3])
        
        dx = (goal_x - x) / max_dist
        dy = (goal_y - y) / max_dist
        dist = math.sqrt(dx**2 + dy**2)
        
        # Basit bir engel sensörü (etrafında engel var mı?)
        # 8 komşudan kaçı engel? (0.0 - 1.0 arası)
        blocked_count = 0
        h, w = obstacle_map.shape
        for mx, my in self.actions:
            nx, ny = x + mx, y + my
            if nx < 0 or ny < 0 or nx >= w or ny >= h or obstacle_map[ny, nx]:
                blocked_count += 1
        obstacle_sense = blocked_count / 8.0
        
        return np.array([dx, dy, dist, obstacle_sense])

    def train_ppo(self, start: Tuple[float, float], goal: Tuple[float, float],
                 obstacles: List[Tuple[float, float, float]],
                 map_bounds: Tuple[float, float, float, float],
                 max_episodes: int = 500):
        
        print(f"🧠 [PPO] Eğitim Başlıyor ({max_episodes} bölüm)...")
        
        obstacle_map = self._create_obstacle_map(obstacles, map_bounds)
        start_grid = self._world_to_grid(start[0], start[1], map_bounds)
        goal_grid = self._world_to_grid(goal[0], goal[1], map_bounds)
        
        memory = Memory()
        update_timestep = 200
        timestep = 0
        
        min_x, max_x, min_y, max_y = map_bounds
        w, h = obstacle_map.shape[1], obstacle_map.shape[0]
        
        for ep in range(max_episodes):
            # Rastgele başlangıç (Curriculum Learning için) veya sabit başlangıç
            if ep % 5 == 0:
                current_x, current_y = start_grid
            else:
                current_x, current_y = random.randint(0, w-1), random.randint(0, h-1)
                while obstacle_map[current_y, current_x]:
                    current_x, current_y = random.randint(0, w-1), random.randint(0, h-1)
            
            ep_reward = 0
            
            for t in range(300): # Max steps per episode
                timestep += 1
                
                # Mevcut durum
                state = self._get_state(current_x, current_y, goal_grid[0], goal_grid[1], obstacle_map, map_bounds)
                
                # Aksiyon seç
                action_idx, log_prob, val = self.agent.select_action(state)
                dx, dy = self.actions[action_idx]
                
                nx, ny = current_x + dx, current_y + dy
                
                # Ödül Mekanizması
                done = False
                reward = 0
                
                # Mesafe bazlı ödül (Shaping)
                dist_old = math.sqrt((current_x - goal_grid[0])**2 + (current_y - goal_grid[1])**2)
                dist_new = math.sqrt((nx - goal_grid[0])**2 + (ny - goal_grid[1])**2)
                
                # Geçersiz hareket / Engel
                if nx < 0 or ny < 0 or nx >= w or ny >= h or obstacle_map[ny, nx]:
                    reward = -10 # Çarpma cezası
                    nx, ny = current_x, current_y # Hareket etme
                    # done = True # İstersek bölümü bitirebiliriz ama öğrenmesi için devam etsin
                
                # Hedef
                elif (nx, ny) == goal_grid:
                    reward = 100
                    done = True
                
                # Hedefe yaklaştı mı?
                else:
                    reward = (dist_old - dist_new) * 10 - 0.1 # Adım maliyeti
                
                # Hafızaya kaydet
                memory.states.append(state)
                memory.actions.append(action_idx)
                memory.logprobs.append(log_prob)
                memory.rewards.append(reward)
                memory.is_terminals.append(done)
                
                current_x, current_y = nx, ny
                ep_reward += reward
                
                # PPO Güncellemesi
                if timestep % update_timestep == 0:
                    self.agent.update(memory)
                    memory.clear()
                    timestep = 0
                
                if done:
                    break
            
            if (ep+1) % 50 == 0:
                print(f"Epizot: {ep+1}, Son Ödül: {ep_reward:.2f}")

        print("✅ [PPO] Eğitim Tamamlandı.")

    def _ppo_heuristic(self, x, y, goal_x, goal_y, obstacle_map, bounds):
        """
        A* için Heuristic Fonksiyonu.
        Sinir ağının 'Critic' çıktısını kullanır.
        Critic 'Value' (Değer) döndürür: Yüksek değer = İyi durum (Hedefe yakın).
        A* ise 'Cost' (Maliyet) ister: Düşük maliyet = İyi durum.
        Bu yüzden: Heuristic = -Value (Negatif Değer)
        """
        state = self._get_state(x, y, goal_x, goal_y, obstacle_map, bounds)
        state_tensor = torch.FloatTensor(state)
        
        with torch.no_grad():
            value = self.agent.policy.get_value(state_tensor).item()
        
        # PPO Value genellikle normalize edilmiştir veya ödül skalasındadır.
        # Bunu A* için pozitif bir uzaklık maliyetine çevirmeliyiz.
        # Negatif value kullanıyoruz ve scale ediyoruz.
        return -value * 2.0 

    def find_path(self, start, goal, obstacles, map_bounds):
        start_grid = self._world_to_grid(start[0], start[1], map_bounds)
        goal_grid = self._world_to_grid(goal[0], goal[1], map_bounds)
        obstacle_map = self._create_obstacle_map(obstacles, map_bounds)
        
        if not self._is_valid(start_grid[0], start_grid[1], obstacle_map):
            print("Geçersiz Başlangıç")
            return None

        # A* Başlat
        open_set = []
        closed_set = set()
        
        # Başlangıç Heuristic: PPO Network'ten al
        h_start = self._ppo_heuristic(start_grid[0], start_grid[1], goal_grid[0], goal_grid[1], obstacle_map, map_bounds)
        
        start_node = AStarNode(start_grid[0], start_grid[1], 0, h_start)
        heappush(open_set, start_node)
        
        iters = 0
        max_iters = 5000
        
        while open_set and iters < max_iters:
            iters += 1
            current = heappop(open_set)
            
            if current.x == goal_grid[0] and current.y == goal_grid[1]:
                path = []
                while current:
                    wx, wy = self._grid_to_world(current.x, current.y, map_bounds)
                    path.append((wx, wy))
                    current = current.parent
                return path[::-1]
            
            closed_set.add((current.x, current.y))
            
            for i, (dx, dy) in enumerate(self.actions):
                nx, ny = current.x + dx, current.y + dy
                
                if not self._is_valid(nx, ny, obstacle_map) or (nx, ny) in closed_set:
                    continue
                
                move_cost = math.sqrt(2) if dx != 0 and dy != 0 else 1.0
                new_g = current.g_cost + move_cost
                
                # PPO Heuristic
                new_h = self._ppo_heuristic(nx, ny, goal_grid[0], goal_grid[1], obstacle_map, map_bounds)
                
                neighbor = AStarNode(nx, ny, new_g, new_h, current)
                
                # Basitleştirilmiş open set kontrolü
                in_open = False
                for node in open_set:
                    if node.x == nx and node.y == ny:
                        if new_g < node.g_cost:
                            node.g_cost = new_g
                            node.f_cost = new_g + node.h_cost
                            node.parent = current
                        in_open = True
                        break
                
                if not in_open:
                    heappush(open_set, neighbor)
                    
        return None

    # --- Yardımcılar ---
    def _world_to_grid(self, wx, wy, bounds):
        return int((wx - bounds[0])/self.grid_size), int((wy - bounds[2])/self.grid_size)
    
    def _grid_to_world(self, gx, gy, bounds):
        return bounds[0] + (gx + 0.5)*self.grid_size, bounds[2] + (gy + 0.5)*self.grid_size
        
    def _is_valid(self, x, y, obs_map):
        if x < 0 or y < 0 or x >= obs_map.shape[1] or y >= obs_map.shape[0]: return False
        return not obs_map[y, x]

    def _create_obstacle_map(self, obstacles, bounds, margin=2.0):
        w = int((bounds[1] - bounds[0]) / self.grid_size) + 1
        h = int((bounds[3] - bounds[2]) / self.grid_size) + 1
        obs_map = np.zeros((h, w), dtype=bool)
        for ox, oy, r in obstacles:
            gx, gy = self._world_to_grid(ox, oy, bounds)
            gr = int((r + margin) / self.grid_size) + 1
            for i in range(-gr, gr+1):
                for j in range(-gr, gr+1):
                    nx, ny = gx+i, gy+j
                    if 0 <= nx < w and 0 <= ny < h:
                        wx, wy = self._grid_to_world(nx, ny, bounds)
                        if math.sqrt((wx-ox)**2 + (wy-oy)**2) <= r + margin:
                            obs_map[ny, nx] = True
        return obs_map

# --- GÖRSELLEŞTİRME VE TEST ---
import matplotlib.pyplot as plt

def visualize_path(path, obstacles, start, goal, bounds):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(bounds[0], bounds[1])
    ax.set_ylim(bounds[2], bounds[3])
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)
    
    for obs in obstacles:
        ax.add_patch(plt.Circle((obs[0], obs[1]), obs[2], color='red', alpha=0.5))
        ax.add_patch(plt.Circle((obs[0], obs[1]), obs[2]+2.0, color='red', alpha=0.1))

    ax.plot(start[0], start[1], 'go', markersize=10, label='Başlangıç')
    ax.plot(goal[0], goal[1], 'bo', markersize=10, label='Hedef')
    
    if path:
        px, py = zip(*path)
        ax.plot(px, py, 'g-', linewidth=2, label='PPO-A* Yolu')
        ax.set_title("PPO (Neural Net) Destekli A*")
    else:
        ax.set_title("Yol Bulunamadı")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    bounds = (0, 30, 0, 30)
    # U Şeklinde Engel
    obstacles = []
    for y in range(10, 20): obstacles.append((10, y, 0.5))
    for x in range(10, 20): obstacles.append((x, 10, 0.5))
    for y in range(10, 20): obstacles.append((20, y, 0.5))
    
    start_pos = (15, 15)
    goal_pos = (25, 25)
    
    planner = PPOAStarPlanner(grid_size=1.0)
    
    # 1. PPO Eğitimi
    planner.train_ppo(start_pos, goal_pos, obstacles, bounds, max_episodes=1000)
    
    # 2. A* ile Yol Bulma (Heuristic olarak PPO Critic ağı kullanılır)
    path = planner.find_path(start_pos, goal_pos, obstacles, bounds)
    
    visualize_path(path, obstacles, start_pos, goal_pos, bounds)