"""
RL-A* (Reinforcement Learning Enhanced A-Star)

Bu modül, Q-Learning kullanarak haritayı tanıyan ve öğrendiği değerleri
A* algoritmasının sezgisel (heuristic) fonksiyonu olarak kullanan hibrid
bir yol bulucuyu implement eder.
"""

import numpy as np
import math
import random
from typing import List, Tuple, Optional, Set, Dict
from heapq import heappush, heappop

class RLNode:
    """A* düğüm sınıfı (Değişmedi)"""
    def __init__(self, x: int, y: int, g_cost: float = 0, h_cost: float = 0, parent=None):
        self.x = x
        self.y = y
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))


class RLAStarPlanner:
    """
    Q-Learning ile güçlendirilmiş A* Planlayıcı.
    """
    
    def __init__(self, grid_size: float = 1.0):
        self.grid_size = grid_size
        # Q-Learning Parametreleri
        self.q_table = {}  # (x, y) -> {action_index: q_value}
        self.learning_rate = 0.1 # Alpha
        self.discount_factor = 0.95 # Gamma
        self.epsilon = 0.1 # Keşif oranı
        self.actions = [ # 8 yönlü hareket
            (-1, -1), (0, -1), (1, -1),
            (-1, 0),           (1, 0),
            (-1, 1),  (0, 1),  (1, 1)
        ]
        
    def _get_state_key(self, x: int, y: int) -> Tuple[int, int]:
        """Q-Table için state anahtarı"""
        return (x, y)

    def _get_q_value(self, x: int, y: int, action_idx: int) -> float:
        """Belirli bir durum ve eylem için Q değerini döndürür"""
        state = self._get_state_key(x, y)
        if state not in self.q_table:
            # Başlangıçta iyimser veya nötr değerler atayabiliriz
            self.q_table[state] = np.zeros(len(self.actions))
        return self.q_table[state][action_idx]

    def _set_q_value(self, x: int, y: int, action_idx: int, value: float):
        """Q değerini günceller"""
        state = self._get_state_key(x, y)
        if state not in self.q_table:
            self.q_table[state] = np.zeros(len(self.actions))
        self.q_table[state][action_idx] = value

    def _get_max_q(self, x: int, y: int) -> float:
        """Bir durumdaki en yüksek Q değerini döndürür (Best Value)"""
        state = self._get_state_key(x, y)
        if state not in self.q_table:
            return 0.0
        return np.max(self.q_table[state])

    def train_agent(self, start: Tuple[float, float], goal: Tuple[float, float],
                    obstacles: List[Tuple[float, float, float]],
                    map_bounds: Tuple[float, float, float, float],
                    episodes: int = 1000):
        """
        RL Ajanını eğiterek haritayı 'öğrenmesini' sağlar.
        Bu süreç Q-Tablosunu doldurur.
        """
        print(f"🤖 [RL] Ajan eğitimi başlıyor ({episodes} bölüm)...")
        
        # Grid hazırlığı
        obstacle_map = self._create_obstacle_map(obstacles, map_bounds)
        min_x, max_x, min_y, max_y = map_bounds
        grid_width = obstacle_map.shape[1]
        grid_height = obstacle_map.shape[0]
        
        goal_grid = self._world_to_grid(goal[0], goal[1], map_bounds)
        
        for episode in range(episodes):
            # Her bölümde başlangıç noktasından veya rastgele bir yerden başlayabiliriz
            # Rastgele başlamak tüm haritayı öğrenmek için daha iyidir
            if episode % 10 == 0:
                current_grid = self._world_to_grid(start[0], start[1], map_bounds)
            else:
                current_grid = (random.randint(0, grid_width-1), random.randint(0, grid_height-1))
                # Engele denk gelirse tekrar seç
                while not self._is_valid(current_grid[0], current_grid[1], obstacle_map, map_bounds):
                     current_grid = (random.randint(0, grid_width-1), random.randint(0, grid_height-1))

            steps = 0
            max_steps = 200 # Sonsuz döngüyü önle
            
            while steps < max_steps:
                cx, cy = current_grid
                
                # Hedefe ulaşıldı mı?
                if (cx, cy) == goal_grid:
                    break
                
                # Eylem Seçimi (Epsilon-Greedy)
                if random.random() < self.epsilon:
                    action_idx = random.randint(0, len(self.actions) - 1)
                else:
                    # En iyi bilinen eylemi seç
                    state_vals = self._get_state_vals(cx, cy)
                    action_idx = np.argmax(state_vals)
                
                dx, dy = self.actions[action_idx]
                nx, ny = cx + dx, cy + dy
                
                # Ödül Hesaplama
                reward = -1.0 # Her adım maliyeti (kısa yolu teşvik eder)
                
                # Geçerlilik kontrolü
                is_valid = self._is_valid(nx, ny, obstacle_map, map_bounds)
                
                if not is_valid:
                    reward = -100.0 # Engel cezası
                    next_max_q = -100.0 # Duvarın değeri kötüdür
                    # Konum değişmez
                    nx, ny = cx, cy
                elif (nx, ny) == goal_grid:
                    reward = 100.0 # Hedef ödülü
                    next_max_q = 0.0 # Hedefin ötesinde maliyet yok
                else:
                    next_max_q = self._get_max_q(nx, ny)
                
                # Q-Learning Formülü: Q(s,a) = Q(s,a) + alpha * [R + gamma * max(Q(s',a')) - Q(s,a)]
                current_q = self._get_q_value(cx, cy, action_idx)
                new_q = current_q + self.learning_rate * (reward + self.discount_factor * next_max_q - current_q)
                self._set_q_value(cx, cy, action_idx, new_q)
                
                # Durum güncelle
                if is_valid:
                    current_grid = (nx, ny)
                
                steps += 1
                
        print("✅ [RL] Eğitim tamamlandı.")

    def _get_state_vals(self, x, y):
        state = self._get_state_key(x, y)
        if state not in self.q_table:
            self.q_table[state] = np.zeros(len(self.actions))
        return self.q_table[state]

    def _learned_heuristic(self, x: int, y: int, goal_x: int, goal_y: int) -> float:
        """
        RL tabanlı heuristic fonksiyonu.
        Normalde A* f = g + h (maliyet) kullanır.
        Q-değerleri ödül (reward) bazlıdır (hedefe yaklaşınca artar, uzadıkça azalır).
        
        Maliyete dönüştürmek için:
        Q değeri ne kadar yüksekse (hedefe o kadar yakın/ödül büyük), maliyet o kadar düşük olmalı.
        
        Burada hibrit bir yaklaşım kullanıyoruz:
        Eğer Q-Tablosunda bu nokta için bilgi varsa onu kullan, yoksa Öklid mesafesini kullan.
        """
        max_q = self._get_max_q(x, y)
        
        # Eğer Q değeri çok düşükse (henüz keşfedilmemiş veya engel), standart heuristic kullan
        if max_q == 0: 
             return math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
        
        # Q değerini maliyete dönüştür. 
        # Hedefteki max Q değeri 0 veya pozitif olabilir. Adımlar negatif.
        # En kısa yol en az negatif (en yüksek) puana sahiptir.
        # A* düşük maliyet arar. Dolayısıyla h = -Q (kabaca)
        # Ölçekleme faktörü ekleyebiliriz.
        return -max_q * 1.5 # 1.5 katsayısı RL'e güveni artırır (Weighted A*)

    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float],
                  obstacles: List[Tuple[float, float, float]],
                  map_bounds: Tuple[float, float, float, float],
                  train_episodes: int = 2000) -> Optional[List[Tuple[float, float]]]:
        
        start_grid = self._world_to_grid(start[0], start[1], map_bounds)
        goal_grid = self._world_to_grid(goal[0], goal[1], map_bounds)
        obstacle_map = self._create_obstacle_map(obstacles, map_bounds)
        
        # 1. Aşama: RL Ajanını Eğit
        # Not: Gerçek uygulamalarda eğitim bir kez yapılır ve kaydedilir.
        self.train_agent(start, goal, obstacles, map_bounds, episodes=train_episodes)
        
        # 2. Aşama: A* Algoritması (Öğrenilmiş Heuristic ile)
        if not self._is_valid(start_grid[0], start_grid[1], obstacle_map, map_bounds):
            print("Başlangıç noktası geçersiz.")
            return None

        open_set = []
        closed_set = set()
        
        # Başlangıç düğümü - Heuristic olarak RL fonksiyonunu kullanıyoruz
        h_start = self._learned_heuristic(start_grid[0], start_grid[1], goal_grid[0], goal_grid[1])
        start_node = RLNode(start_grid[0], start_grid[1], g_cost=0.0, h_cost=h_start)
        heappush(open_set, start_node)
        
        iteration = 0
        while open_set:
            iteration += 1
            current = heappop(open_set)
            
            if current.x == goal_grid[0] and current.y == goal_grid[1]:
                path = []
                while current:
                    wx, wy = self._grid_to_world(current.x, current.y, map_bounds)
                    path.append((wx, wy))
                    current = current.parent
                path.reverse()
                print(f"✅ [RL-A*] Yol bulundu! Adım: {len(path)}, A* İterasyon: {iteration}")
                return path
            
            closed_set.add((current.x, current.y))
            
            for dx, dy in [(-1,-1), (0,-1), (1,-1), (-1,0), (1,0), (-1,1), (0,1), (1,1)]:
                nx, ny = current.x + dx, current.y + dy
                
                if not self._is_valid(nx, ny, obstacle_map, map_bounds) or (nx, ny) in closed_set:
                    continue
                
                # Hareket maliyeti
                move_cost = math.sqrt(2) if dx != 0 and dy != 0 else 1.0
                new_g = current.g_cost + move_cost
                
                # RL Heuristic kullanımı
                new_h = self._learned_heuristic(nx, ny, goal_grid[0], goal_grid[1])
                
                neighbor = RLNode(nx, ny, new_g, new_h, current)
                
                # Basitleştirilmiş open_set kontrolü (daha iyi performans için hash map eklenebilir)
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

    # --- Yardımcı Metodlar (Orijinal Koddan Alındı ve Basitleştirildi) ---
    def _world_to_grid(self, wx, wy, bounds):
        return int((wx - bounds[0])/self.grid_size), int((wy - bounds[2])/self.grid_size)
    
    def _grid_to_world(self, gx, gy, bounds):
        return bounds[0] + (gx + 0.5)*self.grid_size, bounds[2] + (gy + 0.5)*self.grid_size

    def _is_valid(self, x, y, obs_map, bounds):
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

# --- Test Kodu ---
if __name__ == "__main__":
    # Harita Sınırları: (min_x, max_x, min_y, max_y)
    bounds = (0, 20, 0, 20)
    
    # U şeklinde bir engel (Tuzak)
    # A* normalde bu tuzağın içine girer ve sonra çıkar. 
    # RL önceden öğrenirse tuzağın etrafından dolaşmayı heuristic olarak bilecektir.
    obstacles = [
        (10, 5, 1), (10, 6, 1), (10, 7, 1), (10, 8, 1), # Sol duvar
        (10, 4, 1), (11, 4, 1), (12, 4, 1), (13, 4, 1), # Alt duvar
        (13, 5, 1), (13, 6, 1), (13, 7, 1), (13, 8, 1)  # Sağ duvar
    ]
    
    start_pos = (5, 6)
    goal_pos = (18, 6)
    
    planner = RLAStarPlanner(grid_size=1.0)
    
    # Önce RL ajanı çevreyi öğrenir, sonra A* yol bulur
    path = planner.find_path(start_pos, goal_pos, obstacles, bounds, train_episodes=5000)
    
    if path:
        print(f"Yol noktaları (ilk 5): {path[:5]}...")