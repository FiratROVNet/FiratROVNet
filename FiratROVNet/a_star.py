"""
A* (A-Star) Yol Bulma Algoritması - NetworkX Kullanarak

NetworkX kütüphanesinin A* algoritmasını kullanır.
10x10 grid, dairesel engeller (adalar).
"""

import math
import networkx as nx
from typing import List, Tuple, Optional


class AStarPlanner:
    """NetworkX kullanan A* planlayıcı - 10x10 grid"""
    
    GRID_SIZE = 10  # 10x10 grid
    
    def __init__(self):
        pass
    
    def _world_to_grid(self, world_x: float, world_y: float, 
                       map_bounds: Tuple[float, float, float, float]) -> Tuple[int, int]:
        """Dünya koordinatlarını grid koordinatlarına çevir"""
        min_x, max_x, min_y, max_y = map_bounds
        grid_x = int((world_x - min_x) / (max_x - min_x) * self.GRID_SIZE)
        grid_y = int((world_y - min_y) / (max_y - min_y) * self.GRID_SIZE)
        # Sınırları kontrol et
        grid_x = max(0, min(self.GRID_SIZE - 1, grid_x))
        grid_y = max(0, min(self.GRID_SIZE - 1, grid_y))
        return grid_x, grid_y
    
    def _grid_to_world(self, grid_x: int, grid_y: int,
                       map_bounds: Tuple[float, float, float, float]) -> Tuple[float, float]:
        """Grid koordinatlarını dünya koordinatlarına çevir"""
        min_x, max_x, min_y, max_y = map_bounds
        world_x = min_x + (grid_x + 0.5) * (max_x - min_x) / self.GRID_SIZE
        world_y = min_y + (grid_y + 0.5) * (max_y - min_y) / self.GRID_SIZE
        return world_x, world_y
    
    def _create_graph(self, obstacles: List[Tuple[float, float, float]],
                     map_bounds: Tuple[float, float, float, float]) -> nx.Graph:
        """
        Grid'i NetworkX graph'ına çevir.
        Engel olan hücreler graph'a eklenmez (içinden geçilemez).
        """
        G = nx.Graph()
        min_x, max_x, min_y, max_y = map_bounds
        
        # Engel haritası oluştur
        obstacle_map = [[False] * self.GRID_SIZE for _ in range(self.GRID_SIZE)]
        
        for grid_y in range(self.GRID_SIZE):
            for grid_x in range(self.GRID_SIZE):
                world_x, world_y = self._grid_to_world(grid_x, grid_y, map_bounds)
                
                # Her engel (ada) için kontrol et
                for obs_x, obs_y, obs_radius in obstacles:
                    dist = math.sqrt((world_x - obs_x)**2 + (world_y - obs_y)**2)
                    if dist <= obs_radius*1.2:
                        obstacle_map[grid_y][grid_x] = True
                        break
        
        # Geçerli hücreleri graph'a ekle
        for grid_y in range(self.GRID_SIZE):
            for grid_x in range(self.GRID_SIZE):
                if not obstacle_map[grid_y][grid_x]:
                    node = (grid_x, grid_y)
                    G.add_node(node)
                    
                    # 8 yönlü komşuları ekle
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            
                            neighbor_x = grid_x + dx
                            neighbor_y = grid_y + dy
                            
                            # Sınır kontrolü
                            if (0 <= neighbor_x < self.GRID_SIZE and 
                                0 <= neighbor_y < self.GRID_SIZE and
                                not obstacle_map[neighbor_y][neighbor_x]):
                                
                                neighbor = (neighbor_x, neighbor_y)
                                
                                # Hareket maliyeti: düz = 1.0, diyagonal = sqrt(2)
                                if dx == 0 or dy == 0:
                                    weight = 1.0
                                else:
                                    weight = math.sqrt(2.0)
                                
                                # Edge ekle (zaten varsa eklemez)
                                if not G.has_edge(node, neighbor):
                                    G.add_edge(node, neighbor, weight=weight)
        
        return G
    
    def _heuristic(self, node1: Tuple[int, int], node2: Tuple[int, int]) -> float:
        """Euclidean mesafe heuristic (NetworkX için)"""
        x1, y1 = node1
        x2, y2 = node2
        dx = x2 - x1
        dy = y2 - y1
        return math.sqrt(dx * dx + dy * dy)
    
    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float],
                  obstacles: List[Tuple[float, float, float]],
                  map_bounds: Tuple[float, float, float, float],
                  safety_margin: float = 0.0) -> Optional[List[Tuple[float, float]]]:
        """
        NetworkX A* ile yol bul
        
        Args:
            start: (x, y) başlangıç
            goal: (x, y) hedef
            obstacles: [(x, y, radius), ...] dairesel engeller (adalar)
            map_bounds: (min_x, max_x, min_y, max_y)
            safety_margin: Kullanılmıyor (sadeleştirilmiş versiyon)
        
        Returns:
            Yol listesi veya None
        """
        # Grid koordinatlarına çevir
        start_grid = self._world_to_grid(start[0], start[1], map_bounds)
        goal_grid = self._world_to_grid(goal[0], goal[1], map_bounds)
        
        # Graph oluştur
        G = self._create_graph(obstacles, map_bounds)
        
        # Başlangıç ve hedef node'ları geçerli mi?
        if start_grid not in G:
            print(f"⚠️ [A*] Başlangıç engel üzerinde: {start}")
            return None
        
        if goal_grid not in G:
            print(f"⚠️ [A*] Hedef engel üzerinde: {goal}")
            return None
        
        try:
            # NetworkX A* algoritması kullan
            path_grid = nx.astar_path(
                G, 
                start_grid, 
                goal_grid,
                heuristic=self._heuristic,
                weight='weight'
            )
            
            # Grid koordinatlarını dünya koordinatlarına çevir
            path_world = []
            for grid_node in path_grid:
                world_x, world_y = self._grid_to_world(grid_node[0], grid_node[1], map_bounds)
                path_world.append((world_x, world_y))
            
            print(f"✅ [A*] Yol bulundu! {len(path_world)} nokta (NetworkX)")
            return path_world
            
        except nx.NetworkXNoPath:
            print(f"❌ [A*] Yol bulunamadı! (NetworkX)")
            return None
        except Exception as e:
            print(f"❌ [A*] Hata: {e}")
            return None
