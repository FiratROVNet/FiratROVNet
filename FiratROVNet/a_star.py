"""
A* (A-Star) Yol Bulma Algoritması - NetworkX Kullanarak

NetworkX kütüphanesinin A* algoritmasını kullanır.
Grid çözünürlüğü 10x10, dairesel engeller (adalar) için
vektörel (line-segment) çarpışma kontrolü kullanır.
"""

import math
import networkx as nx
from typing import List, Tuple, Optional


class AStarPlanner:
    """NetworkX kullanan A* planlayıcı - 10x10 grid (Vektör tabanlı engel kontrolü)"""
    
    GRID_SIZE = 5  # 10x10 grid (hücreler büyük, ama kenar kontrolü hassas)
    
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

    def _segment_intersects_circle(self, p1: Tuple[float, float], p2: Tuple[float, float], 
                                 circle: Tuple[float, float, float]) -> bool:
        """
        Line segment p1-p2 vs Circle (x, y, r).
        True if intersection or inside.
        """
        x1, y1 = p1
        x2, y2 = p2
        cx, cy, r = circle
        
        # Safety margin multiplier for radius
        # Daha güvenli olması için hem çarpan hem de sabit tampon ekliyoruz
        r_safe = r * 1.3 + 10.0  # %30 + 10 metre güvenlik payı
        
        dx, dy = x2 - x1, y2 - y1
        if dx == 0 and dy == 0:
            return math.hypot(cx - x1, cy - y1) <= r_safe

        # Project circle center onto line containing segment
        # t = dot(CP1, P1P2) / |P1P2|^2
        t = ((cx - x1) * dx + (cy - y1) * dy) / (dx*dx + dy*dy)
        t = max(0, min(1, t))
        
        # Closest point on segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        # Distance from closest point to center
        dist = math.hypot(closest_x - cx, closest_y - cy)
        
        return dist <= r_safe
    
    def _create_graph(self, obstacles: List[Tuple[float, float, float]],
                     map_bounds: Tuple[float, float, float, float]) -> nx.Graph:
        """
        Grid'i NetworkX graph'ına çevir.
        
        Strateji:
        1. Önce düğümlerin (hücre merkezleri) güvenli olup olmadığını kontrol et.
        2. Sonra komşu düğümler arasındaki kenarları (vektörleri) kontrol et.
           Eğer vektör bir engelin içinden geçiyorsa, o kenarı ekleme.
        """
        G = nx.Graph()
        
        # 1. Geçerli düğümleri belirle (merkezi engel içinde olmayanlar)
        valid_nodes = set()
        for grid_y in range(self.GRID_SIZE):
            for grid_x in range(self.GRID_SIZE):
                world_pos = self._grid_to_world(grid_x, grid_y, map_bounds)
                
                # Check if node center is safe
                is_safe = True
                for obs in obstacles:
                    ox, oy, orad = obs
                    # Check if point is inside (with margin)
                    # Node güvenliği için de aynı marjı kullanalım
                    r_safe_node = orad * 1.3 + 10.0
                    if math.hypot(world_pos[0]-ox, world_pos[1]-oy) <= r_safe_node:
                        is_safe = False
                        break
                
                if is_safe:
                    G.add_node((grid_x, grid_y))
                    valid_nodes.add((grid_x, grid_y))

        # 2. Kenarları ekle (vektör kontrolü ile)
        for node in valid_nodes:
            grid_x, grid_y = node
            p1 = self._grid_to_world(grid_x, grid_y, map_bounds)
            
            # 8 yönlü komşuları kontrol et
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    
                    nx_x, nx_y = grid_x + dx, grid_y + dy
                    neighbor = (nx_x, nx_y)
                    
                    # Sadece geçerli ve daha önce eklenmemiş komşulara bak
                    # (G.has_edge kontrolü çift yönlü eklemeyi önler)
                    if neighbor in valid_nodes:
                        if G.has_edge(node, neighbor):
                            continue
                            
                        p2 = self._grid_to_world(nx_x, nx_y, map_bounds)
                        
                        # Vektörün engellerle kesişimini kontrol et
                        edge_safe = True
                        for obs in obstacles:
                            if self._segment_intersects_circle(p1, p2, obs):
                                edge_safe = False
                                break
                        
                        if edge_safe:
                            # Hareket maliyeti: düz = 1.0, diyagonal = sqrt(2)
                            weight = math.sqrt(2.0) if (dx!=0 and dy!=0) else 1.0
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
        
        # Başlangıç veya bitiş noktası graph'ta yoksa (engel içindeyse veya izole ise)
        # En yakın geçerli düğümü bulmaya çalışabiliriz, ama şimdilik None dönelim
        if start_grid not in G or goal_grid not in G:
            return None

        try:
            path_grid = nx.astar_path(G, start_grid, goal_grid, heuristic=self._heuristic, weight='weight')
            return [
                self._grid_to_world(gx, gy, map_bounds)
                for gx, gy in path_grid
            ]
        except (nx.NetworkXNoPath, Exception):
            return None
