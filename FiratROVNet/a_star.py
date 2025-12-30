"""
A* (A-Star) Yol Bulma Algoritması

Bu modül, engelleri dikkate alarak başlangıç noktasından hedef noktaya
en kısa yolu bulan A* algoritmasını implement eder.
"""

import numpy as np
import math
from typing import List, Tuple, Optional, Set
from heapq import heappush, heappop


class AStarNode:
    """A* algoritması için düğüm sınıfı"""
    def __init__(self, x: int, y: int, g_cost: float = 0, h_cost: float = 0, parent=None):
        self.x = x
        self.y = y
        self.g_cost = g_cost  # Başlangıçtan bu noktaya gerçek maliyet
        self.h_cost = h_cost  # Bu noktadan hedefe tahmini maliyet (heuristic)
        self.f_cost = g_cost + h_cost  # Toplam maliyet
        self.parent = parent  # Önceki düğüm (yolu geri takip etmek için)
    
    def __lt__(self, other):
        """Heap için karşılaştırma operatörü"""
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        """Eşitlik kontrolü"""
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        """Hash fonksiyonu (set için)"""
        return hash((self.x, self.y))


class AStarPlanner:
    """
    A* algoritması kullanarak yol planlama yapan sınıf.
    """
    
    def __init__(self, grid_size: float = 1.0):
        """
        Args:
            grid_size: Grid çözünürlüğü (metre cinsinden, varsayılan: 1.0)
        """
        self.grid_size = grid_size
    
    def _heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """
        Heuristic fonksiyonu (Euclidean mesafe).
        
        Args:
            x1, y1: İlk nokta koordinatları
            x2, y2: İkinci nokta koordinatları
        
        Returns:
            float: Tahmini mesafe
        """
        dx = x2 - x1
        dy = y2 - y1
        return math.sqrt(dx * dx + dy * dy)
    
    def _is_valid(self, x: int, y: int, obstacle_map: np.ndarray, 
                   map_bounds: Tuple[float, float, float, float]) -> bool:
        """
        Noktanın geçerli olup olmadığını kontrol eder.
        
        Args:
            x, y: Grid koordinatları
            obstacle_map: Engel haritası (True = engel, False = serbest)
            map_bounds: (min_x, max_x, min_y, max_y) harita sınırları
        
        Returns:
            bool: True if geçerli, False otherwise
        """
        min_x, max_x, min_y, max_y = map_bounds
        
        # Sınır kontrolü
        if x < 0 or y < 0:
            return False
        
        if x >= obstacle_map.shape[1] or y >= obstacle_map.shape[0]:
            return False
        
        # Engel kontrolü
        if obstacle_map[y, x]:
            return False
        
        return True
    
    def _get_neighbors(self, x: int, y: int) -> List[Tuple[int, int]]:
        """
        Bir noktanın komşularını döndürür (8 yönlü hareket).
        
        Args:
            x, y: Grid koordinatları
        
        Returns:
            List[Tuple[int, int]]: Komşu koordinatları
        """
        neighbors = []
        # 8 yönlü hareket (diyagonal dahil)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbors.append((x + dx, y + dy))
        return neighbors
    
    def _get_movement_cost(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """
        İki nokta arasındaki hareket maliyetini hesaplar.
        
        Args:
            x1, y1: İlk nokta
            x2, y2: İkinci nokta
        
        Returns:
            float: Hareket maliyeti
        """
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        
        # Düz hareket: 1.0, Diyagonal hareket: sqrt(2) ≈ 1.414
        if dx == 0 or dy == 0:
            return 1.0
        else:
            return math.sqrt(2.0)
    
    def _world_to_grid(self, world_x: float, world_y: float, 
                       map_bounds: Tuple[float, float, float, float]) -> Tuple[int, int]:
        """
        Dünya koordinatlarını grid koordinatlarına dönüştürür.
        
        Args:
            world_x, world_y: Dünya koordinatları (metre)
            map_bounds: (min_x, max_x, min_y, max_y) harita sınırları
        
        Returns:
            Tuple[int, int]: Grid koordinatları
        """
        min_x, max_x, min_y, max_y = map_bounds
        
        # Grid indekslerini hesapla
        grid_x = int((world_x - min_x) / self.grid_size)
        grid_y = int((world_y - min_y) / self.grid_size)
        
        return grid_x, grid_y
    
    def _grid_to_world(self, grid_x: int, grid_y: int,
                       map_bounds: Tuple[float, float, float, float]) -> Tuple[float, float]:
        """
        Grid koordinatlarını dünya koordinatlarına dönüştürür.
        
        Args:
            grid_x, grid_y: Grid koordinatları
            map_bounds: (min_x, max_x, min_y, max_y) harita sınırları
        
        Returns:
            Tuple[float, float]: Dünya koordinatları (metre)
        """
        min_x, max_x, min_y, max_y = map_bounds
        
        # Dünya koordinatlarını hesapla (grid'in ortası)
        world_x = min_x + (grid_x + 0.5) * self.grid_size
        world_y = min_y + (grid_y + 0.5) * self.grid_size
        
        return world_x, world_y
    
    def _create_obstacle_map(self, obstacles: List[Tuple[float, float, float]],
                            map_bounds: Tuple[float, float, float, float],
                            safety_margin: float = 8.0,
                            polygon_obstacles: Optional[List[List[Tuple[float, float]]]] = None) -> np.ndarray:
        """
        Engel listesinden grid tabanlı engel haritası oluşturur.
        
        Args:
            obstacles: [(x, y, radius), ...] formatında dairesel engel listesi
            map_bounds: (min_x, max_x, min_y, max_y) harita sınırları
            safety_margin: Engel etrafında güvenlik mesafesi (metre)
            polygon_obstacles: [[(x1, y1), (x2, y2), ...], ...] formatında polygon engel listesi
        
        Returns:
            np.ndarray: Engel haritası (True = engel, False = serbest)
        """
        min_x, max_x, min_y, max_y = map_bounds
        
        # Grid boyutlarını hesapla
        grid_width = int((max_x - min_x) / self.grid_size) + 1
        grid_height = int((max_y - min_y) / self.grid_size) + 1
        
        # Engel haritasını oluştur (başlangıçta tümü serbest)
        obstacle_map = np.zeros((grid_height, grid_width), dtype=bool)
        
        # Her engel için grid'i işaretle
        for obs_x, obs_y, obs_radius in obstacles:
            # Güvenlik mesafesi ile genişletilmiş yarıçap
            total_radius = obs_radius + safety_margin
            
            # Grid koordinatlarını hesapla
            grid_obs_x, grid_obs_y = self._world_to_grid(obs_x, obs_y, map_bounds)
            
            # Engel etrafındaki tüm grid noktalarını kontrol et
            # Engel merkezinden total_radius mesafesindeki tüm noktaları işaretle
            radius_grid = int(total_radius / self.grid_size) + 1
            
            for dx in range(-radius_grid, radius_grid + 1):
                for dy in range(-radius_grid, radius_grid + 1):
                    grid_x = grid_obs_x + dx
                    grid_y = grid_obs_y + dy
                    
                    # Sınır kontrolü
                    if grid_x < 0 or grid_x >= grid_width or grid_y < 0 or grid_y >= grid_height:
                        continue
                    
                    # Dünya koordinatlarını hesapla
                    world_x, world_y = self._grid_to_world(grid_x, grid_y, map_bounds)
                    
                    # Mesafe kontrolü
                    dist = math.sqrt((world_x - obs_x)**2 + (world_y - obs_y)**2)
                    if dist <= total_radius:
                        obstacle_map[grid_y, grid_x] = True
        
        # Polygon engelleri işle (ada çevre noktaları gibi)
        if polygon_obstacles:
            for polygon in polygon_obstacles:
                if len(polygon) < 3:
                    continue  # En az 3 nokta gerekli
                
                # Polygon'un bounding box'ını hesapla
                min_poly_x = min(p[0] for p in polygon)
                max_poly_x = max(p[0] for p in polygon)
                min_poly_y = min(p[1] for p in polygon)
                max_poly_y = max(p[1] for p in polygon)
                
                # Güvenlik mesafesi ile genişlet
                min_poly_x -= safety_margin
                max_poly_x += safety_margin
                min_poly_y -= safety_margin
                max_poly_y += safety_margin
                
                # Bounding box içindeki grid noktalarını kontrol et
                grid_min_x, grid_min_y = self._world_to_grid(min_poly_x, min_poly_y, map_bounds)
                grid_max_x, grid_max_y = self._world_to_grid(max_poly_x, max_poly_y, map_bounds)
                
                for grid_x in range(max(0, grid_min_x), min(grid_width, grid_max_x + 1)):
                    for grid_y in range(max(0, grid_min_y), min(grid_height, grid_max_y + 1)):
                        # Dünya koordinatlarını hesapla
                        world_x, world_y = self._grid_to_world(grid_x, grid_y, map_bounds)
                        
                        # Nokta polygon içinde mi kontrol et (point-in-polygon)
                        if self._point_in_polygon(world_x, world_y, polygon, safety_margin):
                            obstacle_map[grid_y, grid_x] = True
        
        return obstacle_map
    
    def _point_in_polygon(self, x: float, y: float, polygon: List[Tuple[float, float]], 
                          safety_margin: float = 0.0) -> bool:
        """
        Noktanın polygon içinde olup olmadığını kontrol eder (ray casting algoritması).
        
        Args:
            x, y: Test edilecek nokta koordinatları
            polygon: [(x1, y1), (x2, y2), ...] formatında polygon noktaları
            safety_margin: Güvenlik mesafesi (polygon dışına genişletme)
        
        Returns:
            bool: True if nokta polygon içinde (veya safety_margin mesafesinde), False otherwise
        """
        if len(polygon) < 3:
            return False
        
        # Ray casting algoritması
        inside = False
        n = len(polygon)
        
        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            
            # Kenar ile kesişim kontrolü
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            
            j = i
        
        # Eğer nokta polygon içindeyse, True döndür
        if inside:
            return True
        
        # Eğer safety_margin > 0 ise, polygon kenarlarına mesafe kontrolü yap
        if safety_margin > 0:
            # Polygon'un her kenarına mesafe kontrolü
            for i in range(n):
                p1 = polygon[i]
                p2 = polygon[(i + 1) % n]
                
                # Noktadan kenara mesafe hesapla
                dist = self._point_to_line_segment_distance(x, y, p1[0], p1[1], p2[0], p2[1])
                if dist <= safety_margin:
                    return True
        
        return False
    
    def _point_to_line_segment_distance(self, px: float, py: float, 
                                       x1: float, y1: float, x2: float, y2: float) -> float:
        """
        Noktadan doğru parçasına mesafe hesaplar.
        
        Args:
            px, py: Nokta koordinatları
            x1, y1: Doğru parçası başlangıç noktası
            x2, y2: Doğru parçası bitiş noktası
        
        Returns:
            float: Mesafe
        """
        # Vektörler
        dx = x2 - x1
        dy = y2 - y1
        
        # Doğru parçası uzunluğu
        seg_len_sq = dx * dx + dy * dy
        
        if seg_len_sq < 1e-10:
            # Nokta gibi doğru parçası
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        
        # Parametrik pozisyon
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))
        
        # En yakın nokta
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        # Mesafe
        return math.sqrt((px - closest_x)**2 + (py - closest_y)**2)
    
    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float],
                  obstacles: List[Tuple[float, float, float]],
                  map_bounds: Tuple[float, float, float, float],
                  safety_margin: float = 8.0,
                  polygon_obstacles: Optional[List[List[Tuple[float, float]]]] = None) -> Optional[List[Tuple[float, float]]]:
        """
        A* algoritması kullanarak başlangıçtan hedefe yol bulur.
        
        Args:
            start: (x, y) başlangıç koordinatları (metre)
            goal: (x, y) hedef koordinatları (metre)
            obstacles: [(x, y, radius), ...] formatında dairesel engel listesi
            map_bounds: (min_x, max_x, min_y, max_y) harita sınırları
            safety_margin: Engel etrafında güvenlik mesafesi (metre, varsayılan: 2.0)
            polygon_obstacles: [[(x1, y1), (x2, y2), ...], ...] formatında polygon engel listesi
        
        Returns:
            Optional[List[Tuple[float, float]]]: Bulunan yol [(x1, y1), (x2, y2), ...] veya None
        """
        start_x, start_y = start
        goal_x, goal_y = goal
        
        # Grid koordinatlarına dönüştür
        start_grid = self._world_to_grid(start_x, start_y, map_bounds)
        goal_grid = self._world_to_grid(goal_x, goal_y, map_bounds)
        
        # Engel haritasını oluştur
        obstacle_map = self._create_obstacle_map(obstacles, map_bounds, safety_margin, polygon_obstacles)
        
        # Başlangıç ve hedef noktalarının geçerliliğini kontrol et
        if not self._is_valid(start_grid[0], start_grid[1], obstacle_map, map_bounds):
            print(f"⚠️ [A*] Başlangıç noktası geçersiz veya engel üzerinde: {start}")
            return None
        
        if not self._is_valid(goal_grid[0], goal_grid[1], obstacle_map, map_bounds):
            print(f"⚠️ [A*] Hedef nokta geçersiz veya engel üzerinde: {goal}")
            return None
        
        # A* algoritması
        open_set = []  # Priority queue (heap)
        closed_set: Set[Tuple[int, int]] = set()  # Ziyaret edilen noktalar
        
        # Başlangıç düğümü
        start_node = AStarNode(start_grid[0], start_grid[1], 
                              g_cost=0.0,
                              h_cost=self._heuristic(start_grid[0], start_grid[1], 
                                                    goal_grid[0], goal_grid[1]))
        heappush(open_set, start_node)
        
        # Ana döngü
        max_iterations = 100000  # Sonsuz döngüyü önlemek için
        iteration = 0
        
        while open_set and iteration < max_iterations:
            iteration += 1
            
            # En düşük f_cost'a sahip düğümü al
            current = heappop(open_set)
            
            # Hedefe ulaşıldı mı?
            if current.x == goal_grid[0] and current.y == goal_grid[1]:
                # Yolu geri takip et
                path = []
                node = current
                while node is not None:
                    world_x, world_y = self._grid_to_world(node.x, node.y, map_bounds)
                    path.append((world_x, world_y))
                    node = node.parent
                
                # Yolu ters çevir (başlangıçtan hedefe)
                path.reverse()
                print(f"✅ [A*] Yol bulundu! {len(path)} nokta, {iteration} iterasyon")
                return path
            
            # Bu düğümü kapalı listeye ekle
            closed_set.add((current.x, current.y))
            
            # Komşuları kontrol et
            for neighbor_x, neighbor_y in self._get_neighbors(current.x, current.y):
                # Geçerlilik kontrolü
                if not self._is_valid(neighbor_x, neighbor_y, obstacle_map, map_bounds):
                    continue
                
                # Zaten ziyaret edildi mi?
                if (neighbor_x, neighbor_y) in closed_set:
                    continue
                
                # Hareket maliyeti
                movement_cost = self._get_movement_cost(current.x, current.y, neighbor_x, neighbor_y)
                new_g_cost = current.g_cost + movement_cost
                
                # Heuristic maliyet
                h_cost = self._heuristic(neighbor_x, neighbor_y, goal_grid[0], goal_grid[1])
                
                # Yeni düğüm oluştur
                neighbor_node = AStarNode(neighbor_x, neighbor_y,
                                          g_cost=new_g_cost,
                                          h_cost=h_cost,
                                          parent=current)
                
                # Open set'te zaten var mı kontrol et
                found_in_open = False
                for existing_node in open_set:
                    if existing_node.x == neighbor_x and existing_node.y == neighbor_y:
                        found_in_open = True
                        # Daha iyi bir yol bulunduysa güncelle
                        if new_g_cost < existing_node.g_cost:
                            existing_node.g_cost = new_g_cost
                            existing_node.f_cost = new_g_cost + existing_node.h_cost
                            existing_node.parent = current
                        break
                
                # Open set'te yoksa ekle
                if not found_in_open:
                    heappush(open_set, neighbor_node)
        
        # Yol bulunamadı
        print(f"❌ [A*] Yol bulunamadı! {iteration} iterasyon sonrası durduruldu.")
        return None

