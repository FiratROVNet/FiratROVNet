import math
import networkx as nx
from typing import List, Tuple, Optional, Any

class AStarPlanner:
    """NetworkX tabanlı 2D (X, Z) Yol Planlayıcı."""
    GRID_RES = 100  # Harita çözünürlüğü artırıldı (daha hassas manevralar)
    FALLBACK_BASE_RADIUS = 20.0
    FALLBACK_ANGLE_STEP_DEG = 60
    ROV_GUVENLIK_YARICAPI = 12.0  # ROV'un adalara/kayalara bindirmesini engelleyecek ekstra emniyet mesafesi

    def _get_bounds(self, havuz_genisligi: float):
        return (-havuz_genisligi, havuz_genisligi, -havuz_genisligi, havuz_genisligi)

    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float],
                  obstacles: List[Any],
                  havuz_genisligi: float) -> Optional[List[Tuple[float, float]]]:
        """
        X ve Z koordinat düzleminde en kısa GÜVENLİ (Pürüzsüz) yolu bulur.
        """
        min_x, max_x, min_z, max_z = self._get_bounds(havuz_genisligi)
        G = nx.Graph()

        # 1. Grid Düğümlerini Oluştur ve Emniyetli Çember Kontrolü Yap
        valid_nodes = set()
        for i in range(self.GRID_RES):
            for j in range(self.GRID_RES):
                wx = min_x + (i + 0.5) * (max_x - min_x) / self.GRID_RES
                wz = min_z + (j + 0.5) * (max_z - min_z) / self.GRID_RES
                
                is_safe = True
                for obs in obstacles:
                    ox = float(obs[0])
                    oz = float(obs[1])
                    # Veride engel çapı yoksa varsayılanı (5m) al, varsa üzerine ROV çapını ekle
                    orad = float(obs[2]) if len(obs) > 2 else 5.0 
                    
                    gerekli_mesafe = orad + self.ROV_GUVENLIK_YARICAPI
                    
                    if math.hypot(wx - ox, wz - oz) < gerekli_mesafe:
                        is_safe = False
                        break
                
                if is_safe:
                    G.add_node((i, j), pos=(wx, wz))
                    valid_nodes.add((i, j))

        # 2. Komşulukları (Edges) Bağla
        for node in valid_nodes:
            for dx, dz in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (node[0] + dx, node[1] + dz)
                if neighbor in valid_nodes:
                    cost = math.sqrt(dx**2 + dz**2)
                    G.add_edge(node, neighbor, weight=cost)

        def get_closest(wx, wz):
            gx = int((wx - min_x) / (max_x - min_x) * self.GRID_RES)
            gz = int((wz - min_z) / (max_z - min_z) * self.GRID_RES)
            gx, gz = max(0, min(self.GRID_RES-1, gx)), max(0, min(self.GRID_RES-1, gz))
            return (gx, gz)

        def valid_node(point):
            if not (min_x <= point[0] <= max_x and min_z <= point[1] <= max_z):
                return None
            
            n = get_closest(*point)
            if n in valid_nodes:
                return n
                
            # Eğer noktamız güvenli değilse (engelin içindeyse) en yakın güvenli çıkış hücresini bul
            min_dist = float('inf')
            best_node = None
            for vn in valid_nodes:
                vpos = G.nodes[vn]['pos']
                dist = math.hypot(point[0] - vpos[0], point[1] - vpos[1])
                if dist < min_dist and dist < self.FALLBACK_BASE_RADIUS:
                    min_dist = dist
                    best_node = vn
            return best_node

        def astar_between(start_point, goal_point):
            start_node = valid_node(start_point)
            goal_node = valid_node(goal_point)
            if start_node is None or goal_node is None:
                return None
            try:
                path = nx.astar_path(G, start_node, goal_node, weight='weight')
                return [G.nodes[n]['pos'] for n in path]
            except Exception:
                return None

        # Ana A* Çağrısı
        path = astar_between(start, goal)
        if path:
            # Doğal ve Pürüzsüz görünmesi için zigzagları temizleyen fonksiyonu çağır
            return self._smooth_path(path, obstacles)

        # Standart yol bulunamazsa genişleyerek (Fallback) güvenli noktalar ara
        for radius in self._fallback_radii(havuz_genisligi):
            start_candidates = self._circle_candidates(start, radius, min_x, max_x, min_z, max_z)
            goal_candidates = self._circle_candidates(goal, radius, min_x, max_x, min_z, max_z)
            
            if valid_node(start) is not None: start_candidates.insert(0, start)
            if valid_node(goal) is not None: goal_candidates.insert(0, goal)

            for sc in start_candidates:
                for gc in goal_candidates:
                    path = astar_between(sc, gc)
                    if path:
                        return self._smooth_path(path, obstacles)
        return None

    def _smooth_path(self, path: List[Tuple[float, float]], obstacles: List[Any]) -> List[Tuple[float, float]]:
        """A* zigzaglarını (merdiven görünümü) temizler, araları düz, net, direkt çizgilere çevirir."""
        if len(path) <= 2:
            return path
            
        smoothed = [path[0]]
        current_idx = 0
        
        while current_idx < len(path) - 1:
            furthest_valid = current_idx + 1
            # Mevcut noktadan "Kuş Uçuşu" engelsiz gidebileceğimiz en uzak noktayı bul
            for i in range(len(path) - 1, current_idx, -1):
                if i == current_idx + 1:
                    furthest_valid = i
                    break
                if self._line_of_sight(path[current_idx], path[i], obstacles):
                    furthest_valid = i
                    break
            smoothed.append(path[furthest_valid])
            current_idx = furthest_valid
            
        return smoothed

    def _line_of_sight(self, p1, p2, obstacles):
        """İki nokta arasında kuş uçuşu direk bir yol var mı? Engel kesişiyor mu?"""
        x1, z1 = p1
        x2, z2 = p2
        dist = math.hypot(x2 - x1, z2 - z1)
        if dist == 0: return True
        
        steps = int(dist / 2.0)  # Her 2 metrede bir sanal ışın (raycast) gönder
        if steps < 1: steps = 1
        
        for i in range(1, steps):
            tx = x1 + (x2 - x1) * (i / steps)
            tz = z1 + (z2 - z1) * (i / steps)
            
            for obs in obstacles:
                ox, oz = float(obs[0]), float(obs[1])
                orad = float(obs[2]) if len(obs) > 2 else 5.0
                
                # Işın (Line) herhangi bir engele veya ROV güvenlik balonuna değerse geçersiz kıl
                if math.hypot(tx - ox, tz - oz) < (orad + self.ROV_GUVENLIK_YARICAPI - 1.0):
                    return False
        return True

    def _fallback_radii(self, havuz_genisligi: float):
        max_radius = max(self.FALLBACK_BASE_RADIUS, float(havuz_genisligi) * 2.0)
        multiplier = 1
        while True:
            radius = self.FALLBACK_BASE_RADIUS * multiplier
            if radius > max_radius:
                break
            yield radius
            multiplier += 1

    def _circle_candidates(self, center, radius, min_x, max_x, min_z, max_z):
        candidates = []
        for angle_deg in range(0, 360, self.FALLBACK_ANGLE_STEP_DEG):
            angle = math.radians(angle_deg)
            point = (
                float(center[0]) + math.cos(angle) * float(radius),
                float(center[1]) + math.sin(angle) * float(radius),
            )
            if min_x <= point[0] <= max_x and min_z <= point[1] <= max_z:
                candidates.append(point)
        return candidates