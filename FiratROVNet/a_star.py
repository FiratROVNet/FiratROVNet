import math
import networkx as nx
from typing import List, Tuple, Optional

class AStarPlanner:
    """NetworkX tabanlı 2D (X, Z) Yol Planlayıcı."""
    GRID_RES = 60  # Toplam 10x10 grid (Daha hassas istersen artırabilirsin)
    FALLBACK_BASE_RADIUS = 20.0
    FALLBACK_ANGLE_STEP_DEG = 60

    def _get_bounds(self, havuz_genisligi: float):
        # -200, 200 gibi sınırları döndürür
        return (-havuz_genisligi, havuz_genisligi, -havuz_genisligi, havuz_genisligi)

    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float],
                  obstacles: List[Tuple[float, float, float]],
                  havuz_genisligi: float) -> Optional[List[Tuple[float, float]]]:
        """
        X ve Z koordinat düzleminde en kısa güvenli yolu bulur.
        obstacles: [(x, z, radius), ...]
        """
        min_x, max_x, min_z, max_z = self._get_bounds(havuz_genisligi)
        G = nx.Graph()

        # 1. Grid Düğümlerini Oluştur ve Engel Kontrolü Yap
        valid_nodes = set()
        for i in range(self.GRID_RES):
            for j in range(self.GRID_RES):
                # Grid'den Dünya koordinatına (X, Z)
                wx = min_x + (i + 0.5) * (max_x - min_x) / self.GRID_RES
                wz = min_z + (j + 0.5) * (max_z - min_z) / self.GRID_RES
                
                # Engel içinde mi? (%20 güvenlik marjı + 5m sabit pay)
                is_safe = True
                for ox, oz, orad in obstacles:
                    if math.hypot(wx - ox, wz - oz) < (orad * 1.1):
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

        # 3. Başlangıç ve Hedefe En Yakın Node'ları Bul
        def get_closest(wx, wz):
            gx = int((wx - min_x) / (max_x - min_x) * self.GRID_RES)
            gz = int((wz - min_z) / (max_z - min_z) * self.GRID_RES)
            gx, gz = max(0, min(self.GRID_RES-1, gx)), max(0, min(self.GRID_RES-1, gz))
            return (gx, gz)

        def inside_bounds(point):
            return min_x <= point[0] <= max_x and min_z <= point[1] <= max_z

        def valid_node(point):
            if not inside_bounds(point):
                return None
            node = get_closest(*point)
            return node if node in G else None

        def astar_between(start_point, goal_point):
            start_node, goal_node = valid_node(start_point), valid_node(goal_point)
            if start_node is None or goal_node is None:
                return None
            try:
                path = nx.astar_path(G, start_node, goal_node, weight='weight')
                return [G.nodes[n]['pos'] for n in path]
            except Exception:
                return None

        path = astar_between(start, goal)
        if path:
            return path

        # Başlangıç veya hedef engel içindeyse ya da grafik kopuksa, 20m'lik
        # halkalardan başlayıp 60 derecelik örneklerle güvenli giriş/çıkış ara.
        for radius in self._fallback_radii(havuz_genisligi):
            start_candidates = self._circle_candidates(start, radius, min_x, max_x, min_z, max_z)
            goal_candidates = self._circle_candidates(goal, radius, min_x, max_x, min_z, max_z)

            if valid_node(start) is not None:
                start_candidates.insert(0, start)
            if valid_node(goal) is not None:
                goal_candidates.insert(0, goal)

            if not start_candidates and not goal_candidates:
                break

            for start_candidate in start_candidates:
                for goal_candidate in goal_candidates:
                    path = astar_between(start_candidate, goal_candidate)
                    if path:
                        return path

        return None

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
