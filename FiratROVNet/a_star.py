import math
import networkx as nx
from typing import List, Tuple, Optional

class AStarPlanner:
    """NetworkX tabanlı 2D (X, Z) Yol Planlayıcı."""
    GRID_RES = 60  # Toplam 10x10 grid (Daha hassas istersen artırabilirsin)

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
                    if math.hypot(wx - ox, wz - oz) < (orad * 1.2 + 5.0):
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

        start_node, goal_node = get_closest(*start), get_closest(*goal)

        if start_node not in G or goal_node not in G: return None

        try:
            path = nx.astar_path(G, start_node, goal_node, weight='weight')
            return [G.nodes[n]['pos'] for n in path]
        except:
            return None