"""
Simülasyon Helper Module
========================
Hesaplama fonksiyonları ve yardımcı işlemler için Ortam sınıfı.
Matematiksel işlemler, koordinat dönüşümleri ve veri hazırlama logic'i.
"""

import math
import os
import random
from typing import List, Optional, Tuple

import torch

from ursina import Vec3, Mesh, Entity, color, distance

try:
    from FiratROVNet.config import GATLimitleri
except ImportError:
    from ..config import GATLimitleri


# -----------------------------------------------------------------------------
# Veri yapıları
# -----------------------------------------------------------------------------

class MiniData:
    """GAT modeli için veri taşıyıcı (x: özellik matrisi, edge_index: kenar indeksleri)."""
    __slots__ = ('x', 'edge_index')

    def __init__(self, x, edge_index):
        self.x = x
        self.edge_index = edge_index


# -----------------------------------------------------------------------------
# Koordinat dönüşümleri (Sim ↔ Ursina)
# -----------------------------------------------------------------------------

def sim_to_ursina(x_2d: float, y_2d: float, z_depth: float) -> Tuple[float, float, float]:
    """Sim (X: sağ-sol, Y: ileri-geri, Z: derinlik) → Ursina (X, Y: yukarı, Z)."""
    return (x_2d, z_depth, y_2d)


def ursina_to_sim(ux: float, uy: float, uz: float) -> Tuple[float, float, float]:
    """Ursina (X, Y: yukarı, Z) → Sim (X: sağ-sol, Y: ileri-geri, Z: derinlik)."""
    return (ux, uz, uy)


# -----------------------------------------------------------------------------
# Statik yardımcı fonksiyonlar
# -----------------------------------------------------------------------------

def dunya_to_harita(x: float, z: float, havuz_genisligi: float) -> Vec3:
    """
    Dünya koordinatlarını (metre) harita lokal koordinatlarına (-0.5, 0.5) çevirir.
    Ursina UI: Y yukarı, Dünya Z = Harita Y.
    """
    factor = 1.0 / (havuz_genisligi * 2)
    return Vec3(x * factor, z * factor, 0)


def grid_step_metre(havuz_genisligi: float, grid_sayisi: Optional[int] = None, grid_unit: float = 50.0) -> float:
    """Grid başına mesafe (m). grid_sayisi verilirse (2*havuz)/N, yoksa grid_unit."""
    half = havuz_genisligi
    if grid_sayisi is not None and grid_sayisi > 0:
        return (2.0 * half) / grid_sayisi
    return float(grid_unit)


def load_obj_as_mesh(obj_path: str) -> Optional[Mesh]:
    """
    OBJ dosyasını yükler, quad yüzleri üçgene çevirir (Ursina 7.x uyumu).
    Returns:
        Ursina Mesh veya None (hata/boş dosya).
    """
    if not obj_path or not os.path.exists(obj_path):
        return None
    v_list, vt_list, vertices, uvs, triangles = [], [], [], [], []
    try:
        with open(obj_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if not parts:
                    continue
                if parts[0] == 'v' and len(parts) >= 4:
                    v_list.append((float(parts[1]), float(parts[2]), float(parts[3])))
                elif parts[0] == 'vt' and len(parts) >= 3:
                    vt_list.append((float(parts[1]), float(parts[2])))
                elif parts[0] == 'f':
                    face_verts = []
                    for i in range(1, len(parts)):
                        seg = parts[i].split('/')
                        try:
                            vi = int(seg[0])
                        except (ValueError, IndexError):
                            continue
                        vi = len(v_list) + vi if vi < 0 else vi - 1
                        vti_raw = int(seg[1]) if len(seg) > 1 and seg[1].strip() else 0
                        vti = len(vt_list) + vti_raw if vti_raw < 0 else (vti_raw - 1 if vti_raw > 0 else 0)
                        if 0 <= vi < len(v_list):
                            u, v = vt_list[vti] if vti < len(vt_list) else (0.0, 0.0)
                            face_verts.append((vi, u, v))
                    if len(face_verts) == 3:
                        base = len(vertices)
                        for vi, u, v in face_verts:
                            vertices.append(v_list[vi])
                            uvs.append((u, v))
                        triangles.append((base, base + 1, base + 2))
                    elif len(face_verts) == 4:
                        base = len(vertices)
                        for vi, u, v in face_verts:
                            vertices.append(v_list[vi])
                            uvs.append((u, v))
                        triangles.append((base, base + 1, base + 2))
                        triangles.append((base, base + 2, base + 3))
    except Exception:
        return None
    if not vertices or not triangles:
        return None
    try:
        return Mesh(vertices=vertices, triangles=triangles, uvs=uvs, mode='triangle', static=True)
    except Exception:
        return None


def kayalari_olustur(
    n_engels: int,
    havuz_genisligi: float,
    sea_floor_y: float,
    water_surface_y_base: float,
    guvenlik_mesafesi: float = 8.0,
    min_boyut: float = 15,
    max_boyut: float = 40,
    max_z_boyut: float = 60,
) -> List:
    """
    Kayaları oluşturur ve güvenli pozisyonlara yerleştirir.
    Returns:
        Oluşturulan kaya Entity listesi.
    """
    engeller = []
    mevcut: List[Tuple[float, float, float]] = []  # [(x, z, yari_cap), ...]

    def _guvenli_pozisyon(yari_cap: float) -> Tuple[float, float]:
        toplam = yari_cap + guvenlik_mesafesi
        mn = -havuz_genisligi + toplam
        mx = havuz_genisligi - toplam
        if mn >= mx:
            mn, mx = -havuz_genisligi + 1, havuz_genisligi - 1
        for _ in range(100):
            x = random.uniform(mn, mx)
            z = random.uniform(mn, mx)
            if all(math.hypot(x - ex, z - ez) >= yari_cap + er + guvenlik_mesafesi for ex, ez, er in mevcut):
                return (x, z)
        return (random.uniform(mn, mx), random.uniform(mn, mx))

    for _ in range(n_engels):
        s_x = random.uniform(min_boyut, max_boyut)
        s_y = random.uniform(min_boyut, max_boyut)
        s_z = random.uniform(min_boyut, max_z_boyut)
        yari_cap = max(s_x, s_y, s_z) / 2.0
        x, z = _guvenli_pozisyon(yari_cap)
        y_alt = sea_floor_y
        y_ust = water_surface_y_base - (s_y / 2) - 2
        y = random.uniform(y_alt, y_ust) if y_ust > y_alt else sea_floor_y + (s_y / 2)
        gri = random.randint(80, 100)
        engel = Entity(
            model='icosphere',
            color=color.rgb(gri, gri, gri),
            texture='noise',
            scale=(s_x, s_y, s_z),
            position=(x, sea_floor_y, z),
            rotation=(random.randint(0, 360), random.randint(0, 360), random.randint(0, 360)),
            collider='sphere',
            unlit=True
        )
        engeller.append(engel)
        mevcut.append((x, z, yari_cap))

    return engeller


def a_star_yol_bul(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    obstacles: List[Tuple[float, float, float]],
    map_bounds: Tuple[float, float, float, float],
) -> Optional[List[Tuple[float, float]]]:
    """
    A* ile yol hesaplar. obstacles: [(x, y, radius), ...], map_bounds: (min_x, max_x, min_y, max_y).
    """
    try:
        from FiratROVNet.a_star import AStarPlanner
        planner = AStarPlanner()
        return planner.find_path(start, goal, obstacles, map_bounds, safety_margin=0.0)
    except Exception:
        return None


def find_safe_island_position(
    placed_positions: List[Tuple[float, float]],
    min_x: float, max_x: float, min_z: float, max_z: float,
    min_distance: float,
    max_attempts: int = 100
) -> Tuple[float, float]:
    """
    Adaların birbirine çakışmaması için güvenli (X, Z) pozisyonu bulur.

    Args:
        placed_positions: Mevcut ada pozisyonları [(x, z), ...]
        min_x, max_x, min_z, max_z: Havuz sınırları
        min_distance: Minimum mesafe (ada yarıçapı * güvenlik payı)
        max_attempts: Maksimum deneme sayısı

    Returns:
        (island_x, island_z): Güvenli ada pozisyonu
    """
    if not placed_positions:
        return (random.uniform(min_x, max_x), random.uniform(min_z, max_z))

    for _ in range(max_attempts):
        cx = random.uniform(min_x, max_x)
        cz = random.uniform(min_z, max_z)
        too_close = False
        for ex, ez in placed_positions:
            d = math.hypot(cx - ex, cz - ez)
            if d < min_distance:
                too_close = True
                break
        if not too_close:
            return (cx, cz)

    # Fallback: mevcut adalardan en uzak nokta
    avg_x = sum(p[0] for p in placed_positions) / len(placed_positions)
    avg_z = sum(p[1] for p in placed_positions) / len(placed_positions)
    fallback_x = max(min_x + 20, avg_x - min_distance) if avg_x > 0 else min(max_x - 20, avg_x + min_distance)
    fallback_z = max(min_z + 20, avg_z - min_distance) if avg_z > 0 else min(max_z - 20, avg_z + min_distance)
    return (fallback_x, fallback_z)


# -----------------------------------------------------------------------------
# OrtamHelper sınıfı
# -----------------------------------------------------------------------------

class OrtamHelper:
    """
    Ortam sınıfı için hesaplama ve veri hazırlama yardımcısı.
    FiloHelper yapısına uyumlu; ortam_ref üzerinden veri erişimi.
    """

    def __init__(self, ortam_ref):
        """
        Args:
            ortam_ref: Ortam örneği referansı (self)
        """
        self.ortam = ortam_ref

    def find_safe_island_position(
        self,
        placed_positions: List[Tuple[float, float]],
        min_x: float, max_x: float, min_z: float, max_z: float,
        min_distance: float,
        max_attempts: int = 100
    ) -> Tuple[float, float]:
        """Ada için güvenli pozisyon bulur (OrtamHelper üzerinden)."""
        return find_safe_island_position(
            placed_positions, min_x, max_x, min_z, max_z, min_distance, max_attempts
        )

    def simden_veriye(self) -> MiniData:
        """
        Fiziksel dünyayı GAT modeli girdisine çevirir.

        Returns:
            MiniData: x (n x 9), edge_index (2 x E)
        """
        engeller = getattr(self.ortam, 'engeller', []) or []
        aktif_rovs = [r for r in getattr(self.ortam, 'rovs', []) if r is not None]

        if not aktif_rovs:
            return MiniData(
                x=torch.zeros((0, 9), dtype=torch.float),
                edge_index=torch.zeros((2, 0), dtype=torch.long)
            )

        L = {
            'LEADER': GATLimitleri.UZAK,
            'DISCONNECT': GATLimitleri.KOPMA,
            'OBSTACLE': GATLimitleri.ENGEL,
            'COLLISION': GATLimitleri.CARPISMA
        }
        n = len(aktif_rovs)
        positions = [r.position for r in aktif_rovs]
        x = torch.zeros((n, 9), dtype=torch.float)

        # Mesafe matrisi (tek seferde hesapla)
        dist_matrix = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                if i != j:
                    dist_matrix[i][j] = distance(positions[i], positions[j])

        sources, targets = [], []
        for i in range(n):
            code = 0
            # Kod 5: Liderden uzak
            if i != 0 and dist_matrix[i][0] > L['LEADER']:
                code = 5
            # Kod 3: Bağlantı kopması
            dists_other = [dist_matrix[i][j] for j in range(n) if j != i]
            if dists_other and min(dists_other) > L['DISCONNECT']:
                code = 3
            # Kod 1: Engel yakınlığı
            min_engel = 999.0
            for engel in engeller:
                d = distance(positions[i], engel.position) - 6.0
                min_engel = min(min_engel, d)
            if min_engel < L['OBSTACLE']:
                code = 1
            # Kod 2: Çarpışma riski
            for j in range(n):
                if i != j and dist_matrix[i][j] < L['COLLISION']:
                    code = 2
                    break

            rov = aktif_rovs[i]
            x[i][0] = code / 5.0
            x[i][1] = getattr(rov, 'battery', 100.0)
            x[i][2] = 0.9
            x[i][3] = min(1.0, abs(getattr(rov, 'y', 0.0)) / 100.0)
            x[i][4] = getattr(getattr(rov, 'velocity', None), 'x', 0.0) or 0.0
            x[i][5] = getattr(getattr(rov, 'velocity', None), 'z', 0.0) or 0.0
            x[i][6] = 1.0 if getattr(rov, 'role', 0) == 1 else 0.0
            if n > 1:
                min_rov_d = min(dist_matrix[i][j] for j in range(n) if j != i)
                x[i][7] = min(min_rov_d / 100.0, 1.0)
            if i != 0:
                x[i][8] = min(dist_matrix[i][0] / 100.0, 1.0)

            for j in range(n):
                if i != j and dist_matrix[i][j] < L['DISCONNECT']:
                    sources.append(i)
                    targets.append(j)

        edge_index = torch.tensor([sources, targets], dtype=torch.long)
        return MiniData(x, edge_index)
