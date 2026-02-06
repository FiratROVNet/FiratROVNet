"""
GNC Helper Module
Mathematical calculations, geometric operations, and complex logic for Filo and TemelGNC classes.
"""

from ast import Pass
import numpy as np
import math
import random

from panda3d.core import loadPrcFileData

# Log seviyesini 'fatal' yaparak sadece hayati hataları gösterir, bilgi mesajlarını gizler
loadPrcFileData('', 'notify-level fatal')
loadPrcFileData('', 'notify-level-util fatal')
from ursina import Vec3, time

# Alpha Shape ve Shapely için import (kontur hesaplama için)
try:
    import alphashape
    ALPHASHAPE_AVAILABLE = True
except ImportError:
    ALPHASHAPE_AVAILABLE = False

try:
    from shapely.geometry import Point, LineString, Polygon, MultiPolygon
    from shapely.ops import unary_union, nearest_points
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False

try:
    from scipy.spatial import ConvexHull
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

# Import from FiratROVNet.config
try:
    from FiratROVNet.config import Formasyon, HareketAyarlari, FizikSabitleri, GATLimitleri
except ImportError:
    # Fallback: try relative import if running from within package
    try:
        from ..FiratROVNet.config import Formasyon, HareketAyarlari, FizikSabitleri, GATLimitleri
    except ImportError:
        # Last resort: try direct import
        import sys
        import os
        sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
        from FiratROVNet.config import Formasyon, HareketAyarlari, FizikSabitleri, GATLimitleri

# #region agent log
import json as _json_mod
_DEBUG_LOG_PATH = "/home/celik/github/ROV/FiratRovNet-org/.cursor/debug.log"
def _agent_log(location, message, data, hypothesis_id, run_id="run1"):
    try:
        with open(_DEBUG_LOG_PATH, "a", encoding="utf-8") as _f:
            _f.write(_json_mod.dumps({"timestamp": __import__("time").time() * 1000, "location": location, "message": message, "data": data, "hypothesisId": hypothesis_id, "sessionId": "debug-session", "runId": run_id}) + "\n")
    except Exception:
        pass
# #endregion


class Hidrodinamik:
    # --- Ortam Sabitleri ---
    SU_YOGUNLUGU = 1000.0  # kg/m^3 (Tatlı su için 1000, Tuzlu su için 1025)
    YER_CEKIMI = 9.81      # m/s^2

    # --- ROV Fiziksel Özellikleri ---
    KUTLE = 12.0           # kg (ROV'un ağırlığı)
    HACIM = 0.0122         # m^3 (Batması/Çıkması için: Hacim * Su Yoğunluğu > Kütle ise yüzer)
                           # Örn: 0.0122 * 1000 = 12.2 kg kaldırma kuvveti (Nötr'e yakın ama hafif pozitif)
    
    # --- Motor Özellikleri ---
    MAX_ITME_KUVVETI = 50.0 # Newton (Toplam motor gücü, örn: T200 motorlar için ~5kgf)
    
    # --- Sürtünme (Drag) Katsayısı ---
    # F_drag = 0.5 * rho * v^2 * Cd * A
    DRAG_KATSAYISI_CD = 0.8  # Kutu gibi şekiller için 0.8 - 1.0 arası
    ON_YUZEY_ALANI = 0.15    # m^2 (ROV'un suyla temas eden ön yüzeyi)



class BasitKalmanFiltresi:
    def __init__(self, R=0.1, Q=0.1, baslangic_degeri=0.0):
        """
        1 Boyutlu Basit Kalman Filtresi.
        Args:
            R: Ölçüm Gürültüsü (Yüksek R = Daha fazla yumuşatma, daha yavaş tepki)
            Q: Süreç Gürültüsü (Yüksek Q = Daha hızlı tepki, daha az yumuşatma)
        """
        self.R = R  # Measurement Noise (Sensör/Girdi hatası varsayımı)
        self.Q = Q  # Process Noise (Sistemin kendi değişim hızı)
        self.P = 1.0  # Estimation Error Covariance (Başlangıç hatası)
        self.x = baslangic_degeri  # State (Tahmin edilen değer)

    def guncelle(self, olcum):
        # 1. Tahmin (Prediction)
        # Hareket komutlarında bir önceki durumun korunduğunu varsayıyoruz
        x_pred = self.x
        p_pred = self.P + self.Q

        # 2. Güncelleme (Update)
        K = p_pred / (p_pred + self.R)  # Kalman Kazancı (Gain)
        self.x = x_pred + K * (olcum - x_pred)  # Yeni tahmin
        self.P = (1 - K) * p_pred  # Hata kovaryansını güncelle
        
        return self.x


class BasitKalmanFiltresi:
    def __init__(self, R=0.1, Q=0.1, baslangic_degeri=0.0):
        """
        1 Boyutlu Basit Kalman Filtresi.
        Args:
            R: Ölçüm Gürültüsü (Yüksek R = Daha fazla yumuşatma, daha yavaş tepki)
            Q: Süreç Gürültüsü (Yüksek Q = Daha hızlı tepki, daha az yumuşatma)
        """
        self.R = R  # Measurement Noise (Sensör/Girdi hatası varsayımı)
        self.Q = Q  # Process Noise (Sistemin kendi değişim hızı)
        self.P = 1.0  # Estimation Error Covariance (Başlangıç hatası)
        self.x = baslangic_degeri  # State (Tahmin edilen değer)

    def guncelle(self, olcum):
        # 1. Tahmin (Prediction)
        # Hareket komutlarında bir önceki durumun korunduğunu varsayıyoruz
        x_pred = self.x
        p_pred = self.P + self.Q

        # 2. Güncelleme (Update)
        K = p_pred / (p_pred + self.R)  # Kalman Kazancı (Gain)
        self.x = x_pred + K * (olcum - x_pred)  # Yeni tahmin
        self.P = (1 - K) * p_pred  # Hata kovaryansını güncelle
        
        return self.x

class FiloHelper:
    """
    Helper class for Filo complex calculations and geometric operations.
    Contains heavy mathematical logic extracted from Filo class.
    Initialized with Filo instance to access self.sistemler and self.ortam_ref.
    """
    
    def __init__(self, filo_ref):
        """
        Initialize helper with Filo instance reference.
        
        Args:
            filo_ref: Reference to Filo instance (self)
        """
        self.filo = filo_ref
        self._vektor_baslangic = None  # int (ROV id) veya (x, z) nokta — başlangıç
        self._vektor_bitis = None     # int (ROV id) veya (x, z) nokta — bitiş
        self._vektor_renk = 'm'       # k=kırmızı, y=yeşil, m=mavi, s=sarı, t=turuncu (varsayılan mavi)
        self._vektor_uzunluk_metre = 10.0  # haritada ok uzunluğu (metre), filo.vektor(..., uzunluk=10)
        self._vektor_reverse = False  # True ise vektör 180° döner (ok ters yön)
        self._apf_vektor_list = []  # apf() ile set edilir: [{'baslangic':(x,z), 'bitis':(x,z), 'renk':str, 'uzunluk':float, 'reverse':bool}, ...]
        self._apf_prev_vektor = {}  # rov_id -> (ux, uz) temporal smoothing için
        # Koordinator'u lazy import için cache (circular import önleme)
        self._koordinator = None

    def get(self, rov_id: int = None, veri_tipi: str = None, taraf: int = None, koordinator=None, sessiz: bool = False):
        """
        ROV bilgilerini alır.

        Args:
            rov_id: ROV ID (0, 1, 2, ...) veya None (tüm ROV'lar için)
            veri_tipi: Veri tipi ('gps', 'hiz', 'batarya', 'rol', 'renk', 'sensör',
                                  'engel_mesafesi', 'iletisim_menzili', 'min_pil_uyarisi',
                                  'kacinma_mesafesi', 'sonar', 'lidar', 'yaw', 'engels')
                                  veya None (tüm ROV'ların GPS koordinatları)
            taraf: Lidar için yön parametresi (sadece 'lidar' için geçerli)
            koordinator: Koordinat dönüştürücü (Koordinator.ursina_to_sim)
            sessiz: Hata mesajlarını bastırır (RL eğitimi için)

        Returns:
            İstenen veri tipine göre değer veya tüm ROV'ların koordinatları
        """
        if rov_id is None and veri_tipi is None:
            return self.filo._get_all_rovs_positions()

        if len(self.filo.sistemler) == 0:
            if not sessiz:
                print("❌ [HATA] GNC sistemleri henüz kurulmamış!")
            return None

        if rov_id is not None and (not isinstance(rov_id, int) or rov_id < 0):
            if not sessiz:
                print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
                print(f"   Mevcut ROV sayısı: {len(self.filo.sistemler)} (0-{len(self.filo.sistemler)-1} arası)")
            return None

        if rov_id is not None and rov_id >= len(self.filo.sistemler):
            if not sessiz:
                print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
                print(f"   Mevcut ROV sayısı: {len(self.filo.sistemler)} (0-{len(self.filo.sistemler)-1} arası)")
            return None

        if rov_id is None:
            print("❌ [HATA] ROV ID belirtilmedi!")
            return None

        # None kontrolü (çıkarılmış ROV'lar için sistem yoksa None olabilir)
        if rov_id < len(self.filo.sistemler) and self.filo.sistemler[rov_id] is None:
            return None

        try:
            # Sistem kontrolü
            if self.filo.sistemler[rov_id] is None:
                if not sessiz:
                    print(f"⚠️ [GET] ROV-{rov_id} için sistem bulunamadı (None)")
                return None
            
            sistem = self.filo.sistemler[rov_id]
            if not hasattr(sistem, 'rov') or sistem.rov is None:
                if not sessiz:
                    print(f"⚠️ [GET] ROV-{rov_id} için ROV entity bulunamadı")
                return None
            
            rov = sistem.rov
            
            if veri_tipi == "lidar":
                deger = rov.get(veri_tipi, taraf=taraf)
            elif veri_tipi == "gps":
                ursina_gps = rov.get("gps")
                if ursina_gps is not None:
                    if isinstance(ursina_gps, np.ndarray):
                        ursina_gps = tuple(ursina_gps.tolist())
                    elif isinstance(ursina_gps, (tuple, list)):
                        ursina_gps = tuple(ursina_gps)
                    if koordinator:
                        deger = koordinator.ursina_to_sim(*ursina_gps)
                    else:
                        deger = ursina_gps
                else:
                    deger = None
            elif veri_tipi == "engels":
                deger = self.filo._compute_obstacle_positions(rov_id)
            else:
                deger = rov.get(veri_tipi)

            if deger is None:
                print(f"⚠️ [GET] ROV-{rov_id} için '{veri_tipi}' veri tipi bulunamadı veya None döndü")
            return deger
        except Exception as e:
            print(f"❌ [HATA] Veri alma sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return None

    def points(self) -> list:
        """
        Tüm ROV koordinatlarını ve tüm engel koordinatlarını birleştirip döndürür.
        """
        all_points = []
        try:
            rovs_positions = self.filo._get_all_rovs_positions()
            for _, position in rovs_positions.items():
                if position is not None:
                    all_points.append(position)

            for rov_id in rovs_positions.keys():
                engels = self.filo._compute_obstacle_positions(rov_id)
                if engels:
                    all_points.extend(engels)
        except Exception as e:
            print(f"❌ [HATA] Points hesaplanırken hata: {e}")
            import traceback
            traceback.print_exc()

        return all_points

    def compute_obstacle_positions(self, rov_id: int) -> list:
        """
        ROV'un tüm lidar sensörlerinden engel koordinatlarını hesaplar.
        Simülasyon formatında (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik) çalışır.
        """
        LIDAR_OFFSETS = {
            0: 0,     # ön
            1: -90,   # sağ
            2: 90     # sol
        }
        obstacles = []

        try:
            gps = self.filo.get(rov_id, "gps")
            if gps is None:
                return []

            x0, y0, z0 = gps[0], gps[1], gps[2]
            yaw_deg = self.filo.get(rov_id, "yaw") or 0.0

            for lidar_indis in [0, 1, 2]:
                distance = self.filo.get(rov_id, "lidar", lidar_indis)
                if distance is not None and distance > 0 and distance != -1:
                    offset = LIDAR_OFFSETS[lidar_indis]
                    theta_rad = math.radians(yaw_deg + offset)
                    ox = x0 + distance * math.sin(theta_rad)
                    oy = y0 + distance * math.cos(theta_rad)
                    oz = z0
                    obstacles.append((ox, oy, oz))
        except Exception as e:
            print(f"❌ [HATA] Engel koordinatları hesaplanırken hata: {e}")
            import traceback
            traceback.print_exc()

        return obstacles

    def _engel_radius_al(self, entity, hit_pt_2d):
        """Hit entity veya ortam.island_positions'tan engel yarıçapını (metre) döndürür."""
        if entity is not None:
            try:
                sx = getattr(entity, 'scale_x', None)
                if sx is not None:
                    return float(sx) / 2.0
                scale = getattr(entity, 'scale', None)
                if scale is not None:
                    if isinstance(scale, (int, float)):
                        return float(scale) / 2.0
                    s0 = scale[0] if hasattr(scale, '__getitem__') else getattr(scale, 'x', 0)
                    return float(s0) / 2.0 if s0 else 0.0
            except (TypeError, ValueError):
                pass
        ortam = getattr(self.filo, 'ortam_ref', None)
        if ortam and getattr(ortam, 'island_positions', None) and hit_pt_2d:
            hx, hz = float(hit_pt_2d[0]), float(hit_pt_2d[1])
            best_r, best_d = 0.0, float('inf')
            for ip in ortam.island_positions:
                if len(ip) < 3:
                    continue
                ix, iz, ir = float(ip[0]), float(ip[1]), float(ip[2])
                d = math.sqrt((hx - ix) ** 2 + (hz - iz) ** 2)
                if d < best_d:
                    best_d, best_r = d, ir
            return best_r if best_d < float('inf') else 0.0
        return 0.0

    def _engel_bul_cache_sonuc(self, rov, rov_id: int, menzil: float) -> list:
        """
        Ana thread dışından engel_bul çağrıldığında: ROV'un sonar/lidar önbelleğinden
        (son_sonar_mesafesi, son_lidar_mesafeleri) engel listesi oluşturur. Raycast atılmaz.
        """
        try:
            from ursina import Vec3
        except ImportError:
            return []
        sonar = getattr(rov, 'son_sonar_mesafesi', -1)
        lidar = getattr(rov, 'son_lidar_mesafeleri', None)
        if lidar is None:
            lidar = {}
        # Önbellekte geçerli mesafe var mı?
        lidar_0 = lidar.get(0, -1)
        lidar_1 = lidar.get(1, -1)
        lidar_2 = lidar.get(2, -1)
        if sonar < 0 and lidar_0 < 0 and lidar_1 < 0 and lidar_2 < 0:
            return []
        # ROV konumu ve yaw (derece) — ana engel_bul ile aynı koordinat dönüşümü
        origin = Vec3(rov.world_position.x, rov.world_position.y, rov.world_position.z) + Vec3(0, 0.5, 0)
        yaw_deg = 0.0
        if hasattr(rov, 'rotation') and rov.rotation is not None:
            if hasattr(rov.rotation, 'y'):
                yaw_deg = float(rov.rotation.y)
            elif isinstance(rov.rotation, (tuple, list)) and len(rov.rotation) >= 2:
                yaw_deg = float(rov.rotation[1])
        yaw_rad = math.radians(yaw_deg)
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        # Ursina: Z=ileri, X=sağ, Y=yukarı. Lokal ileri=(0,0,1), sag=(1,0,0), sol=(-1,0,0)
        def global_vektor(lx, ly, lz):
            gx = lx * c + lz * s
            gz = -lx * s + lz * c
            return Vec3(gx, ly, gz).normalized()
        ileri = global_vektor(0, 0, 1)
        sag = global_vektor(1, 0, 0)
        sol = global_vektor(-1, 0, 0)
        sonuclar = []
        if sonar > 0 and sonar <= menzil:
            sonuclar.append({
                'koordinat': origin + ileri * sonar,
                'mesafe': sonar,
                'vektor': ileri,
                'yon': 'ileri',
                'radius': 0.0,
            })
        if lidar_0 > 0 and lidar_0 <= menzil:
            sonuclar.append({
                'koordinat': origin + ileri * lidar_0,
                'mesafe': lidar_0,
                'vektor': ileri,
                'yon': 'on_lidar',
                'radius': 0.0,
            })
        if lidar_1 > 0 and lidar_1 <= menzil:
            sonuclar.append({
                'koordinat': origin + sol * lidar_1,
                'mesafe': lidar_1,
                'vektor': sol,
                'yon': 'sol_lidar',
                'radius': 0.0,
            })
        if lidar_2 > 0 and lidar_2 <= menzil:
            sonuclar.append({
                'koordinat': origin + sag * lidar_2,
                'mesafe': lidar_2,
                'vektor': sag,
                'yon': 'sag_lidar',
                'radius': 0.0,
            })
        return sonuclar

    def engel_bul(self, rov_id: int, menzil: float = None, debug: bool = False) -> list:
            """
            ROV için 3D çevresel tarama yapar (Çapraz yönler kaldırıldı).
            Aşağı yönlü sensör menzili GATLimitleri.CARPISMA (10m) ile sınırlıdır.
            """
            import math
            from ursina import raycast, Vec3, color, Entity, destroy

            # 1. PARAMETRE VE ROV KONTROLÜ
            if menzil is None: menzil = GATLimitleri.ENGEL
            ASAGI_MENZIL = GATLimitleri.CARPISMA 

            gnc_sistemi = self.filo.sistemler[rov_id] if 0 <= rov_id < len(self.filo.sistemler) else None
            rov = getattr(gnc_sistemi, 'rov', None)
            if not rov: return []

            # 2. THREAD KONTROLÜ (Konsol Uyumu - Cache Döner)
            if not self.filo._is_main_thread():
                son_tespitler = getattr(rov, '_son_engeller', [])
                return [e for e in son_tespitler if e['mesafe'] <= menzil]

            # 3. DEBUG VE IGNORE LİSTESİ TEMİZLİĞİ
            if not hasattr(self.filo, '_debug_noktalari'): self.filo._debug_noktalari = []
            for obj in self.filo._debug_noktalari: destroy(obj)
            self.filo._debug_noktalari.clear()

            # ROV'un kendisini ve diğer tüm ROV'ları görmezden gel
            ignore_list = [rov]
            if getattr(rov, 'safety_zone', None): ignore_list.append(rov.safety_zone)
            ortam = self.filo.ortam_ref
            if ortam and hasattr(ortam, 'rovs'):
                for other in ortam.rovs:
                    if other and other != rov: ignore_list.append(other)
            ignore_tuple = tuple(ignore_list)

            # 4. TARAMA YÖNLERİ (Sadece Ana Eksenler)
            tarama_yonleri = {
                'ileri': rov.forward,
                'geri': -rov.forward,
                'sag': rov.right,
                'sol': -rov.right,
                'yukari': rov.up,
                'asagi': -rov.up
            }

            origin = rov.world_position + Vec3(0, 0.5, 0)
            sonuclar = []

            # 5. RAYCAST DÖNGÜSÜ
            for ad, yon in tarama_yonleri.items():
                # Aşağı bakarken 10m, diğer yönlerde tam menzil kullan
                aktif_menzil = ASAGI_MENZIL if ad == 'asagi' else menzil

                hit = raycast(origin, yon, distance=aktif_menzil, ignore=ignore_tuple, debug=False)
                
                if hit.hit:
                    pt = hit.world_point
                    # Ursina (x, y, z) -> Simülasyon (x, z, -y)
                    sim_koord = (pt.x, pt.z, -pt.y) 
                    
                    res = {
                        'yon': ad,
                        'mesafe': hit.distance,
                        'koordinat': sim_koord,
                        'radius': self._engel_radius_al(hit.entity, (pt.x, pt.z))
                    }
                    sonuclar.append(res)

                    # 6. HARİTAYA (MİNİMAP) EKLEME KOŞULLARI
                    # 'yukari' ve 'asagi' (zemin/tavan) haritada kirlilik yapmasın diye eklenmez
                    if ad not in ['yukari', 'asagi'] and ortam and hasattr(ortam, 'engel_bulutu'):
                        # Yoğunluk filtresi: Aynı noktaya çok yakın (2m) başka nokta yoksa ekle
                        is_unique = True
                        for old_pt in ortam.engel_bulutu[-50:]:
                            if (old_pt[0]-sim_koord[0])**2 + (old_pt[1]-sim_koord[1])**2 < 4.0:
                                is_unique = False
                                break
                        if is_unique:
                            ortam.engel_bulutu.append((sim_koord[0], sim_koord[1]))

                    # 3D Debug Görseli (Kırmızı Küreler)
                    if debug:
                        dot = Entity(model='sphere', color=color.red, scale=0.3, position=pt, unlit=True)
                        self.filo._debug_noktalari.append(dot)

            # 7. CACHE GÜNCELLEME
            rov._son_engeller = sonuclar
            return sonuclar

    def yakinlastir(self, rov_id1: int, rov_id2: int, mesafe: float) -> bool:
        """
        rov_id1'i rov_id2'ye yatay düzlemde (X,Z) mesafe kadar yaklaştırır.
        Sadece rov_id1 hareket eder; hedef konum filo.git() ile atanır (ROV o noktaya gider).
        Ursina: (x, z) yatay; Sim: (x: Sağ-Sol, y: İleri-Geri, z: Derinlik) => Sim(x, Ursina.z, Ursina.y).
        """
        if not getattr(self.filo, 'sistemler', None):
            return False
        n = len(self.filo.sistemler)
        if rov_id1 < 0 or rov_id1 >= n or rov_id2 < 0 or rov_id2 >= n or rov_id1 == rov_id2:
            return False
        rov1 = getattr(self.filo.sistemler[rov_id1], 'rov', None)
        rov2 = getattr(self.filo.sistemler[rov_id2], 'rov', None)
        if rov1 is None or rov2 is None:
            return False
        pos1 = getattr(rov1, 'world_position', rov1)
        pos2 = getattr(rov2, 'world_position', rov2)
        x1 = getattr(pos1, 'x', getattr(rov1, 'x', 0.0))
        y1 = getattr(pos1, 'y', getattr(rov1, 'y', 0.0))
        z1 = getattr(pos1, 'z', getattr(rov1, 'z', 0.0))
        x2 = getattr(pos2, 'x', getattr(rov2, 'x', 0.0))
        z2 = getattr(pos2, 'z', getattr(rov2, 'z', 0.0))
        dx = x2 - x1
        dz = z2 - z1
        d = math.sqrt(dx * dx + dz * dz)
        if d < 1e-9:
            return True
        # rov_id1, rov_id2'ye mesafe kadar yaklaşacak (sadece rov1 hareket eder)
        move = min(mesafe, d)
        ux = dx / d
        uz = dz / d
        new_x1 = x1 + ux * move
        new_z1 = z1 + uz * move
        # Sim format: x = Sağ-Sol (Ursina x), y = İleri-Geri (Ursina z), z = Derinlik (Ursina y)
        sim_x = new_x1
        sim_y = new_z1
        sim_z = y1
        try:
            self.filo.git(rov_id1, sim_x, sim_y, sim_z)
        except Exception:
            return False
        return True

    # Vektör renk kodu -> minimap'te kullanılır: k=kırmızı, y=yeşil, m=mavi, s=sarı, t=turuncu
    VEKTOR_RENK_KODLARI = ('k', 'y', 'm', 's', 't')

    def _vektor_arg_norm(self, arg):
        """Tek argümanı normalize eder: ROV ID (int) veya nokta (x, z) tuple. Nokta = len>=2 sequence."""
        if arg is None:
            return None
        try:
            if hasattr(arg, '__len__') and len(arg) >= 2:
                return (float(arg[0]), float(arg[1]))
        except (TypeError, ValueError, IndexError):
            pass
        return int(arg)
    def vektor(self, ilk=None, ikinci=None,
                    rov_id_ilk=None, rov_id_ikinci=None,
                    baslangic_noktasi=None, bitis_noktasi=None, vektor=None,
                    renk='m', uzunluk=20.0, reverse=False, debug=False, ciz=False):
            """
            Gelişmiş 3D Vektör Metodu.
            - Okun uzunluğu her zaman 'uzunluk' parametresine eşittir.
            - Okun yönü, verilen hedef noktasına (bitis_noktasi) doğrudur.
            - Koordinatlar 'filo.get(id, "gps")' üzerinden (x, y, z) birebir alınır.
            """
            import math
            from ursina import Vec3
            
            # 1. PARAMETRE AYARLARI
            self._vektor_renk = renk if renk in self.VEKTOR_RENK_KODLARI else 'm'
            self._vektor_reverse = bool(reverse)
            # Eğer uzunluk verilmemişse varsayılan 20 birim
            self._vektor_uzunluk_metre = float(uzunluk) if uzunluk is not None else 20.0
            
            ortam = getattr(self.filo, 'ortam_ref', None)

            # 2. BAŞLANGIÇ NOKTASI (POS1) - GPS'ten Birebir
            pos1 = None
            rid1 = rov_id_ilk if rov_id_ilk is not None else (ilk if isinstance(ilk, int) else None)
            
            if baslangic_noktasi is not None:
                pos1 = (float(baslangic_noktasi[0]), float(baslangic_noktasi[1]), float(baslangic_noktasi[2]))
            elif rid1 is not None:
                pos1 = self.filo.get(rid1, "gps") # (x, y, z)

            if pos1 is None: return None

            # 3. YÖN (BİRİM VEKTÖR) HESABI
            ux, uy, uz = 0.0, 0.0, 0.0
            gercek_hedef_pos = None
            gercek_mesafe = 0.0

            # DURUM A: Yön Vektörü Modu (Başlangıç + Yön)
            if vektor is not None:
                try:
                    vx, vy, vz = float(vektor[0]), float(vektor[1]), float(vektor[2])
                    mag = math.sqrt(vx**2 + vy**2 + vz**2)
                    if mag > 1e-9:
                        ux, uy, uz = vx/mag, vy/mag, vz/mag
                    gercek_mesafe = self._vektor_uzunluk_metre
                    gercek_hedef_pos = (pos1[0] + ux * mag, pos1[1] + uy * mag, pos1[2] + uz * mag)
                except: return None

            # DURUM B: İki Nokta Modu (Başlangıç + Hedef)
            else:
                rid2 = rov_id_ikinci if rov_id_ikinci is not None else (ikinci if isinstance(ikinci, int) else None)
                if bitis_noktasi is not None:
                    gercek_hedef_pos = (float(bitis_noktasi[0]), float(bitis_noktasi[1]), float(bitis_noktasi[2]))
                elif rid2 is not None:
                    gercek_hedef_pos = self.filo.get(rid2, "gps")

                if gercek_hedef_pos is not None:
                    dx, dy, dz = gercek_hedef_pos[0]-pos1[0], gercek_hedef_pos[1]-pos1[1], gercek_hedef_pos[2]-pos1[2]
                    gercek_mesafe = math.sqrt(dx**2 + dy**2 + dz**2)
                    if gercek_mesafe > 1e-9:
                        ux, uy, uz = dx/gercek_mesafe, dy/gercek_mesafe, dz/gercek_mesafe
                else:
                    return None

            # Ters Çevirme (Reverse)
            if self._vektor_reverse:
                ux, uy, uz = -ux, -uy, -uz

            # 4. GÖRSEL ÇİZİM NOKTASI (POS2)
            # Okun bittiği yer: Başlangıç + (Yön * Senin İstediğin Sabit Uzunluk)
            pos2_cizim = (
                pos1[0] + ux * self._vektor_uzunluk_metre,
                pos1[1] + uy * self._vektor_uzunluk_metre,
                pos1[2] + uz * self._vektor_uzunluk_metre
            )

            # 5. SONUÇLAR
            ret = {
                'baslangic_3d': pos1,
                'bitis_3d': gercek_hedef_pos,    # Gerçek hedef koordinatın (Data için)
                'birim_vektor_3d': (ux, uy, uz),
                'uzaklik_metre': float(gercek_mesafe)
            }

            # 6. MİNİMAP ÇİZİM LİSTESİ
            if ciz:
                if debug: self._apf_vektor_list = []
                self._apf_vektor_list.append({
                    'baslangic': pos1,
                    'bitis': pos2_cizim,       # Minimap sadece 'uzunluk' kadar çizer
                    'renk': self._vektor_renk,
                    'uzunluk': self._vektor_uzunluk_metre,
                    'rov_id': rid1
                })

            return ret

    def _vektor_poz_al_3d(self, rov_id, ortam):
            """ROV'un 3D pozisyonunu simülasyon formatında döner."""
            if not ortam or not hasattr(ortam, 'rovs'): return None
            for r in ortam.rovs:
                if r and getattr(r, 'id', None) == rov_id:
                    # Ursina (x, y, z) -> Simülasyon (x, y_ileri, z_derinlik)
                    # Ursina Y yukarıdır, Simülasyonda Derinlik (Z) aşağıdır.
                    return (r.x, r.z, -r.y)
            return None
    def get_vektor_renk(self):
        """Minimap vektör çizgisi için renk kodu döner: k, y, m, s, t (varsayılan m)."""
        return getattr(self, '_vektor_renk', 'm')

    def _vektor_verts_birim(self, p1, p2, z_line=-0.37, havuz_genisligi=200.0, uzunluk_metre=None, reverse=False):
        """
        Harita koordinatında p1 -> p2 yönünde (reverse=True ise ters yönde) sabit uzunlukta vektör köşe listesi döner.
        Vektör uzunluğu: merkez–kenar = havuz_genisligi m, harita 0.5 birim;
        birim_uzunluk = uzunluk_metre / (2*havuz_genisligi) harita birimi.
        Ok ucu: vektör uzunluğunun 1/6'ı, 145° ve 225°.
        """
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        d = math.sqrt(dx * dx + dy * dy)
        if d >= 1e-9:
            ux, uy = dx / d, dy / d
            if reverse:
                ux, uy = -ux, -uy
            h = max(float(havuz_genisligi), 1.0)
            if uzunluk_metre is None:
                uzunluk_metre = getattr(self, '_vektor_uzunluk_metre', 10.0)
            birim_uzunluk = float(uzunluk_metre) / (2.0 * h)  # harita birimi (metre ölçeğine göre)
            ex = p1.x + ux * birim_uzunluk
            ey = p1.y + uy * birim_uzunluk
            # Vektör açısı (radyan), ok ucu kanatları bu açıya göre 145° ve 225°
            vektor_aci = math.atan2(uy, ux)
            kanat_uzunluk = birim_uzunluk / 6.0
            aci_145 = vektor_aci + math.radians(145)
            aci_225 = vektor_aci + math.radians(225)
            w1x = ex + kanat_uzunluk * math.cos(aci_145)
            w1y = ey + kanat_uzunluk * math.sin(aci_145)
            w2x = ex + kanat_uzunluk * math.cos(aci_225)
            w2y = ey + kanat_uzunluk * math.sin(aci_225)
            # Ana çizgi + iki kanat (uc -> 145°, uc -> 225°)
            return [
                (p1.x, p1.y, z_line), (ex, ey, z_line),
                (ex, ey, z_line), (w1x, w1y, z_line),
                (ex, ey, z_line), (w2x, w2y, z_line),
            ]
        return [(p1.x, p1.y, z_line), (p1.x, p1.y, z_line)]

    def _vektor_poz_al(self, arg, ortam):
        """Tek vektör argümanını (ROV id veya (x,z) veya (x,y,z) nokta) dünya koordinatına çevirir. Bulunamazsa None.
        - Eğer arg tuple/list ise: 3 elemanlıyse (x,y,z) döner, 2 elemanlıysa (x,z) döner.
        - Eğer arg ROV id ise, geriye uyumluluk için (x,z) döner (y mevcutsa kullanılmaz).
        """
        if arg is None:
            return None
        try:
            if isinstance(arg, (tuple, list)) and len(arg) >= 3:
                return (float(arg[0]), float(arg[1]), float(arg[2]))
            if isinstance(arg, (tuple, list)) and len(arg) >= 2:
                return (float(arg[0]), float(arg[1]))
        except (TypeError, ValueError, IndexError):
            pass
        rid = int(arg)
        if ortam is None or not hasattr(ortam, 'rovs') or not ortam.rovs:
            return None
        for rov in ortam.rovs:
            if rov is None:
                continue
            if getattr(rov, 'id', None) == rid:
                return (rov.x, rov.z)
        return None

    def get_vektor_verts(self, minimap):
        """
        Minimap için vektör çizgi köşe listesini hesaplar (birim vektör, sabit uzunluk).
        Başlangıç/bitiş hibrit: ROV ID ise o ROV konumu, nokta ise verilen (x,z) kullanılır.

        Returns:
            list | None: [(x1,y1,z), (x2,y2,z)] veya None.
        """
        baslangic = getattr(self, '_vektor_baslangic', None)
        bitis = getattr(self, '_vektor_bitis', None)
        if baslangic is None or bitis is None:
            return None
        ortam = getattr(self.filo, 'ortam_ref', None)
        pos1 = self._vektor_poz_al(baslangic, ortam)
        pos2 = self._vektor_poz_al(bitis, ortam)
        if pos1 is None or pos2 is None:
            return None
        z_line = -0.37
        havuz_genisligi = getattr(minimap, 'havuz_genisligi', 200.0)
        uzunluk_metre = getattr(self, '_vektor_uzunluk_metre', 10.0)
        reverse = getattr(self, '_vektor_reverse', False)
        p1 = minimap.dunya_to_harita(pos1[0], pos1[1])
        p2 = minimap.dunya_to_harita(pos2[0], pos2[1])
        return self._vektor_verts_birim(p1, p2, z_line, havuz_genisligi, uzunluk_metre=uzunluk_metre, reverse=reverse)

    def _yatay_mesafe(self, p1, p2):
        """İki nokta (x, z) arasındaki yatay mesafeyi metre cinsinden döner."""
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    def _mesafe(self, p1, p2, is_3d: bool = False):
        """
        İki nokta arasındaki mesafeyi hesaplar. is_3d True ise 3B, değilse 2B (x,z) kullanır.
        p1/p2: (x,z) veya (x,y,z) tuple olabilir.
        """
        try:
            if is_3d:
                x1, y1, z1 = float(p1[0]), float(p1[1]), float(p1[2])
                x2, y2, z2 = float(p2[0]), float(p2[1]), float(p2[2])
                return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
            else:
                return math.sqrt((float(p2[0]) - float(p1[0])) ** 2 + (float(p2[1]) - float(p1[1])) ** 2)
        except Exception:
            return float('inf')

    def _apf_rep_factor_hesapla(self, mesafe: float, r_etki: float, d_min: float = 0.5):
        """
        APF için repulsive factor hesaplar (mesafe ile ters orantılı).
        
        Args:
            mesafe: ROV ile obje arasındaki mesafe (metre)
            r_etki: Etki menzili (metre)
            d_min: Minimum güvenli mesafe (metre, varsayılan 0.5)
        
        Returns:
            float: Repulsive factor (mesafe azaldıkça artar)
        """
        d_safe = max(mesafe, d_min)
        return (1.0 / d_safe - 1.0 / r_etki)

    def _apf_kuvvet_hesapla(self, mesafe: float, birim_vektor: tuple, r_etki: float, 
                           k_itme: float, k_teget: float, r_yakin_artis: float, 
                           yakin_carpan: float, teget_carpan: float = 1.0):
        """
        APF için itme ve teğet kuvvetlerini hesaplar.
        Engel vektörü karesel olarak artar (mesafe azaldıkça).
        
        Args:
            mesafe: ROV ile obje arasındaki mesafe (metre)
            birim_vektor: (ux, uz) birim vektör (ROV'dan objeye doğru)
            r_etki: Etki menzili (metre)
            k_itme: İtme katsayısı
            k_teget: Teğet kuvvet katsayısı
            r_yakin_artis: Yakın artış mesafesi (metre)
            yakin_carpan: Yakın mesafede kuvvet çarpanı
            teget_carpan: Teğet kuvvet çarpanı (varsayılan 1.0, ROV için 1.5)
        
        Returns:
            tuple: (f_rep_ux, f_rep_uz, f_tan_ux, f_tan_uz) kuvvet bileşenleri
        """
        # Menzil dışındaki objeler sıfır kuvvet uygular
        if mesafe >= r_etki:
            return (0.0, 0.0, 0.0, 0.0)
        
        ux, uz = birim_vektor[:2]
        rep_factor = self._apf_rep_factor_hesapla(mesafe, r_etki)
        
        # İtme kuvveti (karesel - mesafe azaldıkça karesel artar)
        # birim_vektor ROV->Obstacle yönünde; itme kuvveti obje→ROV yönünde olmalı (repulsive)
        f_rep = k_itme * rep_factor * rep_factor
        if mesafe < r_yakin_artis:
            # Yakın mesafede ekstra karesel artış
            f_rep *= yakin_carpan * (r_yakin_artis / max(mesafe, 0.1)) ** 2
        
        # İtme vektörü: objeden uzağa (-ux, -uz) = itici yön
        f_rep_ux = -ux * f_rep
        f_rep_uz = -uz * f_rep
        
        # Teğet kuvveti (vortex): itici yön (-ux, -uz) ile 90° dik
        # Perpendicular: (-uz, ux) veya (uz, -ux) — engel etrafında dolaşmak için
        f_tan_ux, f_tan_uz = 0.0, 0.0
        if mesafe < (r_etki / 2.0):
            f_tan = (k_teget * teget_carpan) * (rep_factor * rep_factor)
            if mesafe < r_yakin_artis:
                # Yakın mesafede ekstra karesel artış
                f_tan *= yakin_carpan * (r_yakin_artis / max(mesafe, 0.1)) ** 2
            # Teğet: (-uz, ux) — itici yön (-ux,-uz)'ye dik
            f_tan_ux = -uz * f_tan
            f_tan_uz = ux * f_tan
        
        return (f_rep_ux, f_rep_uz, f_tan_ux, f_tan_uz)

    def vektor_normalize(self, ux=None, uz=None, uy=None, max_mag: float = 1.0, vektor=None):
        """
        Vektörü normalize eder ve maksimum büyüklüğe sınırlar. Hem 2D hem 3D vektörleri destekler.
        
        Args:
            ux: X bileşeni (2D veya 3D modu için)
            uz: Z bileşeni (2D modu için) veya Y bileşeni (3D modu için)
            uy: Y bileşeni (3D modu için, opsiyonel). None ise 2D modu kullanılır.
            max_mag: Maksimum büyüklük (varsayılan 1.0)
            vektor: Alternatif kullanım - vektör tuple/list olarak verilebilir.
                - 2D: (ux, uz) veya [ux, uz]
                - 3D: (ux, uy, uz) veya [ux, uy, uz]
                vektor verilirse ux, uz, uy parametreleri göz ardı edilir.
        
        Returns:
            tuple: 
                - 2D modu: (ux_norm, uz_norm, mag) normalize edilmiş vektör ve büyüklüğü
                - 3D modu: (ux_norm, uy_norm, uz_norm, mag) normalize edilmiş vektör ve büyüklüğü
        
        Örnekler:
            # 2D vektör
            ux, uz, mag = self.vektor_normalize(3.0, 4.0)  # (0.6, 0.8, 1.0)
            ux, uz, mag = self.vektor_normalize(vektor=(3.0, 4.0))  # Tuple ile
            
            # 3D vektör
            ux, uy, uz, mag = self.vektor_normalize(2.0, 3.0, 4.0)  # 3D
            ux, uy, uz, mag = self.vektor_normalize(vektor=(2.0, 3.0, 4.0))  # Tuple ile
        """
        # Vektör tuple/list olarak verilmişse kullan
        if vektor is not None:
            try:
                if isinstance(vektor, (tuple, list)):
                    if len(vektor) >= 3:
                        # 3D vektör
                        ux, uy, uz = float(vektor[0]), float(vektor[1]), float(vektor[2])
                    elif len(vektor) >= 2:
                        # 2D vektör
                        ux, uz = float(vektor[0]), float(vektor[1])
                        uy = None
                    else:
                        return None
            except (TypeError, ValueError, IndexError):
                return None
        
        # Parametre kontrolü
        if ux is None or uz is None:
            return None
        
        # 3D modu: uy verilmişse
        if uy is not None:
            mag = math.sqrt(ux * ux + uy * uy + uz * uz)
            if mag < 1e-9:
                return (0.0, 0.0, 0.0, 0.0)
            if mag > max_mag:
                scale = max_mag / mag
                ux_norm = ux * scale
                uy_norm = uy * scale
                uz_norm = uz * scale
                mag = max_mag
            else:
                ux_norm = ux
                uy_norm = uy
                uz_norm = uz
            return (ux_norm, uy_norm, uz_norm, mag)
        
        # 2D modu: uy verilmemişse
        mag = math.sqrt(ux * ux + uz * uz)
        if mag < 1e-9:
            return (0.0, 0.0, 0.0)
        if mag > max_mag:
            scale = max_mag / mag
            ux_norm = ux * scale
            uz_norm = uz * scale
            mag = max_mag
        else:
            ux_norm = ux
            uz_norm = uz
        return (ux_norm, uz_norm, mag)

    def _apf_motor_guc_hesapla(self, min_dist_to_obj: float, r_guvenlik: float, 
                               base_speed: float = 0.15, total_mag: float = 1.0):
        """
        APF için dinamik motor gücü hesaplar (mesafeye göre).
        
        Args:
            min_dist_to_obj: En yakın objeye mesafe (metre)
            r_guvenlik: Güvenlik mesafesi (metre)
            base_speed: Temel hız (varsayılan 0.15)
            total_mag: Toplam vektör büyüklüğü (varsayılan 1.0)
        
        Returns:
            float: Motor gücü [0.0, 1.0] aralığında
        """
        current_power = base_speed
        if min_dist_to_obj < r_guvenlik:
            current_power = 0.35
        elif min_dist_to_obj < (r_guvenlik * 2):
            current_power = 0.25
        
        final_power = current_power * total_mag
        return max(0.0, min(final_power, 1.0))

    def _apf_hedef_gain_hesapla(self, min_dist_to_obj: float, r_yakin_hedef: float = 10.0, 
                                r_etki: float = 20.0, k_hedef: float = 1.0):
        """
        APF için hedef kazancını hesaplar (engel yaklaştıkça azalır).
        10 metreden yakın olduğunda neredeyse 0 olur.
        
        Args:
            min_dist_to_obj: En yakın objeye mesafe (metre)
            r_yakin_hedef: Yakın hedef mesafesi (metre, varsayılan 10.0) - bu mesafeden yakın olduğunda hedef neredeyse 0
            r_etki: Etki menzili (metre, varsayılan 10.0)
            k_hedef: Hedef katsayısı (varsayılan 1.0)
        
        Returns:
            float: Hedef kazancı (0.0 - k_hedef aralığında)
        """
        # 10 metreden yakın olduğunda neredeyse 0
        if min_dist_to_obj < r_yakin_hedef:
            # Karesel azalma: (mesafe / r_yakin_hedef)^2 ile çarp
            ratio = min_dist_to_obj / r_yakin_hedef
            return k_hedef * (ratio ** 2) * 0.01  # Neredeyse 0 (maksimum %1)
        
        # 10 metreden uzakta normal karesel azalma
        ratio_sq = (min_dist_to_obj / r_etki) ** 2
        return min(k_hedef, k_hedef * ratio_sq)

    def _koordinat_cikar(self, obj, x_attr='x', y_attr='y', z_attr='z'):
        """
        Bir objeden koordinatları çıkarır (getattr ile). 
        Obje Vec3, tuple, list veya attribute'lara sahip bir obje olabilir.
        
        Args:
            obj: Koordinat içeren obje (Vec3, tuple, list veya attribute'lara sahip obje)
            x_attr: X koordinatı için attribute adı (varsayılan 'x')
            y_attr: Y koordinatı için attribute adı (varsayılan 'y')
            z_attr: Z koordinatı için attribute adı (varsayılan 'z')
        
        Returns:
            tuple: (x, y, z) veya (x, z) eğer y bulunamazsa. Bulunamazsa (None, None, None).
        """
        if obj is None:
            return (None, None, None)
        
        # Tuple veya list ise
        if isinstance(obj, (tuple, list)):
            if len(obj) >= 3:
                return (float(obj[0]), float(obj[1]), float(obj[2]))
            elif len(obj) >= 2:
                return (float(obj[0]), None, float(obj[1]))
            return (None, None, None)
        
        # Vec3 veya attribute'lara sahip obje ise
        try:
            x = getattr(obj, x_attr, None)
            y = getattr(obj, y_attr, None)
            z = getattr(obj, z_attr, None)
            
            # Eğer x veya z None ise, tuple/list olarak dene
            if x is None or z is None:
                if isinstance(obj, (tuple, list)) and len(obj) >= 2:
                    x = obj[0] if x is None else x
                    z = obj[1] if z is None else z
            
            if x is not None and z is not None:
                return (float(x), float(y) if y is not None else None, float(z))
        except (TypeError, ValueError, AttributeError):
            pass
        
        return (None, None, None)

    def _rov_pozisyon_ursina(self, rov_id=None, rov_entity=None, ortam=None):
        """
        ROV pozisyonunu Ursina koordinatlarında (x, z) döner.
        Mevcut _vektor_poz_al() fonksiyonunu kullanır veya rov_entity'den direkt alır.
        
        Args:
            rov_id: ROV ID (int) - ortam'dan ROV'u bulmak için
            rov_entity: ROV entity objesi - direkt pozisyon almak için
            ortam: Ortam referansı - ROV'u bulmak için
        
        Returns:
            tuple: (x, z) Ursina koordinatları veya None
        """
        # Eğer rov_entity verilmişse direkt kullan
        if rov_entity is not None:
            x = getattr(rov_entity, 'x', None)
            z = getattr(rov_entity, 'z', None)
            if x is not None and z is not None:
                return (float(x), float(z))
        
        # Eğer rov_id verilmişse _vektor_poz_al() kullan
        if rov_id is not None and ortam is not None:
            return self._vektor_poz_al(rov_id, ortam)
        
        return None

    def _koordinator_al(self):
        """Koordinator'u lazy import ile alır (circular import önleme)."""
        if self._koordinator is None:
            from FiratROVNet.gnc import Koordinator
            self._koordinator = Koordinator
        return self._koordinator

    def apf(self, rov_id: int, hedef: bool = True, engel: bool = False, rov: bool = False):
            """
            3D APF Hesaplama. KeyError korumalı versiyon.
            """
           
            self.apf_temizle(rov_id=rov_id)
                
            toplam_vec = [0.0, 0.0, 0.0]
            out_hedef = None
            out_engeller = []
            out_rovs = []

            def add_vec(res_dict):
                if res_dict and 'birim_vektor_3d' in res_dict:
                    bv = res_dict['birim_vektor_3d']
                    toplam_vec[0] += bv[0]
                    toplam_vec[1] += bv[1]
                    toplam_vec[2] += bv[2]
                    return True
                return False

            # Hedef
            if hedef:
                h_koord = self.filo.hedef(rov_id=rov_id)
                if h_koord:
                    res = self.vektor(rov_id_ilk=rov_id, bitis_noktasi=h_koord, renk='y', ciz=True)
                    if add_vec(res):
                        out_hedef = {
                            'birim_vektor': res.get('birim_vektor_3d', (0,0,0)),
                            'mesafe': res.get('uzaklik_metre', 0.0) # HATA BURADAYDI, .get eklendi
                        }

            # Engeller
            if engel:
                # Doğrudan engel_bul fonksiyonunu çağırıyoruz (Raycast taraması yapar)
                tespit_edilenler = self.engel_bul(rov_id=rov_id, menzil=GATLimitleri.ENGEL)
                
                for e in tespit_edilenler:
                    # Engel koordinatı (Simülasyon formatında: x, y_ileri, z_derinlik)
                    target = e.get('koordinat')
                    
                    if target:
                        # KRİTİK: Mesafeyi doğrudan engel_bul sonucundaki 'mesafe' anahtarından alıyoruz.
                        # Bu değer Ursina'nın raycast hit.distance değeridir ve en hassas veridir.
                        sensor_mesafesi = float(e.get('mesafe', 0.0))
                        
                        # 'vektor' metodunu sadece itme yönünü (birim_vektor_3d) hesaplamak 
                        # ve minimap üzerinde 'k' (kara) renginde ok çizmek için kullanıyoruz.
                        # reverse=True olduğu için vektör engelden dışarı doğru (itme) oluşur.
                        res = self.vektor(
                            rov_id_ilk=rov_id, 
                            bitis_noktasi=target, 
                            reverse=True, 
                            renk='k', 
                            ciz=True
                        )
                        
                        if add_vec(res):
                            out_engeller.append({
                                'birim_vektor': res.get('birim_vektor_3d', (0,0,0)),
                                'mesafe': sensor_mesafesi, # Doğrudan engel_bul'dan gelen ham mesafe
                                'radius': e.get('radius', 0.0)
                            })

            # --- DİĞER ROV'LAR (Collision Avoidance) ---
            if rov:
                # rov_vektor fonksiyonu menzil içindeki ROV'ları ve gerçek mesafelerini bulur
                for r in self.rov_vektor(rov_id=rov_id, menzil=GATLimitleri.CARPISMA):
                    target = r.get('koordinat')
                    
                    if target:
                        # KRİTİK DÜZELTME: Mesafeyi vektor() çıktısından değil, 
                        # rov_vektor'ün halihazırda hesapladığı 'mesafe' anahtarından alıyoruz.
                        gercek_mesafe = float(r.get('mesafe', 0.0))
                        
                        # vektor() metodunu sadece yön (itme vektörü) ve Minimap çizimi ('t' turuncu) için kullanıyoruz.
                        res = self.vektor(
                            rov_id_ilk=rov_id, 
                            bitis_noktasi=target, 
                            reverse=True, # Diğer ROV'dan kaçınmak için itme
                            renk='t', 
                            ciz=True
                        )
                        
                        if add_vec(res):
                            out_rovs.append({
                                'birim_vektor': res.get('birim_vektor_3d', (0,0,0)),
                                'mesafe': gercek_mesafe  # En doğru fiziksel mesafe
                            })

            # Bileşke Vektör
            import math
            mag = math.sqrt(sum(v**2 for v in toplam_vec))
            birim = (toplam_vec[0]/mag, toplam_vec[1]/mag, toplam_vec[2]/mag) if mag > 1e-9 else (0,0,0)

            return {
                'birim_vektor': birim,
                'mesafe': float(mag),
                'hedef': out_hedef,
                'engeller': out_engeller,
                'rovs': out_rovs
            }

    def apf_temizle(self, rov_id=None) -> None:
        """
        APF vektörlerini temizler. rov_id verilirse sadece o ROV'a ait vektörleri siler;
        boş bırakılırsa hepsini temizler.
        """
        if rov_id is None:
            self._apf_vektor_list = []
        else:
            rid = int(rov_id)
            self._apf_vektor_list = [i for i in self._apf_vektor_list if i.get('rov_id') != rid]
        # Minimap entity'lerini anında temizle (beklemeden görsel güncelleme)
        ortam = getattr(self.filo, 'ortam_ref', None)
        if ortam and hasattr(ortam, 'minimap') and ortam.minimap is not None:
            try:
                ortam.minimap._apf_vektorlari_temizle()
            except Exception:
                pass

    def hedef_vektor(self, rov_id: int):
            """
            ROV'un hedefine olan 3B vektör bilgisini döndürür (Çizim yapmaz).
            """
            hedef_koord = self.filo.hedef(rov_id=rov_id) # Sim Format: (x, y, z)
            if hedef_koord is None:
                return None

            # Yeni 3D 'vektor' metodunu kullanarak bilgileri al
            # 'ciz=False' ile sadece hesaplama yapar, 'reverse=False' ile hedefe çeker.
            return self.vektor(
                rov_id_ilk=rov_id, 
                bitis_noktasi=hedef_koord, 
                renk='y', 
                ciz=False
            )

    def rov_vektor(self, rov_id: int, menzil: float = None):
        """
        ROV'un diğer ROV'lara olan 3B kaçınma vektörlerini döndürür (Çizim yapmaz).
        """
        if menzil is None:
            menzil = GATLimitleri.CARPISMA

        ortam = getattr(self.filo, 'ortam_ref', None)
        if not ortam or not hasattr(ortam, 'rovs'):
            return []

        # Kendi 3D pozisyonumuzu al (Simülasyon formatı)
        pos_self = self._vektor_poz_al_3d(rov_id, ortam)
        if not pos_self: return []

        result = []
        for r_ent in ortam.rovs:
            if r_ent is None or getattr(r_ent, 'id', None) == rov_id:
                continue

            # Diğer ROV'un 3D pozisyonu (Simülasyon formatı: X, Y_ileri, Z_derinlik)
            # Ursina (x, y, z) -> Sim (x, z, -y)
            pos_other = (r_ent.x, r_ent.z, -r_ent.y)
            
            # 3D Öklid Mesafesi
            dist = math.sqrt(
                (pos_other[0]-pos_self[0])**2 + 
                (pos_other[1]-pos_self[1])**2 + 
                (pos_other[2]-pos_self[2])**2
            )

            if dist <= menzil:
                # 'reverse=True' ile itme (repulsion) vektörü oluşturulur.
                vb = self.vektor(
                    baslangic_noktasi=pos_self, 
                    bitis_noktasi=pos_other, 
                    reverse=True, 
                    ciz=False
                )
                if vb:
                    result.append({
                        'rov_id': int(r_ent.id),
                        'koordinat': pos_other,
                        'vektor_bilgi': vb,
                        'mesafe': dist
                    })
        return result



    def get_apf_vektor_verts_list(self, minimap):
        """
        APF vektör listesi için minimap köşe ve renk listesi döner.
        apf() çağrıldıktan sonra minimap birden fazla vektörü (engel kırmızı, rov turuncu, hedef yeşil) çizebilir.

        Returns:
            list: [(verts, renk_kodu), ...] veya [] (apf hiç çağrılmadıysa veya liste boşsa).
        """
        apf_list = getattr(self, '_apf_vektor_list', None)
        if not apf_list:
            return []
        z_line = -0.37
        havuz_genisligi = getattr(minimap, 'havuz_genisligi', 200.0)
        sonuc = []
        for item in apf_list:
            p1_xyz = item.get('baslangic')
            p2_xyz = item.get('bitis')
            renk = item.get('renk', 'm')
            uzunluk_metre = item.get('uzunluk', 20.0)
            reverse = item.get('reverse', False)
            if p1_xyz is None or p2_xyz is None or len(p1_xyz) < 2 or len(p2_xyz) < 2:
                continue
            p1 = minimap.dunya_to_harita(float(p1_xyz[0]), float(p1_xyz[1]))
            p2 = minimap.dunya_to_harita(float(p2_xyz[0]), float(p2_xyz[1]))
            verts = self._vektor_verts_birim(p1, p2, z_line, havuz_genisligi, uzunluk_metre=uzunluk_metre, reverse=reverse)
            sonuc.append((verts, renk))  # renk: 'k' kırmızı, 'y' yeşil, 't' turuncu (minimap eşler)
        return sonuc

    def yakinlastir(self, rov_id1: int, rov_id2: int, mesafe: float) -> bool:
        """
        rov_id1'i rov_id2'ye yatay düzlemde (X,Z) mesafe kadar yaklaştırır.
        Sadece rov_id1 hareket eder; hedef konum filo.git() ile atanır (ROV o noktaya gider).
        Ursina: (x, z) yatay; Sim: (x: Sağ-Sol, y: İleri-Geri, z: Derinlik) => Sim(x, Ursina.z, Ursina.y).
        """
        if not getattr(self.filo, 'sistemler', None):
            return False
        n = len(self.filo.sistemler)
        if rov_id1 < 0 or rov_id1 >= n or rov_id2 < 0 or rov_id2 >= n or rov_id1 == rov_id2:
            return False
        rov1 = getattr(self.filo.sistemler[rov_id1], 'rov', None)
        rov2 = getattr(self.filo.sistemler[rov_id2], 'rov', None)
        if rov1 is None or rov2 is None:
            return False
        pos1 = getattr(rov1, 'world_position', rov1)
        pos2 = getattr(rov2, 'world_position', rov2)
        x1 = getattr(pos1, 'x', getattr(rov1, 'x', 0.0))
        y1 = getattr(pos1, 'y', getattr(rov1, 'y', 0.0))
        z1 = getattr(pos1, 'z', getattr(rov1, 'z', 0.0))
        x2 = getattr(pos2, 'x', getattr(rov2, 'x', 0.0))
        z2 = getattr(pos2, 'z', getattr(rov2, 'z', 0.0))
        dx = x2 - x1
        dz = z2 - z1
        d = math.sqrt(dx * dx + dz * dz)
        if d < 1e-9:
            return True
        # rov_id1, rov_id2'ye mesafe kadar yaklaşacak (sadece rov1 hareket eder)
        move = min(mesafe, d)
        ux = dx / d
        uz = dz / d
        new_x1 = x1 + ux * move
        new_z1 = z1 + uz * move
        # Sim format: x = Sağ-Sol (Ursina x), y = İleri-Geri (Ursina z), z = Derinlik (Ursina y)
        sim_x = new_x1
        sim_y = new_z1
        sim_z = y1
        try:
            self.filo.git(rov_id1, sim_x, sim_y, sim_z)
        except Exception:
            return False
        return True

    # Vektör renk kodu -> minimap'te kullanılır: k=kırmızı, y=yeşil, m=mavi, s=sarı, t=turuncu
    VEKTOR_RENK_KODLARI = ('k', 'y', 'm', 's', 't')

    def _vektor_arg_norm(self, arg):
        """Tek argümanı normalize eder: ROV ID (int) veya nokta (x, z) tuple. Nokta = len>=2 sequence."""
        if arg is None:
            return None
        try:
            if hasattr(arg, '__len__') and len(arg) >= 2:
                return (float(arg[0]), float(arg[1]))
        except (TypeError, ValueError, IndexError):
            pass
        return int(arg)

    def vektor(self, ilk=None, ikinci=None,
               rov_id_ilk=None, rov_id_ikinci=None,
               baslangic_noktasi=None, bitis_noktasi=None, vektor=None,
               renk='m', uzunluk=20, reverse=False, debug=False, ciz=True):
        """
        Minimap üzerinde vektör çizer. Keyword argümanlarla net kullanım.

        Modlar:
        1. İki nokta: rov_id_ilk + rov_id_ikinci | baslangic_noktasi + bitis_noktasi | karışık
        2. Nokta + vektör: baslangic_noktasi/rov_id_ilk + vektor=(vx,vz) → başlangıçtan o yönde uzunluk kadar

        Args:
            rov_id_ilk (int): Başlangıç ROV ID.
            rov_id_ikinci (int): Bitiş ROV ID.
            baslangic_noktasi ((x,z)): Başlangıç noktası (metre, dünya koordinatı).
            bitis_noktasi ((x,z)): Bitiş noktası (metre).
            vektor ((vx,vz)): Yön vektörü (birim olması gerekmez). baslangic_noktasi veya rov_id_ilk ile kullan.
            ilk, ikinci: Eski pozisyonel API (geriye uyumluluk).
            renk: 'k','y','m','s','t'. uzunluk: ok uzunluğu (m). reverse, debug, ciz: bool.

        Returns:
            dict | None: baslangic, bitis, birim_vektor, uzunluk veya None.

        Örnekler:
            debug.vektor(rov_id_ilk=2, rov_id_ikinci=5)
            debug.vektor(baslangic_noktasi=(-174, 115), vektor=(0.83, -0.55), uzunluk=20)
            debug.vektor(rov_id_ilk=5, vektor=(0.76, -0.65), uzunluk=30)
            debug.vektor(0, 1)  # Eski API
        """
        if renk in self.VEKTOR_RENK_KODLARI:
            self._vektor_renk = renk
        self._vektor_uzunluk_metre = max(0.0, float(uzunluk))
        self._vektor_reverse = bool(reverse)

        # Geriye uyumluluk: positional ilk, ikinci
        if ilk is not None or ikinci is not None:
            if rov_id_ilk is None and baslangic_noktasi is None:
                if isinstance(ilk, int):
                    rov_id_ilk = ilk
                elif hasattr(ilk, '__len__') and len(ilk) >= 2:
                    try:
                        baslangic_noktasi = (float(ilk[0]), float(ilk[1]))
                    except (TypeError, ValueError, IndexError):
                        pass
            if rov_id_ikinci is None and bitis_noktasi is None and vektor is None and ikinci is not None:
                if isinstance(ikinci, int):
                    rov_id_ikinci = ikinci
                elif hasattr(ikinci, '__len__') and len(ikinci) >= 2:
                    try:
                        v0, v1 = float(ikinci[0]), float(ikinci[1])
                        mag = math.sqrt(v0 * v0 + v1 * v1)
                        # Birim vektör: uzunluk ~1 ve bileşenler [-1.2, 1.2]
                        if mag >= 1e-6 and 0.7 <= mag <= 1.3 and abs(v0) <= 1.2 and abs(v1) <= 1.2:
                            vektor = (v0, v1)
                        else:
                            bitis_noktasi = (v0, v1)
                    except (TypeError, ValueError, IndexError):
                        pass

        ortam = getattr(self.filo, 'ortam_ref', None)
        havuz_genisligi = getattr(ortam, 'havuz_genisligi', 200.0) if ortam else 200.0
        h = max(float(havuz_genisligi), 1.0)
        kosegen = 2.0 * h * math.sqrt(2.0)

        # Başlangıç pozisyonu
        pos1 = None
        if baslangic_noktasi is not None:
            try:
                pos1 = (float(baslangic_noktasi[0]), float(baslangic_noktasi[1]))
            except (TypeError, IndexError, ValueError):
                pass
        if pos1 is None and rov_id_ilk is not None:
            pos1 = self._vektor_poz_al(int(rov_id_ilk), ortam)

        # vektor modu: başlangıç + yön vektörü
        if vektor is not None:
            if pos1 is None:
                if debug:
                    self._apf_vektor_list = []
                return None
            try:
                vx, vz = float(vektor[0]), float(vektor[1])
            except (TypeError, IndexError, ValueError):
                return None
            v_mag = math.sqrt(vx * vx + vz * vz)
            if v_mag < 1e-9:
                ret = {
                    'baslangic': (float(pos1[0]) / h, float(pos1[1]) / h),
                    'bitis': (float(pos1[0]) / h, float(pos1[1]) / h),
                    'birim_vektor': (0.0, 0.0),
                    'uzunluk': 0.0,
                }
                pos2 = pos1
            else:
                ux, uz = vx / v_mag, vz / v_mag
                if self._vektor_reverse:
                    ux, uz = -ux, -uz
                pos2 = (pos1[0] + ux * self._vektor_uzunluk_metre, pos1[1] + uz * self._vektor_uzunluk_metre)
                ret = {
                    'baslangic': (float(pos1[0]) / h, float(pos1[1]) / h),
                    'bitis': (float(pos2[0]) / h, float(pos2[1]) / h),
                    'birim_vektor': (ux, uz),
                    'uzunluk': self._vektor_uzunluk_metre / kosegen,
                }
            if ciz:
                if debug:
                    self._apf_vektor_list = []
                item = {
                    'baslangic': (float(pos1[0]), float(pos1[1])),
                    'bitis': (float(pos2[0]), float(pos2[1])),
                    'renk': self._vektor_renk,
                    'uzunluk': self._vektor_uzunluk_metre,
                    'reverse': self._vektor_reverse,
                }
                if rov_id_ilk is not None:
                    item['rov_id'] = int(rov_id_ilk)
                self._apf_vektor_list.append(item)
            return ret

        # İki nokta modu
        if pos1 is None:
            self._vektor_baslangic = None
            self._vektor_bitis = None
            if debug:
                self._apf_vektor_list = []
            return None

        pos2 = None
        if bitis_noktasi is not None:
            try:
                pos2 = (float(bitis_noktasi[0]), float(bitis_noktasi[1]))
            except (TypeError, IndexError, ValueError):
                pass
        if pos2 is None and rov_id_ikinci is not None:
            pos2 = self._vektor_poz_al(int(rov_id_ikinci), ortam)

        if pos2 is None:
            if debug:
                self._apf_vektor_list = []
            return None

        dx = pos2[0] - pos1[0]
        dz = pos2[1] - pos1[1]
        d = math.sqrt(dx * dx + dz * dz)
        if d < 1e-9:
            ret = {
                'baslangic': (float(pos1[0]) / h, float(pos1[1]) / h),
                'bitis': (float(pos2[0]) / h, float(pos2[1]) / h),
                'birim_vektor': (0.0, 0.0),
                'uzunluk': 0.0,
                'uzaklik_metre': 0.0,
            }
        else:
            ux, uz = dx / d, dz / d
            if self._vektor_reverse:
                ux, uz = -ux, -uz
            ret = {
                'baslangic': (float(pos1[0]) / h, float(pos1[1]) / h),
                'bitis': (float(pos2[0]) / h, float(pos2[1]) / h),
                'birim_vektor': (ux, uz),
                'uzunluk': d / kosegen,
                'uzaklik_metre': float(d),
            }

        if ciz:
            if debug:
                self._apf_vektor_list = []
            item = {
                'baslangic': (float(pos1[0]), float(pos1[1])),
                'bitis': (float(pos2[0]), float(pos2[1])),
                'renk': self._vektor_renk,
                'uzunluk': self._vektor_uzunluk_metre,
                'reverse': self._vektor_reverse,
            }
            if rov_id_ilk is not None:
                item['rov_id'] = int(rov_id_ilk)
            self._apf_vektor_list.append(item)
        return ret

    def get_vektor_renk(self):
        """Minimap vektör çizgisi için renk kodu döner: k, y, m, s, t (varsayılan m)."""
        return getattr(self, '_vektor_renk', 'm')

    def _vektor_verts_birim(self, p1, p2, z_line=-0.37, havuz_genisligi=200.0, uzunluk_metre=None, reverse=False):
        """
        Harita koordinatında p1 -> p2 yönünde (reverse=True ise ters yönde) sabit uzunlukta vektör köşe listesi döner.
        Vektör uzunluğu: merkez–kenar = havuz_genisligi m, harita 0.5 birim;
        birim_uzunluk = uzunluk_metre / (2*havuz_genisligi) harita birimi.
        Ok ucu: vektör uzunluğunun 1/6'ı, 145° ve 225°.
        """
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        d = math.sqrt(dx * dx + dy * dy)
        if d >= 1e-9:
            ux, uy = dx / d, dy / d
            if reverse:
                ux, uy = -ux, -uy
            h = max(float(havuz_genisligi), 1.0)
            if uzunluk_metre is None:
                uzunluk_metre = getattr(self, '_vektor_uzunluk_metre', 10.0)
            birim_uzunluk = float(uzunluk_metre) / (2.0 * h)  # harita birimi (metre ölçeğine göre)
            ex = p1.x + ux * birim_uzunluk
            ey = p1.y + uy * birim_uzunluk
            # Vektör açısı (radyan), ok ucu kanatları bu açıya göre 145° ve 225°
            vektor_aci = math.atan2(uy, ux)
            kanat_uzunluk = birim_uzunluk / 6.0
            aci_145 = vektor_aci + math.radians(145)
            aci_225 = vektor_aci + math.radians(225)
            w1x = ex + kanat_uzunluk * math.cos(aci_145)
            w1y = ey + kanat_uzunluk * math.sin(aci_145)
            w2x = ex + kanat_uzunluk * math.cos(aci_225)
            w2y = ey + kanat_uzunluk * math.sin(aci_225)
            # Ana çizgi + iki kanat (uc -> 145°, uc -> 225°)
            return [
                (p1.x, p1.y, z_line), (ex, ey, z_line),
                (ex, ey, z_line), (w1x, w1y, z_line),
                (ex, ey, z_line), (w2x, w2y, z_line),
            ]
        return [(p1.x, p1.y, z_line), (p1.x, p1.y, z_line)]

    def _vektor_poz_al(self, arg, ortam):
        """Tek vektör argümanını (ROV id veya (x,z) nokta) dünya (x, z) koordinatına çevirir. Bulunamazsa None."""
        if arg is None:
            return None
        try:
            if isinstance(arg, (tuple, list)) and len(arg) >= 2:
                return (float(arg[0]), float(arg[1]))
        except (TypeError, ValueError, IndexError):
            pass
        rid = int(arg)
        if ortam is None or not hasattr(ortam, 'rovs') or not ortam.rovs:
            return None
        for rov in ortam.rovs:
            if rov is None:
                continue
            if getattr(rov, 'id', None) == rid:
                return (rov.x, rov.z)
        return None

    def get_vektor_verts(self, minimap):
        """
        Minimap için vektör çizgi köşe listesini hesaplar (birim vektör, sabit uzunluk).
        Başlangıç/bitiş hibrit: ROV ID ise o ROV konumu, nokta ise verilen (x,z) kullanılır.

        Returns:
            list | None: [(x1,y1,z), (x2,y2,z)] veya None.
        """
        baslangic = getattr(self, '_vektor_baslangic', None)
        bitis = getattr(self, '_vektor_bitis', None)
        if baslangic is None or bitis is None:
            return None
        ortam = getattr(self.filo, 'ortam_ref', None)
        pos1 = self._vektor_poz_al(baslangic, ortam)
        pos2 = self._vektor_poz_al(bitis, ortam)
        if pos1 is None or pos2 is None:
            return None
        z_line = -0.37
        havuz_genisligi = getattr(minimap, 'havuz_genisligi', 200.0)
        uzunluk_metre = getattr(self, '_vektor_uzunluk_metre', 10.0)
        reverse = getattr(self, '_vektor_reverse', False)
        p1 = minimap.dunya_to_harita(pos1[0], pos1[1])
        p2 = minimap.dunya_to_harita(pos2[0], pos2[1])
        return self._vektor_verts_birim(p1, p2, z_line, havuz_genisligi, uzunluk_metre=uzunluk_metre, reverse=reverse)

    def _yatay_mesafe(self, p1, p2):
        """İki nokta (x, z) arasındaki yatay mesafeyi metre cinsinden döner."""
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    def _apf_rep_factor_hesapla(self, mesafe: float, r_etki: float, d_min: float = 0.5):
        """
        APF için repulsive factor hesaplar (mesafe ile ters orantılı).
        
        Args:
            mesafe: ROV ile obje arasındaki mesafe (metre)
            r_etki: Etki menzili (metre)
            d_min: Minimum güvenli mesafe (metre, varsayılan 0.5)
        
        Returns:
            float: Repulsive factor (mesafe azaldıkça artar)
        """
        d_safe = max(mesafe, d_min)
        return (1.0 / d_safe - 1.0 / r_etki)

    def _apf_kuvvet_hesapla(self, mesafe: float, birim_vektor: tuple, r_etki: float, 
                           k_itme: float, k_teget: float, r_yakin_artis: float, 
                           yakin_carpan: float, teget_carpan: float = 1.0):
        """
        APF için itme ve teğet kuvvetlerini hesaplar.
        Engel vektörü karesel olarak artar (mesafe azaldıkça).
        
        Args:
            mesafe: ROV ile obje arasındaki mesafe (metre)
            birim_vektor: (ux, uz) birim vektör (ROV'dan objeye doğru)
            r_etki: Etki menzili (metre)
            k_itme: İtme katsayısı
            k_teget: Teğet kuvvet katsayısı
            r_yakin_artis: Yakın artış mesafesi (metre)
            yakin_carpan: Yakın mesafede kuvvet çarpanı
            teget_carpan: Teğet kuvvet çarpanı (varsayılan 1.0, ROV için 1.5)
        
        Returns:
            tuple: (f_rep_ux, f_rep_uz, f_tan_ux, f_tan_uz) kuvvet bileşenleri
        """
        # Menzil dışındaki objeler sıfır kuvvet uygular
        if mesafe >= r_etki:
            return (0.0, 0.0, 0.0, 0.0)
        
        ux, uz = birim_vektor[:2]
        rep_factor = self._apf_rep_factor_hesapla(mesafe, r_etki)
        
        # İtme kuvveti (karesel - mesafe azaldıkça karesel artar)
        # birim_vektor ROV->Obstacle yönünde; itme kuvveti obje→ROV yönünde olmalı (repulsive)
        f_rep = k_itme * rep_factor * rep_factor
        if mesafe < r_yakin_artis:
            # Yakın mesafede ekstra karesel artış
            f_rep *= yakin_carpan * (r_yakin_artis / max(mesafe, 0.1)) ** 2
        
        # İtme vektörü: objeden uzağa (-ux, -uz) = itici yön
        f_rep_ux = -ux * f_rep
        f_rep_uz = -uz * f_rep
        
        # Teğet kuvveti (vortex): itici yön (-ux, -uz) ile 90° dik
        # Perpendicular: (-uz, ux) veya (uz, -ux) — engel etrafında dolaşmak için
        f_tan_ux, f_tan_uz = 0.0, 0.0
        if mesafe < (r_etki / 2.0):
            f_tan = (k_teget * teget_carpan) * (rep_factor * rep_factor)
            if mesafe < r_yakin_artis:
                # Yakın mesafede ekstra karesel artış
                f_tan *= yakin_carpan * (r_yakin_artis / max(mesafe, 0.1)) ** 2
            # Teğet: (-uz, ux) — itici yön (-ux,-uz)'ye dik
            f_tan_ux = -uz * f_tan
            f_tan_uz = ux * f_tan
        
        return (f_rep_ux, f_rep_uz, f_tan_ux, f_tan_uz)

    def vektor_normalize(self, ux=None, uz=None, uy=None, max_mag: float = 1.0, vektor=None):
        """
        Vektörü normalize eder ve maksimum büyüklüğe sınırlar. Hem 2D hem 3D vektörleri destekler.
        
        Args:
            ux: X bileşeni (2D veya 3D modu için)
            uz: Z bileşeni (2D modu için) veya Y bileşeni (3D modu için)
            uy: Y bileşeni (3D modu için, opsiyonel). None ise 2D modu kullanılır.
            max_mag: Maksimum büyüklük (varsayılan 1.0)
            vektor: Alternatif kullanım - vektör tuple/list olarak verilebilir.
                - 2D: (ux, uz) veya [ux, uz]
                - 3D: (ux, uy, uz) veya [ux, uy, uz]
                vektor verilirse ux, uz, uy parametreleri göz ardı edilir.
        
        Returns:
            tuple: 
                - 2D modu: (ux_norm, uz_norm, mag) normalize edilmiş vektör ve büyüklüğü
                - 3D modu: (ux_norm, uy_norm, uz_norm, mag) normalize edilmiş vektör ve büyüklüğü
        
        Örnekler:
            # 2D vektör
            ux, uz, mag = self.vektor_normalize(3.0, 4.0)  # (0.6, 0.8, 1.0)
            ux, uz, mag = self.vektor_normalize(vektor=(3.0, 4.0))  # Tuple ile
            
            # 3D vektör
            ux, uy, uz, mag = self.vektor_normalize(2.0, 3.0, 4.0)  # 3D
            ux, uy, uz, mag = self.vektor_normalize(vektor=(2.0, 3.0, 4.0))  # Tuple ile
        """
        # Vektör tuple/list olarak verilmişse kullan
        if vektor is not None:
            try:
                if isinstance(vektor, (tuple, list)):
                    if len(vektor) >= 3:
                        # 3D vektör
                        ux, uy, uz = float(vektor[0]), float(vektor[1]), float(vektor[2])
                    elif len(vektor) >= 2:
                        # 2D vektör
                        ux, uz = float(vektor[0]), float(vektor[1])
                        uy = None
                    else:
                        return None
            except (TypeError, ValueError, IndexError):
                return None
        
        # Parametre kontrolü
        if ux is None or uz is None:
            return None
        
        # 3D modu: uy verilmişse
        if uy is not None:
            mag = math.sqrt(ux * ux + uy * uy + uz * uz)
            if mag < 1e-9:
                return (0.0, 0.0, 0.0, 0.0)
            if mag > max_mag:
                scale = max_mag / mag
                ux_norm = ux * scale
                uy_norm = uy * scale
                uz_norm = uz * scale
                mag = max_mag
            else:
                ux_norm = ux
                uy_norm = uy
                uz_norm = uz
            return (ux_norm, uy_norm, uz_norm, mag)
        
        # 2D modu: uy verilmemişse
        mag = math.sqrt(ux * ux + uz * uz)
        if mag < 1e-9:
            return (0.0, 0.0, 0.0)
        if mag > max_mag:
            scale = max_mag / mag
            ux_norm = ux * scale
            uz_norm = uz * scale
            mag = max_mag
        else:
            ux_norm = ux
            uz_norm = uz
        return (ux_norm, uz_norm, mag)

    def _apf_motor_guc_hesapla(self, min_dist_to_obj: float, r_guvenlik: float, 
                               base_speed: float = 0.15, total_mag: float = 1.0):
        """
        APF için dinamik motor gücü hesaplar (mesafeye göre).
        
        Args:
            min_dist_to_obj: En yakın objeye mesafe (metre)
            r_guvenlik: Güvenlik mesafesi (metre)
            base_speed: Temel hız (varsayılan 0.15)
            total_mag: Toplam vektör büyüklüğü (varsayılan 1.0)
        
        Returns:
            float: Motor gücü [0.0, 1.0] aralığında
        """
        current_power = base_speed
        if min_dist_to_obj < r_guvenlik:
            current_power = 0.35
        elif min_dist_to_obj < (r_guvenlik * 2):
            current_power = 0.25
        
        final_power = current_power * total_mag
        return max(0.0, min(final_power, 1.0))

    def _apf_hedef_gain_hesapla(self, min_dist_to_obj: float, r_yakin_hedef: float = 10.0, 
                                r_etki: float = 20.0, k_hedef: float = 1.0):
        """
        APF için hedef kazancını hesaplar (engel yaklaştıkça azalır).
        10 metreden yakın olduğunda neredeyse 0 olur.
        
        Args:
            min_dist_to_obj: En yakın objeye mesafe (metre)
            r_yakin_hedef: Yakın hedef mesafesi (metre, varsayılan 10.0) - bu mesafeden yakın olduğunda hedef neredeyse 0
            r_etki: Etki menzili (metre, varsayılan 10.0)
            k_hedef: Hedef katsayısı (varsayılan 1.0)
        
        Returns:
            float: Hedef kazancı (0.0 - k_hedef aralığında)
        """
        # 10 metreden yakın olduğunda neredeyse 0
        if min_dist_to_obj < r_yakin_hedef:
            # Karesel azalma: (mesafe / r_yakin_hedef)^2 ile çarp
            ratio = min_dist_to_obj / r_yakin_hedef
            return k_hedef * (ratio ** 2) * 0.01  # Neredeyse 0 (maksimum %1)
        
        # 10 metreden uzakta normal karesel azalma
        ratio_sq = (min_dist_to_obj / r_etki) ** 2
        return min(k_hedef, k_hedef * ratio_sq)

    def _koordinat_cikar(self, obj, x_attr='x', y_attr='y', z_attr='z'):
        """
        Bir objeden koordinatları çıkarır (getattr ile). 
        Obje Vec3, tuple, list veya attribute'lara sahip bir obje olabilir.
        
        Args:
            obj: Koordinat içeren obje (Vec3, tuple, list veya attribute'lara sahip obje)
            x_attr: X koordinatı için attribute adı (varsayılan 'x')
            y_attr: Y koordinatı için attribute adı (varsayılan 'y')
            z_attr: Z koordinatı için attribute adı (varsayılan 'z')
        
        Returns:
            tuple: (x, y, z) veya (x, z) eğer y bulunamazsa. Bulunamazsa (None, None, None).
        """
        if obj is None:
            return (None, None, None)
        
        # Tuple veya list ise
        if isinstance(obj, (tuple, list)):
            if len(obj) >= 3:
                return (float(obj[0]), float(obj[1]), float(obj[2]))
            elif len(obj) >= 2:
                return (float(obj[0]), None, float(obj[1]))
            return (None, None, None)
        
        # Vec3 veya attribute'lara sahip obje ise
        try:
            x = getattr(obj, x_attr, None)
            y = getattr(obj, y_attr, None)
            z = getattr(obj, z_attr, None)
            
            # Eğer x veya z None ise, tuple/list olarak dene
            if x is None or z is None:
                if isinstance(obj, (tuple, list)) and len(obj) >= 2:
                    x = obj[0] if x is None else x
                    z = obj[1] if z is None else z
            
            if x is not None and z is not None:
                return (float(x), float(y) if y is not None else None, float(z))
        except (TypeError, ValueError, AttributeError):
            pass
        
        return (None, None, None)

    def _rov_pozisyon_ursina(self, rov_id=None, rov_entity=None, ortam=None):
        """
        ROV pozisyonunu Ursina koordinatlarında (x, z) döner.
        Mevcut _vektor_poz_al() fonksiyonunu kullanır veya rov_entity'den direkt alır.
        
        Args:
            rov_id: ROV ID (int) - ortam'dan ROV'u bulmak için
            rov_entity: ROV entity objesi - direkt pozisyon almak için
            ortam: Ortam referansı - ROV'u bulmak için
        
        Returns:
            tuple: (x, z) Ursina koordinatları veya None
        """
        # Eğer rov_entity verilmişse direkt kullan
        if rov_entity is not None:
            x = getattr(rov_entity, 'x', None)
            z = getattr(rov_entity, 'z', None)
            if x is not None and z is not None:
                return (float(x), float(z))
        
        # Eğer rov_id verilmişse _vektor_poz_al() kullan
        if rov_id is not None and ortam is not None:
            return self._vektor_poz_al(rov_id, ortam)
        
        return None

    def _koordinator_al(self):
        """Koordinator'u lazy import ile alır (circular import önleme)."""
        if self._koordinator is None:
            from FiratROVNet.gnc import Koordinator
            self._koordinator = Koordinator
        return self._koordinator

    def apf(self, rov_id: int, hedef: bool = True, engel: bool = False, rov: bool = False):
        """
        APF: ROV'un mevcut konumundan hedefine/engellere/diğer ROV'lara yönelik vektörleri döndürür.
        Hangi bileşenlerin hesaplanacağı hedef/engel/rov parametreleriyle seçilir.

        Args:
            rov_id: ROV ID
            hedef: True ise hedef vektörü hesaplanır (varsayılan True)
            engel: True ise engellere yönelik vektörler hesaplanır
            rov: True ise diğer ROV'lara yönelik vektörler hesaplanır

        Returns:
            dict: {
                'birim_vektor': (ux, uz),
                'mesafe': mag,
                'hedef': {...} veya None,
                'engeller': [...],
                'rovs': [...]
            }
        """
        # _apf_vektor_list temizlenmez; vektörler birikir. Temizlemek için apf_temizle() çağrılır.

        birim_vektor = (0.0, 0.0)
        mag = 0.0
        out_hedef = None

        if hedef:
            hedef_koord = self.filo.hedef(rov_id=rov_id)
            if hedef_koord is not None:
                Koordinator = self._koordinator_al()
                h_ursina = Koordinator.sim_to_ursina(
                    float(hedef_koord[0]), float(hedef_koord[1]),
                    float(hedef_koord[2]) if len(hedef_koord) > 2 else 0.0
                )
                bitis_2d = (h_ursina[0], h_ursina[2])
                vec = self.vektor(rov_id_ilk=rov_id, bitis_noktasi=bitis_2d, renk='y', reverse=False, ciz=True)
                if vec is not None:
                    birim_vektor = vec['birim_vektor']
                    mag = vec.get('uzaklik_metre', vec.get('uzunluk', 0.0))
                    out_hedef = {'birim_vektor': birim_vektor, 'mesafe': float(mag)}
                else:
                    out_hedef = {'birim_vektor': (0.0, 0.0), 'mesafe': 0.0}
            else:
                out_hedef = {'birim_vektor': (0.0, 0.0), 'mesafe': 0.0}

        out_engeller = []
        if engel:
            engel_listesi = self.engel_vektor(rov_id=rov_id, menzil=GATLimitleri.ENGEL)
            for engel in engel_listesi:
                koord = engel.get('koordinat')
                vb = engel.get('vektor_bilgi')
                if not koord or len(koord) < 2 or not vb:
                    continue
                try:
                    ex = float(koord[0])
                    ez = float(koord[1])
                except (TypeError, ValueError):
                    continue
                self.vektor(
                    rov_id_ilk=rov_id,
                    bitis_noktasi=(ex, ez),
                    reverse=True,
                    renk='k',
                    ciz=True
                )
                out_engeller.append({
                    'birim_vektor': vb.get('birim_vektor'),
                    'mesafe': float(vb.get('uzaklik_metre', engel.get('mesafe', 0.0))),
                    'radius': float(engel.get('radius', 0.0)),
                })

        out_rovs = []
        if rov:
            rov_listesi = self.rov_vektor(rov_id=rov_id, menzil=GATLimitleri.CARPISMA)
            for diger in rov_listesi:
                koord = diger.get('koordinat')
                vb = diger.get('vektor_bilgi')
                if not koord or len(koord) < 2 or not vb:
                    continue
                try:
                    rx = float(koord[0])
                    rz = float(koord[1])
                except (TypeError, ValueError):
                    continue
                self.vektor(
                    rov_id_ilk=rov_id,
                    bitis_noktasi=(rx, rz),
                    reverse=True,
                    renk='t',
                    ciz=True
                )
                out_rovs.append({
                    'birim_vektor': vb.get('birim_vektor'),
                    'mesafe': float(vb.get('uzaklik_metre', diger.get('mesafe', 0.0)))
                })

        out = {
            'birim_vektor': birim_vektor,
            'mesafe': float(mag),
        }

        if hedef:
            out['hedef'] = out_hedef
        if engel:
            out['engeller'] = out_engeller
        if rov:
            out['rovs'] = out_rovs

        return out



    def apf_temizle(self, rov_id=None) -> None:
        """
        APF vektörlerini temizler. rov_id verilirse sadece o ROV'a ait vektörleri siler;
        boş bırakılırsa hepsini temizler.
        """
        if rov_id is None:
            self._apf_vektor_list = []
        else:
            rid = int(rov_id)
            self._apf_vektor_list = [i for i in self._apf_vektor_list if i.get('rov_id') != rid]
        # Minimap entity'lerini anında temizle (beklemeden görsel güncelleme)
        ortam = getattr(self.filo, 'ortam_ref', None)
        if ortam and hasattr(ortam, 'minimap') and ortam.minimap is not None:
            try:
                ortam.minimap._apf_vektorlari_temizle()
            except Exception:
                pass

    def hedef_vektor(self, rov_id: int, menzil: float = None):
        """
        ROV'un hedefine olan vektör bilgisini döndürür (çizim yapılmaz).
        
        Args:
            rov_id (int): ROV ID (0, 1, 2, ...)
            menzil (float): Yatay düzlemde (X,Z) menzil (metre). Varsayılan GATLimitleri.ENGEL.
        
        Returns:
            dict | None: Vektör bilgisi (baslangic, bitis, birim_vektor, uzunluk) veya None.
                Hedef yoksa veya ROV konumu bulunamazsa None döner.
        """
        if menzil is None:
            menzil = GATLimitleri.ENGEL
        ortam = getattr(self.filo, 'ortam_ref', None)
        pos_rov = self._rov_pozisyon_ursina(rov_id=rov_id, ortam=ortam)
        if pos_rov is None:
            return None
        
        sistemler = getattr(self.filo, 'sistemler', None)
        if sistemler and 0 <= rov_id < len(sistemler):
            hedef_obj = getattr(sistemler[rov_id], 'hedef', None)
            if hedef_obj is not None:
                hx_sim, hy_sim, hz_sim = self._koordinat_cikar(hedef_obj, x_attr='x', y_attr='y', z_attr='z')
                if hx_sim is not None and hy_sim is not None:
                    Koordinator = self._koordinator_al()
                    hedef_ursina = Koordinator.sim_to_ursina(hx_sim, hy_sim, hz_sim if hz_sim is not None else 0.0)
                    hedef_pt = (hedef_ursina[0], hedef_ursina[2])
                    
                    # Vektör bilgisini al (çizim yapılmaz)
                    vb = self.vektor(pos_rov, hedef_pt, renk='y', uzunluk=20.0, reverse=False, debug=False, ciz=False)
                    
                    return vb
        return None

    def rov_vektor(self, rov_id: int, menzil: float = None):
        """
        ROV'un diğer ROV'lara olan vektör bilgilerini liste olarak döndürür (çizim yapılmaz).
        
        Args:
            rov_id (int): ROV ID (0, 1, 2, ...)
            menzil (float): Yatay düzlemde (X,Z) menzil (metre). Varsayılan GATLimitleri.CARPISMA.
        
        Returns:
            list: [{'rov_id': int, 'koordinat': (x, z), 'vektor_bilgi': {...}}, ...]
                Her diğer ROV için vektör bilgisi içeren dict listesi.
                ROV konumu bulunamazsa veya menzil içinde ROV yoksa boş liste döner.
        """
        if menzil is None:
            menzil = GATLimitleri.CARPISMA
        ortam = getattr(self.filo, 'ortam_ref', None)
        pos_rov = self._rov_pozisyon_ursina(rov_id=rov_id, ortam=ortam)
        if pos_rov is None:
            return []
        
        result = []
        if ortam and getattr(ortam, 'rovs', None):
            for rov_entity in ortam.rovs:
                if rov_entity is None:
                    continue
                rid = getattr(rov_entity, 'id', None)
                if rid is None or int(rid) == int(rov_id):
                    continue
                rx, _, rz = self._koordinat_cikar(rov_entity, x_attr='x', z_attr='z')
                if rx is None or rz is None:
                    continue
                pt = (rx, rz)
                mesafe = self._yatay_mesafe(pos_rov, pt)
                if mesafe > menzil:
                    continue
                
                # Vektör bilgisini al (çizim yapılmaz)
                vb = self.vektor(pos_rov, pt, renk='t', uzunluk=20.0, reverse=True, debug=False, ciz=False)
                if vb is not None:
                    result.append({'rov_id': int(rid), 'koordinat': pt, 'vektor_bilgi': vb, 'mesafe': mesafe})
        return result

    def engel_vektor(self, rov_id: int, menzil: float = None):
        """
        ROV'un engellere olan vektör bilgilerini liste olarak döndürür (çizim yapılmaz).
        
        Args:
            rov_id (int): ROV ID (0, 1, 2, ...)
            menzil (float): Yatay düzlemde (X,Z) menzil (metre). Varsayılan GATLimitleri.ENGEL.
        
        Returns:
            list: [{'koordinat': (x, z), 'vektor_bilgi': {...}, 'mesafe': float, 'radius': float}, ...]
                Her engel için vektör bilgisi ve yarıçap (metre) içeren dict listesi.
                ROV konumu bulunamazsa veya menzil içinde engel yoksa boş liste döner.
        """
        if menzil is None:
            menzil = GATLimitleri.ENGEL
        ortam = getattr(self.filo, 'ortam_ref', None)
        pos_rov = self._rov_pozisyon_ursina(rov_id=rov_id, ortam=ortam)
        if pos_rov is None:
            return []
        
        result = []
        for s in self.engel_bul(rov_id, menzil=menzil):
            koord = s.get('koordinat')
            if koord is None:
                continue
            kx, _, kz = self._koordinat_cikar(koord, x_attr='x', z_attr='z')
            if kx is None or kz is None:
                continue
            pt = (kx, kz)
            mesafe = self._yatay_mesafe(pos_rov, pt)
            if mesafe > menzil:
                continue
            
            # Vektör bilgisini al (çizim yapılmaz)
            vb = self.vektor(pos_rov, pt, renk='k', uzunluk=20.0, reverse=True, debug=False, ciz=False)
            radius = s.get('radius', 0.0)
            if vb is not None:
                result.append({'koordinat': pt, 'vektor_bilgi': vb, 'mesafe': mesafe, 'radius': radius})
        return result


    def get_apf_vektor_verts_list(self, minimap):
        """
        APF vektör listesi için minimap köşe ve renk listesi döner.
        apf() çağrıldıktan sonra minimap birden fazla vektörü (engel kırmızı, rov turuncu, hedef yeşil) çizebilir.

        Returns:
            list: [(verts, renk_kodu), ...] veya [] (apf hiç çağrılmadıysa veya liste boşsa).
        """
        apf_list = getattr(self, '_apf_vektor_list', None)
        if not apf_list:
            return []
        z_line = -0.37
        havuz_genisligi = getattr(minimap, 'havuz_genisligi', 200.0)
        sonuc = []
        for item in apf_list:
            p1_xyz = item.get('baslangic')
            p2_xyz = item.get('bitis')
            renk = item.get('renk', 'm')
            uzunluk_metre = item.get('uzunluk', 20.0)
            reverse = item.get('reverse', False)
            if p1_xyz is None or p2_xyz is None or len(p1_xyz) < 2 or len(p2_xyz) < 2:
                continue
            p1 = minimap.dunya_to_harita(float(p1_xyz[0]), float(p1_xyz[1]))
            p2 = minimap.dunya_to_harita(float(p2_xyz[0]), float(p2_xyz[1]))
            verts = self._vektor_verts_birim(p1, p2, z_line, havuz_genisligi, uzunluk_metre=uzunluk_metre, reverse=reverse)
            sonuc.append((verts, renk))  # renk: 'k' kırmızı, 'y' yeşil, 't' turuncu (minimap eşler)
        return sonuc

    def get_100_samples(self, hull_output=None, sample_count=100):
        """
        yeni_hull çıktısındaki noktaları alır ve çevre uzunluğu üzerinden
        sabit sayıda (sample_count) örnek nokta döndürür.
        """
        if hull_output is None:
            hull_output = self.filo.yeni_hull(self.filo.ada_cevre())

        points = hull_output.get('points')
        if points is None or len(points) < 2:
            print("⚠️ [SAMPLED] Örnekleme için yetersiz nokta!")
            return None

        if not np.allclose(points[0], points[-1]):
            points = np.vstack([points, points[0]])

        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt((diffs**2).sum(axis=1))
        cumulative_dist = np.concatenate(([0], np.cumsum(segment_lengths)))
        total_perimeter = cumulative_dist[-1]

        if total_perimeter == 0:
            return np.tile(points[0], (sample_count, 1))

        target_dists = np.linspace(0, total_perimeter, sample_count, endpoint=False)
        new_x = np.interp(target_dists, cumulative_dist, points[:, 0])
        new_y = np.interp(target_dists, cumulative_dist, points[:, 1])
        sampled_points = np.column_stack((new_x, new_y))
        return sampled_points

    def uret_rl_egitim_verisi(self):
        """
        RL eğitimi için hızlı senaryo üretir ve sabit boyutlu verileri döner.
        Yeni senaryo sistemi: Ortam bir kez başlatılır, ROV/ada sayıları dinamik olarak ayarlanır.
        """
        from FiratROVNet import senaryo
        import random

        try:
            n_rov_secenekleri = [4, 6, 8]
            secilen_n = random.choice(n_rov_secenekleri)
            n_engels = random.randint(12, 22)
            n_adalar = random.randint(2, 5)  # 2-5 arasında random ada sayısı

            # Senaryo ortamı bir kez başlatılır, sonra sadece parametreler güncellenir
            senaryo.uret(n_rovs=secilen_n, n_engels=n_engels, n_adalar=n_adalar, havuz_genisligi=200, verbose=False)
            aktif_filo = senaryo.filo
            if not aktif_filo:
                return None

            # Lider ROV'u bul (role=1 olan)
            lider_id = None
            for i in range(secilen_n):
                if i < len(senaryo.filo.sistemler) and senaryo.filo.sistemler[i] is not None:
                    rol = senaryo.get(i, "rol")
                    if rol == 1:
                        lider_id = i
                        break
            
            if lider_id is None:
                lider_id = 0  # Fallback: ilk ROV lider olsun

            lider_gps = senaryo.get(lider_id, "gps") if lider_id < len(senaryo.filo.sistemler) else None
            lider_yaw = senaryo.get(lider_id, "yaw") if lider_id < len(senaryo.filo.sistemler) else None

            if lider_gps is None:
                lider_gps = np.array([400.0, 400.0, 400.0])
            if lider_yaw is None:
                lider_yaw = 0.0

            rov_filo_gps = []
            for i in range(8):
                if i < secilen_n:
                    # Senaryo'dan get kullan (sessiz mod - hata mesajları yok)
                    try:
                        # Senaryo instance'ından direkt get kullan (sessiz mod)
                        from FiratROVNet.senaryo import _get_instance
                        senaryo_instance = _get_instance()
                        if senaryo_instance and senaryo_instance.aktif and i < len(senaryo_instance.ortam.rovs):
                            pos = senaryo_instance.get(i, "gps")
                            rov_filo_gps.append(pos if pos is not None else [400.0, 400.0, 400.0])
                        else:
                            rov_filo_gps.append([400.0, 400.0, 400.0])
                    except Exception:
                        # Hata durumunda varsayılan değer
                        rov_filo_gps.append([400.0, 400.0, 400.0])
                else:
                    rov_filo_gps.append([400.0, 400.0, 400.0])
            rov_filo_gps = np.array(rov_filo_gps)

            hull_merkez = np.array([400.0, 400.0])
            hull_noktalar = np.full((100, 2), 400.0)

            ada_cevreleri = aktif_filo.ada_cevre(offset=15.0, sessiz=True)
            hull_dict = aktif_filo.yeni_hull(ada_cevreleri)
            if hull_dict and hull_dict.get('points') is not None:
                center = hull_dict.get('center')
                if center:
                    hull_merkez = np.array([center[0], center[1]])

                samples = self.get_100_samples(hull_dict, 100)
                if samples is not None:
                    hull_noktalar = samples

            # Headless/egitim modunda main thread olmadığından formasyon_sec None dönebilir.
            # Sessiz mod: Log mesajları ve görsel işlemler kapalı (RL eğitimi için)
            out = aktif_filo.helper._formasyon_sec_impl(margin=HareketAyarlari.FORMASYON_MESAFESI, is_3d=False, offset=HareketAyarlari.FORMASYON_OFFSET, sessiz=True)
            
            # Formasyon seçilemediyse None döndür (eğitim verisi oluşturulamadı)
            if out is None:
                # Sessiz modda olduğu için log mesajı yok
                return None
            
            # Not: senaryo.temizle() artık çağrılmıyor - ortam bir kez başlatılıp tekrar kullanılıyor
            # Eğer gerçekten temizlemek isterseniz: senaryo.temizle()

            return {
                "output": out,
                "n_rovs": secilen_n,
                "lider_pozisyon": lider_gps,
                "lider_yaw": lider_yaw,
                "rov_filo_gps": rov_filo_gps,
                "hull_merkez": hull_merkez,
                "hull_noktalar": hull_noktalar
            }
        except Exception as e:
            print(f"❌ [RL_DATA] Veri üretimi sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            # Hata durumunda temizleme yapılabilir (isteğe bağlı)
            # senaryo.temizle()
            return None

    def lider_sec_veri_uret(self, asil_hedef=None):
        """
        RL eğitimi için lider seçim verisi üretir.
        Matematiksel liderlik formülünü 'Label' olarak kullanır.
        Yeni senaryo sistemi: Ortam bir kez başlatılır, ROV/ada sayıları dinamik olarak ayarlanır.
        """
        from FiratROVNet import senaryo
        import random
        try:
            n_rov_list = [4, 6, 8]
            secilen_n = random.choice(n_rov_list)
            n_adalar = random.randint(2, 5)  # 2-5 arasında random ada sayısı
            # Senaryo ortamı bir kez başlatılır, sonra sadece parametreler güncellenir
            senaryo.uret(n_rovs=secilen_n, n_engels=random.randint(10, 20), n_adalar=n_adalar, havuz_genisligi=200, verbose=False)

            if not senaryo.filo:
                return None

            hedef = asil_hedef if asil_hedef else Vec3(random.randint(-100, 100), random.randint(-100, 100), 0)

            rov_data = []
            rov_list_for_calc = []
            for i in range(8):
                if i < secilen_n:
                    # Senaryo instance'ından direkt get kullan (sessiz mod)
                    try:
                        from FiratROVNet.senaryo import _get_instance
                        senaryo_instance = _get_instance()
                        if senaryo_instance and senaryo_instance.aktif and i < len(senaryo_instance.ortam.rovs):
                            bat = senaryo_instance.get(i, "batarya")
                            if bat is None:
                                bat = 1.0  # Varsayılan batarya
                            bat = bat * 100.0
                            gps = senaryo_instance.get(i, "gps")
                            if gps is None:
                                gps = np.array([400.0, 400.0, 400.0])
                            rov_data.append([bat, gps[0], gps[1], gps[2]])
                            rov_list_for_calc.append({'id': i, 'batarya': bat, 'konum': gps})
                        else:
                            rov_data.append([0.0, 400.0, 400.0, 400.0])
                    except Exception:
                        # Hata durumunda varsayılan değer
                        rov_data.append([0.0, 400.0, 400.0, 400.0])
                else:
                    rov_data.append([0.0, 400.0, 400.0, 400.0])

            from RL_PPO.lider_sec.lider_sec import LiderSecimModulu
            lider_modulu = LiderSecimModulu()
            dogru_lider_id, dogru_skor = lider_modulu.lideri_belirle_ve_yazdir(
                rov_list_for_calc, [hedef.x, hedef.y, hedef.z]
            )

            state = np.array(
                [hedef.x, hedef.y, hedef.z] + list(np.array(rov_data).flatten()),
                dtype=np.float32
            )

            # Not: senaryo.temizle() artık çağrılmıyor - ortam bir kez başlatılıp tekrar kullanılıyor
            # Eğer gerçekten temizlemek isterseniz: senaryo.temizle()
            
            return {
                "state": state,
                "target_id": dogru_lider_id,
                "target_skor": dogru_skor
            }
        except Exception as e:
            print(f"❌ Lider veri üretim hatası: {e}")
            import traceback
            traceback.print_exc()
            # Hata durumunda temizleme yapılabilir (isteğe bağlı)
            # senaryo.temizle()
            return None

    def gat_veri_uret(self):
        """
        GAT eğitimi için senaryo verisi üretir.
        Senaryo.py kullanarak 8, 6, 4 rastgele ROV ve 2-5 arası ada ile ortam oluşturur.
        
        Returns:
            dict: {
                'senaryo': Senaryo instance,
                'filo': Filo instance,
                'ortam': Ortam instance,
                'n_rovs': int,
                'n_adalar': int,
                'n_engels': int
            } veya None (hata durumunda)
        """
        from FiratROVNet import senaryo
        from FiratROVNet.senaryo import _get_instance
        import random
        
        try:
            # NOT: random.seed() çağrısını KALDIRDIK - Python'un varsayılan rastgele durumunu kullan
            # Bu, her çağrıda gerçekten farklı senaryolar üretilmesini sağlar
            
            # Senaryo tipi seçimi - farklı GAT kodları için çeşitli senaryolar
            senaryo_tipi = random.choice([
                'normal',       # Standart dağılım
                'yakin',        # ROV'lar yakın (çarpışma riski)
                'dagnik',       # ROV'lar dağınık (kopma riski)
                'tek_kume',     # Tek kümede toplanmış
                'iki_kume',     # İki ayrı kümede
            ])
            
            # Daha geniş parametre aralığı ile çeşitlilik sağla
            n_rov_secenekleri = [4,5,6,7, 8,9, 10,11,12]
            secilen_n = random.choice(n_rov_secenekleri)
            n_engels = random.randint(10, 20)  # Daha geniş aralık
            n_adalar = random.randint(2, 6)  # 2-6 arası (daha geniş)
            
            # Cache'i temizlemek için önce mevcut cache'i sıfırla
            senaryo_instance = _get_instance()
            if senaryo_instance:
                # Cache'i temizle - böylece her zaman yeni senaryo üretilir
                senaryo_instance._cache_n_rovs = None
                senaryo_instance._cache_n_engels = None
                senaryo_instance._cache_n_adalar = None
                senaryo_instance._cache_havuz_genisligi = None
            
            # Havuz genişliğini daha fazla değiştir (cache'i bypass etmek için)
            # Daha geniş aralık ile her epoch'ta kesinlikle farklı senaryo üretilir
            havuz_genisligi = 200 + random.uniform(-5, 5)  # 195-205 arası (daha geniş fark)
            
            # Senaryo ortamı oluştur (her zaman yeni senaryo üretilir)
            senaryo.uret(n_rovs=secilen_n, n_engels=n_engels, n_adalar=n_adalar, havuz_genisligi=havuz_genisligi, verbose=False)
            
            # Senaryo instance'ını al
            senaryo_instance = _get_instance()
            if not senaryo_instance or not senaryo_instance.aktif:
                return None
            
            aktif_filo = senaryo_instance.filo
            if not aktif_filo:
                return None
            
            # ROV pozisyonlarını senaryo tipine göre ayarla (çeşitli GAT kodları için)
            if hasattr(senaryo_instance.ortam, 'rovs') and senaryo_instance.ortam.rovs:
                from ursina import Vec3
                rovs = [r for r in senaryo_instance.ortam.rovs if r is not None]
                n_rovs_actual = len(rovs)
                
                if senaryo_tipi == 'yakin':
                    # ROV'ları yakın yerleştir (çarpışma riski - GAT kodu 2)
                    merkez_x = random.uniform(-50, 50)
                    merkez_z = random.uniform(-50, 50)
                    for i, rov in enumerate(rovs):
                        # Çok yakın mesafeler (3-8 arası)
                        offset_x = random.uniform(-4, 4)
                        offset_z = random.uniform(-4, 4)
                        if hasattr(rov, 'position'):
                            rov.position = Vec3(merkez_x + offset_x, -5, merkez_z + offset_z)
                
                elif senaryo_tipi == 'dagnik':
                    # ROV'ları dağınık yerleştir (kopma riski - GAT kodu 3)
                    for i, rov in enumerate(rovs):
                        # Geniş alana yay (50+ metre arası)
                        pos_x = random.uniform(-80, 80)
                        pos_z = random.uniform(-80, 80)
                        if hasattr(rov, 'position'):
                            rov.position = Vec3(pos_x, -5, pos_z)
                
                elif senaryo_tipi == 'tek_kume':
                    # Tek kümede toplanmış (karma GAT kodları)
                    merkez_x = random.uniform(-40, 40)
                    merkez_z = random.uniform(-40, 40)
                    for i, rov in enumerate(rovs):
                        offset_x = random.uniform(-15, 15)
                        offset_z = random.uniform(-15, 15)
                        if hasattr(rov, 'position'):
                            rov.position = Vec3(merkez_x + offset_x, -5, merkez_z + offset_z)
                
                elif senaryo_tipi == 'iki_kume':
                    # İki ayrı kümede (liderden uzaklık - GAT kodu 4)
                    merkez1_x, merkez1_z = random.uniform(-60, -20), random.uniform(-60, 60)
                    merkez2_x, merkez2_z = random.uniform(20, 60), random.uniform(-60, 60)
                    for i, rov in enumerate(rovs):
                        if i < n_rovs_actual // 2:
                            merkez_x, merkez_z = merkez1_x, merkez1_z
                        else:
                            merkez_x, merkez_z = merkez2_x, merkez2_z
                        offset_x = random.uniform(-10, 10)
                        offset_z = random.uniform(-10, 10)
                        if hasattr(rov, 'position'):
                            rov.position = Vec3(merkez_x + offset_x, -5, merkez_z + offset_z)
                
                # 'normal' durumunda senaryo.uret() tarafından yerleştirilen pozisyonlar kullanılır
            
            # Birkaç adım simülasyon çalıştır (fizik ve sensör güncellemeleri için)
            for _ in range(10):
                senaryo.guncelle(0.016)
            
            # Senaryo instance'ının get metodunu kullanabilmek için wrapper
            # Bu wrapper, ROV ID kontrolü yapar ve sessiz modda çalışır
            class SenaryoWrapper:
                def __init__(self, instance):
                    self.instance = instance
                    self.filo = instance.filo
                    self.ortam = instance.ortam
                    # Sessiz modu aktif et (GAT eğitimi için)
                    if hasattr(self.filo, 'helper'):
                        self.filo.helper._sessiz_mod = True
                
                def get(self, rov_id, veri_tipi):
                    """ROV verisine erişim (sessiz mod - hata mesajları yok)."""
                    # ROV sayısını kontrol et
                    if hasattr(self.instance, 'ortam') and hasattr(self.instance.ortam, 'rovs'):
                        n_rovs = len([r for r in self.instance.ortam.rovs if r is not None])
                        if rov_id >= n_rovs:
                            return None
                    # Sessiz modu aktif et (her çağrıda)
                    if hasattr(self.filo, 'helper'):
                        self.filo.helper._sessiz_mod = True
                    return self.instance.get(rov_id, veri_tipi)
            
            return {
                'senaryo': SenaryoWrapper(senaryo_instance),
                'filo': aktif_filo,
                'ortam': senaryo_instance.ortam,
                'n_rovs': secilen_n,
                'n_adalar': n_adalar,
                'n_engels': n_engels
            }
        except Exception as e:
            print(f"❌ [GAT_DATA] Veri üretimi sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return None

    def hedef_gorsel_olustur(self, x, y, z):
        """
        Hedef pozisyonunu Ursina'da büyük X işareti olarak gösterir.
        (x, y, z) Ursina koordinatlarıdır (çağıran Koordinator.sim_to_ursina ile geçirir).
        """

        x,y,z=(x,z,y)
        if not self.filo.ortam_ref:
            return

        if self.filo.hedef_gorsel:
            try:
                from ursina import destroy
                destroy(self.filo.hedef_gorsel)
            except:
                pass

        from ursina import Entity, color

        self.filo.hedef_gorsel = Entity()
        self.filo.hedef_gorsel.position = (x, y, z)

        x_boyutu = HareketAyarlari.HEDEF_X_BOYUTU
        kalinlik = HareketAyarlari.HEDEF_KALINLIK

        Entity(
            model='cube',
            position=(0, 0, 0),
            rotation=(90, 0, 45),
            scale=(x_boyutu, kalinlik, kalinlik),
            color=color.rgba(255, 0, 0, 0.5),
            parent=self.filo.hedef_gorsel,
            unlit=True,
            billboard=False
        )

        Entity(
            model='cube',
            position=(0, 0, 0),
            rotation=(90, 0, -45),
            scale=(x_boyutu, kalinlik, kalinlik),
            color=color.rgba(255, 0, 0, 0.5),
            parent=self.filo.hedef_gorsel,
            unlit=True,
            billboard=False
        )

        Entity(
            model='sphere',
            position=(0, 0, 0),
            scale=(2, 2, 2),
            color=color.rgba(255, 0, 0, 0.5),
            parent=self.filo.hedef_gorsel,
            unlit=True
        )

        hedef_rengi = color.rgb(0, 255, 120)
        Entity(
            model='circle',
            position=(0, 0, 0),
            rotation=(90, 0, 0),
            scale=(x_boyutu * 1.5, x_boyutu * 1.5, 1),
            color=hedef_rengi,
            parent=self.filo.hedef_gorsel,
            unlit=True,
            wireframe=True
        )

    def hull(self, offset=50.0):
        """
        Güvenlik hull oluşturur (Thread-safe).
        1. Lider ROV'u merkez alarak 'offset' yarıçaplı dairesel noktalar oluşturur.
        2. Yakındaki adaları 'offset' kadar içeri (lider ROV'a doğru) çeken sanal noktaları alır.
        3. Hepsini birleştirerek adayı DIŞARIDA bırakan güvenli alanı hesaplar.
        
        Ana thread'de değilse, komutu queue'ya ekler.
        """
        if not self.filo._is_main_thread():
            try:
                from ursina import invoke
                result = [None]
                def wrapper():
                    result[0] = self.filo._guvenlik_hull_olustur_impl(offset)
                invoke(wrapper)
                return result[0] if result[0] is not None else {'hull': None, 'points': None, 'center': None}
            except (ImportError, AttributeError):
                self.filo._command_queue.put(('hull', (offset,), {}))
                return {'hull': None, 'points': None, 'center': None}

        return self.filo._guvenlik_hull_olustur_impl(offset)

    def git(self, rov_id: int, x, y: float = None, z: float = None, ai: bool = True, sessiz: bool = False) -> None:
        """
        ROV'a hedef koordinatı doğrudan atar. 
        Koordinat Formatı: (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik)
        """
        target_x, target_y, target_z = 0.0, 0.0, z

        # 1. GİRDİ AYRIŞTIRMA (Nokta listesi, Liste veya Float)
        if isinstance(x, (list, tuple)):
            if not x: return # Boş liste kontrolü
            
            # Durum A: Çoklu Nokta Listesi (Rota) -> [[x1,y1], [x2,y2], ...]
            if isinstance(x[0], (list, tuple)):
                self.filo._git_nokta_listesi[rov_id] = [[float(n[0]), float(n[1])] for n in x if len(n) >= 2]
                self.filo._git_mevcut_nokta_indeksi[rov_id] = 0
                target_x, target_y = self.filo._git_nokta_listesi[rov_id][0]
                
            # Durum B: Tekil Koordinat Listesi -> [x, y] veya [x, y, z]
            else:
                target_x = float(x[0])
                target_y = float(x[1])
                if len(x) >= 3: target_z = float(x[2])
        else:
            # Durum C: Doğrudan float değerler (x, y, z)
            if y is None:
                if not sessiz: print("❌ [FİLO] Y koordinatı eksik.")
                return
            target_x, target_y = float(x), float(y)

        # 2. DOĞRUDAN UYGULAMA
        # (Not: Harici log satırı kaldırıldı - hedef atamalarında fazladan stdout yazdırmamak için.)
        # Hiçbir bekletme yapmadan asıl işi yapan alt fonksiyona gönder
        self._git_impl(rov_id, target_x, target_y, target_z, ai, sessiz)

    def git_path(self, rov_id, hedef, ai=True, isaret=False):
        """
        ROV'a bir yol atar ve otomatik moda geçirir (Thread-safe).
        ROV'un mevcut derinliğini korur.
        isaret=True ise bir sonraki waypoint minimapte gösterilir.
        """
        from FiratROVNet.gnc import Koordinator

        # Thread-safe: Ana thread değilse queue'ya ekle (invoke yerine queue = daha güvenilir)
        if not self.filo._is_main_thread():
            self.filo._command_queue.put(('git_path', (rov_id, hedef, ai), {'isaret': isaret}))
            return

        self._git_path_impl(rov_id, hedef, ai, isaret)
    
    def _git_path_impl(self, rov_id, hedef, ai=True, isaret=False):
        """
        git_path() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır).
        ROV'un mevcut derinliğini korur.
        isaret=True ise bir sonraki waypoint minimapte gösterilir.
        """
        from FiratROVNet.gnc import Koordinator

        if not hasattr(self.filo, '_git_isaret'):
            self.filo._git_isaret = {}
        self.filo._git_isaret[rov_id] = bool(isaret)
        
        # ROV'un mevcut derinliğini al (Sim formatında)
        if len(self.filo.sistemler) == 0 or rov_id >= len(self.filo.sistemler):
            print(f"❌ [FİLO] Geçersiz ROV ID: {rov_id}")
            return
        
        current_sim_pos = Koordinator.ursina_to_sim(
            self.filo.sistemler[rov_id].rov.x,
            self.filo.sistemler[rov_id].rov.y,
            self.filo.sistemler[rov_id].rov.z
        )
        current_z = current_sim_pos[2]  # Mevcut derinlik (Sim formatında Z)
        
        # A* için 2D koordinatlar (x, y) - z derinlik bilgisi kullanılmaz
        start_2d = (current_sim_pos[0], current_sim_pos[1])
        
        # Hedef de 2D olmalı (eğer 3D ise ilk 2 elemanı al)
        if isinstance(hedef, (tuple, list)) and len(hedef) >= 2:
            goal_2d = (float(hedef[0]), float(hedef[1]))
        else:
            print(f"❌ [FİLO] Geçersiz hedef formatı: {hedef}")
            return
        
        # Daha güvenli yol için safety_margin artırıldı (ROV boyutu ve manevra alanı için)
        path = self.a_star(start=start_2d, goal=goal_2d, safety_margin=5.0)
        if not isinstance(path, list) or len(path) == 0:
            print(f"❌ [FİLO] Geçersiz yol listesi: {path}")
            return

        #gidilecek_n = self.filo.gidilecek_noktalar(path)
        gidilecek_noktalar = self.filo.gidilecek_noktalar_n(path,10)
        
        # Mevcut derinliği koruyarak git() çağır
        if len(gidilecek_noktalar) > 0:
            self.filo.git(rov_id, gidilecek_noktalar, z=current_z, ai=ai)
        else:
            print(f"⚠️ [FİLO] Gidilecek nokta bulunamadı (yol çok kısa olabilir)")

    def _git_impl(self, rov_id: int, x: float, y: float, z: float = None, ai: bool = True, sessiz: bool = True) -> None:
        """
        git() fonksiyonunun sadeleştirilmiş implementasyonu.
        Bağımlılıklar temizlendi, koordinat hesabı manuel yapıldı.
        """
        # 1. TEMEL KONTROLLER
        if not (0 <= rov_id < len(self.filo.sistemler)):
            print(f"❌ [FİLO] Geçersiz ROV ID: {rov_id}")
            return

        gnc_sistemi = self.filo.sistemler[rov_id]

        # 2. DURUM AYARLARI
        gnc_sistemi.manuel_kontrol = False
        gnc_sistemi.ai_aktif = ai

        # 3. MANUEL DERİNLİK HESABI (Z verilmemişse)
        if z is None:
            # Ursina'da Y dikey eksendir. Simülasyonda Z derinliktir.
            # Ursina'da aşağı indikçe Y değeri negatifleşir.
            # Simülasyon Z (derinlik) ise aşağı indikçe pozitif artar.
            # Bu yüzden: Simülasyon_Z = -Ursina_Y
            z = float(gnc_sistemi.rov.y)

        # 4. HEDEF ATAMA
        try:
            # GNC nesnesine hedefi Simülasyon formatında ilet
            gnc_sistemi.hedef_atama(x, y, z)
            
            # Takip için filo sözlüğüne kaydet
            if not hasattr(self.filo, '_rov_hedefleri'):
                self.filo._rov_hedefleri = {}
            self.filo._rov_hedefleri[rov_id] = (x, y, z)
            
            if not sessiz:
                print(f"✅ [FİLO] ROV-{rov_id} -> Hedef: ({x}, {y}, {z}) | AI: {'AÇIK' if ai else 'KAPALI'}")
                
        except Exception as e:
            if not sessiz: print(f"❌ [HATA] Hedef atanamadı: {e}")

    def harita(self, goster=True, convex=True, a_star=True):
        """Harita penceresini açar, kapatır veya görünürlük ayarlarını yapar."""
        if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'harita') and self.filo.ortam_ref.harita:
            self.filo.ortam_ref.harita.goster(goster, convex, a_star)

    def minimap(self, durum=True, convex=True, a_star=True, scale=None, grid=None, *args, **kwargs):
        """
        Minimap'i açar, kapatır veya durumunu döndürür.
        scale: çarpan (1=taban 0.45, 2=2 katı, 0.1=4.5 vb.); verilirse boyut dinamik güncellenir.
        grid: grid sayısı (None=varsayılan GRID_UNIT m; N=toplam N aralık, 1 grid=(2*havuz)/N m).
        filo.minimap("ekle", filo.ada_cevre()) ile ada çevre noktaları minimapte turuncu-kahverengi çizgi olarak gösterilir.
        """
        # filo.minimap("ekle", points) — ada çevre noktalarını minimapte çiz (durum="ekle", convex=points)
        if durum == "ekle" and convex is not None and hasattr(convex, '__iter__') and not isinstance(convex, (bool, str)):
            points = list(convex)
            if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'minimap') and self.filo.ortam_ref.minimap:
                if hasattr(self.filo.ortam_ref.minimap, 'update_ada_cevre'):
                    self.filo.ortam_ref.minimap.update_ada_cevre(points)
                    print(f"✅ [MİNİMAP] Ada çevre noktaları güncellendi: {points}")
                    if not self.filo.ortam_ref.minimap.visible:
                        self.filo.ortam_ref.minimap.goster(True)
                else:
                    print("⚠️ [MİNİMAP] update_ada_cevre bulunamadı.")
            else:
                print("❌ [MİNİMAP] Minimap sistemi bulunamadı!")
            return

        if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'minimap') and self.filo.ortam_ref.minimap:
            if not hasattr(self.filo.ortam_ref.minimap, 'filo_ref') or self.filo.ortam_ref.minimap.filo_ref != self.filo:
                self.filo.ortam_ref.minimap.filo_ref = self.filo

            if durum is None:
                if scale is not None or grid is not None:
                    self.filo.ortam_ref.minimap.goster(
                        self.filo.ortam_ref.minimap.visible, convex, a_star, scale=scale, grid=grid
                    )
                self.filo.ortam_ref.minimap.visible = not self.filo.ortam_ref.minimap.visible
                status = "AÇIK" if self.filo.ortam_ref.minimap.visible else "KAPALI"
                print(f"🗺️ [MİNİMAP] Minimap şu an {status}")
            else:
                self.filo.ortam_ref.minimap.goster(durum, convex, a_star, scale=scale, grid=grid)
        else:
            print("❌ [MİNİMAP] Minimap sistemi bulunamadı!")

    def a_star(self, start=None, goal=None, safety_margin=2.0, **kwargs):
        """
        A* algoritması kullanarak başlangıçtan hedefe yol hesaplar.
        """
        if start is None:
            start = kwargs.get('start')
        if goal is None:
            goal = kwargs.get('goal')
        if safety_margin == 2.0:
            safety_margin = kwargs.get('safety_margin', 2.0)

        if isinstance(start, int):
            rov_id = start
            try:
                gps_bilgisi = self.filo.get(rov_id, 'gps')
                if gps_bilgisi is None:
                    print(f"❌ [FİLO] ROV-{rov_id} için GPS bilgisi alınamadı!")
                    return None
                if isinstance(gps_bilgisi, (tuple, list)) and len(gps_bilgisi) >= 2:
                    start = (float(gps_bilgisi[0]), float(gps_bilgisi[1]))
                    print(f"✅ [FİLO] ROV-{rov_id}'ın GPS'inden başlangıç: {start}")
                else:
                    print(f"❌ [FİLO] ROV-{rov_id} için geçersiz GPS formatı: {gps_bilgisi}")
                    return None
            except Exception as e:
                print(f"❌ [FİLO] ROV-{rov_id} GPS bilgisi alınırken hata: {e}")
                return None

        if start is None or goal is None:
            print("❌ [FİLO] A* için start ve goal parametreleri gerekli!")
            print("   Kullanım: filo.a_star(start=(x1, y1), goal=(x2, y2), safety_margin=2.0)")
            print("   veya: filo.a_star(start=rov_id, goal=(x2, y2))  # ROV ID ile başlangıç")
            return None

        if not isinstance(start, (tuple, list)) or len(start) < 2:
            print(f"❌ [FİLO] Start parametresi geçersiz format: {start}")
            print("   Format: (x, y) tuple veya [x, y] list olmalı")
            return None
        
        # Start'ı 2D'ye normalize et (eğer 3D ise ilk 2 elemanı al)
        if len(start) >= 2:
            start = (float(start[0]), float(start[1]))
        
        # Goal'ı da kontrol et ve 2D'ye normalize et
        if not isinstance(goal, (tuple, list)) or len(goal) < 2:
            print(f"❌ [FİLO] Goal parametresi geçersiz format: {goal}")
            print("   Format: (x, y) tuple veya [x, y] list olmalı")
            return None
        
        if len(goal) >= 2:
            goal = (float(goal[0]), float(goal[1]))

        if not self.filo.ortam_ref or not hasattr(self.filo.ortam_ref, 'harita') or self.filo.ortam_ref.harita is None:
            print("❌ [FİLO] Harita sistemi bulunamadı!")
            return None

        try:
            return self.filo.ortam_ref.harita.a_star_yolu_hesapla(
                start=start,
                goal=goal,
                safety_margin=safety_margin
            )
        except Exception as e:
            print(f"❌ [FİLO] A* yolu hesaplanırken hata: {e}")
            import traceback
            traceback.print_exc()
            return None

    def _a_star_path_al(self, path=None):
        """path None ise harita.a_star_yolu döner, yoksa []."""
        if path is not None:
            return path
        harita = getattr(getattr(self.filo, 'ortam_ref', None), 'harita', None)
        return getattr(harita, 'a_star_yolu', None) if harita else None

    def gidilecek_noktalar_n(self, path=None, n=10):
        """A* yolu üzerinde her n metre sonra waypoint döndürür. path None ise harita.a_star_yolu kullanılır."""
        path = self._a_star_path_al(path)
        if not path or len(path) < 2 or n <= 0:
            return []

        rotalar, toplam, next_stop = [], 0.0, float(n)
        for i in range(1, len(path)):
            x0, y0 = float(path[i - 1][0]), float(path[i - 1][1])
            x1, y1 = float(path[i][0]), float(path[i][1])
            seg_len = np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
            if seg_len < 1e-9:
                continue
            while toplam + seg_len >= next_stop:
                t = max(0.0, min(1.0, (next_stop - toplam) / seg_len))
                rotalar.append([x0 + t * (x1 - x0), y0 + t * (y1 - y0)])
                next_stop += n
            toplam += seg_len

        son = [float(path[-1][0]), float(path[-1][1])]
        if not rotalar or rotalar[-1][0] != son[0] or rotalar[-1][1] != son[1]:
            rotalar.append(son)
        return rotalar

    def move(self, rov_id: int, yon: str, guc: float = 1.0, sessiz: bool = True) -> None:
        """
        ROV'a güç bazlı hareket komutu verir. Eşzamanlı hareket: sadece ilgili eksen güncellenir,
        diğer eksenler korunur (örn. ileri + sağ = çapraz hareket).
        State: rov.active_forces = {'surge', 'sway', 'heave', 'yaw'} — her frame ROV.update() bunlardan velocity hesaplar.
        sessiz: True (varsayılan) ise log yazılmaz; False ise bilgi/hata mesajları yazdırılır.
        """
        # --- Hata kontrolleri (mevcut kontroller korunur) ---
        if len(self.filo.sistemler) == 0:
            if not sessiz:
                print("❌ [HATA] GNC sistemleri henüz kurulmamış!")
                print("   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return

        if not isinstance(rov_id, int) or rov_id < 0:
            if not sessiz:
                print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
                print(f"   Mevcut ROV sayısı: {len(self.filo.sistemler)} (0-{len(self.filo.sistemler)-1} arası)")
            return

        if rov_id >= len(self.filo.sistemler):
            if not sessiz:
                print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
                print(f"   Mevcut ROV sayısı: {len(self.filo.sistemler)} (0-{len(self.filo.sistemler)-1} arası)")
                print("   💡 Çözüm: filo.ekle() ile daha fazla GNC sistemi ekleyin")
            return

        gecerli_yonler = ['ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur', 'yaw']
        if yon not in gecerli_yonler:
            if not sessiz:
                print(f"❌ [HATA] Geçersiz hareket yönü: '{yon}'")
                print(f"   Geçerli yönler: {', '.join(gecerli_yonler)}")
            return

        if not isinstance(guc, (int, float)):
            if not sessiz:
                print(f"❌ [HATA] Güç değeri sayı olmalı: {guc}")
            return

        if yon == 'yaw':
            guc = max(-1.0, min(1.0, float(guc)))
        else:
            guc = max(0.0, min(1.0, float(guc)))

        try:
            # Hibrit kontrol: move() otonom (AI) sistemini kapatmaz; sadece manuel kuvvet hafızasını günceller.
            # Final_Force = AI_Output + Manual_Input (guncelle() ve ROV.update() içinde vektörel toplanır).
            gnc = self.filo.sistemler[rov_id]
            rov = gnc.rov

            # --- State: active_forces (yoksa oluştur, sadece ilgili eksen güncellenir) ---
            if not hasattr(rov, 'active_forces') or rov.active_forces is None:
                rov.active_forces = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0, 'yaw': 0.0}

            # --- dur: Sadece manuel kuvvetleri (active_forces) sıfırla; otonom hedefe dokunulmaz ---
            if yon == 'dur':
                for k in rov.active_forces:
                    rov.active_forces[k] = 0.0
                if hasattr(rov, 'manuel_hareket'):
                    rov.manuel_hareket['yon'] = 'dur'
                    rov.manuel_hareket['guc'] = 0.0
                # velocity'a dokunmuyoruz: AI hâlâ hedefe gidiyorsa devam eder (hibrit)
                if not sessiz:
                    print(f"🛑 [FİLO] ROV-{rov_id} manuel giriş sıfırlandı (otonom devam edebilir)")
                return

            # --- yaw: Sadece dönüş hızı güncelle (ROV.update() her frame uygular) ---
            if yon == 'yaw':
                rov.active_forces['yaw'] = guc
                if hasattr(rov, 'manuel_hareket'):
                    rov.manuel_hareket['yon'] = 'yaw'
                    rov.manuel_hareket['guc'] = guc
                if not sessiz:
                    guc_yuzdesi = int(abs(guc) * 100)
                    yon_metni = "saat yönünün tersine" if guc > 0 else "saat yönünde"
                    print(f"🔄 [FİLO] ROV-{rov_id} {yon_metni} %{guc_yuzdesi} güçle döndürülüyor (yaw)")
                return

            # --- Havuz / güvenlik sınırları (mevcut kontroller korunur) ---
            if yon == 'bat' and rov.role == 1:
                if not sessiz:
                    print(f"⚠️ [FİLO] ROV-{rov_id} lider, batırılamaz!")
                return

            HAVUZ_GUVENLIK_MESAFESI = 10.0
            if hasattr(rov, 'environment_ref') and rov.environment_ref:
                havuz_genisligi = getattr(rov.environment_ref, 'havuz_genisligi', 200)
                havuz_sinir = havuz_genisligi
                guvenli_sinir = havuz_sinir - HAVUZ_GUVENLIK_MESAFESI

                sinirda_x = abs(rov.x) >= guvenli_sinir * 0.95
                sinirda_z = abs(rov.z) >= guvenli_sinir * 0.95
                sinirda_y_ust = rov.y >= 0.3
                sinirda_y_alt = rov.y <= -95

                if sinirda_x and ((yon == 'sag' and rov.x > 0) or (yon == 'sol' and rov.x < 0)):
                    if not sessiz:
                        print(f"⚠️ [FİLO] ROV-{rov_id} havuz sınırında (X), {yon} yönünde hareket engellendi")
                    return

                if sinirda_z and ((yon == 'ileri' and rov.z > 0) or (yon == 'geri' and rov.z < 0)):
                    if not sessiz:
                        print(f"⚠️ [FİLO] ROV-{rov_id} havuz sınırında (Z), {yon} yönünde hareket engellendi")
                    return

                if sinirda_y_ust and yon == 'cik':
                    if not sessiz:
                        print(f"⚠️ [FİLO] ROV-{rov_id} su yüzeyinde, yukarı hareket engellendi")
                    return

                if sinirda_y_alt and yon == 'bat':
                    if not sessiz:
                        print(f"⚠️ [FİLO] ROV-{rov_id} deniz tabanında, aşağı hareket engellendi")
                    return

            # --- Sadece ilgili ekseni güncelle (diğer eksenlere dokunma) ---
            # Mapping: ileri/geri -> surge, sag/sol -> sway, cik/bat -> heave. guc=0 ise sadece o ekseni sıfırla.
            if yon == 'ileri':
                rov.active_forces['surge'] = guc
            elif yon == 'geri':
                rov.active_forces['surge'] = -guc
            elif yon == 'sag':
                rov.active_forces['sway'] = guc
            elif yon == 'sol':
                rov.active_forces['sway'] = -guc
            elif yon == 'cik':
                rov.active_forces['heave'] = guc
            elif yon == 'bat' and rov.role != 1:
                rov.active_forces['heave'] = -guc

            # Manuel mod: ROV.update() her frame active_forces'tan velocity hesaplayacak
            if hasattr(rov, 'manuel_hareket'):
                rov.manuel_hareket['yon'] = yon
                rov.manuel_hareket['guc'] = guc

            if not sessiz:
                guc_yuzdesi = int(abs(guc) * 100)
                print(f"🔵 [FİLO] ROV-{rov_id} {yon} yönünde %{guc_yuzdesi} güçle hareket ediyor (eşzamanlı mod)")
        except AttributeError as e:
            if not sessiz:
                print(f"❌ [HATA] ROV-{rov_id} için gerekli özellik bulunamadı: {e}")
                print(f"   💡 Debug: GNC sistemi tipi: {type(self.filo.sistemler[rov_id])}")
                import traceback
                traceback.print_exc()
        except Exception as e:
            if not sessiz:
                print(f"❌ [HATA] Hareket komutu sırasında hata: {e}")
                print(f"   💡 Debug: ROV ID: {rov_id}, Yön: {yon}, Güç: {guc}")
                import traceback
                traceback.print_exc()


    def formasyon(self, formasyon_id="LINE", aralik=None, is_3d=False, lider_koordinat=None, dinamik=True):
        """
        Filoyu belirtilen formasyona sokar.
        Formasyon.pozisyonlar() ile pozisyonları alır ve filo.git() ile uygular.
        """
        if aralik is None:
            aralik = HareketAyarlari.FORMASYON_VARSAYILAN_ARALIK
        formasyon_obj = Formasyon(self.filo)
        pozisyonlar = formasyon_obj.pozisyonlar(formasyon_id, aralik, is_3d=is_3d, lider_koordinat=lider_koordinat)

        if not pozisyonlar or len(pozisyonlar) == 0:
            print("❌ [FORMASYON] Pozisyonlar alınamadı!")
            return None if lider_koordinat is not None else None

        if len(pozisyonlar) != len(self.filo.sistemler):
            print(f"⚠️ [FORMASYON] Uyarı: Pozisyon sayısı ({len(pozisyonlar)}) ROV sayısı ({len(self.filo.sistemler)}) ile eşleşmiyor!")

        if lider_koordinat is not None:
            ursina_positions = []
            for pozisyon in pozisyonlar:
                config_x, config_y, config_z = pozisyon
                ursina_positions.append((config_x, config_y, config_z))
            print(f"✅ [FORMASYON] Pozisyonlar hesaplandı: Tip={formasyon_id}, Aralık={aralik}, ROV Sayısı={len(pozisyonlar)}")
            return [(x, z, y) for x, y, z in ursina_positions]

        # Aktif formasyonu kaydet (dinamik takip için)
        if dinamik:
            self.filo.aktif_formasyon = {
                'id': formasyon_id,
                'aralik': aralik,
                'is_3d': is_3d
            }
        else:
            self.filo.aktif_formasyon = None

        # Lideri bul
        lider_id = 0
        for i, sistem in enumerate(self.filo.sistemler):
             if hasattr(sistem, 'rov') and sistem.rov.role == 1:
                 lider_id = i
                 break

        for i, pozisyon in enumerate(pozisyonlar):
            if i >= len(self.filo.sistemler):
                break
            
            # Lider ROV kontrolü: Eğer hareket halindeyse (hedefi varsa), ona dokunma
            if i == lider_id:
                mevcut_hedef = self.filo.hedef(rov_id=i)
                if mevcut_hedef is not None:
                    # Liderin zaten bir hedefi var, dokunma.
                    print(f"ℹ️ [FORMASYON] ROV-{i} (Lider) hareket halinde, mevcut hedefine devam ediyor.")
                    continue

            sim_x, sim_y, sim_z = pozisyon
            if sim_z >= 0:
                sim_z = -10.0
            try:
                self.filo.git(i, sim_x, sim_y, sim_z, ai=True)
                print(f"✅ [FORMASYON] ROV-{i} hedefi ayarlandı: ({sim_x:.2f}, {sim_y:.2f}, {sim_z:.2f})")
            except Exception as e:
                print(f"⚠️ [FORMASYON] ROV-{i} için hedef ayarlanırken hata: {e}")

        print(f"✅ [FORMASYON] Formasyon kuruldu: Tip={formasyon_id}, Aralık={aralik}, ROV Sayısı={len(pozisyonlar)}")
        return None


    def formasyon_sec(self, margin=None, is_3d=False, offset=None, minimap=True, dinamik=True):
            """
            Convex hull kullanarak en uygun formasyonu seçer.
            Yaw senkronizasyon mantığı tamamen kaldırılmıştır (Fizik motoruna devredildi).
            minimap=True ise hesaplanan alan Ursina UI Minimap üzerinde gösterilir.
            """
            # 1. Minimap Kontrolü ve Açılması
            if minimap and self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'minimap'):
                m_ui = self.filo.ortam_ref.minimap
                if m_ui:
                    m_ui.goster(True) # Minimap'i görünür yap

            # 2. Thread Güvenliği (Ursina/Panda3D senkronizasyonu)
            if not self.filo._is_main_thread():
                if hasattr(self.filo, '_command_queue'):
                    self.filo._command_queue.put(('formasyon_sec', (margin, is_3d, offset), {'dinamik': dinamik}))
                return None

            # 3. Ana Hesaplama Fonksiyonunu Çağır
            return self._formasyon_sec_impl(margin, is_3d, offset, dinamik=dinamik)

    def _formasyon_sec_impl(self, margin=None, is_3d=False, offset=None, sessiz=True, dinamik=True):
        """
        formasyon_sec() fonksiyonunun gerçek implementasyonu.
        Hesaplanan Hull verisini doğrudan Minimap (Entity) nesnesine paslar.
        """
        if margin is None: margin = HareketAyarlari.FORMASYON_MESAFESI
        if offset is None: offset = HareketAyarlari.FORMASYON_OFFSET
        
        try:
            self.filo._formasyon_hedefleri.clear()

            # --- 1. GÜVENLİK ALANI (HULL) HESAPLAMA ---
            yasakli_noktalar = self.filo._prepare_forbidden_points()
            guvenlik_hull_dict = self.filo.yeni_hull(
                yasakli_noktalar=yasakli_noktalar,
                offset=offset,
                alpha=2.0,
                buffer_radius=10.0,
                channel_width=10.0
            )

            hull_noktalari = guvenlik_hull_dict.get("hull")
            hull_merkez = guvenlik_hull_dict.get("center")

            if hull_noktalari is None or hull_merkez is None:
                if not sessiz: print("⚠️ [FORMASYON] Güvenlik alanı (Hull) oluşturulamadı.")
                return None

            # --- 2. MİNİMAP UI ÜZERİNDE GÖSTERİM ---
            # Minimap sınıfındaki update_hull metodunu doğrudan tetikliyoruz
            if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'minimap'):
                m_ui = self.filo.ortam_ref.minimap
                if m_ui and hasattr(m_ui, 'update_hull'):
                    # Bu metod senin sınıfında: Eski hull'u siler, yenisini cyan mesh olarak çizer.
                    m_ui.update_hull(hull_noktalari)
            # ---------------------------------------

            # 3. LİDER VE MERKEZ BİLGİLERİ
            hull_merkez = self.filo._normalize_hull_center(hull_merkez)
            lider_id, lider_gps = self.filo._find_leader_info(sessiz=sessiz)
            
            if lider_id is None:
                if not sessiz: print("⚠️ [FORMASYON] Aktif bir lider bulunamadı.")
                return None
            
            # Lider konumu yoksa hull merkezini baz al
            if lider_gps is None: lider_gps = hull_merkez

            # 4. FORMASYON ARAMA VE YERLEŞTİRME
            # (Yaw senkronizasyon değişkenleri buradan temizlendi)
            min_aralik = HareketAyarlari.FORMASYON_MIN_ARALIK
            baslangic_aralik = margin
            
            # Farklı yönlerden (0, 90, 180, 270 derece) sığdırma denemeleri
            arama_noktalari = self.filo._generate_search_points(lider_gps, hull_merkez)
            
            for nokta_adi, merkez_koord in arama_noktalari:
                for deneme_yaw in [0, 90, 180, 270]:
                    denenecek_ids = self.filo._get_formation_ids_to_try()
                    
                    for f_id in denenecek_ids:
                        aralik = baslangic_aralik
                        while aralik >= min_aralik:
                            # _try_formation_fit: Formasyonun hull içine sığıp sığmadığını kontrol eder
                            if self.filo._try_formation_fit(f_id, aralik, is_3d, merkez_koord,
                                                            deneme_yaw, hull_noktalari, lider_id, 
                                                            nokta_adi, sessiz=sessiz, dinamik=dinamik):
                                
                                if not sessiz:
                                    durum = "Dinamik" if dinamik else "Sabit"
                                    print(f"✅ [MİNİMAP] {durum} {f_id} seçildi. Alan haritaya cyan olarak işlendi.")
                                
                                return (f_id, aralik, deneme_yaw, merkez_koord)
                            aralik -= 1.0 # Aralığı daraltarak tekrar dene

            if not sessiz: print("⚠️ [FORMASYON] Mevcut hull içine sığan formasyon bulunamadı.")
            return None

        except Exception as e:
            print(f"❌ [FORMASYON HATASI]: {e}")
            return None
    
    def ada_cevre(self, offset: float = 15.0, sessiz: bool = False) -> list:
        """
        Simülasyondaki adaları tespit edip her ada için eşit çevrede 12 nokta döndürür.
        
        Args:
            offset: Ada yarıçapından uzaklık (metre, varsayılan: 15.0)
        
        Returns:
            list: [(x1, y1, z1), (x2, y2, z2), ...] - Ada çevresi noktaları (Simülasyon formatı)
        """
        if not self.filo.ortam_ref:
            if not sessiz:
                print("⚠️ [UYARI] Ortam referansı bulunamadı!")
            return []
        
        if not hasattr(self.filo.ortam_ref, 'island_positions') or not self.filo.ortam_ref.island_positions:
            if not sessiz:
                print("⚠️ [UYARI] Simülasyonda ada bulunamadı!")
            return []
        
        tum_noktalar = []
        
        for island_data in self.filo.ortam_ref.island_positions:
            # None kontrolü (çıkarılmış adalar için None olabilir)
            if island_data is None:
                continue
            
            if len(island_data) < 3:
                continue
            
            island_x = float(island_data[0])
            island_z = float(island_data[1])
            island_radius = float(island_data[2])
            
            cevre_mesafesi = island_radius + offset
            acilar = [i * 30 for i in range(12)]  # 0°, 30°, 60°, ..., 330° (12 nokta)
            
            for aci in acilar:
                aci_rad = math.radians(aci)
                nokta_x = island_x + cevre_mesafesi * math.sin(aci_rad)
                nokta_y = island_z + cevre_mesafesi * math.cos(aci_rad)
                nokta_z = 0.0
                tum_noktalar.append((nokta_x, nokta_y, nokta_z))
        
        if not sessiz and getattr(self.filo.ortam_ref, "verbose", False):
            aktif_ada_sayisi = sum(1 for ada in self.filo.ortam_ref.island_positions if ada is not None)
            print(f"✅ [ADA_CEVRE] {aktif_ada_sayisi} ada için {len(tum_noktalar)} nokta hesaplandı (offset={offset}m)")
        return tum_noktalar
    
    def yeniden_ciz(self, noktalar: list, yasakli_noktalar: list, alpha: float = 2.0, 
                   buffer_radius: float = 15.0, channel_width: float = 10.0) -> list:
        """
        Verilen nokta kümesini saran, ancak yasaklı noktaları dışarıda bırakacak şekilde
        içeri bükülmüş sınırın koordinatlarını döndürür.
        
        Args:
            noktalar: Nokta listesi [(x1, y1), (x2, y2), ...]
            yasakli_noktalar: Yasaklı nokta listesi [(x1, y1), (x2, y2), ...]
            alpha: Alpha shape parametresi (varsayılan: 2.0)
            buffer_radius: Yasaklı bölge yarıçapı (metre, varsayılan: 15.0)
            channel_width: Kanal genişliği (metre, varsayılan: 10.0)
        
        Returns:
            list: Yeni kontur noktaları [(x1, y1), (x2, y2), ...]
        """
        if not SHAPELY_AVAILABLE:
            print("❌ [HATA] shapely kütüphanesi bulunamadı!")
            return []
        
        try:
            from shapely.geometry import Point, LineString, Polygon, MultiPolygon
            from shapely.ops import unary_union, nearest_points
            from scipy.spatial import ConvexHull
        except ImportError as e:
            print(f"❌ [HATA] Gerekli kütüphaneler eksik: {e}")
            return []
        
        try:
            # Giriş verisini düzenle
            points_cloud = []
            for p in noktalar:
                if len(p) >= 2:
                    points_cloud.append((float(p[0]), float(p[1])))
            
            if len(points_cloud) < 3:
                print("⚠️ [UYARI] Yeterli nokta yok (en az 3 nokta gerekli)")
                return []
            
            # Temel şekli (Convex Hull) oluştur
            try:
                points_np = np.array(points_cloud)
                hull = ConvexHull(points_np)
                hull_points = points_np[hull.vertices]
                base_shape = Polygon(hull_points)
            except Exception as e:
                print(f"❌ [HATA] Başlangıç Hull oluşturulamadı: {e}")
                return []
            
            if not base_shape.is_valid:
                base_shape = base_shape.buffer(0)
            
            final_shape = base_shape
            kesilen_nokta_sayisi = 0
            
            # Yasaklı noktaları kesip çıkar
            if yasakli_noktalar:
                if getattr(self.filo.ortam_ref, "verbose", False):
                    print(f"🔍 [YENIDEN_CIZ] Kontrol edilecek yasaklı nokta: {len(yasakli_noktalar)}")
                
                for i, fp in enumerate(yasakli_noktalar):
                    if len(fp) < 2:
                        continue
                    
                    p_obj = Point(float(fp[0]), float(fp[1]))
                    
                    if not final_shape.contains(p_obj):
                        continue
                    
                    kesilen_nokta_sayisi += 1
                    
                    # Yasaklı Bölge (Güvenlik Çemberi)
                    forbidden_zone = p_obj.buffer(buffer_radius)
                    
                    # Kanal Açma (En kısa yoldan dışarı tünel)
                    exterior_line = final_shape.exterior
                    p1, p2 = nearest_points(forbidden_zone, exterior_line)
                    
                    channel_line = LineString([p_obj, p2])
                    channel_poly = channel_line.buffer(max(channel_width, buffer_radius * 0.5))
                    
                    # Kesme işlemi
                    cut_area = unary_union([forbidden_zone, channel_poly])
                    final_shape = final_shape.difference(cut_area)
                    
                    # Parçalanma kontrolü
                    if isinstance(final_shape, MultiPolygon):
                        if not final_shape.is_empty:
                            final_shape = max(final_shape.geoms, key=lambda a: a.area)
                        else:
                            final_shape = base_shape
            
            if getattr(self.filo.ortam_ref, "verbose", False):
                print(f"✅ [YENIDEN_CIZ] İşlem tamam. Kesilen engel sayısı: {kesilen_nokta_sayisi}")
            
            # Sonuç koordinatlarını döndür
            if isinstance(final_shape, Polygon):
                return list(final_shape.exterior.coords)
            else:
                print("⚠️ [UYARI] Sonuç bir Polygon değil.")
                return []
        
        except Exception as e:
            print(f"❌ [HATA] Kontur hesaplama genel hatası: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    def yeni_hull(self, yasakli_noktalar: list, offset: float = 40.0, alpha: float = 2.0,
                  buffer_radius: float = 20.0, channel_width: float = 15.0) -> dict:
        """
        Mevcut hull noktalarını alır, yasaklı bölgeleri kesip çıkarır.
        
        Args:
            yasakli_noktalar: Yasaklı nokta listesi
            offset: ROV hull genişletme mesafesi
            alpha: Alpha shape parametresi
            buffer_radius: Yasaklı bölge yarıçapı
            channel_width: Kanal genişliği
        
        Returns:
            dict: {'hull': hull_obj, 'points': points_array, 'center': center_tuple}
        """
        try:
            if not SHAPELY_AVAILABLE:
                return {'hull': None, 'points': None, 'center': None}
            
            from shapely.geometry import Point, Polygon
            
            # Mevcut Hull'ı Al
            guvenlik_hull_dict = self.filo.hull_manager.hull(offset=offset)
            hull_noktalari = guvenlik_hull_dict.get("points")
            eski_hull_merkez = guvenlik_hull_dict.get("center")
            
            if hull_noktalari is None:
                return {'hull': None, 'points': None, 'center': None}
            
            # Noktaları Hazırla
            hull_noktalari_2d = []
            if isinstance(hull_noktalari, np.ndarray):
                hull_noktalari_2d = [[float(p[0]), float(p[1])] for p in hull_noktalari]
            else:
                hull_noktalari_2d = [[float(p[0]), float(p[1])] for p in hull_noktalari if len(p) >= 2]
            
            yasakli_noktalar_2d = []
            if yasakli_noktalar:
                for nokta in yasakli_noktalar:
                    if len(nokta) >= 2:
                        yasakli_noktalar_2d.append([float(nokta[0]), float(nokta[1])])
            
            # Yeniden Çiz
            if yasakli_noktalar_2d:
                yeni_kontur_noktalari = self.yeniden_ciz(
                    noktalar=hull_noktalari_2d,
                    yasakli_noktalar=yasakli_noktalar_2d,
                    alpha=alpha,
                    buffer_radius=buffer_radius,
                    channel_width=channel_width
                )
            else:
                yeni_kontur_noktalari = hull_noktalari_2d
            
            # Sonuçları Paketle
            if yeni_kontur_noktalari and len(yeni_kontur_noktalari) >= 3:
                kontur_noktalari_np = np.array(yeni_kontur_noktalari)
                
                yeni_poly = Polygon(yeni_kontur_noktalari)
                if not yeni_poly.is_valid:
                    yeni_poly = yeni_poly.buffer(0)
                
                # Merkez hesapla
                eski_merkez_2d = (eski_hull_merkez[0], eski_hull_merkez[1])
                if yeni_poly.contains(Point(eski_merkez_2d)):
                    final_merkez_2d = eski_merkez_2d
                else:
                    guvenli_nokta = yeni_poly.representative_point()
                    final_merkez_2d = (guvenli_nokta.x, guvenli_nokta.y)
                
                eski_z = eski_hull_merkez[2] if eski_hull_merkez and len(eski_hull_merkez) >= 3 else 0.0
                yeni_hull_merkez = (float(final_merkez_2d[0]), float(final_merkez_2d[1]), float(eski_z))
                
                # SahteHull sınıfı
                class SahteHull:
                    def __init__(self, points, polygon_obj):
                        self.points = points
                        self.polygon = polygon_obj
                        self.vertices = np.arange(len(points))
                        self.simplices = []
                        for i in range(len(points)):
                            self.simplices.append([i, (i + 1) % len(points)])
                        self.simplices = np.array(self.simplices)

                    def __len__(self):
                        return len(self.points)

                    def __iter__(self):
                        return iter(self.points)

                    def __getitem__(self, idx):
                        return self.points[idx]

                custom_hull = SahteHull(kontur_noktalari_np, yeni_poly)
                
                # Haritaya gönder (convex_hull_data; harita otomatik açılmaz — filo.harita() / filo.minimap() ile açılır)
                if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'harita') and self.filo.ortam_ref.harita:
                    hull_data = {
                        'hull': custom_hull,
                        'points': kontur_noktalari_np,
                        'center': yeni_hull_merkez
                    }
                    self.filo.ortam_ref.harita.convex_hull_data = hull_data
                
                return {
                    'hull': custom_hull,
                    'points': kontur_noktalari_np,
                    'center': yeni_hull_merkez
                }
            else:
                return {'hull': None, 'points': None, 'center': None}
        
        except Exception as e:
            print(f"❌ [HATA] Yeni hull oluşturulurken hata: {e}")
            import traceback
            traceback.print_exc()
            return {'hull': None, 'points': None, 'center': None}
    
    def prepare_forbidden_points(self) -> list:
        """Ada çevre noktalarını yasaklı nokta listesine dönüştürür."""
        ada_cevre_noktalari = self.ada_cevre()
        yasakli_noktalar = []
        if ada_cevre_noktalari:
            for nokta in ada_cevre_noktalari:
                if len(nokta) >= 2:
                    yasakli_noktalar.append([float(nokta[0]), float(nokta[1])])
        return yasakli_noktalar
    
    def normalize_hull_center(self, hull_merkez) -> tuple:
        """Hull merkezini Sim formatına dönüştürür (z=0 yapar)."""
        hull_merkez_liste = list(hull_merkez)
        hull_merkez_liste[2] = 0
        return tuple(hull_merkez_liste)
    
    def find_leader_info(self, sessiz: bool = False) -> tuple:
        """Lider ROV ID ve GPS koordinatını bulur."""
        lider_rov_id = None
        lider_gps = None
        
        # Debug: Sistemler listesi kontrolü
        if not self.filo.sistemler or len(self.filo.sistemler) == 0:
            if not sessiz:
                print("⚠️ [FORMASYON] Sistemler listesi boş! ROV'lar filo sistemine eklenmemiş olabilir.")
            return None, None
        
        for rov_id in range(len(self.filo.sistemler)):
            # None kontrolü (çıkarılmış ROV'lar için sistem yoksa None olabilir)
            if rov_id < len(self.filo.sistemler) and self.filo.sistemler[rov_id] is None:
                continue
            
            # Rol bilgisini al
            rol = self.filo.get(rov_id, "rol")
            
            if rol == 1:
                lider_rov_id = rov_id
                gps = self.filo.get(rov_id, "gps")
                if gps:
                    lider_gps = (float(gps[0]), float(gps[1]), float(gps[2]))
                break
        
        # Not: find_leader_info() içindeki print mesajları kaldırıldı (sessiz mod için)
        return lider_rov_id, lider_gps
    
    def generate_search_points(self, lider_gps: tuple, hull_merkez: tuple) -> list:
        """Lider GPS'ten hull merkezine kadar ara noktalar oluşturur."""
        arama_noktalari = [("Lider GPS", lider_gps)]
        
        lider_x, lider_y, lider_z = lider_gps
        hull_x, hull_y, hull_z = hull_merkez
        
        dx = hull_x - lider_x
        dy = hull_y - lider_y
        mesafe_2d = math.sqrt(dx**2 + dy**2)
        
        if mesafe_2d > 10.0 and mesafe_2d > 0.001:
            yon_x = dx / mesafe_2d
            yon_y = dy / mesafe_2d
            
            dilim_boyutu = 10.0
            mevcut_mesafe = dilim_boyutu
            
            while mevcut_mesafe < mesafe_2d:
                ara_x = lider_x + (yon_x * mevcut_mesafe)
                ara_y = lider_y + (yon_y * mevcut_mesafe)
                ara_z = lider_z
                
                arama_noktalari.append((f"Ara Nokta ({mevcut_mesafe:.1f}m)", (ara_x, ara_y, ara_z)))
                mevcut_mesafe += dilim_boyutu
        
        arama_noktalari.append(("Hull Merkezi", hull_merkez))
        return arama_noktalari
    
    def get_formation_ids_to_try(self) -> list:
        """Denenecek formasyon ID'lerini pool'dan alır."""
        denenecek_formasyon_idleri = []
        pool_kopyasi = self.filo._formasyon_id_pool.copy()
        
        while len(denenecek_formasyon_idleri) < len(Formasyon.TIPLER) and len(pool_kopyasi) > 0:
            denenecek_formasyon_idleri.append(pool_kopyasi.pop(0))
        
        if len(denenecek_formasyon_idleri) < len(Formasyon.TIPLER):
            kalan_idler = [i for i in range(len(Formasyon.TIPLER)) if i not in denenecek_formasyon_idleri]
            random.shuffle(kalan_idler)
            denenecek_formasyon_idleri.extend(kalan_idler)
        
        return denenecek_formasyon_idleri
    
    def try_formation_fit(self, formasyon_id: int, aralik: float, is_3d: bool, 
                          merkez_koordinat: tuple, deneme_yaw: float, hull, 
                          lider_rov_id: int, nokta_adi: str, sessiz: bool = False, dinamik: bool = True) -> bool:
        """Formasyonun geçerli olup olmadığını kontrol eder ve uygular."""
        formasyon_obj = Formasyon(self.filo)
        pozisyonlar = formasyon_obj.pozisyonlar(
            formasyon_id,
            aralik=aralik,
            is_3d=is_3d,
            lider_koordinat=merkez_koordinat,
            yaw=deneme_yaw
        )
        
        if not pozisyonlar:
            return False
        
        # Pozisyonları Ursina formatına dönüştür
        ursina_positions = []
        for pozisyon in pozisyonlar:
            config_x, config_y, config_z = pozisyon
            ursina_positions.append((config_x, config_z, config_y))
        
        if not self.filo._formasyon_gecerli_mi(ursina_positions, hull, aralik):
            return False
        
        # Başarılı formasyon bulundu! Uygula
        
        # Aktif formasyonu kaydet (dinamik takip için)
        if dinamik:
            self.filo.aktif_formasyon = {
                'id': formasyon_id,
                'aralik': aralik,
                'is_3d': is_3d
            }
        else:
            self.filo.aktif_formasyon = None
        
        self.filo.set(lider_rov_id, 'yaw', float(deneme_yaw))
        
        if nokta_adi != "Lider GPS":
            # Lider ROV kontrolü: Eğer hareket halindeyse (hedefi varsa), ona dokunma
            mevcut_hedef = self.filo.hedef(rov_id=lider_rov_id)
            if mevcut_hedef is None:
                self.filo.git(lider_rov_id, merkez_koordinat[0], merkez_koordinat[1],
                            merkez_koordinat[2], ai=True, sessiz=sessiz)
            else:
                if not sessiz:
                    print(f"ℹ️ [FORMASYON] Lider ROV-{lider_rov_id} hareket halinde, mevcut hedefine devam ediyor.")
        
        # Takipçi ROV'ları formasyon pozisyonlarına gönder
        for rov_id, pozisyon in enumerate(pozisyonlar):
            if rov_id >= len(self.filo.sistemler):
                break
            
            # None kontrolü (çıkarılmış ROV'lar için sistem yoksa None olabilir)
            if rov_id < len(self.filo.sistemler) and self.filo.sistemler[rov_id] is None:
                continue
            
            if rov_id == lider_rov_id:
                continue
            
            sim_x, sim_y, sim_z = pozisyon
            
            if sim_z >= 0:
                sim_z = -10.0
            
            self.filo._formasyon_hedefleri[rov_id] = {
                'pozisyon': (sim_x, sim_y, sim_z),
                'hedef_yaw': deneme_yaw
            }
            
            self.filo.git(rov_id, sim_x, sim_y, sim_z, ai=True, sessiz=sessiz)
        
        return True


class TemelGNCHelper:
    """
    Helper class for TemelGNC physics and calculation logic.
    Contains mathematical operations extracted from TemelGNC class.
    Initialized with ROV entity and optional Filo reference.
    """
    
    # Sabitler
    HEDEF_TOLERANSI = 0.5
    YAVASLAMA_MESAFESI = 4.0
    
    def __init__(self, rov_entity, filo_ref=None, gnc_ref=None):
        """
        Initialize helper with ROV entity and optional Filo reference.
        
        Args:
            rov_entity: ROV entity (for velocity and rotation access)
            filo_ref: Optional Filo reference (for future use)
        """
        # ... diğer kodların ...
        
        # Kalman Filtreleri (X, Y ve Z ekseni için ayrı ayrı)
        # R değerini artırırsan (örn: 0.5) ROV daha "ağır" ama pürüzsüz hareket eder.
        # R değerini azaltırsan (örn: 0.01) titreme artar ama tepki hızlanır.
        self.kf_x = BasitKalmanFiltresi(R=0.4, Q=0.01)
        self.kf_y = BasitKalmanFiltresi(R=0.4, Q=0.01)
        self.kf_z = BasitKalmanFiltresi(R=0.4, Q=0.01)


        self.rov = rov_entity
        self.filo_ref = filo_ref
        self.gnc_ref = gnc_ref
        self.sayac=0
        # Koordinator'u lazy import için cache (circular import önleme)
        self._koordinator = None
        # APF smoothing state (low-pass filter for jitter reduction)
        self._last_sim_vektor = None
        self._last_guc = None
    
    def hiz_hesapla(self, mesafe: float) -> float:
        """
        Hedefe yaklaşırken hızı azaltır.
        
        Args:
            mesafe: Hedefe olan mesafe (metre)
        
        Returns:
            float: Hız çarpanı (0.2 - 1.0 arası)
        """
        if mesafe < self.YAVASLAMA_MESAFESI:
            return max(0.1, min(1.0, mesafe / self.YAVASLAMA_MESAFESI))
        return 1.0
    
    def yaw_ayarla(self, fark_vektoru: Vec3, ani: bool = False):
        """
        Yaw açısını hedefe doğru ayarlar.

        Args:
            fark_vektoru: Hedefe olan fark vektörü (Sim formatında)
            ani: Ani dönüş yapılsın mı (varsayılan: False - kademeli)
        """
        dx, dy = fark_vektoru.x, fark_vektoru.y
        if abs(dx) < 0.01 and abs(dy) < 0.01:
            return

        hedef_yaw = math.degrees(math.atan2(dx, dy)) % 360

        if hasattr(self.rov, 'rotation_y'):
            mevcut = self.rov.rotation_y
            if ani:
                self.rov.rotation_y = hedef_yaw
            else:
                delta = (hedef_yaw - mevcut + 180) % 360 - 180
                max_step = 3.0
                delta = max(-max_step, min(max_step, delta))
                self.rov.rotation_y = (mevcut + delta) % 360

    def waypoint_izdusum(self, rov_id: int, mevcut_waypoint, sonraki_waypoint):
        """
        Waypoint doğrultusuna göre ROV konumunun izdüşümünü hesaplar.

        Args:
            rov_id: ROV kimliği (ileride genişletme için tutulur)
            mevcut_waypoint: (x, y[, z])
            sonraki_waypoint: (x, y[, z])

        Returns:
            dict: {
                'axis': 'x' veya 'y',
                'proj_rov': (x, y),
                'proj_mevcut': (x, y),
                'proj_sonraki': (x, y)
            } veya None
        """
        if not mevcut_waypoint or not sonraki_waypoint:
            return None

        mx = float(mevcut_waypoint[0])
        my = float(mevcut_waypoint[1])
        nx = float(sonraki_waypoint[0])
        ny = float(sonraki_waypoint[1])

        dx = nx - mx
        dy = ny - my
        uzunluk = math.hypot(dx, dy)

        if uzunluk < 1e-9:
            eksen = 'x'
        else:
            cos_theta = dx / uzunluk
            sin_theta = dy / uzunluk
            eksen = 'x' if abs(cos_theta) >= abs(sin_theta) else 'y'

        koordinator = self._koordinator_al()
        if koordinator:
            sim_pos = koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z)
        else:
            sim_pos = (self.rov.x, self.rov.z)

        if eksen == 'x':
            proj_rov = (float(sim_pos[0]), 0.0)
            proj_mevcut = (mx, 0.0)
            proj_sonraki = (nx, 0.0)
        else:
            proj_rov = (0.0, float(sim_pos[1]))
            proj_mevcut = (0.0, my)
            proj_sonraki = (0.0, ny)

        return {
            'axis': eksen,
            'proj_rov': proj_rov,
            'proj_mevcut': proj_mevcut,
            'proj_sonraki': proj_sonraki,
        }

    def _waypoint_izdusum_gecildi_mi(self, rov_id: int, mevcut_waypoint, sonraki_waypoint) -> bool:
        """
        ROV'un izdüşümünün, mevcut waypoint'e kıyasla sonraki waypoint'e daha yakın olup olmadığını döndürür.
        """
        if not mevcut_waypoint or not sonraki_waypoint:
            return False

        izdusum = self.waypoint_izdusum(rov_id, mevcut_waypoint, sonraki_waypoint)
        if not izdusum:
            return False

        if izdusum['axis'] == 'x':
            rov_fark = abs(izdusum['proj_rov'][0] - izdusum['proj_sonraki'][0])
            mevcut_fark = abs(izdusum['proj_mevcut'][0] - izdusum['proj_sonraki'][0])
        else:
            rov_fark = abs(izdusum['proj_rov'][1] - izdusum['proj_sonraki'][1])
            mevcut_fark = abs(izdusum['proj_mevcut'][1] - izdusum['proj_sonraki'][1])

        return rov_fark < mevcut_fark

    def _kalman_vektor_filtrele(self, v):
            """
            Gelen ham vektörü (v) Kalman filtresinden geçirir ve temizlenmiş vektörü döner.
            """
            if v is None:
                return Vec3(0,0,0)
                
            # Her ekseni kendi filtresinden geçir
            yeni_x = self.kf_x.guncelle(v.x)
            yeni_y = self.kf_y.guncelle(v.y)
            yeni_z = self.kf_z.guncelle(v.z)
            
            return Vec3(yeni_x, yeni_y, yeni_z)

    def batma_orani_hesapla(self):
        """
        ROV gövdesinin suyun içindeki yüzdesini döner (0.0 - 1.0)
        """
        su_yuzeyi = 0.0  # Su seviyesi Y ekseninde 0
        
        # ROV'un Ursina dünyasındaki yüksekliği (scale.y)
        # Eğer modelin dikey ekseni Z ise bunu rov.scale.z yapmalısın
        rov_yukseklik = self.rov.scale.y *100
        
        rov_y = self.rov.y  # Mevcut dikey pozisyon
        
        # Sınır noktalarını belirle
        en_ust_nokta = rov_y + (rov_yukseklik / 2)
        en_alt_nokta = rov_y - (rov_yukseklik / 2)
        
        # 1. Tamamen suyun üzerinde
        if en_alt_nokta >= su_yuzeyi:
            return 0.0
            
        # 2. Tamamen suyun altında
        if en_ust_nokta <= su_yuzeyi:
            return 1.0
            
        # 3. Kısmen suyun içinde (Yüzeyde dalgalanıyor)
        # Suyun altında kalan kısmın uzunluğu:
        suyun_altindaki_kisim = su_yuzeyi - en_alt_nokta
        
        # Oran = (Altta kalan uzunluk) / (Toplam uzunluk)
        oran = suyun_altindaki_kisim / rov_yukseklik
        
        return max(0.0, min(1.0, oran)) # Güvenlik için 0-1 arasına sıkıştır

    def fizik_uygula(self, hedef_yon_ursina: Vec3, guc_orani: float, dt: float, uygula: bool = True):
            """
            SAF FİZİK MOTORU: 
            uygula=True ise: Newton fiziğini hesaplar ve yeni HIZ (velocity) vektörünü döndürür.
            uygula=False ise: Gelen hedef vektörünü olduğu gibi döndürür.
            """
            # Eğer fizik uygulanmayacaksa gelen vektörü direkt geri gönder
            if not uygula:
                #ivme=guc_orani*Vec3(hedef_yon_ursina.x, hedef_yon_ursina.y, hedef_yon_ursina.z)
                #yeni_hiz = self.rov.velocity + (ivme * dt)
                return hedef_yon_ursina*guc_orani*2

            # --- 1. KUVVET HESAPLAMALARI ---
            # A) Motor İtme (Thrust)
            f_thrust = hedef_yon_ursina * guc_orani * Hidrodinamik.MAX_ITME_KUVVETI

            # B) Hidrodinamik Direnç (Drag)
            mevcut_hiz = self.rov.velocity
            hiz_buyuklugu = mevcut_hiz.length()
            f_drag = Vec3(0, 0, 0)
            if hiz_buyuklugu > 0.001:
                drag_magnitude = 0.5 * Hidrodinamik.SU_YOGUNLUGU * Hidrodinamik.DRAG_KATSAYISI_CD * \
                                Hidrodinamik.ON_YUZEY_ALANI * (hiz_buyuklugu ** 2)
                f_drag = -mevcut_hiz.normalized() * drag_magnitude

            # C) Statik Kuvvetler (Yerçekimi ve Kaldırma)
            batma = self.batma_orani_hesapla()
            su_icindeki_hacim = Hidrodinamik.HACIM * batma
            f_yercekimi = Vec3(0, -Hidrodinamik.KUTLE * Hidrodinamik.YER_CEKIMI, 0)
            f_kaldirma  = Vec3(0, su_icindeki_hacim * Hidrodinamik.SU_YOGUNLUGU * Hidrodinamik.YER_CEKIMI, 0)

            # --- 2. ENTEGRASYON ---
            f_net = f_thrust + f_drag + f_yercekimi + f_kaldirma
            ivme = f_net / Hidrodinamik.KUTLE

            # Yeni hızı hesapla (v = v0 + a*dt)
            yeni_hiz = self.rov.velocity + (ivme * dt)
            
            return yeni_hiz

    def vektor_to_motor_sim(self, v_sim_dir: Vec3, guc_orani: float):
        """
        GNC YÜRÜTÜCÜ:
        Fizikten gelen vektörü ROV'un gövdesine (pozisyon ve rotasyon) uygular.
        """
        # 1. Koordinat Dönüşümü (Sim -> Ursina)
        # X: Sağ, Z: İleri, Y: Derinlik (Kullanıcının tercih ettiği mapping)
        hedef_yon_ursina = Vec3(v_sim_dir.x, v_sim_dir.z, v_sim_dir.y).normalized()
        guc_orani = max(0.0, min(1.0, guc_orani))
        dt = time.dt

        # 2. Fizik Hesaplamasını Tetikle (Vektörü al)
        hesaplanan_hiz = self.fizik_uygula(hedef_yon_ursina, guc_orani, dt, uygula=False)


        self.rov.velocity = self._kalman_vektor_filtrele(hesaplanan_hiz)
            
            # Burnunu hareket yönüne çevir (Yaw)
        if guc_orani > 0.01:
            self.yaw_ayarla(v_sim_dir, ani=False)

        # Pozisyonu Güncelle (Fiziksel yer değiştirme)

        
        GORSEL_HIZ_CARPANI = 10.0
        self.rov.position += self.rov.velocity * dt * GORSEL_HIZ_CARPANI

        # Hız Vektörünü Minimap'te Çiz
        if guc_orani > 0.02 and hasattr(self.rov, 'velocity'):
            #print(guc_orani,self.rov.velocity)
            #v_sim_dir=self._kalman_vektor_filtrele(v_sim_dir)
            if hasattr(self, 'filo_ref') and self.filo_ref.helper:
                    self.filo_ref.helper.vektor(
                        rov_id_ilk=self.rov.id, 
                        vektor=v_sim_dir, 
                        renk='m', 
                        ciz=True
                    )

        else:
            self.rov.hedef = None

        return hesaplanan_hiz
    def _guncelle_kontroller(self):
        """
        Temel kontrolleri yapar: gnc_ref, rov ve manuel_kontrol kontrolü.
        Basit bir kontrol fonksiyonu - modülerlik için ayrı tutulmuştur.
        
        Returns:
            bool: Kontroller geçildiyse True, aksi halde False.
        """
        if self.gnc_ref is None:
            return False
        if self.rov is None:
            return False
        if self.gnc_ref.manuel_kontrol:
            return False
        return True

    def _koordinator_al(self):
        """Koordinator'u lazy import ile alır (circular import önleme)."""
        if self._koordinator is None:
            from FiratROVNet.gnc import Koordinator
            self._koordinator = Koordinator
        return self._koordinator

    def _rov_pozisyon_sim(self):
        """
        ROV'un mevcut pozisyonunu Sim koordinat sistemine çevirir.
        Mevcut Koordinator.ursina_to_sim() fonksiyonunu kullanır.
        
        Returns:
            Vec3: ROV pozisyonu Sim koordinatlarında (X, Y, Z) veya None.
        """
        if self.rov is None:
            return None
        
        Koordinator = self._koordinator_al()
        # ROV pozisyonunu Ursina'dan Sim'e çevir
        current_sim_pos = Vec3(*Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z))
        return current_sim_pos

    def _engel_rov_kaçınma_vektörü(self, engel_listesi, rov_listesi):
        """
        Engel ve ROV repulsif vektörlerini birleştirir (hedef yokken kaçınma).
        Her vektör mesafe ile ağırlıklandırılır: yakın = daha güçlü itme.
        """
        import math
        toplam_x, toplam_y = 0.0, 0.0
        for item in (engel_listesi or []) + (rov_listesi or []):
            bv = item.get('birim_vektor')
            m = max(float(item.get('mesafe', 0.0)), 0.5)
            if bv and len(bv) >= 2:
                w = 1.0 / m  # yakın = büyük ağırlık
                toplam_x += float(bv[0]) * w
                toplam_y += float(bv[1]) * w
        n = math.sqrt(toplam_x * toplam_x + toplam_y * toplam_y)
        if n < 1e-9:
            return None
        return (toplam_x / n, toplam_y / n)
    def _guc_orani_hesapla(self, mesafe: float):
        # HEDEFE VARDI MI? (Ölü Bölge Kontrolü)
        


        if mesafe < 10: # 50 cm tolerans
            guc=mesafe/(np.sqrt(3)*400)
            guc=np.log(guc*10+1)/np.log(11) # Logaritmik azalma
            
            
        # YANAŞMA MODU (3 metre kala yavaşla)
        else:

            guc= 1.0 # En az %5 güç ver ki akıntıya karşı direnebilsin
            
        return guc

    def hedef_sifirla(self):
        self.rov.hedef = None
    def _formasyon_dinamik_guncelle(self, rov_id: int):
            """
            Eğer aktif bir formasyon varsa, takipçilerin hedeflerini 
            liderin o anki konumuna ve yönüne göre günceller.
            """
            aktif = getattr(self.filo_ref, 'aktif_formasyon', None)
            if not aktif:
                return

            # Lideri bul (role == 1)
            lider = next((r for r in self.filo_ref.ortam_ref.rovs if r and r.role == 1), None)
            if not lider:
                return

            # Lider kendisi ise hedef güncellemesi yapma (Lider özgürdür)
            if self.rov.role == 1:
                return

            f_obj = Formasyon(self.filo_ref)
            
            # Liderin o anki GPS konumunu baz alarak tüm filo için olması gereken yerleri hesapla
            lider_pos_sim = (lider.x, lider.z, lider.y) # Ursina -> Sim
            yeni_pozisyonlar = f_obj.pozisyonlar(
                aktif['id'], 
                aktif['aralik'], 
                is_3d=aktif['is_3d'], 
                lider_koordinat=lider_pos_sim
            )

            # 2. Bu ROV'un payına düşen hedefi güncelle
            if yeni_pozisyonlar and rov_id < len(yeni_pozisyonlar):
                hedef = yeni_pozisyonlar[rov_id]
                # GNC sistemine yeni hedefi sessizce ata (Sürekli print basmaması için sessiz)
                self.filo_ref.hedef((hedef[0], hedef[1], hedef[2]),rov_id=self.rov.id)
        
    def _guncelle_hareket_uygula(self, rov_id: int):
        """
        APF vektörlerini toplar ve ROV'u hedefe yönlendirir.
        Gereksiz döngü içi tip kontrolleri temizlendi, vektörel matematik kullanıldı.
        """
        """
        APF öncesi dinamik formasyon kontrolü eklenmiş ana hareket döngüsü.
        """
        engel_tespiti=False
        # --- KRİTİK EKLEME: DİNAMİK FORMASYON TAKİBİ ---
        self._formasyon_dinamik_guncelle(rov_id)
        # -----------------------------------------------

        hedef_koordinat = self.filo_ref.hedef(rov_id=rov_id)
        
        # 1. APF Verilerini Al
        sonuc = self.filo_ref.helper.apf(
            rov_id=rov_id, hedef=(hedef_koordinat is not None),
            engel=True, rov=True
        )
        if not sonuc: return

        toplam=Vec3(0,0,0)

        # Hedef bilgilerini güvenli bir şekilde al
        h_info = sonuc.get('hedef') or {'birim_vektor': [0, 0, 0], 'mesafe': 0.0}

        
        mag = float(h_info.get('mesafe', 0.0))
        mag1=0
        mag2=0
        guc = self._guc_orani_hesapla(mag)

        hv=Vec3(*h_info.get('birim_vektor', [0, 0, 0]))
        

        for engel_info in sonuc.get('engeller', []):
            bv = engel_info.get('birim_vektor')
            if bv and len(bv) >= 2:

                m = float(engel_info.get('mesafe', 0.0))
                mag1=1-m/GATLimitleri.ENGEL
                carpan=mag1*0.5
                if not engel_tespiti and self.filo_ref.get(rov_id, 'rol')==1 and mag1>0.3:
                    self.filo_ref.formasyon_sec(dinamik=True)
                    engel_tespiti=True

                #print(f"ROV {rov_id} - Mag1: {m}")
                bv=Vec3(bv[0], bv[1], 0)*carpan
                toplam += Vec3(bv[0], bv[1], 0)
                
        for rov_info in sonuc.get('rovs', []):
            bv = rov_info.get('birim_vektor')
            if bv and len(bv) >= 2:

                guc=0.5
                m= float(rov_info.get('mesafe', 0.0))
                mag2=1-m/GATLimitleri.CARPISMA
                carpan2=mag2*0.3
                #print(f"ROV {rov_id} - Mag2: {m}")
                toplam += Vec3(bv[0], bv[1], 0)*carpan2
              


        hv=hv*(1-mag1)*0.1 + hv*(1-mag2)*0.1
        toplam += hv
        t_bv=toplam.normalized() if toplam.length() >0.001 else Vec3(0,0,0)
        t_bv=t_bv/t_bv.length() if t_bv.length()>0.001 else Vec3(0,0,0)

        self.vektor_to_motor_sim(t_bv, guc)

    def guncelle(self, gat_kodu=None):
        """
        GNC güncelleme: Hedef varsa APF ile vektör hesaplar ve motor komutlarını uygular;
        hedef yoksa sönümler.
        """
        # Temel kontroller
        if not self._guncelle_kontroller():
            return

        # APF ile vektör hesapla, motor komutunu uygula ve yaw ayarla
        self._guncelle_hareket_uygula(rov_id=self.rov.id)

        

