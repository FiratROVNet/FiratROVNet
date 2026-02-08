"""
GNC Helper Module
Mathematical calculations, geometric operations, and complex logic for Filo and TemelGNC classes.
"""

from ast import Pass
import numpy as np
import math
import random

from panda3d.core import loadPrcFileData
from itertools import product
# Log seviyesini 'fatal' yaparak sadece hayati hataları gösterir, bilgi mesajlarını gizler
loadPrcFileData('', 'notify-level fatal')
loadPrcFileData('', 'notify-level-util fatal')
from ursina import Entity, color , Text, destroy,Vec3,time
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

class FiloHelper:
    """
    Helper class for Filo complex calculations and geometric operations.
    Contains heavy mathematical logic extracted from Filo class.
    Initialized with Filo instance to access self.sistemler and self.ortam_ref.
    """
    
    # Vektör renk kodu -> minimap'te kullanılır: k=kırmızı, y=yeşil, m=mavi, s=sarı, t=turuncu
    VEKTOR_RENK_KODLARI = ('k', 'y', 'm', 's', 't')
    
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
        self.kalici_hedefler = {}

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
            ROV için 3D çevresel tarama yapar ve bulguları Minimap/GNC sistemine raporlar.
            """
            import math
            from ursina import raycast, Vec3, color, Entity, destroy

            # 1. TEMEL KONTROLLER
            if menzil is None: menzil = GATLimitleri.ENGEL
            

            # ROV Nesnesine Eriş
            gnc_sistemi = self.filo.sistemler[rov_id] if 0 <= rov_id < len(self.filo.sistemler) else None
            rov = getattr(gnc_sistemi, 'rov', None)
            if not rov: return []

            # THREAD GÜVENLİĞİ: Konsol thread'inden çağrılıyorsa raycast yapma, cache dön.
            if not self.filo._is_main_thread():
                return getattr(rov, '_son_engeller', [])

            # 2. IGNORE LİSTESİ (Kritik: ROV'un tüm parçalarını görmezden gel)
            # Sadece ROV değil, onun tüm çocuklarını (children) da listeye ekliyoruz.
            ignore_list = [rov]
            if hasattr(rov, 'children'):
                ignore_list.extend(rov.children)
            
            ortam = self.filo.ortam_ref
            if ortam and hasattr(ortam, 'rovs'):
                for other in ortam.rovs:
                    if other and other != rov: 
                        ignore_list.append(other)
                        if hasattr(other, 'children'): ignore_list.extend(other.children)
            
            ignore_tuple = tuple(ignore_list)

            # 3. TARAMA YÖNLERİ
            # rov.forward, rov.right vb. dünya koordinat sistemindeki yön vektörleridir.
            tarama_yonleri = {
                'ileri': rov.forward,
                'geri': -rov.forward,
                'sag': rov.right,
                'sol': -rov.right,
                'yukari': Vec3(0, 1, 0), # Mutlak yukarı
                'asagi': Vec3(0, -1, 0)   # Mutlak aşağı
            }

            # Işın başlangıç noktası (ROV merkezinden biraz yukarıda, zeminle çakışmasın)
            origin = rov.world_position + Vec3(0, 0.2, 0)
            sonuclar = []

            # 4. RAYCAST DÖNGÜSÜ
            for ad, yon in tarama_yonleri.items():
                aktif_menzil = menzil

                hit = raycast(origin, yon, distance=aktif_menzil, ignore=ignore_tuple, debug=False)
                
                if hit.hit:
                    pt = hit.world_point
                    
                    # URSINA -> SİMÜLASYON KOORDİNAT DÖNÜŞÜMÜ
                    # Ursina X = Sim X
                    # Ursina Z = Sim Y (Yatay düzlem)
                    # Ursina Y = -Sim Z (Derinlik)
                    sim_koord = (pt.x, pt.z, pt.y) 
                    
                    # Radius Güvenli Hesaplama
                    radius = 5.0 # Varsayılan
                    if hasattr(self, '_engel_radius_al'):
                        radius = self._engel_radius_al(hit.entity, (pt.x, pt.z))
                    
                    res = {
                        'yon': ad,
                        'mesafe': hit.distance,
                        'koordinat': sim_koord,
                        'radius': radius,
                        'entity': hit.entity
                    }
                    sonuclar.append(res)

                    # 5. HARİTAYA (MİNİMAP) NOKTA EKLEME
                    # Sadece yatay engelleri (ileri, geri, sağ, sol) ekle. 
                    # Zemin ve tavan haritada kirlilik yaratır.
                    if ad in ['ileri', 'geri', 'sag', 'sol','asagi'] and ortam and hasattr(ortam, 'engel_bulutu'):
                        # Filtre: Noktalar arası mesafe kontrolü (Jitter önleme)
                        # Sadece son eklenen 20 noktaya bakmak performans için yeterlidir.
                        is_unique = True
                        for old_pt in ortam.engel_bulutu[-20:]:
                            # 1.5 metre mesafe filtresi
                            if (old_pt[0]-pt.x)**2 + (old_pt[1]-pt.z)**2 < 2.25:
                                is_unique = False
                                break
                        
                        if is_unique:
                            # DİKKAT: Minimap 'dunya_to_harita' Ursina X ve Z'sini bekler.
                            ortam.engel_bulutu.append((pt.x, pt.z))

                    # 6. DEBUG GÖRSELİ
                    if debug:
                        if not hasattr(self, '_debug_ents'): self._debug_ents = []
                        # Önceki debug noktalarını temizle
                        if len(self._debug_ents) > 30: 
                            destroy(self._debug_ents.pop(0))
                        
                        dot = Entity(model='sphere', color=color.red, scale=0.4, position=pt, unlit=True)
                        self._debug_ents.append(dot)

            # 7. CACHE GÜNCELLEME VE DÖNÜŞ
            rov._son_engeller = sonuclar
            return sonuclar

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
            Gelişmiş 3D Vektör Metodu (Ursina Koordinat Sistemi Uyumu).
            - Eksenler: x (yatay), z (yatay), y_depth (derinlik/düşey)
            - Format: (x, z, y_depth)
            """
            import math
            
            # 1. PARAMETRE AYARLARI
            self._vektor_renk = renk if renk in self.VEKTOR_RENK_KODLARI else 'm'
            self._vektor_reverse = bool(reverse)
            self._vektor_uzunluk_metre = float(uzunluk) if uzunluk is not None else 20.0
            
            # 2. BAŞLANGIÇ NOKTASI (POS1) - (x, z, y_depth)
            pos1 = None
            rid1 = rov_id_ilk if rov_id_ilk is not None else (ilk if isinstance(ilk, int) else None)
            
            if baslangic_noktasi is not None:
                # Gelen veri tuple/liste: (x, z, y_depth)
                pos1 = (float(baslangic_noktasi[0]), float(baslangic_noktasi[1]), float(baslangic_noktasi[2]))
            elif rid1 is not None:
                # filo.get artık ursina formatında (x, z, y_depth) döndürüyor
                pos1 = self.filo.get(rid1, "gps") 

            if pos1 is None: return None

            # 3. YÖN (BİRİM VEKTÖR) HESABI
            ux, uz, uy_depth = 0.0, 0.0, 0.0
            gercek_hedef_pos = None
            gercek_mesafe = 0.0

            # DURUM A: Doğrudan Vektör Verildiğinde (vx, vz, vy_depth)
            if vektor is not None:
                try:
                    vx, vz, vy_depth = float(vektor[0]), float(vektor[1]), float(vektor[2])
                    mag = math.sqrt(vx**2 + vz**2 + vy_depth**2)
                    if mag > 1e-9:
                        ux, uz, uy_depth = vx/mag, vz/mag, vy_depth/mag
                    gercek_mesafe = self._vektor_uzunluk_metre
                    # Sanal bitiş noktası
                    gercek_hedef_pos = (pos1[0] + ux * mag, pos1[1] + uz * mag, pos1[2] + uy_depth * mag)
                except: return None

            # DURUM B: İki Nokta Arası Vektör (Başlangıç -> Hedef)
            else:
                rid2 = rov_id_ikinci if rov_id_ikinci is not None else (ikinci if isinstance(ikinci, int) else None)
                if bitis_noktasi is not None:
                    gercek_hedef_pos = (float(bitis_noktasi[0]), float(bitis_noktasi[1]), float(bitis_noktasi[2]))
                elif rid2 is not None:
                    gercek_hedef_pos = self.filo.get(rid2, "gps") # (x, z, y_depth)

                if gercek_hedef_pos is not None:
                    dx = gercek_hedef_pos[0] - pos1[0]
                    dz = gercek_hedef_pos[1] - pos1[1]
                    dy_depth = gercek_hedef_pos[2] - pos1[2]
                    
                    gercek_mesafe = math.sqrt(dx**2 + dz**2 + dy_depth**2)
                    if gercek_mesafe > 1e-9:
                        ux, uz, uy_depth = dx/gercek_mesafe, dz/gercek_mesafe, dy_depth/gercek_mesafe
                else:
                    return None

            # Ters Çevirme (Kuvvet vektörleri için itme yönü)
            if self._vektor_reverse:
                ux, uz, uy_depth = -ux, -uz, -uy_depth

            # 4. GÖRSEL ÇİZİM BİTİŞ NOKTASI (POS2)
            # Ursina Minimap için çizilecek okun uç noktası
            pos2_cizim = (
                pos1[0] + ux * self._vektor_uzunluk_metre,
                pos1[1] + uz * self._vektor_uzunluk_metre,
                pos1[2] + uy_depth * self._vektor_uzunluk_metre
            )

            # 5. ÇIKTI VERİSİ (x, z, y_depth)
            ret = {
                'baslangic_3d': pos1,                 # (x, z, y_depth)
                'bitis_3d': gercek_hedef_pos,          # (x, z, y_depth)
                'birim_vektor_3d': (ux, uz, uy_depth), # (ux, uz, uy_depth)
                'uzaklik_metre': float(gercek_mesafe)
            }

            # 6. MİNİMAP ÇİZİM LİSTESİNE EKLE
            if ciz:
                # Debug modu aktifse listeyi temizle (tek ok göstermek için)
                if debug: self._apf_vektor_list = []
                
                self._apf_vektor_list.append({
                    'baslangic': pos1,                 # (x, z, y_depth)
                    'bitis': pos2_cizim,               # (x, z, y_depth)
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
    def hedef_gorsel_olustur(self, x, y, z, id=None, debug=True):
        """
        Hedef pozisyonunu hem 3D dünyada hem de Minimap üzerinde gösterir.
        x, y, z: Simülasyon koordinatları (Koordinator.sim_to_ursina tarafından çevrilmiş).
        id: Noktanın kimliği (debug=False ise zorunludur).
        debug=True: Geçici kırmızı X işareti (Her yeni hedefte eskiyi siler).
        debug=False: Kalıcı, ID numaralı mavi çember (Ekranda kalır).
        """
        from ursina import Entity, color, destroy, Text
        
        # --- 1. KOORDİNAT HAZIRLIĞI ---
        # Ursina'da dikey eksen Y'dir. Simülasyondan gelen (x, y, z) -> (x, z, y) dönüşümü:
        x_urs, y_urs, z_urs = x, z, y

        if not self.filo.ortam_ref:
            return

        # Kalıcı hedefler sözlüğünü kontrol et yoksa oluştur
        if not hasattr(self.filo, 'kalici_hedefler'):
            self.filo.kalici_hedefler = {}

        # --- 2. 3D DÜNYA GÖRSELLEŞTİRME ---

        # DURUM A: GEÇİCİ HEDEF (debug=True)
        if debug:
            if self.filo.hedef_gorsel:
                try: destroy(self.filo.hedef_gorsel)
                except: pass

            self.filo.hedef_gorsel = Entity()
            self.filo.hedef_gorsel.position = (x_urs, y_urs, z_urs)

            # HareketAyarlari'ndan sabitleri al (Erişilemiyorsa varsayılan değer kullan)
            x_boyutu = getattr(HareketAyarlari, 'HEDEF_X_BOYUTU', 15)
            kalinlik = getattr(HareketAyarlari, 'HEDEF_KALINLIK', 0.5)

            # Kırmızı X İşareti
            Entity(model='cube', rotation=(90, 0, 45), scale=(x_boyutu, kalinlik, kalinlik),
                   color=color.rgba(255, 0, 0, 0.5), parent=self.filo.hedef_gorsel, unlit=True)
            Entity(model='cube', rotation=(90, 0, -45), scale=(x_boyutu, kalinlik, kalinlik),
                   color=color.rgba(255, 0, 0, 0.5), parent=self.filo.hedef_gorsel, unlit=True)
            Entity(model='sphere', scale=(2, 2, 2), color=color.rgba(255, 0, 0, 0.5), 
                   parent=self.filo.hedef_gorsel, unlit=True)
            # Yeşil Çember
            Entity(model='circle', rotation=(90, 0, 0), scale=(x_boyutu * 1.5, x_boyutu * 1.5, 1),
                   color=color.rgb(0, 255, 120), parent=self.filo.hedef_gorsel, unlit=True, wireframe=True)

        # DURUM B: KALICI HEDEF (debug=False)
        else:
            if id is None:
                print("⚠️ Hata: debug=False iken bir id belirtmelisiniz!")
                return

            # Eğer bu ID ile bir hedef zaten varsa önce onu temizle (üst üste binmemesi için)
            self.hedef_sil(id)

            # Yeni kalıcı hedef objesi
            yeni_hedef = Entity(position=(x_urs, y_urs, z_urs))
            
            # Yer çemberi (Cyan)
            Entity(model='circle', parent=yeni_hedef, rotation=(90, 0, 0), scale=5,
                   color=color.cyan, wireframe=True, unlit=True)

            # Billboard ID Yazısı (Kameraya her zaman bakar)
            Text(text=str(id), parent=yeni_hedef, y=1.5, scale=25, 
                 color=color.yellow, origin=(0, 0), billboard=True)

            # Listeye kaydet
            self.kalici_hedefler[id] = yeni_hedef

        # --- 3. MİNİMAP GÜNCELLEMESİ ---
        # Minimap sınıfına bu noktayı gönderiyoruz.
        # Not: Minimap dunya_to_harita fonksiyonunda X ve Z kullanır.
        if hasattr(self.filo.ortam_ref, 'minimap') and self.filo.ortam_ref.minimap:
            self.filo.ortam_ref.minimap.hedef_isaretle(x_urs, z_urs, id=id, debug=debug)

    def hedef_sil(self, id):
        """Spesifik bir ID'ye sahip hedefi hem 3D dünyadan hem de Minimap'ten temizler."""
        
        # 3D'den sil
        if hasattr(self, 'kalici_hedefler') and id in self.kalici_hedefler:
            destroy(self.kalici_hedefler[id])
            del self.kalici_hedefler[id]
            
        # Minimap'ten sil
        if hasattr(self.filo.ortam_ref, 'minimap') and self.filo.ortam_ref.minimap:
            self.filo.ortam_ref.minimap.hedef_sil(id)

    def debug_hedefleri_temizle(self):
        """Ekrandaki tüm kalıcı (ID'li) hedefleri ve geçici hedefleri temizler."""
        # Sözlükteki tüm kalıcı ID'leri tek tek sil
        if hasattr(self, 'kalici_hedefler'):
            ids = list(self.kalici_hedefler.keys())
            for hid in ids:
                self.hedef_sil(hid)
        
        # Geçici hedefi (X işareti) sil
        from ursina import destroy
        if self.filo.hedef_gorsel:
            destroy(self.filo.hedef_gorsel)
            self.filo.hedef_gorsel = None
            
        # Minimap'i tamamen süpür
        if hasattr(self.filo.ortam_ref, 'minimap') and self.filo.ortam_ref.minimap:
            self.filo.ortam_ref.minimap.hedefleri_temizle()



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

    def git_path(self, rov_id, hedef, ai=True, isaret=True):
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
            A* ile yol planlar ve ROV'u mevcut derinliğini koruyarak o yola sokar.
            """
            if not hasattr(self.filo, '_git_isaret'): self.filo._git_isaret = {}
            self.filo._git_isaret[rov_id] = bool(isaret)
            
            if not (0 <= rov_id < len(self.filo.sistemler)):
                print(f"❌ [FİLO] Geçersiz ROV ID: {rov_id}")
                return

            # 1. Mevcut Pozisyonu Al (Ursina Sistemi: x, y_depth, z)
            # get("gps") bize doğrudan güncel koordinatları verir
            pos = self.filo.get(rov_id, "gps")
            current_x, current_y, current_z = pos[0], pos[1], pos[2]
            
            # 2. A* için 2D Başlangıç ve Hedef (x, z)
            start_2d = (current_x, current_y)
            
            if isinstance(hedef, (tuple, list)) and len(hedef) >= 2:
                # Hedefin x ve z'sini al (y_depth göz ardı edilir)
                goal_2d = (float(hedef[0]), float(hedef[1]))
            else:
                print(f"❌ [FİLO] Hedef formatı hatalı: {hedef}")
                return
            
            # 3. A* Yol Planlama
            yol_noktalari = self._a_star_path_planla(start_2d, goal_2d)
            #print(yol_noktalari)
            
            if not yol_noktalari:
                print(f"⚠️ [FİLO] Yol bulunamadı, doğrudan gidiliyor.")
                self.filo.git(rov_id, goal_2d[0],  goal_2d[1],current_z, ai=ai)
                return

            # 4. Minimap Güncelleme
            ortam = self.filo.ortam_ref
            if ortam and ortam.minimap:
                ortam.minimap.update_path(yol_noktalari)
            
            # 5. Yolu Atama ve Başlatma
            # git() fonksiyonuna yol listesi ve derinlik parametresini gönderiyoruz
            self.filo.git(rov_id, yol_noktalari, z=current_z, ai=ai)

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

    def _a_star_path_planla(self, start_2d: tuple, goal_2d: tuple) -> list:
        """
        A* yol planlamas yapan ortak helper metodu.
        FiratROVNet/a_star.py'deki AStarPlanner sınıfını kullanır.
        Ortamdaki engelleri (adaları) otomatik olarak alır.
        
        Args:
            start_2d: Başlangıç noktası (x, z)
            goal_2d: Hedef noktası (x, z)
        
        Returns:
            list: Yol noktaları listesi veya boş liste
        """
        try:
            # FiratROVNet/a_star.py'den AStarPlanner import et
            from FiratROVNet.a_star import AStarPlanner
        except ImportError:
            try:
                # Fallback: Relative import
                from ..a_star import AStarPlanner
            except ImportError:
                print("❌ [PATH] AStarPlanner modülü yüklenemedi!")
                return []
        
        ortam = self.filo.ortam_ref
        if not ortam or not hasattr(ortam, 'island_positions'):
            return []
        
        # Ortamdaki adaları engel olarak al (X, Z, R)
        adalar = [(p[0], p[1], p[2]) for p in ortam.island_positions if p]
        
        # A* planner örneği oluştur ve path planning yap
        planner = AStarPlanner()
        yol_noktalari = planner.find_path(
            start=start_2d, 
            goal=goal_2d, 
            obstacles=adalar, 
            havuz_genisligi=ortam.havuz_genisligi
        )
        
        return yol_noktalari if isinstance(yol_noktalari, list) else []


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
<<<<<<< HEAD
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
=======
                return None

            # 3. Ana Hesaplama Fonksiyonunu Çağır
            return self._formasyon_sec_impl(margin, is_3d, offset, dinamik=dinamik)

    def _formasyon_sec_impl(self, margin=None, is_3d=False, offset=None, sessiz=True, dinamik=True):
            initial_margin = margin if margin is not None else HareketAyarlari.FORMASYON_OFFSET
            min_aralik = HareketAyarlari.FORMASYON_MIN_ARALIK
            offset = offset if offset is not None else HareketAyarlari.FORMASYON_OFFSET
            
            try:
                self.filo._formasyon_hedefleri.clear()
                lider_id, lider_gps = self.filo._find_leader_info(sessiz=sessiz)
                if lider_id is None: return None

                lider_mevcut_hedef = self.filo.hedef(rov_id=lider_id)
                lider_hareket_halinde = lider_mevcut_hedef is not None

                # 1. Hull ve Arama Hattı Hazırlığı
                hull_data = self.yeni_hull(yasakli_noktalar=self.filo.ada_cevre(), offset=offset)
                hull_obj = hull_data.get("hull")
                hull_merkez = hull_data.get("center")
                if not hull_obj: return None

                # ==========================================
                # 🖼️ HARİTA GÖRSELLEŞTİRMEYİ TETİKLE
                # ==========================================
                if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'minimap'):
                    m_ui = self.filo.ortam_ref.minimap
                    if m_ui:
                        # Minimap kapalıysa aç ve Cyan alanı çiz
                        if hasattr(m_ui, 'goster'): m_ui.goster(True)
                        if hasattr(m_ui, 'update_hull'): m_ui.update_hull(hull_obj)
                # ==========================================

                # 3D Referans ve Arama Vektörü
                ref_pos_3d = lider_gps if lider_gps else (hull_merkez[0], hull_merkez[1], 0.0)
                start_pos_2d, unit_dir_2d, total_dist = self.generate_search_points(ref_pos_3d, hull_merkez)

                # 2. Sabitler
                denenecek_ids = self.filo._get_formation_ids_to_try()
                yaw_secenekleri = [0, 90, 180, 270]
                formasyon_motoru = Formasyon(self.filo)
                best_overall = None
                
                # --- KONUM İÇİN BINARY SEARCH ---
                low_d, high_d = 0.0, total_dist
                for _ in range(7): 
                    mid_d = (low_d + high_d) / 2
                    curr_2d = start_pos_2d + unit_dir_2d * mid_d
                    merkez_3d = (float(curr_2d[0]), float(curr_2d[1]), float(ref_pos_3d[2]))
                    
                    found_at_this_pos = False
                    for deneme_yaw in yaw_secenekleri:
                        for f_id in denenecek_ids:
                            # --- ARALIK İÇİN BINARY SEARCH ---
                            low_a, high_a = min_aralik, initial_margin
                            current_best_a = -1
                            current_best_p = None
                            
                            while low_a <= high_a:
                                mid_a = (low_a + high_a) / 2
                                p = formasyon_motoru.pozisyonlar(f_id, mid_a, is_3d, merkez_3d, deneme_yaw)
                                
                                if p and self.filo.hull_manager.formasyon_gecerli_mi(p, hull_obj, mid_a):
                                    current_best_a = mid_a
                                    current_best_p = p
                                    low_a = mid_a + 1.0
                                else:
                                    high_a = mid_a - 1.0
                            
                            if current_best_a != -1:
                                best_overall = {
                                    'f_id': f_id, 'aralik': current_best_a, 'yaw': deneme_yaw,
                                    'merkez': merkez_3d, 'pozisyonlar': current_best_p
                                }
                                found_at_this_pos = True
                                break
                        if found_at_this_pos: break
                    
                    if found_at_this_pos:
                        high_d = mid_d - 2.0
                    else:
                        low_d = mid_d + 2.0

                # 3. Sonuç Uygulama
                if best_overall:
                    b = best_overall
                    self._apply_formation_results(
                        b['f_id'], b['aralik'], b['yaw'], b['merkez'], 
                        b['pozisyonlar'], lider_id, is_3d, dinamik, sessiz, lider_hareket_halinde
                    )
                    
                    if not sessiz:
                        durum = "Dinamik" if dinamik else "Sabit"
                        print(f"✅ [MİNİMAP] {durum} {b['f_id']} seçildi. Alan Cyan olarak işlendi.")

                    return {
                        'f_id': int(b['f_id']),
                        'aralik': round(float(b['aralik']), 1),
                        'merkez': (round(b['merkez'][0], 2), round(b['merkez'][1], 2)),
                        'yaw': float(b['yaw'])
                    }

                return None
                
            except Exception as e:
                print(f"❌ [FORMASYON HATASI]: {e}")
                return None
        
    def _apply_formation_results(self, f_id, aralik, yaw, merkez, pozisyonlar, lider_id, is_3d, dinamik, sessiz, lider_hareket_halinde):
            """Hesaplaması bitmiş formasyonu uygular. Lider hareket halindeyse ona dokunmaz."""
            
            # Formasyon bilgisini kaydet (Dinamik takip için şart)
            self.filo.aktif_formasyon = {
                'id': f_id, 
                'aralik': aralik, 
                'is_3d': is_3d,
                'yaw': yaw # Referans yaw
            } if dinamik else None
            
            for i, pos in enumerate(pozisyonlar):
                if i >= len(self.filo.sistemler) or self.filo.sistemler[i] is None: continue
                
                # LİDER KONTROLÜ: Eğer liderin hedefi varsa, ona 'git' komutu gönderme
                if i == lider_id and lider_hareket_halinde:
                    if not sessiz: print(f"ℹ️ [FORMASYON] Lider ROV-{i} görevine devam ediyor, takipçiler eklemleniyor.")
                    continue

                sim_x, sim_y, sim_z = pos
                final_z = -10.0 if sim_z >= 0 else sim_z
                
                # Takipçiler için hedef kaydı
                if i != lider_id:
                    self.filo._formasyon_hedefleri[i] = {'pozisyon': (sim_x, sim_y, final_z), 'hedef_yaw': yaw}
                
                # Komutu gönder
                self.filo.git(i, sim_x, sim_y, final_z, ai=True, sessiz=sessiz)
                
    def normalize_hull_center(self, hull_merkez) -> tuple:
        """Hull merkezini Sim formatına dönüştürür (z=0 yapar)."""
        hull_merkez_liste = list(hull_merkez)
        hull_merkez_liste[2] = 0
        return tuple(hull_merkez_liste)
>>>>>>> develop
    
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
    
    def yeni_hull(self, yasakli_noktalar=None, offset=50.0, buffer_radius=10.0, **kwargs):
            """
            Filoya ait hull oluşturur. 
            Debug logları eklenmiştir.
            """
            import math
            #print(f"🛠️ [DEBUG] yeni_hull çalıştı. Offset: {offset}")

            # 1️⃣ Base Points (Temel Noktalar) Hazırla
            base_points = []
            
            # Lider ROV'u bulmaya çalış
            lider_bilgisi = self.find_leader_info(sessiz=False) # Hata varsa gör
            lider_gps = lider_bilgisi[1] if lider_bilgisi else None

            if lider_gps is None:
                print("⚠️ [UYARI] Lider ROV bulunamadı! Merkez (0,0,0) kabul ediliyor.")
                lx, ly = 0.0, 0.0 # Fallback: Lider yoksa merkeze çiz
            else:
                lx, ly = lider_gps[0], lider_gps[1]
                #print(f"✅ [DEBUG] Lider bulundu: ({lx}, {ly})")

            # Daire oluştur (Güvenlik alanı)
            radius = max(15.0, float(offset))
            for i in range(16):
                angle = math.radians(i * 22.5)
                nx = lx + math.cos(angle) * radius
                ny = ly + math.sin(angle) * radius
                base_points.append([nx, ny])
            
            #print(f"✅ [DEBUG] {len(base_points)} adet temel nokta oluşturuldu.")

            # 2️⃣ HullManager'ı Başlat
            try:
                from FiratROVNet.hull import HullManager
                #print("✅ [DEBUG] HullManager başarıyla yüklendi.")
            except ImportError:
                try:
                    from .hull import HullManager
                except ImportError:
                    #print("❌ [KRİTİK] HullManager import edilemedi!")
                    return {'points': None, 'center': None}
                
<<<<<<< HEAD
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
                
                # Ortam üzerinde convex_hull_data sakla (minimap/diğer UI bunları kullanabilir)
                if self.filo.ortam_ref:
                    hull_data = {
                        'hull': custom_hull,
                        'points': kontur_noktalari_np,
                        'center': yeni_hull_merkez
                    }
                    try:
                        self.filo.ortam_ref.convex_hull_data = hull_data
                    except Exception:
                        pass
                
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
    
=======
            mgr = HullManager()

            # 3️⃣ Hesaplamayı Yap
            sonuc = mgr.yeni_hull(
                base_points=base_points,
                yasakli_noktalar=yasakli_noktalar,
                offset=0.0, # Daireyi zaten geniş çizdik
                buffer_radius=buffer_radius
            )

            if sonuc.get('points') is None:
                print("❌ [DEBUG] HullManager boş sonuç döndürdü!")

            return sonuc

>>>>>>> develop
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
    
    def generate_search_points(self, lider_gps: tuple, hull_merkez: tuple):
            """Arama hattı için vektörel verileri hazırlar."""
            # Girişleri sayısal garantiye al
            lider_pos_2d = np.array([float(lider_gps[0]), float(lider_gps[1])])
            merkez_pos_2d = np.array([float(hull_merkez[0]), float(hull_merkez[1])])
            
            vektor = merkez_pos_2d - lider_pos_2d
            toplam_mesafe = np.linalg.norm(vektor)
            
            if toplam_mesafe < 0.1:
                return lider_pos_2d, np.array([0.0, 0.0]), 0.0
                
            birim_yon = vektor / toplam_mesafe
            return lider_pos_2d, birim_yon, toplam_mesafe
    
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
<<<<<<< HEAD
                return hedef_yon_ursina*guc_orani*2
=======
                return hedef_yon_ursina*guc_orani
>>>>>>> develop

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
        guc_orani = max(0.0, min(1.0, guc_orani))
        dt = time.dt
<<<<<<< HEAD
=======
        vim_dir=v_sim_dir.normalized() if v_sim_dir.length() > 0.001 else Vec3(0,0,0)
>>>>>>> develop

        # 2. Fizik Hesaplamasını Tetikle (Vektörü al)
        hesaplanan_hiz = self.fizik_uygula(v_sim_dir, guc_orani, dt, uygula=False)


<<<<<<< HEAD
        self.rov.velocity = self._kalman_vektor_filtrele(v_sim_dir)
=======
        self.rov.velocity = self._kalman_vektor_filtrele(hesaplanan_hiz)
>>>>>>> develop
            
            # Burnunu hareket yönüne çevir (Yaw)
        if guc_orani > 0.01:
            self.yaw_ayarla(self.rov.velocity, ani=False)

        # Pozisyonu Güncelle (Fiziksel yer değiştirme)
        ursina_rov_velocity=Vec3(self.rov.velocity.x, self.rov.velocity.z, self.rov.velocity.y)

        
<<<<<<< HEAD
        GORSEL_HIZ_CARPANI = 20.0
=======
        GORSEL_HIZ_CARPANI = 30.0
>>>>>>> develop
        self.rov.position += ursina_rov_velocity * dt * GORSEL_HIZ_CARPANI
        # Hız Vektörünü Minimap'te Çiz
        if guc_orani > 0.02 and hasattr(self.rov, 'velocity'):
            #print(guc_orani,self.rov.velocity)
            #v_sim_dir=self._kalman_vektor_filtrele(v_sim_dir)
            if hasattr(self, 'filo_ref') and self.filo_ref.helper:
                    self.filo_ref.helper.vektor(
                        rov_id_ilk=self.rov.id, 
                        vektor=self.rov.velocity, 
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
        


        if mesafe < 5: # 50 cm tolerans
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
        
    def _guncelle_waypoint_takip(self, rov_id: int):
            """
            A* çıktısı olan (x, z) noktalarını takip eder.
            Ursina: x=Yatay, z=Yatay, y=Derinlik.
            """
            import math
            from ursina import Vec3

            # 1. Rota listesini al
            nokta_listesi = getattr(self.filo_ref, '_git_nokta_listesi', {}).get(rov_id)
            if not nokta_listesi:
                return None, False

            mevcut_indeks = getattr(self.filo_ref, '_git_mevcut_nokta_indeksi', {}).get(rov_id, 0)

            # 2. Rota bitti mi?
            if mevcut_indeks >= len(nokta_listesi):
                self.filo_ref._git_nokta_listesi.pop(rov_id, None)
                self.filo_ref._git_mevcut_nokta_indeksi.pop(rov_id, None)
                return None, True

            # 3. Mevcut Waypoint (A* sadece x ve z üretir)
            wp = nokta_listesi[mevcut_indeks]
            current_gps = self.filo_ref.get(rov_id, "gps") # [x, y_depth, z]

            # --- HATA ÇÖZÜMÜ: Sadece x ve z'yi (0. ve 1. indeks) alıyoruz ---
            target_x = float(wp[0])
            target_y = float(wp[1])
            # Derinlik (y) mevcut derinlikte sabit kalır
            target_z = current_gps[2]

            waypoint_hedef = Vec3(target_x,target_y, target_z)

            # 4. Mesafe Hesabı (Yatay düzlemde x ve z üzerinden)
            dist_x = waypoint_hedef.x - current_gps[0]
            dist_y = waypoint_hedef.y - current_gps[1]
            mesafe_yatay = math.sqrt(dist_x**2 + dist_y**2)

            # 5. Waypoint'e ulaşıldı mı?
            if mesafe_yatay < 3.5: # 3.5 metre tolerans
                #print(f"✅ ROV-{rov_id} waypoint {mevcut_indeks} noktasına ulaştı: ({target_x:.1f}, {target_y:.1f})")
                self.filo_ref._git_mevcut_nokta_indeksi[rov_id] = mevcut_indeks + 1
                
                # AI (APF) hedefini bir sonraki noktaya güncelle
                if mevcut_indeks + 1 < len(nokta_listesi):
                    next_wp = nokta_listesi[mevcut_indeks + 1]
<<<<<<< HEAD
                    self.filo_ref.hedef((next_wp[0], next_wp[1],target_z),rov_id=rov_id)
=======
                    self.filo_ref.hedef((next_wp[0], next_wp[1],target_z),rov_id=rov_id,ciz=False)
>>>>>>> develop
                
                return waypoint_hedef, False

            return waypoint_hedef, False

    def _guncelle_hareket_uygula(self, rov_id: int):
            """
            APF kullanarak ROV hareketini yönetir. 
            Waypoint takip mekanizması ile git_path() çağrılarını destekler.
            Engellerden kaçarken batma/çıkma sorununu önlemek için dikey kuvvetler filtrelenmiştir.
            """
            # 0. WAYPOINT TAKİP MEKANIZMASI (git_path ile uyumlu)
            waypoint_hedef, _ = self._guncelle_waypoint_takip(rov_id)
            if waypoint_hedef:
                self.rov.hedef = waypoint_hedef
            
            # 1. Hazırlık ve Formasyon Güncelleme
            self._formasyon_dinamik_guncelle(rov_id)
            hedef_koordinat = self.filo_ref.hedef(rov_id=rov_id)
            
            sonuc = self.filo_ref.helper.apf(
                rov_id=rov_id, 
                hedef=(hedef_koordinat is not None),
                engel=True, 
                rov=True
            )
            if not sonuc: return

            bileske_vektor = Vec3(0, 0, 0)
            max_engel_etkisi = 0.0
            max_rov_etkisi = 0.0

            # 2. HEDEF ÇEKİM KUVVETİ (Attractive Force)
            # Hedefin Y (derinlik) bileşeni korunur, çünkü hedefe gitmek için batması/çıkması gerekebilir.
            h_info = sonuc.get('hedef') or {}
            h_mesafe = float(h_info.get('mesafe', 0.0))
            h_birim = Vec3(*h_info.get('birim_vektor', [0, 0, 0]))
            guc = self._guc_orani_hesapla(h_mesafe)

            # 3. ENGEL KAÇINMA KUVVETİ (Repulsive Force - Obstacles)
            for e_info in sonuc.get('engeller', []):
                bv = Vec3(*e_info.get('birim_vektor', [0, 0, 0]))
                mesafe = float(e_info.get('mesafe', 0.0))
                
                etki = 1.0 - (mesafe / GATLimitleri.ENGEL)
                max_engel_etkisi = max(max_engel_etkisi, etki)
                
                if etki > 0.2 and self.filo_ref.get(rov_id, 'rol') == 1:
                    self.filo_ref.formasyon_sec(dinamik=True)

                # --- DÜZELTME: Sadece Yatay Kaçınma ---
                # Engelden kaçarken batmaması için kaçınma vektörünün Y (düşey) etkisini sıfırlıyoruz.
                bv_yatay = Vec3(bv.x, bv.y, bv.z) 
                bileske_vektor += bv_yatay * etki * 0.37 # Kaçınma ağırlığı biraz artırıldı

            # 4. ROV KAÇINMA KUVVETİ (Repulsive Force - Swarm)
            for r_info in sonuc.get('rovs', []):
                bv = Vec3(*r_info.get('birim_vektor', [0, 0, 0]))
                mesafe = float(r_info.get('mesafe', 0.0))
                
                etki = 1.0 - (mesafe / GATLimitleri.CARPISMA)
                max_rov_etkisi = max(max_rov_etkisi, etki)
                
                # --- DÜZELTME: Diğer ROV'lardan yatayda kaçın ---
                bv_yatay = Vec3(bv.x, bv.y,bv.z)
                bileske_vektor += bv_yatay * etki * 0.25

            # 5. HEDEF VE KAÇINMA DENGESİ (Blending)
            # Kaçınma sırasında hedef çekimi azaltılır ama Y (derinlik) bileşeni 
            # sadece hedefe odaklı kalır.
            hedef_agirligi = (1.0 - max_engel_etkisi) * 0.24 + (1.0 - max_rov_etkisi) * 0.24
            bileske_vektor += h_birim * hedef_agirligi

            # 6. MOTOR KOMUTU GÖNDER
            final_yön = bileske_vektor.normalized() if bileske_vektor.length() > 0.001 else Vec3(0,0,0)
            
            self.vektor_to_motor_sim(final_yön, guc)



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

        

