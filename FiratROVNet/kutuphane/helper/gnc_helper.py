"""
GNC Helper Module
Mathematical calculations, geometric operations, and complex logic for Filo and TemelGNC classes.
"""

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
    from FiratROVNet.config import Formasyon, HareketAyarlari
except ImportError:
    # Fallback: try relative import if running from within package
    try:
        from ..FiratROVNet.config import Formasyon, HareketAyarlari
    except ImportError:
        # Last resort: try direct import
        import sys
        import os
        sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
        from FiratROVNet.config import Formasyon, HareketAyarlari


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
            })
        if lidar_0 > 0 and lidar_0 <= menzil:
            sonuclar.append({
                'koordinat': origin + ileri * lidar_0,
                'mesafe': lidar_0,
                'vektor': ileri,
                'yon': 'on_lidar',
            })
        if lidar_1 > 0 and lidar_1 <= menzil:
            sonuclar.append({
                'koordinat': origin + sol * lidar_1,
                'mesafe': lidar_1,
                'vektor': sol,
                'yon': 'sol_lidar',
            })
        if lidar_2 > 0 and lidar_2 <= menzil:
            sonuclar.append({
                'koordinat': origin + sag * lidar_2,
                'mesafe': lidar_2,
                'vektor': sag,
                'yon': 'sag_lidar',
            })
        return sonuclar

    def engel_bul(self, rov_id: int, menzil: float = 10.0, debug: bool = False, _sonuc_yazdir: bool = False) -> list:
        """
        ROV için çevresel tarama (sonar/lidar benzeri). İleri, sağ, sol, sağ-çapraz,
        sol-çapraz, yukarı, aşağı yönlerinde raycast atar; engellerin dünya koordinatlarını döndürür.
        debug=True ise çarpışma noktalarında kırmızı küre oluşturur (önceki debug noktaları temizlenir).
        Ursina/Panda3D thread-safe değildir: Ana thread dışından (örn. konsol) çağrılırsa raycast
        atılmaz; ROV'un sonar/lidar önbelleğinden engel listesi oluşturulur (filo.get(0,\"sonar\") ile aynı kaynak).
        """
        # --- ROV ve Filo kontrolleri (hem ana thread hem konsol path için) ---
        if not getattr(self.filo, 'sistemler', None) or rov_id < 0 or rov_id >= len(self.filo.sistemler):
            return []
        gnc_sistem = self.filo.sistemler[rov_id]
        rov = getattr(gnc_sistem, 'rov', None)
        if rov is None:
            return []

        # --- 1. Thread güvenliği: Ana thread değilse raycast atma; ROV önbelleğindeki sonar/lidar ile engel listesi döndür ---
        if not getattr(self.filo, '_is_main_thread', lambda: True)():
            _cache = self._engel_bul_cache_sonuc(rov, rov_id, menzil)
            if _cache and getattr(self.filo, 'ortam_ref', None) and getattr(self.filo.ortam_ref, 'harita', None):
                try:
                    harita = self.filo.ortam_ref.harita
                    noktalar_2d = [(s['koordinat'].x, s['koordinat'].z) for s in _cache if s.get('koordinat')]
                    if noktalar_2d and hasattr(harita, 'tespit_engelleri_guncelle'):
                        harita.tespit_engelleri_guncelle(noktalar_2d, debug=debug)
                except Exception:
                    pass
            if not getattr(self.filo, '_engel_bul_console_warned', False):
                self.filo._engel_bul_console_warned = True
                print("⚠️ [ENGEL_BUL] Konsol thread'den çağrıldı; sonuçlar ROV sonar/lidar önbelleğinden. Canlı raycast için ana thread'de veya simülasyon update döngüsünde çağırın.")
            return _cache

        # --- 3. Debug yönetimi: Yeni raycast öncesi eski debug Entity'leri yok et, listeyi temizle ---
        if not hasattr(self.filo, '_debug_noktalari'):
            self.filo._debug_noktalari = []
        try:
            from ursina import destroy
            for obj in list(self.filo._debug_noktalari):
                try:
                    destroy(obj)
                except Exception:
                    pass
            self.filo._debug_noktalari.clear()
        except ImportError:
            pass

        # --- Ursina raycast ve görsel için import ---
        try:
            from ursina import raycast, Vec3, Entity, color
        except ImportError:
            return []

        # --- 4. ROV konumu ve yaw (derece) — rotasyon matrisi ile vektör döndürme ---
        origin = Vec3(rov.world_position.x, rov.world_position.y, rov.world_position.z) + Vec3(0, 0.5, 0)
        yaw_deg = 0.0
        if hasattr(rov, 'rotation') and rov.rotation is not None:
            if hasattr(rov.rotation, 'y'):
                yaw_deg = float(rov.rotation.y)
            elif isinstance(rov.rotation, (tuple, list)) and len(rov.rotation) >= 2:
                yaw_deg = float(rov.rotation[1])
        yaw_rad = math.radians(yaw_deg)
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)

        # --- Lokal yönler (ROV'a göre): Ursina'da X=sağ, Y=yukarı, Z=ileri ---
        lokal_yonler = [
            ('ileri', Vec3(0, 0, 1)),
            ('sag', Vec3(1, 0, 0)),
            ('sol', Vec3(-1, 0, 0)),
            ('sag_capraz', Vec3(1, 0, 1).normalized()),
            ('sol_capraz', Vec3(-1, 0, 1).normalized()),
            ('yukari', Vec3(0, 1, 0)),
            ('asagi', Vec3(0, -1, 0)),
        ]

        # --- Ignore listesi (ROV ve safety_zone) ---
        ignore_list = [rov]
        if getattr(rov, 'safety_zone', None) is not None:
            ignore_list.append(rov.safety_zone)
        ignore_tuple = tuple(ignore_list)

        sonuclar = []
        for _ad, lok in lokal_yonler:
            # Lokal vektörü yaw ile global (yatay düzlem) dönüştür: world_x = lx*c + lz*s, world_z = -lx*s + lz*c, world_y = ly
            gx = lok.x * c + lok.z * s
            gz = -lok.x * s + lok.z * c
            gy = lok.y
            global_vektor = Vec3(gx, gy, gz)
            if global_vektor.length() < 0.001:
                continue
            global_vektor = global_vektor.normalized()

            try:
                hit_info = raycast(
                    origin,
                    global_vektor,
                    distance=menzil,
                    ignore=ignore_tuple,
                    debug=False,
                )
            except Exception:
                continue

            if hit_info and getattr(hit_info, 'hit', False):
                mesafe = getattr(hit_info, 'distance', 0.0) or 0.0
                # Engel koordinatı: raycast world_point varsa onu kullan, yoksa vektörel hesaplama
                if getattr(hit_info, 'world_point', None) is not None:
                    koordinat = Vec3(
                        hit_info.world_point.x,
                        hit_info.world_point.y,
                        hit_info.world_point.z,
                    )
                else:
                    koordinat = origin + global_vektor * mesafe
                sonuclar.append({'koordinat': koordinat, 'mesafe': mesafe, 'vektor': global_vektor, 'yon': _ad})

                # --- Debug: Çarpışma noktasında kırmızı küre ---
                if debug:
                    try:
                        nokta = Entity(
                            model='sphere',
                            color=color.red,
                            scale=0.2,
                            position=koordinat,
                            unlit=True,
                        )
                        self.filo._debug_noktalari.append(nokta)
                    except Exception:
                        pass

        # --- 2. Sonuç gösterimi (callback): invoke ile çağrıldığında sonuçlar konsola yazdırılır ---
        if _sonuc_yazdir and sonuclar:
            print(f"✅ [ENGEL_BUL] ROV-{rov_id} — {len(sonuclar)} engel tespit edildi (menzil={menzil}m):")
            for i, s in enumerate(sonuclar):
                k = s.get('koordinat')
                koord_str = f"({k.x:.2f}, {k.y:.2f}, {k.z:.2f})" if k else "—"
                print(f"   {i+1}. yön={s.get('yon', '—')} mesafe={s.get('mesafe', 0):.2f}m koordinat={koord_str}")

        # --- Haritada tespit edilen engelleri kırmızı noktalar olarak güncelle (Ursina x,z -> harita 2D) ---
        if sonuclar and getattr(self.filo, 'ortam_ref', None) and getattr(self.filo.ortam_ref, 'harita', None):
            try:
                harita = self.filo.ortam_ref.harita
                noktalar_2d = []
                for s in sonuclar:
                    k = s.get('koordinat')
                    if k is not None and hasattr(k, 'x') and hasattr(k, 'z'):
                        noktalar_2d.append((k.x, k.z))
                if noktalar_2d and hasattr(harita, 'tespit_engelleri_guncelle'):
                    harita.tespit_engelleri_guncelle(noktalar_2d, debug=debug)
            except Exception as e:
                pass  # Harita güncellemesi başarısız olursa sessizce devam et

        return sonuclar

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
                if i < secilen_n and i < len(senaryo.filo.sistemler):
                    pos = senaryo.get(i, "gps")
                    rov_filo_gps.append(pos if pos is not None else [400.0, 400.0, 400.0])
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
            out = aktif_filo.helper._formasyon_sec_impl(margin=30, is_3d=False, offset=20.0, sessiz=True)
            
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
                    bat = senaryo.get(i, "batarya")
                    if bat is None:
                        bat = 1.0  # Varsayılan batarya
                    bat = bat * 100.0
                    gps = senaryo.get(i, "gps")
                    if gps is None:
                        gps = np.array([400.0, 400.0, 400.0])
                    rov_data.append([bat, gps[0], gps[1], gps[2]])
                    rov_list_for_calc.append({'id': i, 'batarya': bat, 'konum': gps})
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

    def hull(self, offset=40.0):
        """
        Güvenlik hull oluşturur (Thread-safe).
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
        ROV'a hedef koordinatı atar ve otomatik moda geçirir (Thread-safe).
        Tüm girişler Simülasyon formatındadır: (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik)
        
        Args:
            sessiz: Log mesajlarını kapatır (RL eğitimi için)
        """
        if isinstance(x, (list, tuple)) and len(x) > 0:
            if isinstance(x[0], (list, tuple)) and len(x[0]) >= 2:
                nokta_listesi = [[float(n[0]), float(n[1])] for n in x if len(n) >= 2]
                if len(nokta_listesi) == 0:
                    if not sessiz:
                        print(f"❌ [FİLO] Geçersiz nokta listesi: {x}")
                    return
                self.filo._git_nokta_listesi[rov_id] = nokta_listesi
                self.filo._git_mevcut_nokta_indeksi[rov_id] = 0
                ilk_nokta = nokta_listesi[0]
                self.filo._command_queue.put(('git', (rov_id, ilk_nokta[0], ilk_nokta[1], z, ai, sessiz), {}))
                return
            else:
                if len(x) >= 2:
                    x_val, y_val = float(x[0]), float(x[1])
                    z_val = float(x[2]) if len(x) >= 3 else z
                else:
                    if not sessiz:
                        print(f"❌ [FİLO] Geçersiz koordinat formatı: {x}")
                    return
        else:
            x_val, y_val = float(x), float(y) if y is not None else None
            z_val = z

        if y_val is None:
            if not sessiz:
                print("❌ [FİLO] Y koordinatı gerekli! (x liste değilse)")
            return

        if not self.filo._is_main_thread():
            try:
                from ursina import invoke
                invoke(self._git_impl, rov_id, x_val, y_val, z_val, ai, sessiz)
                return
            except (ImportError, AttributeError):
                self.filo._command_queue.put(('git', (rov_id, x_val, y_val, z_val, ai, sessiz), {}))
                return

        self._git_impl(rov_id, x_val, y_val, z_val, ai, sessiz)

    def git_path(self, rov_id, hedef, ai=True):
        """
        ROV'a bir yol atar ve otomatik moda geçirir (Thread-safe).
        """
        path = self.filo.a_star(rov_id, hedef)
        if not isinstance(path, list) or len(path) == 0:
            print(f"❌ [FİLO] Geçersiz yol listesi: {path}")
            return

        gidilecek_n = self.filo.gidilecek_noktalar(path)
        self.filo.git(rov_id, gidilecek_n, ai)

    def _git_impl(self, rov_id: int, x: float, y: float, z: float = None, ai: bool = True, sessiz: bool = False) -> None:
        """git() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır)."""
        from FiratROVNet.gnc import Koordinator

        if len(self.filo.sistemler) == 0:
            print("❌ [HATA] GNC sistemleri henüz kurulmamış!")
            print("   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return

        if not isinstance(rov_id, int) or rov_id < 0:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.filo.sistemler)} (0-{len(self.filo.sistemler)-1} arası)")
            return

        if rov_id >= len(self.filo.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.filo.sistemler)} (0-{len(self.filo.sistemler)-1} arası)")
            print("   💡 Çözüm: filo.ekle() ile daha fazla GNC sistemi ekleyin")
            return

        self.filo.sistemler[rov_id].manuel_kontrol = False
        self.filo.sistemler[rov_id].ai_aktif = ai

        current_sim_pos = Koordinator.ursina_to_sim(
            self.filo.sistemler[rov_id].rov.x,
            self.filo.sistemler[rov_id].rov.y,
            self.filo.sistemler[rov_id].rov.z
        )
        current_x, current_y, current_z = current_sim_pos

        if z is None:
            z = current_z

        dx = x - current_x
        dy = y - current_y
        mesafe = math.sqrt(dx**2 + dy**2)
        if mesafe > 0.1:
            yaw_rad = math.atan2(dx, dy)
            yaw_deg = math.degrees(yaw_rad)
            while yaw_deg >= 360:
                yaw_deg -= 360
            while yaw_deg < 0:
                yaw_deg += 360
            self.filo._git_hedef_yaw[rov_id] = yaw_deg

        try:
            self.filo.sistemler[rov_id].hedef_atama(x, y, z)
            if not sessiz:
                ai_durum = "AÇIK" if ai else "KAPALI (Kör Mod)"
                print(f"✅ [FİLO] ROV-{rov_id} Hedef: X:{x}, Y:{y}, Z:{z} (Sim Formatı) | AI: {ai_durum}")
        except Exception as e:
            if not sessiz:
                print(f"❌ [HATA] Hedef atama sırasında hata: {e}")
            import traceback
            traceback.print_exc()

    def harita(self, goster=True, convex=True, a_star=True):
        """Harita penceresini açar, kapatır veya görünürlük ayarlarını yapar."""
        if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'harita') and self.filo.ortam_ref.harita:
            self.filo.ortam_ref.harita.goster(goster, convex, a_star)

    def minimap(self, durum=True, convex=True, a_star=True):
        """
        Minimap'i açar, kapatır veya durumunu döndürür.
        Harita fonksiyonunun tüm işlevlerine sahiptir.
        """
        if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'minimap') and self.filo.ortam_ref.minimap:
            if not hasattr(self.filo.ortam_ref.minimap, 'filo_ref') or self.filo.ortam_ref.minimap.filo_ref != self.filo:
                self.filo.ortam_ref.minimap.filo_ref = self.filo

            if durum is None:
                self.filo.ortam_ref.minimap.visible = not self.filo.ortam_ref.minimap.visible
                status = "AÇIK" if self.filo.ortam_ref.minimap.visible else "KAPALI"
                print(f"🗺️ [MİNİMAP] Minimap şu an {status}")
            else:
                self.filo.ortam_ref.minimap.goster(durum, convex, a_star)
        else:
            print("❌ [MİNİMAP] Minimap sistemi bulunamadı!")

    def a_star(self, start=None, goal=None, safety_margin=15.0, **kwargs):
        """
        A* algoritması kullanarak başlangıçtan hedefe yol hesaplar.
        """
        if start is None:
            start = kwargs.get('start')
        if goal is None:
            goal = kwargs.get('goal')
        if safety_margin == 8.0:
            safety_margin = kwargs.get('safety_margin', 8.0)

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

    def gidilecek_noktalar(self, path=None, r=10, derece_threshold=15):
        """
        A* yolu üzerinden gidilecek noktaları filtreler.
        Mesafe ve eğim açısına göre gereksiz noktaları çıkarır.
        """
        if path is None:
            if not self.filo.ortam_ref or not hasattr(self.filo.ortam_ref, 'harita') or self.filo.ortam_ref.harita is None:
                print("❌ [FİLO] Harita sistemi bulunamadı!")
                return []

            if not hasattr(self.filo.ortam_ref.harita, 'a_star_yolu') or self.filo.ortam_ref.harita.a_star_yolu is None:
                print("⚠️ [FİLO] A* yolu henüz hesaplanmamış!")
                print("   Önce filo.a_star(start=(x1, y1), goal=(x2, y2)) çağırın.")
                return []

            path = self.filo.ortam_ref.harita.a_star_yolu

        if len(path) == 0:
            return []

        gidilecek_noktalar = []
        x_baslangic, y_baslangic = path[0]
        gidilecek_noktalar.append([x_baslangic, y_baslangic])

        aci_radyan = np.arctan2(y_baslangic, x_baslangic)
        ilk_derece = np.degrees(aci_radyan)

        for i in range(1, len(path)):
            x_son, y_son = path[i]
            mesafe = np.sqrt(
                (x_son - x_baslangic) ** 2 +
                (y_son - y_baslangic) ** 2
            )
            if mesafe >= r:
                aci_radyan = np.arctan2(
                    y_son - y_baslangic,
                    x_son - x_baslangic
                )
                son_derece = np.degrees(aci_radyan)
                fark = ilk_derece - son_derece
                if abs(fark) >= derece_threshold:
                    ilk_derece = son_derece
                    gidilecek_noktalar.append([x_son, y_son])
                    x_baslangic, y_baslangic = x_son, y_son

        if len(path) > 1:
            son_nokta = path[-1]
            if son_nokta not in gidilecek_noktalar:
                gidilecek_noktalar.append([son_nokta[0], son_nokta[1]])

        return gidilecek_noktalar

    def move(self, rov_id: int, yon: str, guc: float = 1.0) -> None:
        """
        ROV'a güç bazlı hareket komutu verir. Eşzamanlı hareket: sadece ilgili eksen güncellenir,
        diğer eksenler korunur (örn. ileri + sağ = çapraz hareket).
        State: rov.active_forces = {'surge', 'sway', 'heave', 'yaw'} — her frame ROV.update() bunlardan velocity hesaplar.
        """
        # --- Hata kontrolleri (mevcut kontroller korunur) ---
        if len(self.filo.sistemler) == 0:
            print("❌ [HATA] GNC sistemleri henüz kurulmamış!")
            print("   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return

        if not isinstance(rov_id, int) or rov_id < 0:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.filo.sistemler)} (0-{len(self.filo.sistemler)-1} arası)")
            return

        if rov_id >= len(self.filo.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.filo.sistemler)} (0-{len(self.filo.sistemler)-1} arası)")
            print("   💡 Çözüm: filo.ekle() ile daha fazla GNC sistemi ekleyin")
            return

        gecerli_yonler = ['ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur', 'yaw']
        if yon not in gecerli_yonler:
            print(f"❌ [HATA] Geçersiz hareket yönü: '{yon}'")
            print(f"   Geçerli yönler: {', '.join(gecerli_yonler)}")
            return

        if not isinstance(guc, (int, float)):
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
                print(f"🛑 [FİLO] ROV-{rov_id} manuel giriş sıfırlandı (otonom devam edebilir)")
                return

            # --- yaw: Sadece dönüş hızı güncelle (ROV.update() her frame uygular) ---
            if yon == 'yaw':
                rov.active_forces['yaw'] = guc
                if hasattr(rov, 'manuel_hareket'):
                    rov.manuel_hareket['yon'] = 'yaw'
                    rov.manuel_hareket['guc'] = guc
                guc_yuzdesi = int(abs(guc) * 100)
                yon_metni = "saat yönünün tersine" if guc > 0 else "saat yönünde"
                print(f"🔄 [FİLO] ROV-{rov_id} {yon_metni} %{guc_yuzdesi} güçle döndürülüyor (yaw)")
                return

            # --- Havuz / güvenlik sınırları (mevcut kontroller korunur) ---
            if yon == 'bat' and rov.role == 1:
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
                    print(f"⚠️ [FİLO] ROV-{rov_id} havuz sınırında (X), {yon} yönünde hareket engellendi")
                    return

                if sinirda_z and ((yon == 'ileri' and rov.z > 0) or (yon == 'geri' and rov.z < 0)):
                    print(f"⚠️ [FİLO] ROV-{rov_id} havuz sınırında (Z), {yon} yönünde hareket engellendi")
                    return

                if sinirda_y_ust and yon == 'cik':
                    print(f"⚠️ [FİLO] ROV-{rov_id} su yüzeyinde, yukarı hareket engellendi")
                    return

                if sinirda_y_alt and yon == 'bat':
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

            guc_yuzdesi = int(abs(guc) * 100)
            print(f"🔵 [FİLO] ROV-{rov_id} {yon} yönünde %{guc_yuzdesi} güçle hareket ediyor (eşzamanlı mod)")
        except AttributeError as e:
            print(f"❌ [HATA] ROV-{rov_id} için gerekli özellik bulunamadı: {e}")
            print(f"   💡 Debug: GNC sistemi tipi: {type(self.filo.sistemler[rov_id])}")
            import traceback
            traceback.print_exc()
        except Exception as e:
            print(f"❌ [HATA] Hareket komutu sırasında hata: {e}")
            print(f"   💡 Debug: ROV ID: {rov_id}, Yön: {yon}, Güç: {guc}")
            import traceback
            traceback.print_exc()


    def formasyon(self, formasyon_id="LINE", aralik=15, is_3d=False, lider_koordinat=None):
        """
        Filoyu belirtilen formasyona sokar.
        Formasyon.pozisyonlar() ile pozisyonları alır ve filo.git() ile uygular.
        """
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

        for i, pozisyon in enumerate(pozisyonlar):
            if i >= len(self.filo.sistemler):
                break
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

    def formasyon_sec(self, margin=30, is_3d=False, offset=20.0, harita=False, yaw_senkronizasyon_mesafesi=5.0, maksimum_yaw_donme_hizi=90.0):
        """
        Convex hull kullanarak en uygun formasyonu seçer (Thread-safe).
        """
        if harita:
            if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'harita') and self.filo.ortam_ref.harita:
                self.filo.ortam_ref.harita.goster(True, True)

        self.filo._formasyon_yaw_senkronizasyon_mesafesi = yaw_senkronizasyon_mesafesi
        self.filo._maksimum_yaw_donme_hizi = maksimum_yaw_donme_hizi

        if not self.filo._is_main_thread():
            try:
                from ursina import invoke
                result = [None]
                def wrapper():
                    result[0] = self._formasyon_sec_impl(margin, is_3d, offset)
                invoke(wrapper)
                if result[0] is None:
                    print("⚠️ [FORMASYON] Formasyon seçilemedi (thread-safe mod)")
                return result[0]
            except (ImportError, AttributeError):
                self.filo._command_queue.put(('formasyon_sec', (margin, is_3d, offset), {}))
                print("ℹ️ [FORMASYON] Formasyon seçimi komut kuyruğuna eklendi (thread-safe mod)")
                return None

        result = self._formasyon_sec_impl(margin, is_3d, offset)
        if result is None:
            print("⚠️ [FORMASYON] Formasyon seçilemedi")
        return result

    def _formasyon_sec_impl(self, margin: float = 30, is_3d: bool = False, offset: float = 20.0, sessiz: bool = False):
        """
        formasyon_sec() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır).
        
        Args:
            margin: Formasyon aralığı
            is_3d: 3D formasyon modu
            offset: Hull offset değeri
            sessiz: Log mesajlarını kapatır (RL eğitimi için)
        """
        try:
            self.filo._formasyon_hedefleri.clear()

            yasakli_noktalar = self.filo._prepare_forbidden_points()
            guvenlik_hull_dict = self.filo.yeni_hull(
                yasakli_noktalar=yasakli_noktalar,
                offset=offset,
                alpha=2.0,
                buffer_radius=10.0,
                channel_width=10.0
            )

            hull = guvenlik_hull_dict.get("hull")
            hull_merkez = guvenlik_hull_dict.get("center")
            if hull is None or hull_merkez is None:
                if not sessiz:
                    print("⚠️ [FORMASYON] Hull oluşturulamadı veya hull merkezi bulunamadı")
                return None

            hull_merkez = self.filo._normalize_hull_center(hull_merkez)
            lider_rov_id, lider_gps = self.filo._find_leader_info(sessiz=sessiz)
            if lider_rov_id is None:
                if not sessiz:
                    print("⚠️ [FORMASYON] Lider ROV bulunamadı")
                return None
            if lider_gps is None:
                lider_gps = hull_merkez
                if not sessiz:
                    print(f"ℹ️ [FORMASYON] Lider GPS bulunamadı, hull merkezi kullanılıyor: {hull_merkez}")

            min_aralik = margin * 0.2
            baslangic_aralik = margin * 0.6
            adim = 1.0
            yaw_acilari = [0, 90, 180, 270]

            arama_noktalari = self.filo._generate_search_points(lider_gps, hull_merkez)
            for nokta_adi, merkez_koordinat in arama_noktalari:
                for deneme_yaw in yaw_acilari:
                    denenecek_formasyon_idleri = self.filo._get_formation_ids_to_try()
                    for i in denenecek_formasyon_idleri:
                        aralik = baslangic_aralik
                        while aralik >= min_aralik:
                            if self.filo._try_formation_fit(i, aralik, is_3d, merkez_koordinat,
                                                            deneme_yaw, hull, lider_rov_id, nokta_adi, sessiz=sessiz):
                                if i in self.filo._formasyon_id_pool:
                                    self.filo._formasyon_id_pool.remove(i)
                                if not sessiz:
                                    print(f"✅ [FORMASYON] Formasyon seçildi: Tip={i}, Aralık={aralik:.2f}, Yaw={deneme_yaw:.1f}°, Konum={nokta_adi}")
                                return (i, aralik, deneme_yaw, merkez_koordinat)
                            aralik -= adim
            if not sessiz:
                print("⚠️ [FORMASYON] Uygun formasyon bulunamadı (tüm denemeler başarısız)")
            return None
        except Exception as e:
            import traceback
            traceback.print_exc()
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
                          lider_rov_id: int, nokta_adi: str, sessiz: bool = False) -> bool:
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
        self.filo.set(lider_rov_id, 'yaw', float(deneme_yaw))
        
        if nokta_adi != "Lider GPS":
            self.filo.git(lider_rov_id, merkez_koordinat[0], merkez_koordinat[1],
                        merkez_koordinat[2], ai=True, sessiz=sessiz)
        
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
    YAVASLAMA_MESAFESI = 2.0
    
    def __init__(self, rov_entity, filo_ref=None, gnc_ref=None):
        """
        Initialize helper with ROV entity and optional Filo reference.
        
        Args:
            rov_entity: ROV entity (for velocity and rotation access)
            filo_ref: Optional Filo reference (for future use)
        """
        self.rov = rov_entity
        self.filo_ref = filo_ref
        self.gnc_ref = gnc_ref
    
    def hiz_hesapla(self, mesafe: float) -> float:
        """
        Hedefe yaklaşırken hızı azaltır.
        
        Args:
            mesafe: Hedefe olan mesafe (metre)
        
        Returns:
            float: Hız çarpanı (0.2 - 1.0 arası)
        """
        if mesafe < self.YAVASLAMA_MESAFESI:
            return max(0.2, min(1.0, mesafe / self.YAVASLAMA_MESAFESI))
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
            delta = (hedef_yaw - mevcut + 180) % 360 - 180
            
            if ani:
                self.rov.rotation_y = mevcut + delta
            else:
                max_step = 5.0
                step = max(-max_step, min(max_step, delta))
                self.rov.rotation_y = mevcut + step
    
    def vektor_to_motor_sim(self, v_sim: Vec3, guc: float = 0.4):
        """
        Vektörü Simülasyon eksenlerinden Ursina motor komutlarına çevirir.
        Global koordinatlara göre direkt hareket eder (yaw açısından bağımsız).
        
        Args:
            v_sim: Simülasyon formatında vektör (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik)
            guc: Güç çarpanı (varsayılan: 0.4)
        """
        if v_sim.length() < 0.01:
            return
        
        guc = max(0.0, min(2.0, guc))
        v = v_sim.normalized()
        
        thrust = (guc * 100.0) * time.dt * HareketAyarlari.MOTOR_GUC_KATSAYISI
        
        if abs(v.x) > 0.01:
            self.rov.velocity.x += v.x * thrust
        if abs(v.y) > 0.01:
            self.rov.velocity.z += v.y * thrust
        if abs(v.z) > 0.01:
            self.rov.velocity.y += v.z * thrust  # Sim Z -> Ursina Y
        
        limit = guc * 100.0
        if self.rov.velocity.length() > limit:
            self.rov.velocity = self.rov.velocity.normalized() * limit

    def guncelle(self, gat_kodu=None):
        """
        GNC Güncelleme: Hedef varsa hedefe doğru kuvvet uygular (AI çıktısı).
        Hibrit mod: AI çıktısı velocity'e eklenir; move() sadece active_forces günceller,
        manuel_kontrol = True yapmaz, bu yüzden move() ile birlikte AI da çalışır.
        Final_Force = AI_Output + Manual_Input (vektörel toplama).
        manuel_kontrol sadece manuel_kontrol_all(True) ile True yapılır; o zaman AI kapalı olur.
        """
        if self.gnc_ref is None or self.rov is None:
            return

        # manuel_kontrol True ise AI katkısı yok (manuel_kontrol_all ile toplu kapatma)
        if self.gnc_ref.manuel_kontrol:
            return

        if self.gnc_ref.hedef is None:
            if self.rov.velocity.length() > 1:
                self.rov.velocity *= 0.4
            return

        from FiratROVNet.gnc import Koordinator

        current_sim_pos = Vec3(*Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z))
        fark = self.gnc_ref.hedef - current_sim_pos
        mesafe = fark.length()

        hedef_tol = getattr(self.gnc_ref, "HEDEF_TOLERANSI", self.HEDEF_TOLERANSI)
        if mesafe <= hedef_tol:
            self.gnc_ref._hedefe_varis_islemleri(fark)
            return

        hiz_carpani = self.hiz_hesapla(mesafe)
        hareket_vektoru = fark / mesafe if mesafe > 0.01 else Vec3(0, 0, 0)

        self.yaw_ayarla(hareket_vektoru, ani=False)

        guc = 0.4 * hiz_carpani
        self.vektor_to_motor_sim(hareket_vektoru, guc=guc)
