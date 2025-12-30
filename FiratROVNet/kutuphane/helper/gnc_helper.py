"""
GNC Helper Module
Mathematical calculations, geometric operations, and complex logic for Filo and TemelGNC classes.
"""

import numpy as np
import math
import random
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

# Import from parent package (FiratROVNet.config)
try:
    from ...config import Formasyon
except ImportError:
    # Fallback: try absolute import
    from FiratROVNet.config import Formasyon


class FiloHelper:
    """
    Helper class for Filo complex calculations and geometric operations.
    Contains heavy mathematical logic extracted from Filo class.
    """
    
    @staticmethod
    def ada_cevre(ortam_ref, offset: float = 15.0) -> list:
        """
        Simülasyondaki adaları tespit edip her ada için eşit çevrede 12 nokta döndürür.
        
        Args:
            ortam_ref: Ortam referansı (island_positions'a erişim için)
            offset: Ada yarıçapından uzaklık (metre, varsayılan: 15.0)
        
        Returns:
            list: [(x1, y1, z1), (x2, y2, z2), ...] - Ada çevresi noktaları (Simülasyon formatı)
        """
        if not ortam_ref:
            print("⚠️ [UYARI] Ortam referansı bulunamadı!")
            return []
        
        if not hasattr(ortam_ref, 'island_positions') or not ortam_ref.island_positions:
            print("⚠️ [UYARI] Simülasyonda ada bulunamadı!")
            return []
        
        tum_noktalar = []
        
        for island_data in ortam_ref.island_positions:
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
        
        print(f"✅ [ADA_CEVRE] {len(ortam_ref.island_positions)} ada için {len(tum_noktalar)} nokta hesaplandı (offset={offset}m)")
        return tum_noktalar
    
    @staticmethod
    def yeniden_ciz(noktalar: list, yasakli_noktalar: list, alpha: float = 2.0, 
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
    
    @staticmethod
    def yeni_hull(filo_ref, yasakli_noktalar: list, offset: float = 40.0, alpha: float = 2.0,
                  buffer_radius: float = 20.0, channel_width: float = 15.0) -> dict:
        """
        Mevcut hull noktalarını alır, yasaklı bölgeleri kesip çıkarır.
        
        Args:
            filo_ref: Filo referansı (hull_manager ve ortam_ref'e erişim için)
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
            guvenlik_hull_dict = filo_ref.hull_manager.hull(offset=offset)
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
                yeni_kontur_noktalari = FiloHelper.yeniden_ciz(
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
                
                # Haritaya gönder
                if filo_ref.ortam_ref and hasattr(filo_ref.ortam_ref, 'harita') and filo_ref.ortam_ref.harita:
                    hull_data = {
                        'hull': custom_hull,
                        'points': kontur_noktalari_np,
                        'center': yeni_hull_merkez
                    }
                    filo_ref.ortam_ref.harita.convex_hull_data = hull_data
                    filo_ref.ortam_ref.harita.goster(True, True)
                
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
    
    @staticmethod
    def prepare_forbidden_points(filo_ref) -> list:
        """Ada çevre noktalarını yasaklı nokta listesine dönüştürür."""
        ada_cevre_noktalari = FiloHelper.ada_cevre(filo_ref.ortam_ref)
        yasakli_noktalar = []
        if ada_cevre_noktalari:
            for nokta in ada_cevre_noktalari:
                if len(nokta) >= 2:
                    yasakli_noktalar.append([float(nokta[0]), float(nokta[1])])
        return yasakli_noktalar
    
    @staticmethod
    def normalize_hull_center(hull_merkez) -> tuple:
        """Hull merkezini Sim formatına dönüştürür (z=0 yapar)."""
        hull_merkez_liste = list(hull_merkez)
        hull_merkez_liste[2] = 0
        return tuple(hull_merkez_liste)
    
    @staticmethod
    def find_leader_info(filo_ref) -> tuple:
        """Lider ROV ID ve GPS koordinatını bulur."""
        lider_rov_id = None
        lider_gps = None
        for rov_id in range(len(filo_ref.sistemler)):
            if filo_ref.get(rov_id, "rol") == 1:
                lider_rov_id = rov_id
                gps = filo_ref.get(rov_id, "gps")
                if gps:
                    lider_gps = (float(gps[0]), float(gps[1]), float(gps[2]))
                break
        return lider_rov_id, lider_gps
    
    @staticmethod
    def generate_search_points(lider_gps: tuple, hull_merkez: tuple) -> list:
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
    
    @staticmethod
    def get_formation_ids_to_try(formasyon_id_pool: list) -> list:
        """Denenecek formasyon ID'lerini pool'dan alır."""
        denenecek_formasyon_idleri = []
        pool_kopyasi = formasyon_id_pool.copy()
        
        while len(denenecek_formasyon_idleri) < len(Formasyon.TIPLER) and len(pool_kopyasi) > 0:
            denenecek_formasyon_idleri.append(pool_kopyasi.pop(0))
        
        if len(denenecek_formasyon_idleri) < len(Formasyon.TIPLER):
            kalan_idler = [i for i in range(len(Formasyon.TIPLER)) if i not in denenecek_formasyon_idleri]
            random.shuffle(kalan_idler)
            denenecek_formasyon_idleri.extend(kalan_idler)
        
        return denenecek_formasyon_idleri
    
    @staticmethod
    def try_formation_fit(filo_ref, formasyon_id: int, aralik: float, is_3d: bool,
                          merkez_koordinat: tuple, deneme_yaw: float, hull,
                          lider_rov_id: int, nokta_adi: str, formasyon_hedefleri: dict) -> bool:
        """Formasyonun geçerli olup olmadığını kontrol eder ve uygular."""
        formasyon_obj = Formasyon(filo_ref)
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
        
        if not filo_ref._formasyon_gecerli_mi(ursina_positions, hull, aralik):
            return False
        
        # Başarılı formasyon bulundu! Uygula
        filo_ref.set(lider_rov_id, 'yaw', float(deneme_yaw))
        
        if nokta_adi != "Lider GPS":
            filo_ref.git(lider_rov_id, merkez_koordinat[0], merkez_koordinat[1],
                        merkez_koordinat[2], ai=True)
        
        # Takipçi ROV'ları formasyon pozisyonlarına gönder
        for rov_id, pozisyon in enumerate(pozisyonlar):
            if rov_id >= len(filo_ref.sistemler):
                break
            
            if rov_id == lider_rov_id:
                continue
            
            sim_x, sim_y, sim_z = pozisyon
            
            if sim_z >= 0:
                sim_z = -10.0
            
            formasyon_hedefleri[rov_id] = {
                'pozisyon': (sim_x, sim_y, sim_z),
                'hedef_yaw': deneme_yaw
            }
            
            filo_ref.git(rov_id, sim_x, sim_y, sim_z, ai=True)
        
        return True


class TemelGNCHelper:
    """
    Helper class for TemelGNC physics and calculation logic.
    Contains mathematical operations extracted from TemelGNC class.
    """
    
    # Sabitler
    HEDEF_TOLERANSI = 0.5
    YAVASLAMA_MESAFESI = 2.0
    
    @staticmethod
    def hiz_hesapla(mesafe: float) -> float:
        """
        Hedefe yaklaşırken hızı azaltır.
        
        Args:
            mesafe: Hedefe olan mesafe (metre)
        
        Returns:
            float: Hız çarpanı (0.2 - 1.0 arası)
        """
        if mesafe < TemelGNCHelper.YAVASLAMA_MESAFESI:
            return max(0.2, min(1.0, mesafe / TemelGNCHelper.YAVASLAMA_MESAFESI))
        return 1.0
    
    @staticmethod
    def yaw_ayarla(rov_entity, fark_vektoru: Vec3, ani: bool = False, filo_ref=None):
        """
        Yaw açısını hedefe doğru ayarlar.
        
        Args:
            rov_entity: ROV entity (rotation_y'ye erişim için)
            fark_vektoru: Hedefe olan fark vektörü (Sim formatında)
            ani: Ani dönüş yapılsın mı (varsayılan: False - kademeli)
            filo_ref: Filo referansı (opsiyonel, şu an kullanılmıyor)
        """
        dx, dy = fark_vektoru.x, fark_vektoru.y
        if abs(dx) < 0.01 and abs(dy) < 0.01:
            return
        
        hedef_yaw = math.degrees(math.atan2(dx, dy)) % 360
        
        if hasattr(rov_entity, 'rotation_y'):
            mevcut = rov_entity.rotation_y
            delta = (hedef_yaw - mevcut + 180) % 360 - 180
            
            if ani:
                rov_entity.rotation_y = mevcut + delta
            else:
                max_step = 5.0
                step = max(-max_step, min(max_step, delta))
                rov_entity.rotation_y = mevcut + step
    
    @staticmethod
    def vektor_to_motor_sim(rov_entity, v_sim: Vec3, guc: float = 0.4):
        """
        Vektörü Simülasyon eksenlerinden Ursina motor komutlarına çevirir.
        Global koordinatlara göre direkt hareket eder (yaw açısından bağımsız).
        
        Args:
            rov_entity: ROV entity (velocity'ye erişim için)
            v_sim: Simülasyon formatında vektör (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik)
            guc: Güç çarpanı (varsayılan: 0.4)
        """
        if v_sim.length() < 0.01:
            return
        
        guc = max(0.0, min(2.0, guc))
        v = v_sim.normalized()
        
        # Import from parent package (FiratROVNet.config)
        try:
            from ...config import HareketAyarlari
        except ImportError:
            # Fallback: try absolute import
            from FiratROVNet.config import HareketAyarlari
        
        thrust = (guc * 100.0) * time.dt * HareketAyarlari.MOTOR_GUC_KATSAYISI
        
        if abs(v.x) > 0.01:
            rov_entity.velocity.x += v.x * thrust
        if abs(v.y) > 0.01:
            rov_entity.velocity.z += v.y * thrust
        if abs(v.z) > 0.01:
            rov_entity.velocity.y += v.z * thrust  # Sim Z -> Ursina Y
        
        limit = guc * 100.0
        if rov_entity.velocity.length() > limit:
            rov_entity.velocity = rov_entity.velocity.normalized() * limit

