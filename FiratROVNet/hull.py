"""
Convex Hull Yönetimi Modülü

Bu modül, ROV filosu için güvenli alan (Convex Hull) hesaplamalarını yönetir.
ROV pozisyonları, adalar ve lidar engelleri dahil edilerek dinamik güvenli alan oluşturur.
"""

import numpy as np
import math
from shapely.geometry import Point
from shapely.ops import unary_union
# Convex Hull için scipy import
try:
    from scipy.spatial import ConvexHull
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("⚠️ [UYARI] scipy.spatial.ConvexHull bulunamadı. ConvexHull fonksiyonu çalışmayacak.")


class HullManager:
    """
    Convex Hull hesaplamalarını yöneten sınıf.
    Filo referansı üzerinden ROV pozisyonlarına ve ortam bilgilerine erişir.
    """
    
    def __init__(self, filo_ref):
        """
        Args:
            filo_ref: Filo sınıfı referansı (ROV pozisyonları ve ortam bilgileri için)
        """
        self.filo = filo_ref
    
    

    # HullManager sınıfının içindeki fonksiyonu güncelleyin:
    def is_point_inside_hull(self, point, hull):
        """
        Noktanın hull içinde olup olmadığını kontrol eder.
        Hem Scipy ConvexHull hem de Shapely Polygon (SahteHull) destekler.
        """
        if hull is None:
            return False

        # 1. YÖNTEM: Eğer bizim oluşturduğumuz SahteHull ise (Polygon içeriyorsa)
        if hasattr(hull, 'polygon') and hull.polygon is not None:
            # Point sadece X ve Y (2D) alır
            p = Point(point[0], point[1])
            # contains() metodu nokta sınırın içindeyse True döner
            return hull.polygon.contains(p)

        # 2. YÖNTEM: Standart Scipy Convex Hull ise (equations içeriyorsa)
        if hasattr(hull, 'equations'):
            # Noktayı uygun boyuta getir (equations genellikle 2D için 3 elemanlıdır: ax+by+c <= 0)
            # ConvexHull hesaplaması 2D yapıldıysa point de 2D olmalı
            point_check = np.array(point[:2]) # Sadece X ve Y al
            
            for eq in hull.equations:
                # Dot product: normal * point + offset <= 0 ise içeridedir
                # eq[:-1] normal vektörü, eq[-1] offset
                if np.dot(eq[:-1], point_check) + eq[-1] > 1e-6: # Biraz tolerans
                    return False
            return True
            
        return False
    
    def genisletilmis_rov_hull_olustur(self, offset=20.0):
        """
        ROV poligonunu dışarı doğru 'offset' kadar genişletir.
        ROV'ların mavi çizginin içinde kalmasını sağlar.
        
        Args:
            offset (float): Hull köşelerinden dışarı offset mesafesi (metre, varsayılan: 20.0)
        
        Returns:
            list: [(x, y, z), ...] - Genişletilmiş sanal engel noktaları
        """
        if not SCIPY_AVAILABLE:
            return []
        
        try:
            rovs_positions = self.filo._get_all_rovs_positions()
            if len(rovs_positions) < 3:
                return []
            
            # 1. Noktaları al (Simülasyon X, Y)
            points = np.array([[p[0], p[1]] for p in rovs_positions.values()])
            z_avg = np.mean([p[2] for p in rovs_positions.values()])
            
            # 2. Hull oluştur ve vertexleri sıralı al (CCW)
            hull = ConvexHull(points)
            vertices = points[hull.vertices]
            n = len(vertices)
            
            genisletilmis_noktalar = []
            
            for i in range(n):
                prev = vertices[(i - 1) % n]
                curr = vertices[i]
                nxt = vertices[(i + 1) % n]
                
                # Kenar vektörleri
                v1 = (curr - prev)
                v2 = (nxt - curr)
                
                v1_norm = np.linalg.norm(v1)
                v2_norm = np.linalg.norm(v2)
                
                if v1_norm < 1e-6 or v2_norm < 1e-6:
                    continue
                
                v1_u = v1 / v1_norm
                v2_u = v2 / v2_norm
                
                # DIŞ NORMALLER (CCW bir poligonda sağa dönüş dışarı bakar)
                # (x, y) -> (y, -x)
                n1 = np.array([v1_u[1], -v1_u[0]])
                n2 = np.array([v2_u[1], -v2_u[0]])
                
                # Açıortay (bisector) yönü
                bisector = (n1 + n2)
                b_norm = np.linalg.norm(bisector)
                
                if b_norm < 1e-6:
                    bisector_unit = n1
                else:
                    bisector_unit = bisector / b_norm
                
                # Köşeyi DIŞARI it (offset kadar)
                # Not: Tam dairesel genişleme için offset / cos(theta) gerekebilir 
                # ama güvenli alan için basit itme yeterlidir.
                p_offset = curr + bisector_unit * offset
                
                genisletilmis_noktalar.append((p_offset[0], p_offset[1], z_avg))
            
            return genisletilmis_noktalar
        except Exception as e:
            print(f"❌ [HATA] Genişletme hatası: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    def lidar_engel_noktalari(self):
        """
        Tüm ROV'lardan lidar ile tespit edilen gerçek engel koordinatlarını toplar.
        
        Returns:
            list: [(x, y, z), ...] - Tüm tespit edilen engellerin koordinatları
        """
        tum_engeller = []
        
        try:
            # Tüm ROV'lar için
            for rov_id in range(len(self.filo.sistemler)):
                engels = self.filo._compute_obstacle_positions(rov_id)
                if engels:
                    tum_engeller.extend(engels)
        
        except Exception as e:
            print(f"❌ [HATA] Lidar engel noktaları toplanırken hata: {e}")
            import traceback
            traceback.print_exc()
        
        return tum_engeller
    
    def ada_engel_noktalari(self, yakinlik_siniri=200.0):
        """
        Simülasyondaki adaları bulur ve sınır noktalarını 0 offset ile döndürür.
        Mavi çizginin adanın içinden geçmesini engellemek için adayı sınır olarak belirler.
        
        Args:
            yakinlik_siniri (float): Filoya maksimum mesafe (varsayılan: 200.0)
        
        Returns:
            list: [(x, y, z), ...] - Adaların sınır noktaları (Simülasyon formatı)
        """
        # Koordinator'ı lokal import et (circular import'u önlemek için)
        from .gnc import Koordinator
        
        noktalar = []
        if not self.filo.ortam_ref:
            return noktalar

        try:
            # ROV grubunun merkezini bul (tüm haritayı işlememek için)
            rov_pos_list = list(self.filo._get_all_rovs_positions().values())
            if not rov_pos_list:
                return []
            filo_merkez = np.mean([p[:2] for p in rov_pos_list], axis=0)

            # Ursina'daki nesne listesini kontrol et
            # Loglara göre 'rocks' listesi en muhtemel olanı
            adalar = getattr(self.filo.ortam_ref, 'rocks', [])
            if not adalar:
                adalar = getattr(self.filo.ortam_ref, 'engeller', [])

            for ada in adalar:
                if not ada:
                    continue
                
                # Nesne pozisyonunu al (Ursina -> Sim)
                u_pos = (ada.x, ada.y, ada.z)
                sim_pos = Koordinator.ursina_to_sim(*u_pos)
                
                # Sadece filoya yakın adaları işle (performans için)
                dist = np.linalg.norm(np.array(sim_pos[:2]) - filo_merkez)
                if dist > yakinlik_siniri:
                    continue

                # Adanın gerçek yarıçapını tespit et (Genelde scale'in yarısıdır)
                # Kahverengi adalar için scale_x ve scale_z kullanılır
                if hasattr(ada, 'scale_x'):
                    yari_cap = (ada.scale_x / 2.0)
                else:
                    yari_cap = 15.0  # Varsayılan

                # Adanın çevresinde 16 nokta oluştur (Daha pürüzsüz bir yaslanma için)
                # Offset = 0 (Tam sınırından geçsin)
                for i in range(16):
                    aci = math.radians(i * (360 / 16))
                    nx = sim_pos[0] + math.cos(aci) * yari_cap
                    ny = sim_pos[1] + math.sin(aci) * yari_cap
                    # Adanın üzerinden geçmemesi için z'yi sabit tutuyoruz
                    noktalar.append((nx, ny, sim_pos[2]))
                
        except Exception as e:
            print(f"⚠️ [ADA HATASI] Adalar işlenemedi: {e}")
            import traceback
            traceback.print_exc()
        
        return noktalar
    
    def ada_engel_noktalari_pro(self, yakinlik_siniri=100.0, offset=20.0):
        """
        Filoya yakın adaları tespit eder ve bu adaların noktalarını offset mesafesi
        kadar filoya yakınlaştırarak sanal bariyer noktaları oluşturur.
        Bu, adaların Hull (mavi alan) dışında kalmasını garanti eder.
        
        Args:
            yakinlik_siniri (float): Filoya maksimum mesafe (varsayılan: 100.0)
            offset (float): Ada noktalarını filoya doğru kaydırma mesafesi (varsayılan: 20.0)
        
        Returns:
            list: [(x, y, z), ...] - Sanal bariyer noktaları (Simülasyon formatı)
        """
        from .gnc import Koordinator
        
        sanal_bariyer_noktalari = []
        if not self.filo.ortam_ref:
            return sanal_bariyer_noktalari

        try:
            # 1. Filo merkezini bul (referans noktası)
            rov_pos_list = list(self.filo._get_all_rovs_positions().values())
            if not rov_pos_list:
                return []
            filo_merkezi = np.mean([p[:2] for p in rov_pos_list], axis=0)

            # 2. Adaları tara (rocks veya engeller)
            adalar = getattr(self.filo.ortam_ref, 'rocks', [])
            if not adalar:
                adalar = getattr(self.filo.ortam_ref, 'engeller', [])

            for ada in adalar:
                if not ada:
                    continue
                
                # Adanın pozisyonunu Sim formatına çevir
                if hasattr(ada, 'x') and hasattr(ada, 'y') and hasattr(ada, 'z'):
                    u_pos = (ada.x, ada.y, ada.z)
                elif hasattr(ada, 'position') and ada.position is not None:
                    u_pos = (ada.position.x, ada.position.y, ada.position.z)
                else:
                    continue
                
                sim_pos = Koordinator.ursina_to_sim(*u_pos)
                ada_merkez_2d = np.array([sim_pos[0], sim_pos[1]])

                # 3. Mesafe Kontrolü: 100 metre sınırı
                mesafe = np.linalg.norm(ada_merkez_2d - filo_merkezi)
                if mesafe > yakinlik_siniri:
                    continue

                # 4. Yön Vektörü: Adadan Filo Merkezine doğru
                yon_vektoru = filo_merkezi - ada_merkez_2d
                yon_norm = np.linalg.norm(yon_vektoru)
                if yon_norm < 1e-6:
                    # Ada filo merkezinde, her yöne eşit mesafede kaydır
                    birim_yon = np.array([1.0, 0.0])
                else:
                    birim_yon = yon_vektoru / yon_norm

                # 5. Adanın yarıçapını al
                if hasattr(ada, 'scale_x'):
                    yari_cap = ada.scale_x / 2.0
                else:
                    yari_cap = 15.0  # Varsayılan

                # 6. Adanın çevresindeki noktaları içeri (filoya doğru) kaydır
                # Adanın her noktasını filoya 'offset' kadar daha yakınmış gibi hesaplıyoruz
                for i in range(12):
                    aci = math.radians(i * 30)
                    # Orijinal ada yüzey noktası
                    nx = sim_pos[0] + math.cos(aci) * yari_cap
                    ny = sim_pos[1] + math.sin(aci) * yari_cap
                    nokta_2d = np.array([nx, ny])

                    # NOKTA KAYDIRMA: Bu noktayı filoya doğru 'offset' kadar itiyoruz
                    # Böylece Hull bu noktayı birleştirince gerçek ada dışarıda kalıyor.
                    kaydirilmis_nokta = nokta_2d + (birim_yon * offset)
                    
                    sanal_bariyer_noktalari.append((kaydirilmis_nokta[0], kaydirilmis_nokta[1], sim_pos[2]))

        except Exception as e:
            print(f"⚠️ [BARİYER HATASI] {e}")
            import traceback
            traceback.print_exc()
        
        return sanal_bariyer_noktalari
    
    def hull(self, offset=40.0):
        """
        1. ROV'ları 20m dışarı iten noktaları alır.
        2. Yakındaki adaları 20m içeri (filoya doğru) çeken sanal noktaları alır.
        3. Hepsini birleştirerek adayı DIŞARIDA bırakan güvenli alanı hesaplar.
        
        Args:
            offset (float): ROV hull genişletme mesafesi (varsayılan: 20.0)
        
        Returns:
            dict: {
                'hull': ConvexHull objesi (2D) veya None,
                'points': numpy array - Hull hesaplamasında kullanılan noktalar (2D),
                'center': (x, y, z) - Hull merkezi veya None
            }
        """
        return self.guvenlik_hull_olustur(offset=offset)
    
    def guvenlik_hull_olustur(self, offset=40.0):
        """
        1. ROV'ları 20m dışarı iten noktaları alır.
        2. Yakındaki adaları 20m içeri (filoya doğru) çeken sanal noktaları alır.
        3. Hepsini birleştirerek adayı DIŞARIDA bırakan güvenli alanı hesaplar.
        
        Args:
            offset (float): ROV hull genişletme mesafesi (varsayılan: 20.0)
        
        Returns:
            dict: {
                'hull': ConvexHull objesi (2D) veya None,
                'points': numpy array - Hull hesaplamasında kullanılan noktalar (2D),
                'center': (x, y, z) - Hull merkezi veya None
            }
        """
        if not SCIPY_AVAILABLE:
            return {'hull': None, 'points': None, 'center': None}
        
        try:
            # ROV'ların dış çeperi (+offset)
            sanal_rov_noktalari = self.genisletilmis_rov_hull_olustur(offset=offset)
            
            # Adaların sanal bariyerleri (Adadan filoya doğru itilmiş noktalar)
    
            # Sadece bu sanal noktaları birleştiriyoruz (Gerçek ROV veya gerçek ADA merkezlerini değil!)
            tum_noktalar = sanal_rov_noktalari
            
            if len(tum_noktalar) < 3:
                hull_data = {
                    'hull': None,
                    'points': None,
                    'center': None
                }
                
                # Hull bilgisini haritaya aktar (eğer harita varsa)
                if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'harita') and self.filo.ortam_ref.harita:
                    self.filo.ortam_ref.harita.convex_hull_data = hull_data
                
                return hull_data

            # 2D Projeksiyon
            points_2d = np.array([[p[0], p[1]] for p in tum_noktalar])
            points_2d = np.unique(np.round(points_2d, 3), axis=0)

            # Hull Hesaplama
            hull_2d = ConvexHull(points_2d, qhull_options='QJ')
            
            # Convex hull çizgisi üzerinde her 5 metrede bir nokta ekle
            points_2d_genisletilmis = self._hull_kenarlarina_nokta_ekle(
                hull_2d, points_2d, nokta_araligi=5.0
            )
            
            center_2d = np.mean(points_2d_genisletilmis, axis=0)
            z_avg = np.mean([p[2] for p in tum_noktalar])

            hull_data = {
                'hull': hull_2d, 
                'points': points_2d_genisletilmis,  # Genişletilmiş noktalar
                'center': (center_2d[0], center_2d[1], z_avg)
            }

            if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'harita'):
                self.filo.ortam_ref.harita.convex_hull_data = hull_data

            return hull_data
        except Exception as e:
            print(f"❌ [HATA] Pro-Hull oluşturulamadı: {e}")
            import traceback
            traceback.print_exc()
            return {'hull': None, 'points': None, 'center': None}
    
    def convex_hull_3d(self, points, test_point, margin=0.0):
        """
        3D Convex Hull oluşturur ve test noktasının hull içinde olup olmadığını kontrol eder.
        
        Args:
            points: Nx3 numpy array veya liste - Convex hull oluşturmak için kullanılacak noktalar
            test_point: (x, y, z) tuple veya liste - Test edilecek nokta
            margin: float - Minimum mesafe (hull yüzeyinden ne kadar uzakta olmalı) - Şu an kullanılmıyor
        
        Returns:
            dict: {
                'inside': bool - Test noktası hull içinde mi?
                'center': (x, y, z) - Convex hull'un merkezi (3D koordinat)
                'hull': ConvexHull objesi (None if scipy not available)
            }
        
        Örnekler:
            points = np.array([[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0], [0, 0, 2], [2, 2, 2]])
            test_point = [1, 1, 1]
            result = hull_manager.convex_hull_3d(points, test_point, margin=0.2)
            print(f"İçinde mi: {result['inside']}, Merkez: {result['center']}")
        """
        if not SCIPY_AVAILABLE:
            print("❌ [HATA] scipy.spatial.ConvexHull bulunamadı!")
            return {
                'inside': False,
                'center': None,
                'hull': None
            }
        
        try:
            # Points'i numpy array'e çevir
            points = np.asarray(points)
            if points.ndim != 2 or points.shape[1] != 3:
                print(f"❌ [HATA] Points Nx3 formatında olmalı! Alınan shape: {points.shape}")
                return {
                    'inside': False,
                    'center': None,
                    'hull': None
                }
            
            # Test point'i numpy array'e çevir
            test_point = np.asarray(test_point)
            if test_point.shape != (3,):
                print(f"❌ [HATA] Test point (x, y, z) formatında olmalı! Alınan shape: {test_point.shape}")
                return {
                    'inside': False,
                    'center': None,
                    'hull': None
                }
            
            # En az 4 nokta gerekli (3D convex hull için)
            if len(points) < 4:
                print(f"⚠️ [UYARI] 3D Convex Hull için en az 4 nokta gerekli! Alınan: {len(points)}")
                # Yeterli nokta yoksa, merkezi hesapla ve inside=False döndür
                center = np.mean(points, axis=0)
                return {
                    'inside': False,
                    'center': tuple(center),
                    'hull': None
                }
            
            # Convex Hull oluştur
            hull = ConvexHull(points)
            
            # Hull merkezini hesapla (tüm noktaların ortalaması)
            center = np.mean(points, axis=0)
            
            # Test noktasının hull içinde olup olmadığını kontrol et
            inside = self.is_point_inside_hull(test_point, hull)
            
            return {
                'inside': inside,
                'center': tuple(center),
                'hull': hull
            }
            
        except Exception as e:
            print(f"❌ [HATA] ConvexHull hesaplama sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return {
                'inside': False,
                'center': None,
                'hull': None
            }
    
    def formasyon_gecerli_mi(self, test_points, hull, formasyon_aralik):
        """
        Formasyon pozisyonlarının geçerli olup olmadığını kontrol eder.
        
        Args:
            test_points: list - [(x, z, y), ...] Ursina formatında formasyon pozisyonları
            hull: ConvexHull - Güvenlik hull (2D, Simülasyon formatında)
            formasyon_aralik: float - ROV'lar arası minimum mesafe
        
        Returns:
            bool: True if formasyon geçerli, False otherwise
        """
        if hull is None or test_points is None or len(test_points) == 0:
            return False
        
        try:
            # 1. Tüm pozisyonlar hull içinde mi?
            for tp in test_points:
                # formasyon() fonksiyonu (ursina_x, ursina_z, ursina_y) döner.
                # Bu zaten (Sim X, Sim Y, Sim Z) demektir.
                # Sadece ilk iki bileşeni (X, Y) kontrol etmek yeterli.
                if not self.is_point_inside_hull(tp, hull):
                    return False
            
            # 2. Mesafe kontrolü
            for i in range(len(test_points)):
                for j in range(i + 1, len(test_points)):
                    p1 = np.array(test_points[i])
                    p2 = np.array(test_points[j])
                    if np.linalg.norm(p1 - p2) < formasyon_aralik:
                        return False
            return True
        except Exception as e:
            print(f"❌ [HATA] Formasyon geçerliliği kontrolü sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _hull_kenarlarina_nokta_ekle(self, hull, points_2d, nokta_araligi=5.0):
        """
        Convex hull çizgisi üzerinde belirli aralıklarla noktalar ekler.
        
        Args:
            hull: ConvexHull objesi
            points_2d: numpy array - Hull vertex noktaları (Nx2)
            nokta_araligi: float - Noktalar arası mesafe (metre, varsayılan: 5.0)
        
        Returns:
            numpy array: Genişletilmiş noktalar (orijinal vertex'ler + interpolasyon noktaları)
        """
        if hull is None or points_2d is None or len(points_2d) == 0:
            return points_2d
        
        try:
            # Yeni noktalar listesi
            yeni_noktalar = []
            
            # Hull'un kenarlarını (edges) al
            # hull.vertices sıralı vertex'leri içerir (CCW veya CW)
            vertices = hull.vertices
            n_vertices = len(vertices)
            
            # Her kenar için interpolasyon yap
            for i in range(n_vertices):
                # Mevcut ve sonraki vertex'i al
                curr_idx = vertices[i]
                next_idx = vertices[(i + 1) % n_vertices]
                
                p1 = points_2d[curr_idx]
                p2 = points_2d[next_idx]
                
                # Kenar uzunluğunu hesapla
                kenar_uzunlugu = np.linalg.norm(p2 - p1)
                
                # Başlangıç vertex'ini ekle (sadece ilk iterasyonda)
                if i == 0:
                    yeni_noktalar.append(p1.copy())
                
                # Eğer kenar uzunluğu nokta_araligi'ndan büyükse, interpolasyon yap
                if kenar_uzunlugu > nokta_araligi:
                    # Kaç nokta ekleyeceğimizi hesapla
                    n_ek_nokta = int(kenar_uzunlugu / nokta_araligi)
                    
                    # Kenar üzerinde interpolasyon noktaları ekle
                    for j in range(1, n_ek_nokta + 1):
                        t = j * nokta_araligi / kenar_uzunlugu
                        # t değeri 1.0'ı geçmemeli (son vertex'i ayrı ekleyeceğiz)
                        if t < 1.0:
                            interpolasyon_noktasi = p1 + t * (p2 - p1)
                            yeni_noktalar.append(interpolasyon_noktasi)
                
                # Son vertex'i ekle (kapalı çizgi için gerekli)
                # Not: Son kenar için son vertex, ilk vertex ile aynı olacak ama unique() ile tekrar kaldırılacak
                yeni_noktalar.append(p2.copy())
            
            # Eğer hiç nokta eklenmediyse, orijinal noktaları döndür
            if len(yeni_noktalar) == 0:
                return points_2d
            
            # Numpy array'e çevir
            yeni_noktalar_array = np.array(yeni_noktalar)
            
            # Tekrarları kaldır ama sıralamayı koru (her kenar için eklenen noktalar sıralı)
            # Basit yaklaşım: İlk görünen noktayı tut
            seen = set()
            yeni_noktalar_sirali = []
            for nokta in yeni_noktalar_array:
                nokta_tuple = tuple(np.round(nokta, 3))
                if nokta_tuple not in seen:
                    seen.add(nokta_tuple)
                    yeni_noktalar_sirali.append(nokta)
            
            yeni_noktalar_array = np.array(yeni_noktalar_sirali)
            
            # Noktaları convex hull kenarlarına göre açısal sırala (merkezden)
            # Bu, harita çizimi için doğru sıralamayı sağlar
            if len(yeni_noktalar_array) > 0:
                # Merkezi hesapla
                merkez = np.mean(yeni_noktalar_array, axis=0)
                # Her nokta için açı hesapla (atan2: -pi ile pi arası)
                noktalar_merkezden = yeni_noktalar_array - merkez
                acilar = np.arctan2(noktalar_merkezden[:, 1], noktalar_merkezden[:, 0])
                # Açıya göre sırala (counter-clockwise)
                siralama_indeksleri = np.argsort(acilar)
                yeni_noktalar_array = yeni_noktalar_array[siralama_indeksleri]
            
            return yeni_noktalar_array
            
        except Exception as e:
            print(f"⚠️ [UYARI] Hull kenarlarına nokta eklenirken hata: {e}")
            import traceback
            traceback.print_exc()
            # Hata durumunda orijinal noktaları döndür
            return points_2d

