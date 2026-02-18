import numpy as np
from shapely.geometry import Point, Polygon, MultiPolygon, MultiPoint
from shapely.ops import unary_union
from shapely.prepared import prep
from scipy.spatial.distance import pdist  # Hızlı mesafe kontrolü için

try:
    from scipy.spatial import ConvexHull
    SCIPY_AVAILABLE = True
except ImportError:
    print("❌ [HULL-INIT] Scipy kütüphanesi eksik!")
    SCIPY_AVAILABLE = False

class SahteHull:
    """
    Shapely Polygon nesnesini ve köşe noktalarını bir arada tutan yardımcı sınıf.
    """
    def __init__(self, points, polygon_obj):
        self.points = points
        self.polygon = polygon_obj

    def __len__(self): return len(self.points)
    def __getitem__(self, index): return self.points[index]
    def __iter__(self): return iter(self.points)

class HullManager:
    def __init__(self, filo_ref=None):
        self.filo = filo_ref

    def _noktalari_ayikla(self, veri, g_id=0):
        """
        Gelen veriyi (Liste veya Sözlük) analiz eder ve saf koordinat listesine (NumPy) çevirir.
        
        Args:
            veri: Liste [(x,y,z)...] veya Sözlük {id: (x,y,z)...}
            g_id: Grup ID (Varsayılan 0). İleride filtreleme gerekirse kullanılır.
        
        Returns:
            np.ndarray: (N, 2) boyutunda sadece X ve Y koordinatlarını içeren dizi.
        """
        nokta_listesi = []

        # 1. Durum: Sözlük (Dictionary) gelirse {id: (x,y,z)}
        if isinstance(veri, dict):
            # FormasyonYoneticisi'nden gelen veri zaten filtrelenmiş olduğu için
            # doğrudan values() alıyoruz.
            nokta_listesi = list(veri.values())
            
        # 2. Durum: Liste (List) veya Tuple gelirse [(x,y,z), ...]
        elif isinstance(veri, (list, tuple, np.ndarray)):
            nokta_listesi = veri
            
        # Veri temizleme ve formatlama
        valid_points = []
        if nokta_listesi is not None:
            for p in nokta_listesi:
                # None kontrolü ve uzunluk kontrolü (en az x,y olmalı)
                if p is not None and len(p) >= 2:
                    try:
                        valid_points.append([float(p[0]), float(p[1])])
                    except (ValueError, TypeError):
                        continue
                        
        return np.array(valid_points)

    def yeni_hull(
            self,
            base_points,
            yasakli_noktalar: list | None = None,
            offset: float = 60.0,
            buffer_radius: float = 10.0,
            g_id: int = 0,
            **kwargs
        ) -> dict:
            """
            Verilen noktalardan güvenli bir Convex Hull (Dış Kabuk) oluşturur.
            base_points: Liste veya Sözlük olabilir.
            """
            
            BOS = {'points': None, 'center': None, 'hull': None}

            # 1. Veriyi Hazırla (Sözlük/Liste Ayrımı ve NumPy dönüşümü)
            pts_array = self._noktalari_ayikla(base_points, g_id=g_id)

            # Yetersiz nokta kontrolü
            if len(pts_array) < 3:
                # print("⚠️ [HULL] Yetersiz başlangıç noktası (En az 3 nokta gerekli).")
                return BOS

            # 2. Hull Oluştur (Scipy ConvexHull)
            hull_points = self._convex_hull(pts_array)
            if hull_points is None:
                print("❌ [HULL] ConvexHull oluşturulamadı.")
                return BOS

            try:
                # 3. Polygon Oluştur
                poly = Polygon(hull_points)
                if not poly.is_valid:
                    poly = poly.buffer(0) # Kendini kesen geometrileri düzelt

                # Offset Uygula (Alan Genişletme)
                if offset != 0.0:
                    poly = poly.buffer(offset)

                # 4. Yasaklı Bölgeleri Çıkar
                if yasakli_noktalar:
                    yasakli_bufferlar = []
                    # Yasaklı noktaları da normalize et
                    yasakli_arr = self._noktalari_ayikla(yasakli_noktalar, g_id=0)
                    
                    for p in yasakli_arr:
                        yasakli_bufferlar.append(Point(p[0], p[1]).buffer(buffer_radius))
                    
                    if yasakli_bufferlar:
                        engeller = unary_union(yasakli_bufferlar)
                        poly = poly.difference(engeller)

                # 5. Geometri Kontrolü
                if poly.is_empty:
                    print("❌ [HULL] Sonuç alanı boş (Engeller tüm alanı kaplamış).")
                    return BOS

                # MultiPolygon Kontrolü (Alan bölündüyse en büyüğünü seç)
                if isinstance(poly, MultiPolygon):
                    if not poly.geoms: return BOS
                    poly = max(poly.geoms, key=lambda a: a.area)

                if not hasattr(poly, "exterior"):
                    return BOS

                # 6. Sonuçları Paketle
                # coords[:-1] -> Kapanış noktasını tekrar etmemek için
                new_points = np.array(poly.exterior.coords)
                
                # Merkez Noktası (Görselleştirme için)
                rp = poly.representative_point()
                center = (float(rp.x), float(rp.y), 0.0)

                # Wrapper Sınıfı
                custom_hull = SahteHull(new_points, poly)
                
                return {
                    'points': new_points,
                    'center': center,
                    'hull': custom_hull 
                }

            except Exception as e:
                print(f"❌ [HULL KRİTİK HATA] İşlem sırasında istisna oluştu: {e}")
                return BOS

    def _convex_hull(self, points_array):
        """Scipy kullanarak noktaların dış kabuğunu hesaplar."""
        if not SCIPY_AVAILABLE or len(points_array) < 3:
            return None
        try:
            # points_array zaten _noktalari_ayikla ile np.array yapıldı
            hull = ConvexHull(points_array, qhull_options='QJ') # QJ: Joggled input (Hata toleransı için)
            return points_array[hull.vertices]
        except Exception as e:
            print(f"❌ [HULL] _convex_hull hatası: {e}")
            return None

    def is_point_inside_hull(self, point, hull):
        """
        Tek bir noktanın hull içinde olup olmadığını kontrol eder.
        point: (x, y) veya (x, y, z) tuple/list
        """
        if hull is None: return False
        
        try:
            # Noktayı hazırla
            if hasattr(point, '__len__') and len(point) >= 2:
                p_arr = Point(float(point[0]), float(point[1]))
            else:
                return False

            # Shapely (SahteHull) Kontrolü
            if hasattr(hull, 'polygon') and hull.polygon is not None:
                return hull.polygon.contains(p_arr)
            
            return False
        except Exception:
            return False

    def formasyon_gecerli_mi(self, test_points, hull, formasyon_aralik, g_id=0):
        """
        Formasyon noktalarının Hull sınırları içinde olup olmadığını ve
        birbirlerine çok yakın olup olmadıklarını kontrol eder.
        
        Args:
            test_points: Liste veya Sözlük {id: (x,y,z)}
            hull: SahteHull objesi
            formasyon_aralik: Minimum mesafe
            g_id: Grup ID (Varsayılan 0)
        """
        # Veri yoksa veya Hull yoksa geçersiz
        if hull is None or not test_points: 
            return False

        try:
            # 1. Veriyi Temizle ve NumPy Array'e Çevir
            # Dictionary gelirse values() alınır, List gelirse olduğu gibi alınır.
            pts = self._noktalari_ayikla(test_points, g_id=g_id)
            
            if len(pts) == 0:
                return False

            # 2. Hull İçinde Kalma Kontrolü (Shapely - Prepared Geometry)
            if hasattr(hull, 'polygon'):
                # 'prep' poligonu sorgular için optimize eder (STRtree mantığı)
                prepared_poly = prep(hull.polygon)
                
                # Tüm noktaları MultiPoint objesine çevirip tek seferde soruyoruz
                mp = MultiPoint(pts)
                
                if not prepared_poly.contains(mp):
                    # Detaylı hata ayıklama istenirse burası açılabilir
                    # print("⚠️ [HULL] Bazı noktalar güvenli alanın dışında!")
                    return False
            else:
                return False

            # 3. ROV'lar Arası Mesafe Kontrolü (pdist - O(N^2) ama C ile optimize)
            # Eğer sadece 1 ROV varsa mesafe kontrolüne gerek yok
            if len(pts) > 1:
                # pdist: Noktalar arası ikili mesafeleri hesaplar
                mesafeler = pdist(pts)
                
                # Herhangi bir mesafe, belirlenen aralıktan (toleranslı) küçükse çarpışma riski var
                # Tolerans: Formasyon aralığının %30'undan daha yakınlarsa hata ver
                min_mesafe_limiti = formasyon_aralik * 0.3
                
                if np.any(mesafeler < min_mesafe_limiti):
                    # print("⚠️ [HULL] ROV'lar birbirine çok yakın (Çarpışma riski)!")
                    return False
            
            return True

        except Exception as e:
            print(f"⚠️ [HULL] Vektörel kontrol hatası: {e}")
            import traceback
            traceback.print_exc()
            return False