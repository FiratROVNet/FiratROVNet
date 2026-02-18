import numpy as np
from shapely.geometry import Point, Polygon, MultiPolygon, MultiPoint
from shapely.ops import unary_union
from shapely.prepared import prep
from scipy.spatial.distance import pdist  # Hızlı mesafe kontrolü için

try:
    from scipy.spatial import ConvexHull, cKDTree
    SCIPY_AVAILABLE = True
except ImportError:
    print("❌ [HULL-INIT] Scipy kütüphanesi eksik!")
    SCIPY_AVAILABLE = False
    cKDTree = None

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

    def dinamik_engelleri_basitlestir(
            self,
            engel_bulutu: list,
            kume_mesafesi: float = 25.0,
            buffer_radius: float = 5.0,
            min_kume_boyutu: int = 3
        ) -> list:
        """
        Lidar/Sonar'dan gelen binlerce engel noktasını (engel_bulutu) 
        basitleştirir ve geometrik engellere dönüştürür.
        
        Args:
            engel_bulutu: [(x, y), ...] formatında engel noktaları listesi
            kume_mesafesi: Kümeleme için maksimum intra-cluster mesafe (default 25m)
            buffer_radius: Tek/çift nokta için daire yarıçapı (default 5m)
            min_kume_boyutu: Geçerli küme için minimum nokta sayısı (default 3)
        
        Returns:
            Polygon listesi veya boş liste
            Liste elemanları: Shapely Polygon nesneleri
                - Çıkış her poligonun merkezi (centroid) ve kapsayan radius bilgisi depolanır:
                  polygon.centroid -> Point(x, y)
                  polygon.bounds -> (minx, miny, maxx, maxy)
        
        Örnek Kullanım:
        ```python
        engel_bulutu = [(x1, y1), (x2, y2), ...]
        dinamik_engeller = manager.dinamik_engelleri_basitlestir(engel_bulutu)
        
        # A* için hazırlama (radius bilgisi ile)
        obstacles_for_astar = []
        for poly in dinamik_engeller:
            centroid = poly.centroid
            bounds = poly.bounds  # (minx, miny, maxx, maxy)
            radius = max(bounds[2] - centroid.x, bounds[3] - centroid.y)
            obstacles_for_astar.append((centroid.x, centroid.y, radius))
        ```
        """
        
        if not engel_bulutu or len(engel_bulutu) == 0:
            return []
        
        if not SCIPY_AVAILABLE or cKDTree is None:
            print("❌ [DINAMIK-ENGEL] Scipy.spatial.cKDTree mevcut değil!")
            return []
        
        try:
            # 1. Veriyi NumPy array'e çevir ve temizle
            points_array = np.array(engel_bulutu, dtype=np.float32)
            
            if len(points_array) == 0 or len(points_array.shape) != 2 or points_array.shape[1] < 2:
                return []
            
            # 2. cKDTree ile hızlı kümeleme (Density-based clustering simulasyonu)
            # Tüm nokta çiftlerini query ederiz ve gruplayız
            points_2d = points_array[:, :2]
            tree = cKDTree(points_2d)
            
            # Ziyaret edilmemiş noktaları takip et
            visited = np.zeros(len(points_2d), dtype=bool)
            kumeler = []
            
            for i in range(len(points_2d)):
                if visited[i]:
                    continue
                
                # Mevcut noktanın kume_mesafesi içindeki komşularını bul
                indices = tree.query_ball_point(points_2d[i], kume_mesafesi)
                
                kume = points_2d[indices]
                visited[indices] = True
                
                # Kümeyi sakla
                kumeler.append(kume)
            
            # 3. Her küme için geometri oluştur
            poligonlar = []
            
            for kume in kumeler:
                if len(kume) == 0:
                    continue
                
                # Küme boyutuna göre işle
                if len(kume) == 1:
                    # Tek nokta -> Daire (buffer)
                    pt = Point(kume[0, 0], kume[0, 1])
                    poly = pt.buffer(buffer_radius)
                    poligonlar.append(poly)
                
                elif len(kume) == 2:
                    # İki nokta -> Kapsül şekli (buffer ile)
                    pt1 = Point(kume[0, 0], kume[0, 1])
                    pt2 = Point(kume[1, 0], kume[1, 1])
                    line = pt1.buffer(buffer_radius).union(pt2.buffer(buffer_radius))
                    poligonlar.append(line)
                
                else:
                    # 3+ nokta -> Convex Hull
                    try:
                        if len(kume) >= 3:
                            hull = ConvexHull(kume, qhull_options='QJ')
                            hull_points = kume[hull.vertices]
                            poly = Polygon(hull_points)
                            
                            if not poly.is_valid:
                                poly = poly.buffer(0)
                            
                            # Hull'u hafif Bufferleme (Kütüphane kenarı düzeltme)
                            poly = poly.buffer(buffer_radius)
                            poligonlar.append(poly)
                    except Exception as e:
                        # Hull hesaplanamadıysa buffer kullan
                        centroid = np.mean(kume, axis=0)
                        pt = Point(centroid[0], centroid[1])
                        poly = pt.buffer(buffer_radius)
                        poligonlar.append(poly)
            
            # 4. Poligonları birleştir (Unary Union)
            if len(poligonlar) == 0:
                return []
            
            elif len(poligonlar) == 1:
                gecerli_poligonlar = [poligonlar[0]]
            
            else:
                birlestirilmis = unary_union(poligonlar)
                
                # MultiPolygon ise ayrıştır
                if isinstance(birlestirilmis, MultiPolygon):
                    gecerli_poligonlar = [geom for geom in birlestirilmis.geoms if not geom.is_empty]
                elif isinstance(birlestirilmis, Polygon) and not birlestirilmis.is_empty:
                    gecerli_poligonlar = [birlestirilmis]
                else:
                    gecerli_poligonlar = []
            
            # 5. Sonuç döndür
            return gecerli_poligonlar
        
        except Exception as e:
            print(f"❌ [DINAMIK-ENGEL KRİTİK HATA] {e}")
            import traceback
            traceback.print_exc()
            return []
            return False