import numpy as np
from shapely.geometry import Point, Polygon, MultiPolygon, MultiPoint
from shapely.ops import unary_union
from shapely.prepared import prep
from scipy.spatial.distance import pdist # Hızlı mesafe kontrolü için

try:
    from scipy.spatial import ConvexHull
    SCIPY_AVAILABLE = True
except ImportError:
    print("❌ [HULL-INIT] Scipy kütüphanesi eksik!")
    SCIPY_AVAILABLE = False

class SahteHull:
    def __init__(self, points, polygon_obj):
        self.points = points
        self.polygon = polygon_obj

    def __len__(self): return len(self.points)
    def __getitem__(self, index): return self.points[index]
    def __iter__(self): return iter(self.points)

class HullManager:
    def __init__(self, filo_ref=None):
        self.filo = filo_ref

    def yeni_hull(
            self,
            base_points: list,
            yasakli_noktalar: list | None = None,
            offset: float = 0.0,
            buffer_radius: float = 10.0,
            **kwargs
        ) -> dict:
            
            #print(f"🔍 [HULL] Hesaplama başladı. Base Points: {len(base_points) if base_points else 0}, Offset: {offset}")
            BOS = {'points': None, 'center': None, 'hull': None}

            # 1. Kontrol
            if not base_points or len(base_points) < 3:
                print("❌ [HULL] Yetersiz başlangıç noktası.")
                return BOS

            # 2. Hull Oluştur
            hull_points = self._convex_hull(base_points)
            if hull_points is None:
                print("❌ [HULL] ConvexHull oluşturulamadı (Scipy hatası veya geometri hatası).")
                return BOS

            try:
                # 3. Polygon Oluştur
                poly = Polygon(hull_points)
                if not poly.is_valid:
                    print("⚠️ [HULL] Polygon geçersiz, buffer(0) ile düzeltiliyor...")
                    poly = poly.buffer(0)

                # Offset Uygula
                if offset != 0.0:
                    poly = poly.buffer(offset)

                # 4. Yasaklı Bölgeleri Çıkar
                if yasakli_noktalar:
                    #print(f"🛡️ [HULL] {len(yasakli_noktalar)} adet yasaklı nokta/engel işleniyor...")
                    yasakli_bufferlar = []
                    for p in yasakli_noktalar:
                        if len(p) >= 2:
                            yasakli_bufferlar.append(Point(float(p[0]), float(p[1])).buffer(buffer_radius))
                    
                    if yasakli_bufferlar:
                        engeller = unary_union(yasakli_bufferlar)
                        poly = poly.difference(engeller)

                # 5. Geometri Kontrolü
                if poly.is_empty:
                    print("❌ [HULL] Sonuç alanı boş (Engeller tüm alanı kaplamış olabilir).")
                    return BOS

                # MultiPolygon Kontrolü (Alan parçalandıysa en büyüğünü al)
                if isinstance(poly, MultiPolygon):
                    # print("⚠️ [HULL] Alan parçalandı (MultiPolygon), en büyük parça seçiliyor.")
                    if not poly.geoms:
                        return BOS
                    poly = max(poly.geoms, key=lambda a: a.area)

                if not hasattr(poly, "exterior"):
                    print(f"❌ [HULL] Polygon yapısı bozuk. Tip: {type(poly)}")
                    return BOS

                # 6. Sonuçları Paketle
                # coords[:-1] -> Kapanış noktasını tekrar etmemek için
                new_points = np.array(poly.exterior.coords) 
                
                # Merkez
                rp = poly.representative_point()
                center = (float(rp.x), float(rp.y), 0.0)

                # Wrapper Sınıfı Kullan
                custom_hull = SahteHull(new_points, poly)

                #print(f"✅ [HULL] Başarılı! {len(new_points)} nokta üretildi.")
                
                return {
                    'points': new_points,
                    'center': center,
                    'hull': custom_hull 
                }

            except Exception as e:
                print(f"❌ [HULL KRİTİK HATA] İşlem sırasında istisna oluştu: {e}")
                import traceback
                traceback.print_exc()
                return BOS

    def _convex_hull(self, points: list):
        if not SCIPY_AVAILABLE:
            return None
        try:
            # Veri temizleme
            pts_clean = []
            for p in points:
                if isinstance(p, (list, tuple, np.ndarray)) and len(p) >= 2:
                    pts_clean.append([float(p[0]), float(p[1])])
            
            pts = np.array(pts_clean)
            if len(pts) < 3:
                return None
                
            hull = ConvexHull(pts, qhull_options='QJ')
            return pts[hull.vertices]
        except Exception as e:
            print(f"❌ [HULL] _convex_hull hatası: {e}")
            return None

    def is_point_inside_hull(self, point, hull):
        """
        Noktanın hull içinde olup olmadığını döngüsüz (vektörel) kontrol eder.
        """
        if hull is None: return False
        p_arr = np.array(point[:2])

        # 1. Shapely (SahteHull) Kontrolü
        if hasattr(hull, 'polygon') and hull.polygon is not None:
            return hull.polygon.contains(Point(p_arr))

        # 2. Scipy ConvexHull Kontrolü (Matris Çarpımı ile O(1) hızında)
        if hasattr(hull, 'equations'):
            # Ax + By + C <= 0 denklemini tüm kenarlar için tek seferde çözer
            return np.all(hull.equations[:, :-1] @ p_arr + hull.equations[:, -1] <= 1e-6)
        
        return False

    def formasyon_gecerli_mi(self, test_points, hull, formasyon_aralik):
        """
        Tüm formasyonun geçerliliğini SIFIR DÖNGÜ (O(N) ve O(N^2) vektörel) ile kontrol eder.
        """
        if hull is None or not test_points: return False

        try:
            # Test noktalarını hızlıca NumPy array'e çevir (X ve Y)
            pts = np.array(test_points)[:, :2]

            # 1. Hull İçinde mi? (Prepared Geometry ve MultiPoint ile ultra hızlı)
            if hasattr(hull, 'polygon'):
                # 'prep' poligonu sorgular için optimize eder (STRtree benzeri yapı)
                prepared_poly = prep(hull.polygon)
                if not prepared_poly.contains(MultiPoint(pts)):
                    return False
            elif hasattr(hull, 'equations'):
                # Scipy Hull için matrisel kontrol: (Kenar Sayısı x Nokta Sayısı)
                res = hull.equations[:, :-1] @ pts.T + hull.equations[:, -1][:, np.newaxis]
                if not np.all(res <= 1e-6):
                    return False

            # 2. ROV'lar Arası Mesafe Kontrolü (pdist ile O(N^2) ama C seviyesinde hızlı)
            if len(pts) > 1:
                # Tüm noktaların birbirine olan mesafesini tek seferde hesaplar
                mesafeler = pdist(pts)
                if np.any(mesafeler < formasyon_aralik):
                    return False
            
            return True

        except Exception as e:
            print(f"⚠️ [HULL] Vektörel kontrol hatası: {e}")
            return False