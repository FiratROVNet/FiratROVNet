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

<<<<<<< HEAD
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
    
    def guvenlik_hull_olustur(self, offset=50.0):
        """
        1. Lider ROV'u merkez alarak 'offset' yarıçaplı dairesel noktalar oluşturur.
        2. Yakındaki adaları 'offset' kadar içeri (lider ROV'a doğru) çeken sanal noktaları alır.
        3. Hepsini birleştirerek adayı DIŞARIDA bırakan güvenli alanı hesaplar.
        4. Havuz duvarlarını kontrol eder ve hull'ı duvarların içinde tutar.
        
        Args:
            offset (float): Güvenlik hull yarıçapı (metre, varsayılan: 50.0)
        
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
            # 1. Lider ROV merkezli dairesel noktalar oluştur
            sanal_rov_noktalari = []
            
            # Lider ROV'u bul
            lider_id = 0  # Varsayılan
            if hasattr(self.filo, 'sistemler'):
                for i, sistem in enumerate(self.filo.sistemler):
                    if hasattr(sistem, 'rov') and sistem.rov.role == 1:
                        lider_id = i
                        break
            
            # Lider pozisyonunu al
            lider_gps = self.filo.get(lider_id, "gps")
            if lider_gps is None:
                # Lider bulunamazsa hull oluşturma
                return {'hull': None, 'points': None, 'center': None}
            
            lx, ly, lz = lider_gps
            
            # Havuz sınırlarını al (Simülasyon formatında: X ve Y)
            havuz_genisligi = 200.0  # Varsayılan
            if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'havuz_genisligi'):
                havuz_genisligi = self.filo.ortam_ref.havuz_genisligi
            
            # Güvenlik payı (duvardan ne kadar içeride olmalı)
            duvar_guvenlik_payi = 5.0
            max_limit = havuz_genisligi - duvar_guvenlik_payi
            min_limit = -havuz_genisligi + duvar_guvenlik_payi
            
            # Dairesel noktalar (16 nokta)
            for i in range(16):
                aci = math.radians(i * (360 / 16))
                nx = lx + math.cos(aci) * offset
                ny = ly + math.sin(aci) * offset
                
                # Duvar kontrolü: Eğer nokta duvarın dışındaysa, duvara çek
                nx = max(min_limit, min(max_limit, nx))
                ny = max(min_limit, min(max_limit, ny))
                
                sanal_rov_noktalari.append((nx, ny, lz))
            
            # 2. Adaların sanal bariyerleri (Adadan filoya doğru itilmiş noktalar)
            # Not: ada_engel_noktalari_pro filoya yakın adaları zaten işliyor
            # Ancak şimdi filo merkezi yerine lider pozisyonunu kullanmalıyız
            # ada_engel_noktalari_pro fonksiyonu filo._get_all_rovs_positions kullanıyor
            # Bu yüzden onu değiştirmeden kullanmak yerine, burada manuel işlem yapabiliriz
            # veya mevcut yapıyı koruyabiliriz. Basitlik için sadece dairesel hull kullanalım.
            
            # Hull noktaları sadece lider etrafındaki daire olsun
            # Adalar bu dairenin içinde kalırsa 'yeniden_ciz' fonksiyonu onları çıkaracaktır.
            tum_noktalar = sanal_rov_noktalari
            
            if len(tum_noktalar) < 3:
                hull_data = {
                    'hull': None,
                    'points': None,
                    'center': None
                }
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
            
            # Merkez liderin kendisi
            center_2d = np.array([lx, ly])
            z_avg = lz

            hull_data = {
                'hull': hull_2d, 
                'points': points_2d_genisletilmis,  # Genişletilmiş noktalar
                'center': (center_2d[0], center_2d[1], z_avg)
            }

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
    
=======
>>>>>>> develop
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