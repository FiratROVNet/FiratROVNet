"""
Senaryo Üretim Modülü - Yapay Zeka Eğitimi İçin Veri Üretimi

Bu modül, GUI olmadan (headless) simülasyon ortamları oluşturur ve
yapay zeka algoritmalarını eğitmek için veri üretir.

Kullanım:
    from FiratROVNet import senaryo
    
    # Senaryo oluştur
    senaryo.uret(n_rovs=4, n_engels=20, havuz_genisligi=200)
    
    # Veri al
    batarya = senaryo.get(0, "batarya")
    gps = senaryo.get(0, "gps")
    sonar = senaryo.get(0, "sonar")
    
    # veya Filo üzerinden
    batarya = senaryo.filo.get(0, "batarya")
"""

import os
import sys

# Ursina'yı headless modda başlat
os.environ['URSINA_HEADLESS'] = '1'

from ursina import *
from FiratROVNet.simulasyon import Ortam, ROV
from FiratROVNet.gnc import Filo
import numpy as np
import random

# Global senaryo instance
_senaryo_instance = None


class Senaryo:
    """
    Senaryo üretim sınıfı - Headless simülasyon ortamı oluşturur.
    
    Bu sınıf, GUI olmadan simülasyon ortamları oluşturur ve
    yapay zeka eğitimi için veri üretir.
    """
    
    def __init__(self):
        self.app = None
        self.filo = None
        self.ortam = None
        self.aktif = False
        
        # Önbellek mekanizması (hızlı senaryo üretimi için)
        self._cache_n_rovs = None
        self._cache_n_engels = None
        self._cache_havuz_genisligi = None
        
    def uret(self, n_rovs=None, n_engels=None, havuz_genisligi=None, 
             engel_tipleri=None, baslangic_pozisyonlari=None,
             modem_ayarlari=None, sensor_ayarlari=None):
        """
        Senaryo ortamı oluşturur (GUI olmadan).
        
        Args:
            n_rovs (int, optional): ROV sayısı. None ise önbellekten kullanılır veya varsayılan: 3
            n_engels (int, optional): Engel sayısı. None ise önbellekten kullanılır veya varsayılan: 15
            havuz_genisligi (float, optional): Havuz genişliği. None ise önbellekten kullanılır veya varsayılan: 200
            engel_tipleri (list, optional): Engel tipleri listesi
                Örnek: ['kaya', 'kaya', 'agac'] veya None (hepsi kaya)
            baslangic_pozisyonlari (dict, optional): ROV başlangıç pozisyonları
                Örnek: {0: (0, -5, 0), 1: (10, -5, 10)}
            modem_ayarlari (dict, optional): Modem ayarları
            sensor_ayarlari (dict, optional): Sensör ayarları
        
        Returns:
            Senaryo: Kendi instance'ını döndürür (method chaining için)
        
        Örnek:
            # İlk çağrı - yeni ortam oluştur
            senaryo.uret(n_rovs=4, n_engels=20, havuz_genisligi=200)
            
            # Sonraki çağrılar - parametresiz, önbellekten kullanır
            senaryo.uret()  # Aynı parametrelerle farklı pozisyonlar
        """
        # Parametre kontrolü - yeni parametreler varsa önbelleği güncelle
        yeni_ortam_gerekli = False
        
        if n_rovs is not None:
            if self._cache_n_rovs != n_rovs:
                yeni_ortam_gerekli = True
                self._cache_n_rovs = n_rovs
        elif self._cache_n_rovs is None:
            n_rovs = 3  # Varsayılan
            self._cache_n_rovs = n_rovs
            yeni_ortam_gerekli = True
        else:
            n_rovs = self._cache_n_rovs
        
        if n_engels is not None:
            if self._cache_n_engels != n_engels:
                yeni_ortam_gerekli = True
                self._cache_n_engels = n_engels
        elif self._cache_n_engels is None:
            n_engels = 15  # Varsayılan
            self._cache_n_engels = n_engels
            yeni_ortam_gerekli = True
        else:
            n_engels = self._cache_n_engels
        
        if havuz_genisligi is not None:
            if self._cache_havuz_genisligi != havuz_genisligi:
                yeni_ortam_gerekli = True
                self._cache_havuz_genisligi = havuz_genisligi
        elif self._cache_havuz_genisligi is None:
            havuz_genisligi = 200  # Varsayılan
            self._cache_havuz_genisligi = havuz_genisligi
            yeni_ortam_gerekli = True
        else:
            havuz_genisligi = self._cache_havuz_genisligi
        
        # Yeni ortam oluştur veya mevcut ortamı güncelle
        if yeni_ortam_gerekli or self.ortam is None:
            # Ursina'yı headless modda başlat
            if self.app is None:
                # Headless mod için özel ayarlar
                os.environ['URSINA_HEADLESS'] = '1'
                
                try:
                    self.app = Ursina(
                        vsync=False,
                        development_mode=False,
                        show_ursina_splash=False,
                        borderless=True,
                        title="FıratROVNet Senaryo Üretimi (Headless)"
                    )
                    
                    # Window özelliklerini güvenli şekilde ayarla
                    try:
                        if hasattr(window, 'fullscreen'):
                            window.fullscreen = False
                    except:
                        pass
                    
                    try:
                        if hasattr(window, 'show'):
                            window.show = False
                    except:
                        pass
                    
                    try:
                        if hasattr(window, 'fps_counter'):
                            window.fps_counter.enabled = False
                    except:
                        pass
                        
                except Exception as e:
                    # Ursina başlatılamazsa minimal ortam oluştur
                    print(f"⚠️ Ursina headless mod başlatılamadı: {e}")
                    print("   Minimal ortam modu kullanılıyor...")
                    self.app = None
            
            # Ortam oluştur - Gerçek Ortam sınıfını kullan (simulasyon.py'den)
            # Bu sayede Ada ve ROV fonksiyonları kullanılabilir
            try:
                from FiratROVNet.simulasyon import Ortam as OrtamSinifi
                self.ortam = OrtamSinifi()
                # Headless mod için görsel özellikleri kapat
                if hasattr(self.ortam, 'app'):
                    try:
                        if hasattr(self.ortam.app, 'window'):
                            self.ortam.app.window.show = False
                    except:
                        pass
            except Exception as e:
                # Ortam sınıfı yüklenemezse minimal ortam objesi oluştur
                print(f"⚠️ Ortam sınıfı yüklenemedi, minimal mod kullanılıyor: {e}")
                self.ortam = type('Ortam', (), {
                    'rovs': [],
                    'engeller': [],
                    'havuz_genisligi': havuz_genisligi,
                    'filo': None,
                    'island_positions': []  # Ada pozisyonları için
                })()
            
            # Havuz genişliğini kaydet
            self.ortam.havuz_genisligi = havuz_genisligi
            
            # Gerçek Ortam sınıfı ise sim_olustur kullan
            if hasattr(self.ortam, 'sim_olustur'):
                # Ortam'ın sim_olustur metodunu kullan (daha sağlıklı)
                self.ortam.sim_olustur(n_rovs=n_rovs, n_engels=n_engels, havuz_genisligi=havuz_genisligi)
            else:
                # Minimal ortam için manuel oluşturma
                # Engelleri oluştur
                self.ortam.engeller = []
                for i in range(n_engels):
                    engel_tipi = 'kaya'  # Varsayılan
                    if engel_tipleri and i < len(engel_tipleri):
                        engel_tipi = engel_tipleri[i]
                    
                    # Engel pozisyonu (havuz içinde rastgele)
                    x = random.uniform(-havuz_genisligi/2, havuz_genisligi/2)
                    z = random.uniform(-havuz_genisligi/2, havuz_genisligi/2)
                    y = random.uniform(-90, 0)  # Su altı derinliği
                    
                    # Engel boyutları
                    if engel_tipi == 'kaya':
                        s_x = random.uniform(15, 40)
                        s_y = random.uniform(15, 40)
                        s_z = random.uniform(15, 40)
                        gri = random.randint(80, 100)
                        engel_rengi = color.rgb(gri, gri, gri)
                    elif engel_tipi == 'agac':
                        s_x = random.uniform(5, 10)
                        s_y = random.uniform(20, 40)
                        s_z = random.uniform(5, 10)
                        engel_rengi = color.rgb(34, 139, 34)  # Yeşil
                    else:  # Varsayılan
                        s_x = random.uniform(10, 30)
                        s_y = random.uniform(10, 30)
                        s_z = random.uniform(10, 30)
                        engel_rengi = color.gray
                    
                    # Engel entity oluştur (headless, görsel olmayan)
                    # Headless mod için minimal engel objesi (Ursina Entity yerine)
                    try:
                        # Ursina Entity oluşturmayı dene
                        if self.app is not None:
                            try:
                                engel = Entity(
                                    model='icosphere',
                                    color=engel_rengi,
                                    scale=(s_x, s_y, s_z),
                                    position=(x, y, z),
                                    rotation=(random.randint(0, 360), random.randint(0, 360), random.randint(0, 360)),
                                    collider='mesh',
                                    unlit=True
                                )
                                try:
                                    engel.visible = False
                                except:
                                    pass
                            except Exception as engel_error:
                                raise Exception(f"Engel oluşturulamadı: {engel_error}")
                        else:
                            raise Exception("Ursina app yok")
                    except Exception as e:
                        # Entity oluşturulamazsa minimal engel objesi
                        engel = type('Engel', (), {
                            'position': Vec3(x, y, z) if hasattr(Vec3, '__init__') else type('Vec3', (), {'x': x, 'y': y, 'z': z})(),
                            'scale_x': s_x,
                            'scale_y': s_y,
                            'scale_z': s_z,
                            'scale': (s_x, s_y, s_z)
                        })()
                        # Vec3 oluşturma hatası için fallback
                        if not hasattr(engel.position, 'x'):
                            engel.position = type('Vec3', (), {'x': x, 'y': y, 'z': z})()
                    
                    self.ortam.engeller.append(engel)
                
                # ROV'ları oluştur (minimal ortam için)
                self.ortam.rovs = []
                for i in range(n_rovs):
                    # Başlangıç pozisyonu
                    if baslangic_pozisyonlari and i in baslangic_pozisyonlari:
                        pozisyon = baslangic_pozisyonlari[i]
                    else:
                        # Rastgele başlangıç pozisyonu
                        x = random.uniform(-10, 10)
                        z = random.uniform(-10, 10)
                        pozisyon = (x, -2, z)
                    
                    # ROV oluştur (headless mod için)
                    try:
                        # Ursina ROV oluşturmayı dene
                        if self.app is not None:
                            try:
                                rov = ROV(rov_id=i, position=pozisyon)
                                rov.environment_ref = self.ortam
                                
                                # Headless modda görsel özellikleri kapat
                                try:
                                    rov.visible = False
                                except:
                                    pass
                                try:
                                    if hasattr(rov, 'label'):
                                        rov.label.enabled = False
                                except:
                                    pass
                            except Exception as rov_error:
                                # ROV oluşturma hatası
                                raise Exception(f"ROV oluşturulamadı: {rov_error}")
                        else:
                            raise Exception("Ursina app veya ROV sınıfı yok")
                    except Exception as e:
                        # ROV oluşturulamazsa minimal ROV objesi
                        # Vec3 oluşturma
                        try:
                            pos_vec = Vec3(*pozisyon) if len(pozisyon) == 3 else Vec3(pozisyon[0], pozisyon[1] if len(pozisyon) > 1 else -2, pozisyon[2] if len(pozisyon) > 2 else 0)
                            vel_vec = Vec3(0, 0, 0)
                        except:
                            # Vec3 yoksa minimal vektör
                            pos_vec = type('Vec3', (), {'x': pozisyon[0], 'y': pozisyon[1] if len(pozisyon) > 1 else -2, 'z': pozisyon[2] if len(pozisyon) > 2 else 0})()
                            vel_vec = type('Vec3', (), {'x': 0, 'y': 0, 'z': 0})()
                        
                        rov = type('ROV', (), {
                            'id': i,
                            'position': pos_vec,
                            'velocity': vel_vec,
                            'battery': 1.0,
                            'role': 0,
                            'sensor_config': {
                                "engel_mesafesi": 20.0,
                                "iletisim_menzili": 35.0,
                                "min_pil_uyarisi": 10.0,
                                "kacinma_mesafesi": 8.0
                            },
                            'environment_ref': self.ortam,
                            'y': pozisyon[1] if len(pozisyon) > 1 else -2,
                            'x': pozisyon[0],
                            'z': pozisyon[2] if len(pozisyon) > 2 else 0,
                            'modem': None
                        })()
                        
                        # set ve get metodlarını ekle
                        def set_method(key, val):
                            if key == 'rol':
                                rov.role = int(val)  # Rol değerini integer'a çevir
                                # Renk güncellemesi (opsiyonel)
                                try:
                                    if hasattr(rov, 'color'):
                                        if int(val) == 1:  # Lider
                                            rov.color = type('color', (), {'rgb': lambda *args: None})()  # Minimal color
                                        else:  # Takipçi
                                            rov.color = type('color', (), {'rgb': lambda *args: None})()  # Minimal color
                                except:
                                    pass
                            elif key in rov.sensor_config:
                                rov.sensor_config[key] = val
                        
                        def get_method(key):
                            if key == 'batarya':
                                return rov.battery
                            elif key == 'gps':
                                return np.array([rov.x, rov.y, rov.z])
                            elif key == 'hiz':
                                v = rov.velocity
                                return np.array([v.x if hasattr(v, 'x') else 0, v.y if hasattr(v, 'y') else 0, v.z if hasattr(v, 'z') else 0])
                            elif key == 'rol':
                                return rov.role
                            elif key == 'sonar':
                                # Sonar hesaplama (minimal - engel mesafesi)
                                if not hasattr(rov, 'environment_ref') or not rov.environment_ref:
                                    return -1
                                min_dist = 999.0
                                for engel in rov.environment_ref.engeller:
                                    if hasattr(engel, 'position'):
                                        engel_pos = engel.position
                                        # Pozisyon erişimi (Vec3 veya dict)
                                        if hasattr(engel_pos, 'x'):
                                            engel_x, engel_y, engel_z = engel_pos.x, engel_pos.y, engel_pos.z
                                        elif isinstance(engel_pos, (list, tuple)) and len(engel_pos) >= 3:
                                            engel_x, engel_y, engel_z = engel_pos[0], engel_pos[1], engel_pos[2]
                                        else:
                                            continue
                                        
                                        # Mesafe hesaplama
                                        dx = rov.x - engel_x
                                        dy = rov.y - engel_y
                                        dz = rov.z - engel_z
                                        d = np.sqrt(dx*dx + dy*dy + dz*dz)
                                        
                                        # Engel boyutunu hesaba kat
                                        if hasattr(engel, 'scale_x'):
                                            scale_z = engel.scale_z if hasattr(engel, 'scale_z') else engel.scale_x
                                            avg_scale = (engel.scale_x + scale_z) / 2
                                            d = max(0, d - (avg_scale / 2))  # Negatif olmamalı
                                        
                                        if d < min_dist:
                                            min_dist = d
                                menzil = rov.sensor_config.get("engel_mesafesi", 20.0)
                                return min_dist if min_dist < menzil else -1
                            elif key in rov.sensor_config:
                                return rov.sensor_config[key]
                            return None
                        
                        def move_method(komut, guc=1.0):
                            """ROV move metodu (minimal implementasyon)"""
                            if rov.battery <= 0:
                                return
                            
                            # time.dt için varsayılan değer (Ursina'dan al veya varsayılan kullan)
                            try:
                                import time as ursina_time
                                dt = getattr(ursina_time, 'dt', 0.016)
                            except:
                                dt = 0.016  # 60 FPS varsayılan
                            
                            thrust = guc * 30.0 * dt  # HIZLANMA_CARPANI = 30
                            
                            v = rov.velocity
                            if not hasattr(v, 'x'):
                                # Velocity Vec3 değilse oluştur
                                v = type('Vec3', (), {'x': 0, 'y': 0, 'z': 0})()
                                rov.velocity = v
                            
                            if komut == "ileri":  
                                v.z = (v.z if hasattr(v, 'z') else 0) + thrust
                            elif komut == "geri": 
                                v.z = (v.z if hasattr(v, 'z') else 0) - thrust
                            elif komut == "sag":  
                                v.x = (v.x if hasattr(v, 'x') else 0) + thrust
                            elif komut == "sol":  
                                v.x = (v.x if hasattr(v, 'x') else 0) - thrust
                            elif komut == "cik":  
                                v.y = (v.y if hasattr(v, 'y') else 0) + thrust
                            elif komut == "bat":  
                                if rov.role != 1:  # Lider batırılamaz
                                    v.y = (v.y if hasattr(v, 'y') else 0) - thrust
                            elif komut == "dur":
                                v.x = v.y = v.z = 0
                            
                            rov.velocity = v
                        
                        rov.set = set_method
                        rov.get = get_method
                        rov.move = move_method
                    
                        self.ortam.rovs.append(rov)
            
            # Filo sistemini kur (her iki durumda da)
            if not hasattr(self.ortam, 'rovs') or not self.ortam.rovs:
                # ROV'lar oluşturulmadıysa hata ver
                raise ValueError("ROV'lar oluşturulamadı!")
            
            self.filo = Filo()
            
            # Filo otomatik kurulum (rol ataması otomatik_kurulum içinde yapılıyor)
            # Senaryo modülü için baslangic_hedefleri None bırakılıyor ki formasyon hesaplaması yapılmasın
            # ROV pozisyonları başlangıç pozisyonlarında kalacak
            tum_modemler = self.filo.otomatik_kurulum(
                rovs=self.ortam.rovs,
                lider_id=0,
                modem_ayarlari=modem_ayarlari,
                baslangic_hedefleri={},  # Boş dict = formasyon hesaplaması yapılmasın, ROV pozisyonları korunsun
                sensor_ayarlari=sensor_ayarlari
            )
            
            # Ortam referansını filo'ya ekle
            self.ortam.filo = self.filo
            
            # Aktif durumu
            self.aktif = True
            
            print(f"✅ Yeni senaryo oluşturuldu: {n_rovs} ROV, {n_engels} Engel, Havuz: {havuz_genisligi}x{havuz_genisligi}")
        else:
            # Mevcut ortamı güncelle - sadece pozisyonları değiştir
            # ROV'ları rastgele yerlere taşı
            if hasattr(self.ortam, 'rovs') and self.ortam.rovs:
                for rov in self.ortam.rovs:
                    # Rastgele pozisyon
                    x = random.uniform(-10, 10)
                    z = random.uniform(-10, 10)
                    pozisyon = (x, -2, z)
                    
                    # Pozisyonu güncelle
                    if hasattr(rov, 'position'):
                        if hasattr(rov.position, 'x'):
                            rov.position.x = pozisyon[0]
                            rov.position.y = pozisyon[1]
                            rov.position.z = pozisyon[2]
                    if hasattr(rov, 'x'):
                        rov.x = pozisyon[0]
                        rov.y = pozisyon[1]
                        rov.z = pozisyon[2]
            
            print(f"🔄 Senaryo pozisyonları güncellendi: {n_rovs} ROV, {n_engels} Engel")
        
        return self
    
    def Ada(self, ada_id, x=None, y=None):
        """
        Ada pozisyonunu değiştirir veya konumunu döndürür.
        Simulasyon.py'deki Ortam.Ada() metodunu kullanır.
        
        Args:
            ada_id: Ada ID'si
            x: Yeni X koordinatı (None ise mevcut konumu döndürür)
            y: Yeni Y koordinatı (Z ekseni, None ise mevcut konumu döndürür)
        
        Returns:
            tuple: (x, y) koordinatları veya None
        
        Örnek:
            # Ada konumunu değiştir
            senaryo.Ada(0, 50, 60)
            
            # Ada konumunu al
            konum = senaryo.Ada(0)  # (x, y) tuple döner
        """
        if not self.aktif or not self.ortam:
            raise ValueError("Önce senaryo.uret() çağırın.")
        
        # Ortam gerçek Ortam sınıfı ise direkt kullan
        if hasattr(self.ortam, 'Ada'):
            return self.ortam.Ada(ada_id, x, y)
        else:
            # Minimal ortam için fallback
            if not hasattr(self.ortam, 'island_positions'):
                self.ortam.island_positions = []
            while len(self.ortam.island_positions) <= ada_id:
                self.ortam.island_positions.append((0, 0, 50.0))
            
            if x is not None and y is not None:
                radius = self.ortam.island_positions[ada_id][2] if len(self.ortam.island_positions[ada_id]) > 2 else 50.0
                self.ortam.island_positions[ada_id] = (x, y, radius)
                print(f"✅ Ada-{ada_id} pozisyonu güncellendi: ({x}, {y})")
                return (x, y)
            else:
                if ada_id < len(self.ortam.island_positions):
                    ada_pos = self.ortam.island_positions[ada_id]
                    return (ada_pos[0], ada_pos[1])
                return None
    
    def ROV(self, rov_id, x=None, y=None, z=None):
        """
        ROV pozisyonunu değiştirir veya konumunu döndürür.
        Simulasyon.py'deki Ortam.ROV() metodunu kullanır.
        
        Args:
            rov_id: ROV ID'si
            x: Yeni X koordinatı (None ise mevcut konumu döndürür)
            y: Yeni Y koordinatı (derinlik, None ise mevcut konumu döndürür)
            z: Yeni Z koordinatı (None ise mevcut konumu döndürür)
        
        Returns:
            tuple: (x, y, z) koordinatları veya None
        
        Örnek:
            # ROV konumunu değiştir
            senaryo.ROV(0, 10, -5, 20)
            
            # ROV konumunu al
            konum = senaryo.ROV(0)  # (x, y, z) tuple döner
        """
        if not self.aktif or not self.ortam:
            raise ValueError("Önce senaryo.uret() çağırın.")
        
        # Ortam gerçek Ortam sınıfı ise direkt kullan
        if hasattr(self.ortam, 'ROV'):
            return self.ortam.ROV(rov_id, x, y, z)
        else:
            # Minimal ortam için fallback
            if rov_id >= len(self.ortam.rovs):
                print(f"⚠️ ROV ID {rov_id} bulunamadı.")
                return None
            
            rov = self.ortam.rovs[rov_id]
            
            if x is not None and y is not None and z is not None:
                pozisyon = (x, y, z)
                if hasattr(rov, 'position'):
                    if hasattr(rov.position, 'x'):
                        rov.position.x = pozisyon[0]
                        rov.position.y = pozisyon[1]
                        rov.position.z = pozisyon[2]
                if hasattr(rov, 'x'):
                    rov.x = pozisyon[0]
                    rov.y = pozisyon[1]
                    rov.z = pozisyon[2]
                print(f"✅ ROV-{rov_id} pozisyonu güncellendi: ({x}, {y}, {z})")
                return (x, y, z)
            else:
                if hasattr(rov, 'position') and hasattr(rov.position, 'x'):
                    return (rov.position.x, rov.position.y, rov.position.z)
                elif hasattr(rov, 'x'):
                    return (rov.x, rov.y, rov.z)
                return None
    
    def guncelle(self, delta_time=0.016):
        """
        Senaryo ortamını bir adım günceller (simülasyon adımı).
        
        Args:
            delta_time (float): Geçen süre (saniye, varsayılan: 0.016 = ~60 FPS)
        
        Örnek:
            senaryo.guncelle(0.016)  # 1 frame güncelle
        """
        if not self.aktif:
            print("⚠️ Senaryo aktif değil. Önce senaryo.uret() çağırın.")
            return
        
        # Ursina time.dt'yi ayarla (eğer varsa)
        try:
            import time as ursina_time
            if hasattr(ursina_time, 'dt'):
                ursina_time.dt = delta_time
        except:
            pass
        
        # ROV'ları güncelle (sadece update metodu varsa)
        for rov in self.ortam.rovs:
            try:
                if hasattr(rov, 'update') and callable(getattr(rov, 'update', None)):
                    rov.update()
                else:
                    # Minimal ROV için basit fizik güncellemesi
                    if hasattr(rov, 'velocity'):
                        v = rov.velocity
                        # Hızı pozisyona uygula
                        if hasattr(v, 'x'):
                            # Pozisyonu güncelle (x, y, z attribute'ları üzerinden)
                            rov.x += v.x * delta_time
                            rov.y += v.y * delta_time
                            rov.z += v.z * delta_time
                            
                            # Position objesini de güncelle (varsa)
                            if hasattr(rov, 'position'):
                                if hasattr(rov.position, 'x'):
                                    rov.position.x = rov.x
                                    rov.position.y = rov.y
                                    rov.position.z = rov.z
                            
                            # Sürtünme (basit)
                            v.x *= 0.95
                            v.y *= 0.95
                            v.z *= 0.95
            except Exception as e:
                # Update hatası görmezden gel (headless mod)
                pass
        
        # Filo sistemini güncelle (GAT kodları olmadan, sadece fizik)
        if self.filo:
            # GAT kodları olmadan güncelle (varsayılan: 0 = OK)
            tahminler = np.zeros(len(self.ortam.rovs), dtype=int)
            self.filo.guncelle_hepsi(tahminler)
    
    def get(self, rov_id, veri_tipi):
        """
        ROV sensör verilerine erişim (Filo üzerinden).
        
        Args:
            rov_id (int): ROV ID'si
            veri_tipi (str): Veri tipi
                - "batarya": Batarya seviyesi (0-1)
                - "gps": GPS koordinatları [x, y, z]
                - "hiz": Hız vektörü [vx, vy, vz]
                - "sonar": Sonar mesafesi
                - "rol": ROV rolü (0=takipçi, 1=lider)
                - "engel_mesafesi": Engel tespit mesafesi
                - "iletisim_menzili": İletişim menzili
        
        Returns:
            Veri tipine göre değer veya None
        
        Örnek:
            batarya = senaryo.get(0, "batarya")
            gps = senaryo.get(0, "gps")
            sonar = senaryo.get(0, "sonar")
        """
        if not self.aktif:
            print("⚠️ Senaryo aktif değil. Önce senaryo.uret() çağırın.")
            return None
        
        if not self.filo:
            print("⚠️ Filo sistemi kurulmamış.")
            return None
        
        # Filo üzerinden veri al
        veri = self.filo.get(rov_id, veri_tipi)
        
        # Eğer filo None döndürdüyse, direkt ROV'tan al (fallback)
        if veri is None and rov_id < len(self.ortam.rovs):
            rov = self.ortam.rovs[rov_id]
            if hasattr(rov, 'get'):
                veri = rov.get(veri_tipi)
        
        return veri
    
    def set(self, rov_id, ayar_adi, deger):
        """
        ROV ayarlarını değiştirir.
        
        Args:
            rov_id (int): ROV ID'si
            ayar_adi (str): Ayar adı
            deger: Ayar değeri
        
        Örnek:
            senaryo.set(0, "engel_mesafesi", 25.0)
        """
        if not self.aktif:
            print("⚠️ Senaryo aktif değil.")
            return
        
        if self.filo:
            self.filo.set(rov_id, ayar_adi, deger)
    
    def git(self, rov_id, x, z, y=None, ai=True):
        """
        ROV'a hedef atar.
        
        Args:
            rov_id (int): ROV ID'si
            x (float): X koordinatı
            z (float): Z koordinatı
            y (float, optional): Y koordinatı (derinlik)
            ai (bool): AI aktif mi?
        
        Örnek:
            senaryo.git(0, 50, 60, -10)  # ROV-0'a hedef atar
        """
        if not self.aktif:
            print("⚠️ Senaryo aktif değil.")
            return
        
        if self.filo:
            self.filo.git(rov_id, x, z, y, ai)
    
    def temizle(self):
        """
        Senaryo ortamını temizler ve kaynakları serbest bırakır.
        """
        if self.ortam:
            # ROV'ları temizle
            for rov in self.ortam.rovs:
                if hasattr(rov, 'destroy'):
                    rov.destroy()
            self.ortam.rovs = []
            
            # Engelleri temizle
            for engel in self.ortam.engeller:
                if hasattr(engel, 'destroy'):
                    engel.destroy()
            self.ortam.engeller = []
        
        self.filo = None
        self.ortam = None
        self.aktif = False
        
        # Önbelleği temizle
        self._cache_n_rovs = None
        self._cache_n_engels = None
        self._cache_havuz_genisligi = None
        
        print("✅ Senaryo temizlendi")


# Global fonksiyonlar (kolay kullanım için)
def uret(n_rovs=None, n_engels=None, havuz_genisligi=None, **kwargs):
    """
    Senaryo oluşturur (global fonksiyon).
    
    Args:
        n_rovs (int, optional): ROV sayısı. None ise önbellekten kullanılır veya varsayılan: 3
        n_engels (int, optional): Engel sayısı. None ise önbellekten kullanılır veya varsayılan: 15
        havuz_genisligi (float, optional): Havuz genişliği. None ise önbellekten kullanılır veya varsayılan: 200
        **kwargs: Diğer parametreler (engel_tipleri, baslangic_pozisyonlari, vb.)
    
    Returns:
        Senaryo: Senaryo instance'ı
    
    Örnek:
        # İlk çağrı - yeni ortam oluştur
        senaryo.uret(n_rovs=4, n_engels=20, havuz_genisligi=200)
        
        # Sonraki çağrılar - parametresiz, önbellekten kullanır
        senaryo.uret()  # Aynı parametrelerle farklı pozisyonlar
    """
    global _senaryo_instance
    if _senaryo_instance is None:
        _senaryo_instance = Senaryo()
    return _senaryo_instance.uret(n_rovs=n_rovs, n_engels=n_engels, 
                                   havuz_genisligi=havuz_genisligi, **kwargs)


# Global instance (kolay erişim için)
def _get_instance():
    """Global senaryo instance'ını döndürür."""
    global _senaryo_instance
    if _senaryo_instance is None:
        _senaryo_instance = Senaryo()
    return _senaryo_instance


# Module-level functions (kolay erişim için)
def get(rov_id, veri_tipi):
    """ROV verisine erişim."""
    instance = _get_instance()
    return instance.get(rov_id, veri_tipi) if instance.aktif else None


def set(rov_id, ayar_adi, deger):
    """ROV ayarını değiştirir."""
    instance = _get_instance()
    if instance.aktif:
        instance.set(rov_id, ayar_adi, deger)


def git(rov_id, x, z, y=None, ai=True):
    """ROV'a hedef atar."""
    instance = _get_instance()
    if instance.aktif:
        instance.git(rov_id, x, z, y, ai)


def guncelle(delta_time=0.016):
    """Senaryo ortamını günceller."""
    instance = _get_instance()
    if instance.aktif:
        instance.guncelle(delta_time)


def temizle():
    """Senaryo ortamını temizler."""
    global _senaryo_instance
    if _senaryo_instance:
        _senaryo_instance.temizle()
        _senaryo_instance = None


# Module-level filo erişimi için __getattr__ kullan
def __getattr__(name):
    """Module-level attribute erişimi (senaryo.filo, senaryo.Ada, senaryo.ROV için)."""
    if name == 'filo':
        instance = _get_instance()
        return instance.filo if instance.aktif else None
    elif name == 'Ada':
        instance = _get_instance()
        if instance.aktif:
            return lambda ada_id, x=None, y=None: instance.Ada(ada_id, x, y)
        return None
    elif name == 'ROV':
        instance = _get_instance()
        if instance.aktif:
            return lambda rov_id, x=None, y=None, z=None: instance.ROV(rov_id, x, y, z)
        return None
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
