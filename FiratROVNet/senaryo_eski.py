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
        
        # Önbellek mekanizması (aynı parametrelerle tekrar çağrı için)
        self._cache_params = None  # Son kullanılan parametreler
        self._cache_ortam_olusturuldu = False  # Ortam oluşturuldu mu?
        self._cache_n_rovs = None
        self._cache_n_engels = None
        self._cache_havuz_genisligi = None
        
    def uret(self, n_rovs=3, n_engels=15, havuz_genisligi=200, 
             engel_tipleri=None, baslangic_pozisyonlari=None,
             modem_ayarlari=None, sensor_ayarlari=None):
        """
        Senaryo ortamı oluşturur (GUI olmadan).
        
        Args:
            n_rovs (int): ROV sayısı (varsayılan: 3)
            n_engels (int): Engel sayısı (varsayılan: 15)
            havuz_genisligi (float): Havuz genişliği (varsayılan: 200)
            engel_tipleri (list, optional): Engel tipleri listesi
                Örnek: ['kaya', 'kaya', 'agac'] veya None (hepsi kaya)
            baslangic_pozisyonlari (dict, optional): ROV başlangıç pozisyonları
                Örnek: {0: (0, -5, 0), 1: (10, -5, 10)}
            modem_ayarlari (dict, optional): Modem ayarları
            sensor_ayarlari (dict, optional): Sensör ayarları
        
        Returns:
            Senaryo: Kendi instance'ını döndürür (method chaining için)
        
        Örnek:
            senaryo = Senaryo()
            senaryo.uret(n_rovs=4, n_engels=20)
            batarya = senaryo.filo.get(0, "batarya")
        """
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
        
        # Ortam oluştur (headless - minimal)
        # Ortam sınıfı yerine minimal bir ortam objesi oluştur
        self.ortam = type('Ortam', (), {
            'rovs': [],
            'engeller': [],
            'havuz_genisligi': havuz_genisligi,
            'filo': None
        })()
        
        # Havuz genişliğini kaydet
        self.ortam.havuz_genisligi = havuz_genisligi
        
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
        
        # ROV'ları oluştur
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
        
        # Filo sistemini kur
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
        
        print(f"✅ Senaryo oluşturuldu: {n_rovs} ROV, {n_engels} Engel, Havuz: {havuz_genisligi}x{havuz_genisligi}")
        
        return self
    
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
        
        print("✅ Senaryo temizlendi")


# Global fonksiyonlar (kolay kullanım için)
def uret(n_rovs=3, n_engels=15, havuz_genisligi=200, **kwargs):
    """
    Senaryo oluşturur (global fonksiyon).
    
    Args:
        n_rovs (int): ROV sayısı
        n_engels (int): Engel sayısı
        havuz_genisligi (float): Havuz genişliği
        **kwargs: Diğer parametreler (engel_tipleri, baslangic_pozisyonlari, vb.)
    
    Returns:
        Senaryo: Senaryo instance'ı
    
    Örnek:
        senaryo.uret(n_rovs=4, n_engels=20)
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
    """Module-level attribute erişimi (senaryo.filo için)."""
    if name == 'filo':
        instance = _get_instance()
        return instance.filo if instance.aktif else None
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
