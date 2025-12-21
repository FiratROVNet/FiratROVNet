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
import networkx as nx

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


# ============================================================
# GAT EĞİTİMİ İÇİN VERİ ÜRETİMİ
# ============================================================
def veri_uret_gat(n_rovs=None, n_engels=None, havuz_genisligi=200):
    """
    Senaryo ortamından GAT eğitimi için PyTorch Geometric Data formatında veri üretir.
    
    Args:
        n_rovs (int, optional): ROV sayısı (None ise rastgele 4-10 arası)
        n_engels (int, optional): Engel sayısı (None ise rastgele 10-30 arası)
        havuz_genisligi (float): Havuz genişliği
    
    Returns:
        torch_geometric.data.Data: GAT modeli için hazırlanmış veri
    """
    from torch_geometric.data import Data
    import torch
    import networkx as nx
    
    # Senaryo oluştur
    if n_rovs is None:
        n_rovs = np.random.randint(4, 10)
    if n_engels is None:
        n_engels = np.random.randint(10, 30)
    
    senaryo = uret(n_rovs=n_rovs, n_engels=n_engels, havuz_genisligi=havuz_genisligi)
    
    # Birkaç adım simülasyon çalıştır (dinamik veri için)
    for _ in range(np.random.randint(5, 20)):
        guncelle(delta_time=0.016)
    
    # Veri toplama
    rovs = senaryo.ortam.rovs
    engeller = senaryo.ortam.engeller
    n = len(rovs)
    
    if n == 0:
        # Fallback: Eğer ROV yoksa minimal veri döndür
        return veri_uret_gat(n_rovs=4, n_engels=10, havuz_genisligi=havuz_genisligi)
    
    x = torch.zeros((n, 7), dtype=torch.float)
    sources, targets = [], []
    danger_map = {}
    
    # Limitler
    L = {'LEADER': 60.0, 'DISCONNECT': 35.0, 'OBSTACLE': 20.0, 'COLLISION': 8.0}
    
    # Pozisyonları topla
    positions = []
    for rov in rovs:
        if hasattr(rov, 'position'):
            pos = rov.position
            if hasattr(pos, 'x'):
                positions.append([pos.x, pos.y, pos.z])
            elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                positions.append([pos[0], pos[1], pos[2]])
            else:
                positions.append([getattr(rov, 'x', 0), getattr(rov, 'y', -2), getattr(rov, 'z', 0)])
        else:
            positions.append([getattr(rov, 'x', 0), getattr(rov, 'y', -2), getattr(rov, 'z', 0)])
    
    # GAT girdilerini oluştur
    for i in range(n):
        code = 0
        pos_i = np.array(positions[i][:2])  # X, Z (2D düzlem)
        
        # Liderden uzak mı?
        if i != 0 and len(positions) > 0:
            pos_0 = np.array(positions[0][:2])
            if np.linalg.norm(pos_i - pos_0) > L['LEADER']:
                code = 5
        
        # Diğer ROV'lardan uzak mı? (Kopma)
        dists = []
        for j in range(n):
            if i != j:
                pos_j = np.array(positions[j][:2])
                dist = np.linalg.norm(pos_i - pos_j)
                dists.append(dist)
                if dist < L['DISCONNECT']:
                    sources.append(i)
                    targets.append(j)
        
        if dists and min(dists) > L['DISCONNECT']:
            code = 3
        
        # Engel kontrolü
        min_engel_dist = 999.0
        for engel in engeller:
            if hasattr(engel, 'position'):
                engel_pos = engel.position
                if hasattr(engel_pos, 'x'):
                    engel_x, engel_z = engel_pos.x, engel_pos.z
                elif isinstance(engel_pos, (list, tuple)) and len(engel_pos) >= 2:
                    engel_x, engel_z = engel_pos[0], engel_pos[2] if len(engel_pos) > 2 else engel_pos[1]
                else:
                    continue
                
                dist = np.linalg.norm(pos_i - np.array([engel_x, engel_z]))
                
                # Engel boyutunu hesaba kat
                if hasattr(engel, 'scale_x'):
                    scale_avg = (engel.scale_x + (engel.scale_z if hasattr(engel, 'scale_z') else engel.scale_x)) / 2
                    dist = max(0, dist - scale_avg / 2)
                
                if dist < min_engel_dist:
                    min_engel_dist = dist
        
        if min_engel_dist < L['OBSTACLE']:
            code = 1
        
        # Çarpışma kontrolü
        for j in range(n):
            if i != j:
                pos_j = np.array(positions[j][:2])
                if np.linalg.norm(pos_i - pos_j) < L['COLLISION']:
                    code = 2
                    break
        
        # GAT özellik vektörü
        x[i][0] = code / 5.0  # GAT kodu (normalize)
        
        # Batarya
        if hasattr(rovs[i], 'battery'):
            x[i][1] = float(rovs[i].battery) if rovs[i].battery <= 1.0 else rovs[i].battery / 100.0
        elif hasattr(rovs[i], 'get'):
            bat = rovs[i].get('batarya')
            x[i][1] = float(bat) if bat is not None else 0.8
        else:
            x[i][1] = np.random.uniform(0.5, 1.0)
        
        # SNR (İletişim kalitesi)
        x[i][2] = 0.9  # Varsayılan
        
        # Derinlik
        if len(positions[i]) > 1:
            x[i][3] = abs(float(positions[i][1])) / 100.0
        else:
            x[i][3] = np.random.uniform(0.0, 1.0)
        
        # Hız (velocity)
        if hasattr(rovs[i], 'velocity'):
            vel = rovs[i].velocity
            if hasattr(vel, 'x'):
                x[i][4] = float(vel.x)
                x[i][5] = float(vel.z) if hasattr(vel, 'z') else 0.0
            else:
                x[i][4] = np.random.uniform(-1, 1)
                x[i][5] = np.random.uniform(-1, 1)
        elif hasattr(rovs[i], 'get'):
            hiz = rovs[i].get('hiz')
            if hiz is not None and len(hiz) >= 2:
                x[i][4] = float(hiz[0])
                x[i][5] = float(hiz[2]) if len(hiz) > 2 else float(hiz[1])
            else:
                x[i][4] = np.random.uniform(-1, 1)
                x[i][5] = np.random.uniform(-1, 1)
        else:
            x[i][4] = np.random.uniform(-1, 1)
            x[i][5] = np.random.uniform(-1, 1)
        
        # Rol (0=takipçi, 1=lider)
        if hasattr(rovs[i], 'role'):
            x[i][6] = float(rovs[i].role)
        elif hasattr(rovs[i], 'get'):
            rol = rovs[i].get('rol')
            x[i][6] = float(rol) if rol is not None else (1.0 if i == 0 else 0.0)
        else:
            x[i][6] = 1.0 if i == 0 else 0.0
        
        if code > 0:
            danger_map[i] = code
    
    # Edge index
    edge_index = torch.tensor([sources, targets], dtype=torch.long) if sources else torch.zeros((2, 0), dtype=torch.long)
    
    # Hedef etiketler (y)
    y = torch.zeros(n, dtype=torch.long)
    G = nx.Graph()
    G.add_nodes_from(range(n))
    if len(sources) > 0:
        G.add_edges_from(zip(sources, targets))
    
    for i in range(n):
        if i in danger_map:
            y[i] = danger_map[i]
        elif i in G and len(danger_map) > 0:
            # Graf üzerinden en yüksek öncelikli tehlikeli duruma bağlan
            priority = {2: 0, 1: 1, 3: 2, 5: 3, 0: 4}
            sorted_dangers = sorted(danger_map.items(), key=lambda k: priority.get(k[1], 10))
            for d_node, d_code in sorted_dangers:
                if nx.has_path(G, i, d_node):
                    y[i] = d_code
                    break
    
    # Senaryoyu temizle (bellek için)
    temizle()
    
    return Data(x=x, edge_index=edge_index, y=y)
