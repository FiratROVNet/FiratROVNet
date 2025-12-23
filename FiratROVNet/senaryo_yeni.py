"""
Senaryo Modülü - Geliştirilmiş Versiyon
Önbellek mekanizması ve Ada/ROV pozisyon yönetimi ile
"""

import os
import sys
import random
import numpy as np

# Ursina'yı headless modda başlat
os.environ['URSINA_HEADLESS'] = '1'

from ursina import *
from FiratROVNet.simulasyon import Ortam, ROV, Ada, ROV_Pozisyon
from FiratROVNet.gnc import Filo
from FiratROVNet.config import SensorAyarlari, ModemAyarlari

# Global senaryo instance
_senaryo_instance = None


class Senaryo:
    """
    Senaryo üretim sınıfı - Headless simülasyon ortamı oluşturur.
    Önbellek mekanizması ile hızlı senaryo üretimi sağlar.
    """
    
    def __init__(self):
        self.app = None
        self.filo = None
        self.ortam = None  # Gerçek Ortam sınıfı
        self.aktif = False
        
        # Önbellek mekanizması
        self._cache_params = None
        self._cache_n_rovs = None
        self._cache_n_engels = None
        self._cache_havuz_genisligi = None
        
    def _cakisma_kontrolu(self, x, z, y=None, tip='rov', min_mesafe=15.0):
        """
        Çakışma kontrolü yapar (ada-ada, rov-rov, ada-rov).
        
        Args:
            x, z: 2D pozisyon (X ve Z koordinatları)
            y: Y koordinatı (derinlik, optional)
            tip: 'rov' veya 'ada'
            min_mesafe: Minimum mesafe
        
        Returns:
            bool: True ise çakışma yok, False ise çakışma var
        """
        if not self.ortam:
            return True
        
        # Ada çakışma kontrolü
        if hasattr(self.ortam, 'island_positions') and self.ortam.island_positions:
            for ada_pos in self.ortam.island_positions:
                ada_x, ada_z, ada_radius = ada_pos[0], ada_pos[1], ada_pos[2] if len(ada_pos) > 2 else 30.0
                mesafe = np.sqrt((x - ada_x)**2 + (z - ada_z)**2)
                if mesafe < (ada_radius + min_mesafe):
                    return False
        
        # ROV çakışma kontrolü
        if hasattr(self.ortam, 'rovs') and self.ortam.rovs:
            for rov in self.ortam.rovs:
                if hasattr(rov, 'position'):
                    rov_pos = rov.position
                    rov_x = rov_pos.x if hasattr(rov_pos, 'x') else getattr(rov, 'x', 0)
                    rov_z = rov_pos.z if hasattr(rov_pos, 'z') else getattr(rov, 'z', 0)
                else:
                    rov_x = getattr(rov, 'x', 0)
                    rov_z = getattr(rov, 'z', 0)
                
                mesafe = np.sqrt((x - rov_x)**2 + (z - rov_z)**2)
                if mesafe < min_mesafe:
                    return False
        
        return True
    
    def _güvenli_pozisyon_bul(self, tip='rov', max_attempts=100):
        """
        Çakışma olmayan güvenli pozisyon bulur.
        
        Args:
            tip: 'rov' veya 'ada'
            max_attempts: Maksimum deneme sayısı
        
        Returns:
            (x, z): Güvenli pozisyon veya None
        """
        if not self.ortam:
            return None
        
        havuz_genisligi = getattr(self.ortam, 'havuz_genisligi', 200)
        havuz_sinir = havuz_genisligi
        
        min_mesafe = 15.0 if tip == 'rov' else 50.0  # Ada için daha büyük mesafe
        
        for _ in range(max_attempts):
            x = random.uniform(-havuz_sinir + 20, havuz_sinir - 20)
            z = random.uniform(-havuz_sinir + 20, havuz_sinir - 20)
            
            if self._cakisma_kontrolu(x, z, tip=tip, min_mesafe=min_mesafe):
                return (x, z)
        
        return None
    
    def uret(self, n_rovs=None, n_engels=None, havuz_genisligi=None, 
             engel_tipleri=None, baslangic_pozisyonlari=None,
             modem_ayarlari=None, sensor_ayarlari=None):
        """
        Senaryo ortamı oluşturur veya mevcut ortamı günceller.
        
        Args:
            n_rovs (int, optional): ROV sayısı. None ise önbellekten kullanılır
            n_engels (int, optional): Engel sayısı. None ise önbellekten kullanılır
            havuz_genisligi (float, optional): Havuz genişliği. None ise önbellekten kullanılır
            engel_tipleri (list, optional): Engel tipleri
            baslangic_pozisyonlari (dict, optional): ROV başlangıç pozisyonları
            modem_ayarlari (dict, optional): Modem ayarları
            sensor_ayarlari (dict, optional): Sensör ayarları
        
        Returns:
            dict: {
                'rov_sayisi': int,
                'engel_sayisi': int,
                'havuz_boyutu': float
            }
        
        Örnek:
            # İlk çağrı - yeni ortam oluştur
            sonuc = senaryo.uret(n_rovs=4, n_engels=20, havuz_genisligi=200)
            
            # Sonraki çağrılar - sadece pozisyonları değiştir
            sonuc = senaryo.uret()  # Aynı parametrelerle farklı pozisyonlar
        """
        # Parametre kontrolü - yeni parametreler varsa önbelleği sıfırla
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
            # Ursina'yı başlat
            if self.app is None:
                os.environ['URSINA_HEADLESS'] = '1'
                try:
                    self.app = Ursina(
                        vsync=False,
                        development_mode=False,
                        show_ursina_splash=False,
                        borderless=True,
                        title="FıratROVNet Senaryo Üretimi (Headless)"
                    )
                    window.fullscreen = False
                    window.show = False
                    window.fps_counter.enabled = False
                except Exception as e:
                    print(f"⚠️ Ursina başlatılamadı: {e}")
                    self.app = None
            
            # Gerçek Ortam sınıfını kullan (Ursina app ile)
            if self.app is None:
                raise RuntimeError("Ursina app başlatılamadı")
            
            # Ortam sınıfı Ursina app'i kullanır
            self.ortam = Ortam()
            # sim_olustur metodunu çağır
            self.ortam.sim_olustur(n_rovs=n_rovs, n_engels=n_engels, havuz_genisligi=havuz_genisligi)
            
            # Ada ve ROV_Pozisyon için ortam referansını ayarla
            Ada.set_ortam(self.ortam)
            ROV_Pozisyon.set_ortam(self.ortam)
            
            # Filo sistemini kur
            self.filo = Filo()
            self.filo.otomatik_kurulum(
                rovs=self.ortam.rovs,
                lider_id=0,
                modem_ayarlari=modem_ayarlari,
                baslangic_hedefleri={},
                sensor_ayarlari=sensor_ayarlari
            )
            
            self.ortam.filo = self.filo
            self.aktif = True
            
            print(f"✅ Yeni senaryo oluşturuldu: {n_rovs} ROV, {n_engels} Engel, Havuz: {havuz_genisligi}x{havuz_genisligi}")
        else:
            # Mevcut ortamı güncelle - sadece pozisyonları değiştir
            # Ada ve ROV_Pozisyon için ortam referansını ayarla
            Ada.set_ortam(self.ortam)
            ROV_Pozisyon.set_ortam(self.ortam)
            
            # Adaları rastgele yerlere taşı
            if hasattr(self.ortam, 'island_positions') and self.ortam.island_positions:
                for ada_id in range(len(self.ortam.island_positions)):
                    güvenli_pos = self._güvenli_pozisyon_bul(tip='ada', max_attempts=50)
                    if güvenli_pos:
                        Ada(ada_id, güvenli_pos[0], güvenli_pos[1], ortam_ref=self.ortam)
            
            # ROV'ları rastgele yerlere taşı
            if hasattr(self.ortam, 'rovs') and self.ortam.rovs:
                for rov_id in range(len(self.ortam.rovs)):
                    güvenli_pos = self._güvenli_pozisyon_bul(tip='rov', max_attempts=50)
                    if güvenli_pos:
                        y_pos = -2 if getattr(self.ortam.rovs[rov_id], 'role', 0) != 1 else 0
                        ROV_Pozisyon(rov_id, güvenli_pos[0], y_pos, güvenli_pos[1], ortam_ref=self.ortam)
            
            ada_sayisi = len(self.ortam.island_positions) if hasattr(self.ortam, 'island_positions') and self.ortam.island_positions else 0
            print(f"🔄 Senaryo pozisyonları güncellendi: {n_rovs} ROV, {ada_sayisi} Ada")
        
        # Dönüş değeri
        return {
            'rov_sayisi': n_rovs,
            'engel_sayisi': n_engels,
            'havuz_boyutu': havuz_genisligi
        }
    
    def Ada(self, ada_id, x=None, y=None):
        """
        Ada pozisyonunu değiştirir (wrapper).
        
        Args:
            ada_id: Ada ID'si
            x: Yeni X koordinatı
            y: Yeni Y koordinatı (Z ekseni)
        
        Returns:
            Ada: Ada instance'ı
        """
        if not self.aktif or not self.ortam:
            raise ValueError("Önce senaryo.uret() çağırın.")
        return Ada(ada_id, x, y, ortam_ref=self.ortam)
    
    def ROV(self, rov_id, x=None, y=None, z=None):
        """
        ROV pozisyonunu değiştirir (wrapper).
        
        Args:
            rov_id: ROV ID'si
            x: Yeni X koordinatı
            y: Yeni Y koordinatı (derinlik)
            z: Yeni Z koordinatı
        
        Returns:
            ROV_Pozisyon: ROV_Pozisyon instance'ı
        """
        if not self.aktif or not self.ortam:
            raise ValueError("Önce senaryo.uret() çağırın.")
        return ROV_Pozisyon(rov_id, x, y, z, ortam_ref=self.ortam)
    
    def get(self, rov_id, veri_tipi):
        """ROV verisine erişim."""
        if not self.aktif or not self.filo:
            return None
        return self.filo.get(rov_id, veri_tipi)
    
    def set(self, rov_id, ayar_adi, deger):
        """ROV ayarını değiştirir."""
        if not self.aktif or not self.filo:
            return
        self.filo.set(rov_id, ayar_adi, deger)
    
    def git(self, rov_id, x, z, y=None, ai=True):
        """ROV'a hedef atar."""
        if not self.aktif or not self.filo:
            return
        self.filo.git(rov_id, x, z, y, ai)
    
    def guncelle(self, delta_time=0.016):
        """Senaryo ortamını günceller."""
        if not self.aktif:
            return
        # Ortam güncellemesi (Ursina update döngüsü)
        if self.ortam and hasattr(self.ortam, 'rovs'):
            for rov in self.ortam.rovs:
                if hasattr(rov, 'update'):
                    try:
                        rov.update()
                    except:
                        pass
    
    def temizle(self):
        """Senaryo ortamını temizler."""
        if self.ortam:
            # Ortam temizleme (Ursina entity'leri destroy edilir)
            pass
        self.filo = None
        self.ortam = None
        self.aktif = False
        self._cache_params = None
        print("✅ Senaryo temizlendi")


# Global fonksiyonlar
def uret(n_rovs=None, n_engels=None, havuz_genisligi=None, **kwargs):
    """Senaryo oluşturur (global fonksiyon)."""
    global _senaryo_instance
    if _senaryo_instance is None:
        _senaryo_instance = Senaryo()
    return _senaryo_instance.uret(n_rovs=n_rovs, n_engels=n_engels, 
                                   havuz_genisligi=havuz_genisligi, **kwargs)


def _get_instance():
    """Global senaryo instance'ını döndürür."""
    global _senaryo_instance
    if _senaryo_instance is None:
        _senaryo_instance = Senaryo()
    return _senaryo_instance


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


# Module-level attribute erişimi
def __getattr__(name):
    """Module-level attribute erişimi."""
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


