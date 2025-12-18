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
            self.app = Ursina(
                vsync=False,
                development_mode=False,
                show_ursina_splash=False,
                borderless=True,
                title="FıratROVNet Senaryo Üretimi (Headless)",
                window_type='offscreen'  # Headless mod
            )
            
            # Window'u gizle (headless)
            try:
                window.show = False
            except:
                pass
            
            # FPS counter'ı kapat
            try:
                window.fps_counter.enabled = False
            except:
                pass
        
        # Ortam oluştur (headless)
        self.ortam = Ortam()
        self.ortam.app = self.app
        
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
            engel = Entity(
                model='icosphere',
                color=engel_rengi,
                scale=(s_x, s_y, s_z),
                position=(x, y, z),
                rotation=(random.randint(0, 360), random.randint(0, 360), random.randint(0, 360)),
                collider='mesh',
                unlit=True
            )
            # Headless modda görsel özellikleri kapat
            engel.visible = False
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
            
            # ROV oluştur
            rov = ROV(rov_id=i, position=pozisyon)
            rov.environment_ref = self.ortam
            
            # Headless modda görsel özellikleri kapat
            rov.visible = False
            if hasattr(rov, 'label'):
                rov.label.enabled = False
            
            self.ortam.rovs.append(rov)
        
        # Filo sistemini kur
        self.filo = Filo()
        
        # ROV rollerini ayarla (ilk ROV lider)
        if len(self.ortam.rovs) > 0:
            self.ortam.rovs[0].set("rol", 1)  # İlk ROV lider
            for i in range(1, len(self.ortam.rovs)):
                self.ortam.rovs[i].set("rol", 0)  # Diğerleri takipçi
        
        # Filo otomatik kurulum
        tum_modemler = self.filo.otomatik_kurulum(
            rovs=self.ortam.rovs,
            lider_id=0,
            modem_ayarlari=modem_ayarlari,
            baslangic_hedefleri=None,  # Varsayılan hedefler kullanılacak
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
        
        # Ursina time.dt'yi ayarla
        import time as ursina_time
        ursina_time.dt = delta_time
        
        # ROV'ları güncelle
        for rov in self.ortam.rovs:
            if hasattr(rov, 'update'):
                rov.update()
        
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
        
        return self.filo.get(rov_id, veri_tipi)
    
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
