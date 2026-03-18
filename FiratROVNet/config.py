import numpy as np
import math

# ==========================================
# FİZİK VE FİLTRE SINIF

# ==========================================
class Hidrodinamik:
    SU_YOGUNLUGU = 1000.0
    YER_CEKIMI = 9.81
    KUTLE = 8.0
    HACIM = 0.0122
    MAX_ITME_KUVVETI = 50.0
    DRAG_KATSAYISI_CD = 0.4
    ON_YUZEY_ALANI = 0.02
    LINEAR_DAMPING = 0  # Su direnci nedeniyle doğrusal sönümleme
    ANGULAR_DAMPING = 0  # Su direnci nedeniyle açısal sönümleme

class BasitKalmanFiltresi:
    def __init__(self, R=0.1, Q=0.1, baslangic_degeri=0.0):
        self.R = R
        self.Q = Q
        self.P = 1.0
        self.x = baslangic_degeri

    def guncelle(self, olcum):
        x_pred = self.x
        p_pred = self.P + self.Q
        K = p_pred / (p_pred + self.R)
        self.x = x_pred + K * (olcum - x_pred)
        self.P = (1 - K) * p_pred
        return self.x

# ==========================================
# SİSTEM AYARLARI
# ==========================================
class Ayarlar:
    """
    Sistemin loglarını canlı olarak açıp kapatmak için kontrol paneli.
    True = Mesajları Göster
    False = Sustur
    """
    goster_modem = False   # İletişim mesajları (Gönderildi/Alındı/Hata)
    goster_gnc = False     # Navigasyon mesajları (Hedef alındı/GPS güncellendi)
    goster_sistem = True  # Genel sistem mesajları

# Bu nesneyi diğer dosyalardan çağıracağız
cfg = Ayarlar()


# ==========================================
# GAT VE SENSÖR AYARLARI (Eğitim ve Kullanım İçin Ortak)
# ==========================================
class GATLimitleri:
    """
    GAT kodları için limitler - Eğitim ve kullanımda tutarlı olmalı!
    Bu limitler gnc.py ve simulasyon.py (kullanım) tarafından kullanılır.
    """
    # GAT Kod Limitleri (metre cinsinden)
    CARPISMA = 15    # Kod 2: Çarpışma riski mesafesi
    ENGEL = 20.0      # Kod 1: Engel yakınlığı mesafesi
    KOPMA = 50.0      # Kod 3: Bağlantı kopması mesafesi
    UZAK = 80.0       # Kod 5: Liderden uzaklık mesafesi
    ILETISIM_MENZILI = 100.0  # Sonar iletişim maksimum menzili (metre)
    
    @classmethod
    def dict(cls):
        """Dictionary formatında limitleri döndürür."""
        return {
            'CARPISMA': cls.CARPISMA,
            'ENGEL': cls.ENGEL,
            'KOPMA': cls.KOPMA,
            'UZAK': cls.UZAK,
            'ILETISIM_MENZILI': cls.ILETISIM_MENZILI
        }


class SensorAyarlari:
    """
    Sensör ayarları - Eğitim ve kullanımda tutarlı olmalı!
    Bu ayarlar GAT limitleri ile uyumlu olmalı:
    - engel_mesafesi: Sonar ve lidar maksimum algılama menzili (metre). Varsayılan GATLimitleri.ENGEL = 10.0 m.
    - iletisim_menzili >= GATLimitleri.KOPMA (35.0)
    - kacinma_mesafesi <= GATLimitleri.CARPISMA (5.0)
    """
    # Lider ROV için varsayılan ayarlar
    LIDER = {
        'engel_mesafesi': GATLimitleri.ENGEL,       # GATLimitleri.ENGEL ile aynı
        'iletisim_menzili': GATLimitleri.UZAK,     # GATLimitleri.KOPMA ile aynı
        'min_pil_uyarisi': 0.2,       # Normalize edilmiş (0.0-1.0)
        'kacinma_mesafesi': GATLimitleri.CARPISMA,       # GATLimitleri.CARPISMA ile aynı
    }
    
    # Takipçi ROV için varsayılan ayarlar
    TAKIPCI = {
        'engel_mesafesi': GATLimitleri.ENGEL,       # GATLimitleri.ENGEL ile aynı
        'iletisim_menzili': GATLimitleri.UZAK,     # GATLimitleri.KOPMA ile aynı
        'min_pil_uyarisi': 0.15,      # Normalize edilmiş (0.0-1.0)
        'kacinma_mesafesi': GATLimitleri.CARPISMA,       # GATLimitleri.CARPISMA ile aynı
    }
    
    # Genel varsayılan ayarlar (fallback için)
    VARSAYILAN = {
        'engel_mesafesi': GATLimitleri.ENGEL,
        'iletisim_menzili': GATLimitleri.UZAK,
        'min_pil_uyarisi': 0.2,
        'kacinma_mesafesi': GATLimitleri.CARPISMA
    }


class HareketAyarlari:
    """
    Hareket ve formasyon ayarları - GAT limitlerine bağlı, sadece kullanılan değerler.
    """
    # Formasyon (GAT.ENGEL ile uyumlu tek aralık)
    FORMASYON_MIN_ARALIK = GATLimitleri.CARPISMA
    FORMASYON_VARSAYILAN_ARALIK = GATLimitleri.CARPISMA
    FORMASYON_OFFSET = 60

    YAKIN_MESAFE_ESIGI = GATLimitleri.KOPMA * 0.375

    # Hedef görselleştirme
    HEDEF_X_BOYUTU = 10.0
    HEDEF_KALINLIK = 1.0

    # Senaryo: Adalardan min mesafe (GAT.ENGEL * 3)
    RANDOM_HEDEF_MIN_MESAFE_ADA = GATLimitleri.ENGEL * 3


class FizikSabitleri:
    """
    Fizik simülasyonu için sabitler - Sadece kullanılan değerler.
    """
    BATARYA_SOMURME_KATSAYISI = 0.001    # Batarya sömürme katsayısı (sabit azalma)
    BATARYA_HIZ_KATSAYISI = 0.0005       # Hıza göre batarya azalma katsayısı (hız*katsayı*dt)


class ROVModelleri:
    """
    ROV 3D model seçenekleri. sim_olustur(rov_model='bluerov2') veya rov_model='submarine' ile seçilebilir.
    """
    # Mevcut modeller: 'bluerov2' (varsayılan), 'submarine'
    VARSAYILAN = 'bluerov2'
    MODELLER = {
        'bluerov2': {
            'path': 'Models-3D/BlueRov2/Bluerov2.glb',
            'scale': (0.025, 0.025, 0.025),  # 100x küçültme + 2x büyütme
        },
        'submarine': {
            'path': 'Models-3D/water/my_models/submarine/submarine1.fbx',
            'scale': (0.009, 0.009, 0.009),  # FBX 1000x küçültme + %25 büyütme bir rovun boyutu artık 1000*0.009=9 mtre boyunda
        },
    }


class HavuzAyarlari:
    """
    Havuz (pool) boyutu ve sınırları.
    """
    HAVUZ_TAM_GENISLIK = 200  # Havuz genişliği (metre)