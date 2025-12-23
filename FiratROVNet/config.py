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
    Bu limitler ortam.py (eğitim) ve gnc.py/simulasyon.py (kullanım) tarafından kullanılır.
    """
    # GAT Kod Limitleri (metre cinsinden)
    CARPISMA = 8.0    # Kod 2: Çarpışma riski mesafesi
    ENGEL = 20.0      # Kod 1: Engel yakınlığı mesafesi
    KOPMA = 35.0      # Kod 3: Bağlantı kopması mesafesi
    UZAK = 60.0       # Kod 5: Liderden uzaklık mesafesi
    
    @classmethod
    def dict(cls):
        """Dictionary formatında limitleri döndürür."""
        return {
            'CARPISMA': cls.CARPISMA,
            'ENGEL': cls.ENGEL,
            'KOPMA': cls.KOPMA,
            'UZAK': cls.UZAK
        }


class SensorAyarlari:
    """
    Sensör ayarları - Eğitim ve kullanımda tutarlı olmalı!
    Bu ayarlar GAT limitleri ile uyumlu olmalı:
    - engel_mesafesi >= GATLimitleri.ENGEL (20.0)
    - iletisim_menzili >= GATLimitleri.KOPMA (35.0)
    - kacinma_mesafesi <= GATLimitleri.CARPISMA (8.0)
    """
    # Lider ROV için varsayılan ayarlar
    LIDER = {
        'engel_mesafesi': 10.0,      # GATLimitleri.ENGEL ile aynı
        'iletisim_menzili': 50.0,     # GATLimitleri.KOPMA ile aynı
        'min_pil_uyarisi': 0.2,       # Normalize edilmiş (0.0-1.0)
        'kacinma_mesafesi': 5.0       # GATLimitleri.CARPISMA ile aynı
    }
    
    # Takipçi ROV için varsayılan ayarlar
    TAKIPCI = {
        'engel_mesafesi': 10.0,       # GATLimitleri.ENGEL ile aynı
        'iletisim_menzili': 50.0,     # GATLimitleri.KOPMA ile aynı
        'min_pil_uyarisi': 0.15,      # Normalize edilmiş (0.0-1.0)
        'kacinma_mesafesi': 5.0       # GATLimitleri.CARPISMA ile aynı
    }
    
    # Genel varsayılan ayarlar (fallback için)
    VARSAYILAN = {
        'engel_mesafesi': 10.0,
        'iletisim_menzili': 50.0,
        'min_pil_uyarisi': 0.2,
        'kacinma_mesafesi': 5.0
    }


class HareketAyarlari:
    """
    Hareket ve formasyon ayarları - Tüm sistemde kullanılan katsayılar ve mesafeler.
    """
    # Takipçi hareket eşikleri
    HAREKET_ESIGI_KATSAYISI = 0.7      # İletişim menzilinin %70'i (lider uzaklaşma eşiği)
    HISTERESIS_KATSAYISI = 0.9         # Aktif moddan pasif moda geçiş toleransı
    
    # Hedef toleransları (metre)
    HEDEF_TOLERANS_LIDER = 0.5         # Lider için hedef toleransı
    HEDEF_TOLERANS_TAKIPCI = 2.0       # Takipçi için hedef toleransı
    
    # Formasyon ayarları
    FORMASYON_MESAFESI = 15.0          # Varsayılan formasyon mesafesi (metre)
    FORMASYON_VARSAYILAN_ARALIK = 15.0  # Formasyon aralığı (metre)
    
    # Kaçınma ayarları
    KACINMA_MESAFESI_FALLBACK_KATSAYISI = 0.2  # Engel mesafesinin %20'si (fallback)
    MINIMUM_MESAFE_KACINMA = 2.0       # Minimum mesafe (çok yakınsa kaçınma yok)
    YAKIN_MESAFE_ESIGI = 10.0          # Yakın mesafe eşiği (iletişim kopmasını önleme)
    
    # Dikey toleranslar (metre)
    DIKEY_TOLERANS_ENGEL = 10.0         # Engel algılama için dikey tolerans
    DIKEY_TOLERANS_ADA = 5.0            # Ada algılama için dikey tolerans
    
    # Vektör birleştirme katsayıları (0.0-1.0)
    VEKTOR_BIRLESTIRME_NORMAL_KACINMA = 0.5      # Normal durumda kaçınma ağırlığı
    VEKTOR_BIRLESTIRME_TAKIPCI_KACINMA = 0.8      # Takipçi için kaçınma ağırlığı
    VEKTOR_BIRLESTIRME_TAKIPCI_HEDEF = 0.2        # Takipçi için hedef ağırlığı
    
    # Güç ayarları (0.0-1.0)
    GUC_ENGEL = 0.5                    # Engel durumunda motor gücü
    GUC_PASIF_MOD = 0.1                # Pasif modda motor gücü
    GUC_UZAK = 1.5                     # Uzak durumda motor gücü (takipçi)
    
    # Hedef görselleştirme
    HEDEF_X_BOYUTU = 10.0              # Hedef X işareti boyutu
    HEDEF_KALINLIK = 1.0               # Hedef X işareti kalınlığı
    
    # Random hedef oluşturma
    RANDOM_HEDEF_HAVUZ_KATSAYISI = 0.7  # Havuz genişliğinin %70'i içinde
    RANDOM_HEDEF_MIN_MESAFE_ADA = 30.0  # Adalardan minimum mesafe
    
    # Havuz sınırları
    HAVUZ_SINIR_TOLERANS = 0.95        # Havuz sınır toleransı (%95)
    HAVUZ_SINIR_Y_UST = 0.3            # Üst yüzey sınırı
    HAVUZ_SINIR_Y_ALT = -95.0          # Alt derinlik sınırı
    
    # Formasyon şekil katsayıları
    V_FORMASYON_X_KATSAYISI = 0.8      # V formasyonu X ekseni katsayısı
    V_FORMASYON_Z_KATSAYISI = 0.6      # V formasyonu Z ekseni katsayısı
    OK_FORMASYON_X_KATSAYISI = 0.8     # Ok formasyonu X ekseni katsayısı
    OK_FORMASYON_Z_KATSAYISI = 1.5     # Ok formasyonu Z ekseni katsayısı
    
    # Uzaklaşma gücü katsayıları (0.0-1.0)
    UZAKLASMA_GUC_KATSAYISI = 0.3      # Uzaklaşma gücü katsayısı (%30)
    YUMUSAKLIK_CARPANI = 0.2            # Yumuşaklık çarpanı (%20)
    
    # Diğer ayarlar
    PASIF_MOD_MIN_HAREKET_MESAFESI = 5.0  # Pasif modda minimal hareket mesafesi (metre)
    VELOCITY_THRESHOLD = 0.1              # Hız eşiği (normalize edilmiş)
    MOTOR_GUC_KATSAYISI = 0.5              # Manuel hareket güç katsayısı


class ModemAyarlari:
    """
    Modem ayarları - İletişim parametreleri
    """
    # Lider modem için varsayılan ayarlar
    LIDER = {
        'gurultu_orani': 0.05,    # Gürültü oranı (0.0-1.0)
        'kayip_orani': 0.1,       # Paket kayıp oranı (0.0-1.0)
        'gecikme': 0.5            # Gecikme (saniye)
    }
    
    # Takipçi modem için varsayılan ayarlar
    TAKIPCI = {
        'gurultu_orani': 0.1,     # Gürültü oranı (0.0-1.0)
        'kayip_orani': 0.1,       # Paket kayıp oranı (0.0-1.0)
        'gecikme': 0.5            # Gecikme (saniye)
    }

