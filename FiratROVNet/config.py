
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
    YAKIN_MESAFE_ESIGI = 15.0          # Yakın mesafe eşiği (iletişim kopmasını önleme)
    
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


class Formasyon:
    """
    Popüler ve işlevsel 10 formasyon tipini ROV sayısına göre dinamik olarak saklar.
    Her formasyon tipi, lider (index 0) ve takipçiler (index 1+) için pozisyon ofsetlerini döndürür.
    
    Ofsetler (x, y, z) formatında:
    - x: Sağ-sol (pozitif = sağ) - 2D koordinat
    - y: İleri-geri (pozitif = ileri) - 2D koordinat
    - z: Derinlik (pozitif = yukarı, negatif = aşağı) - genelde 0
    """

    def __init__(self,Filo=None):
        self.Filo=Filo
    
    # Formasyon isimleri (10 tip)
    TIPLER = [
        "LINE",      # 0: Çizgi formasyonu (tek sıra)
        "V_SHAPE",   # 1: V şekli (uçan kazlar)
        "DIAMOND",   # 2: Elmas formasyonu
        "SQUARE",    # 3: Kare formasyonu
        "CIRCLE",    # 4: Daire formasyonu
        "ARROW",     # 5: Ok şekli
        "WEDGE",     # 6: Kama şekli
        "ECHELON",   # 7: Eşelon (çapraz sıra)
        "COLUMN",    # 8: Sütun (dikey sıra)
        "SPREAD"     # 9: Yayılım (geniş yayılım)
    ]
    
    
    def pozisyonlar(self, tip, aralik=15.0):
        """
        Belirtilen formasyon tipi ve ROV sayısına göre pozisyon ofsetlerini döndürür.
        
        Args:
            tip (int veya str): Formasyon tipi (0-9 arası indeks veya isim)
            n_rovs (int): Toplam ROV sayısı (lider dahil)
            aralik (float): ROV'lar arası mesafe (varsayılan: 15.0)
        
        Returns:
            list: [(x0, y0, z0), (x1, y1, z1), ...] formatında pozisyon listesi
                  İlk eleman lider (0, 0, 0), diğerleri takipçiler için ofsetler
                  x, y: 2D koordinatlar (yatay düzlem)
                  z: derinlik (dikey)
        
        Örnek:
            pozisyonlar = Formasyon.pozisyonlar("V_SHAPE", n_rovs=5, aralik=20.0)
            # [(0, 0, 0), (-20, -20, 0), (20, -20, 0), (-40, -40, 0), (40, -40, 0)]
        """
        if isinstance(tip, str):
            tip = tip.upper()
            if tip in Formasyon.TIPLER:
                tip_index = Formasyon.TIPLER.index(tip)
            else:
                tip_index = 0  # Varsayılan: LINE
        else:
            tip_index = int(tip) % len(Formasyon.TIPLER)

        # Lider her zaman (0, 0, 0) pozisyonunda
        pozisyonlar = [(0.0, 0.0, 0.0)]
        
        n_rovs=len(self.Filo.sistemler)
        if n_rovs <= 1:
            return pozisyonlar
        
        # Eğer Filo verilmişse, lider pozisyonunu Filo'dan al
        lider_pozisyon_offset = (0.0, 0.0, 0.0)
        if self.Filo is not None:
            try:
                for rov_id in range(n_rovs):
                    rol = self.Filo.get(rov_id, 'rol')
                    if rol == 1:  # Lider bulundu
                        gps = self.Filo.get(rov_id, "gps")
                        if gps is not None:
                            # GPS koordinatlarını ofset olarak kullan
                            # Ursina formatından (x, y, z) -> Bizim format (x, y, z) çevir
                            # Ursina: (x, y, z) -> x: sağ-sol, y: derinlik, z: ileri-geri
                            # Bizim: (x, y, z) -> x: sağ-sol, y: ileri-geri, z: derinlik
                            # Yani: Ursina (x, y, z) -> Bizim (x, z, y)
                            lider_pozisyon_offset = (float(gps[0]), float(gps[2]), float(gps[1]))
                        break
            except:
                pass  # Filo kullanılamazsa varsayılan (0,0,0) kullan
        
        # Lider her zaman (0, 0, 0) pozisyonunda

        
  
        
        # Takipçi sayısı
        n_takipci = n_rovs - 1
        
        # Formasyon tipine göre pozisyonları hesapla
        # Format: (x, y, z) - x,y: 2D koordinatlar, z: derinlik
        if tip_index == 0:  # LINE (Çizgi)
            for i in range(n_takipci):
                pozisyonlar.append((0.0, -aralik * (i + 1), 0.0))  # (x, y, z)
        
        elif tip_index == 1:  # V_SHAPE (V şekli)
            for i in range(n_takipci):
                row = (i + 2) // 2  # Satır numarası (1, 1, 2, 2, 3, 3, ...)
                # İlk takipçi sağda, ikinci solda, üçüncü sağda, dördüncü solda...
                side = 1 if (i + 1) % 2 == 1 else -1  # Tek indeksler sağ, çift indeksler sol
                pozisyonlar.append((side * aralik * row, -aralik * row, 0.0))  # (x, y, z)
        
        elif tip_index == 2:  # DIAMOND (Elmas)
            # Elmas şekli: merkez lider, etrafında eşit dağılım
            import math
            for i in range(n_takipci):
                angle = 2 * math.pi * i / n_takipci
                radius = aralik * (1 + (i // n_takipci))
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                pozisyonlar.append((x, y, 0.0))  # (x, y, z)
        
        elif tip_index == 3:  # SQUARE (Kare)
            # Kare formasyonu: mümkün olduğunca kare şeklinde
            import math
            side_length = int(math.ceil(math.sqrt(n_takipci)))
            idx = 0
            for row in range(side_length):
                for col in range(side_length):
                    if idx < n_takipci:
                        x = (col - side_length / 2 + 0.5) * aralik
                        y = -row * aralik
                        pozisyonlar.append((x, y, 0.0))  # (x, y, z)
                        idx += 1
        
        elif tip_index == 4:  # CIRCLE (Daire)
            # Dairesel formasyon: lider merkezde, takipçiler çember üzerinde
            import math
            for i in range(n_takipci):
                angle = 2 * math.pi * i / n_takipci
                radius = aralik * 1.5
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                pozisyonlar.append((x, y, 0.0))  # (x, y, z)
        
        elif tip_index == 5:  # ARROW (Ok)
            # Ok şekli: lider önde, takipçiler arkada ok şeklinde
            for i in range(n_takipci):
                row = i // 3 + 1
                col = (i % 3) - 1  # -1, 0, 1 (sol, orta, sağ)
                pozisyonlar.append((col * aralik * 0.8, -row * aralik * 1.2, 0.0))  # (x, y, z)
        
        elif tip_index == 6:  # WEDGE (Kama)
            # Kama şekli: V'ye benzer ama daha dar
            for i in range(n_takipci):
                row = (i + 2) // 2  # Satır numarası (1, 1, 2, 2, ...)
                side = 1 if (i + 1) % 2 == 1 else -1  # Tek indeksler sağ, çift indeksler sol
                pozisyonlar.append((side * aralik * row * 0.6, -aralik * row * 0.8, 0.0))  # (x, y, z)
        
        elif tip_index == 7:  # ECHELON (Eşelon)
            # Eşelon: çapraz sıra
            for i in range(n_takipci):
                pozisyonlar.append((aralik * (i + 1) * 0.7, -aralik * (i + 1) * 0.7, 0.0))  # (x, y, z)
        
        elif tip_index == 8:  # COLUMN (Sütun)
            # Sütun: dikey sıra (yan yana)
            for i in range(n_takipci):
                pozisyonlar.append((aralik * (i + 1), 0.0, 0.0))  # (x, y, z)
        
        elif tip_index == 9:  # SPREAD (Yayılım)
            # Yayılım: geniş açılı dağılım
            import math
            for i in range(n_takipci):
                angle = math.pi * (i + 1) / (n_takipci + 1) - math.pi / 2
                radius = aralik * 2.0
                x = radius * math.sin(angle)
                y = -radius * math.cos(angle) * 0.5
                pozisyonlar.append((x, y, 0.0))  # (x, y, z)
        

        lider_x, lider_y, lider_z = lider_pozisyon_offset
        # Tüm pozisyonlara lider pozisyonunu ekle
        # Format: (x, y, z) - x,y: 2D koordinatlar, z: derinlik
        pozisyonlar = [(x + lider_x, y + lider_y, z + lider_z) for x, y, z in pozisyonlar]
        
        return pozisyonlar
    
    @staticmethod
    def tum_formasyonlar(n_rovs, aralik=15.0):
        """
        Tüm formasyon tipleri için pozisyonları döndürür.
        
        Args:
            n_rovs (int): Toplam ROV sayısı
            aralik (float): ROV'lar arası mesafe
        
        Returns:
            dict: {formasyon_ismi: [pozisyon_listesi], ...} formatında sözlük
        
        Örnek:
            tum = Formasyon.tum_formasyonlar(n_rovs=5, aralik=20.0)
            print(tum["V_SHAPE"])  # V_SHAPE formasyonu pozisyonları
        """
        tum = {}
        for tip in Formasyon.TIPLER:
            tum[tip] = Formasyon.pozisyonlar(tip, n_rovs, aralik)
        return tum
    
    @staticmethod
    def formasyon_ismi(tip_index):
        """
        Formasyon indeksine göre isim döndürür.
        
        Args:
            tip_index (int): Formasyon indeksi (0-9)
        
        Returns:
            str: Formasyon ismi
        """
        return Formasyon.TIPLER[tip_index % len(Formasyon.TIPLER)]

