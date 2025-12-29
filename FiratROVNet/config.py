
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
        'engel_mesafesi': 40.0,      # GATLimitleri.ENGEL ile aynı
        'iletisim_menzili': 80.0,     # GATLimitleri.KOPMA ile aynı
        'min_pil_uyarisi': 0.2,       # Normalize edilmiş (0.0-1.0)
        'kacinma_mesafesi': 10.0       # GATLimitleri.CARPISMA ile aynı
    }
    
    # Takipçi ROV için varsayılan ayarlar
    TAKIPCI = {
        'engel_mesafesi': 40.0,       # GATLimitleri.ENGEL ile aynı
        'iletisim_menzili': 80.0,     # GATLimitleri.KOPMA ile aynı
        'min_pil_uyarisi': 0.15,      # Normalize edilmiş (0.0-1.0)
        'kacinma_mesafesi': 10.0       # GATLimitleri.CARPISMA ile aynı
    }
    
    # Genel varsayılan ayarlar (fallback için)
    VARSAYILAN = {
        'engel_mesafesi': 40.0,
        'iletisim_menzili': 80.0,
        'min_pil_uyarisi': 0.2,
        'kacinma_mesafesi': 10.0
    }


class HareketAyarlari:
    """
    Hareket ve formasyon ayarları - Tüm sistemde kullanılan katsayılar ve mesafeler.
    """
    # Takipçi hareket eşikleri
    HAREKET_ESIGI_KATSAYISI = 0.7      # İletişim menzilinin %70'i (lider uzaklaşma eşiği)
    HISTERESIS_KATSAYISI = 0.9         # Aktif moddan pasif moda geçiş toleransı
    
    # Hedef toleransları (metre)
    HEDEF_TOLERANS_LIDER = 1         # Lider için hedef toleransı
    HEDEF_TOLERANS_TAKIPCI = 2.0       # Takipçi için hedef toleransı
    
    # Formasyon ayarları
    FORMASYON_MESAFESI = 15.0          # Varsayılan formasyon mesafesi (metre)
    FORMASYON_VARSAYILAN_ARALIK = 15.0  # Formasyon aralığı (metre)
    
    # Kaçınma ayarları
    KACINMA_MESAFESI_FALLBACK_KATSAYISI = 0.2  # Engel mesafesinin %20'si (fallback)
    MINIMUM_MESAFE_KACINMA = 5.0       # Minimum mesafe (çok yakınsa kaçınma yok)
    YAKIN_MESAFE_ESIGI = 15.0          # Yakın mesafe eşiği (iletişim kopmasını önleme)
    
    # Dikey toleranslar (metre)
    DIKEY_TOLERANS_ENGEL = 10.0         # Engel algılama için dikey tolerans
    DIKEY_TOLERANS_ADA = 5.0            # Ada algılama için dikey tolerans
    
    # Vektör birleştirme katsayıları (0.0-1.0)
    VEKTOR_BIRLESTIRME_NORMAL_KACINMA = 0.8      # Normal durumda kaçınma ağırlığı
    VEKTOR_BIRLESTIRME_TAKIPCI_KACINMA = 0.8      # Takipçi için kaçınma ağırlığı
    VEKTOR_BIRLESTIRME_TAKIPCI_HEDEF = 0.2        # Takipçi için hedef ağırlığı
    
    # Güç ayarları (0.0-1.0)
    GUC_ENGEL = 0.5                    # Engel durumunda motor gücü
    GUC_PASIF_MOD = 0.5                # Pasif modda motor gücü
    GUC_UZAK = 1.0                     # Uzak durumda motor gücü (takipçi)
    
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


class FizikSabitleri:
    """
    Fizik simülasyonu için sabitler - ROV hareketi ve fizik motoru ayarları.
    """
    # Sürtünme ve hareket
    SURTUNME_KATSAYISI = 0.95            # Sürtünme katsayısı (0.0-1.0)
    HIZLANMA_CARPANI = 30.0              # Hızlanma çarpanı (hareket gücü)
    KALDIRMA_KUVVETI = 2.0               # Kaldırma kuvveti (lider için yüzeye çıkma)
    BATARYA_SOMURME_KATSAYISI = 0.001    # Batarya sömürme katsayısı (maksimum güçte ~66 saniye dayanır)
    
    # Hız ve momentum limitleri
    MAX_HIZ = 50.0                       # Maksimum hız limiti (aşırı hızlanmayı önle)
    VELOCITY_DURMA_ESIGI = 0.1           # Hız durma eşiği (momentum korunumu için)
    VELOCITY_DURMA_CARPANI = 0.7         # Durma sırasında hız çarpanı
    
    # Çarpışma ve itme
    CARPISMA_ITME_MESAFESI = 2.0         # Çarpışma sonrası itme mesafesi
    CARPISMA_HIZ_YANSIMA = 0.7           # Çarpışma sonrası hız yansıma katsayısı
    CARPISMA_HIZ_SIFIRLAMA_ESIGI = 0.5   # Çarpışma sonrası hız sıfırlama eşiği
    
    # ROV kütlesi ve boyutları
    ROV_KUTLESI = 1.0                    # ROV kütlesi (basitleştirilmiş)
    ROV_MINIMUM_MESAFE = 2.0             # ROV'lar arası minimum mesafe (çarpışma önleme)
    
    # Lider yüzey kontrolü
    LIDER_YUZEY_ALT_SINIR = -2.0         # Lider için alt derinlik sınırı
    LIDER_YUZEY_UST_SINIR = 0.5          # Lider için üst yüzey sınırı
    LIDER_YUZEY_YAKINLIK = -0.5           # Lider yüzeye yakınlık eşiği
    LIDER_YUZEY_HIZ_CARPANI = 0.5        # Lider yüzeye yakınken hız çarpanı
    
    # Takipçi derinlik kontrolü
    TAKIPCI_YUZEY_SINIRI = 0.0           # Takipçi için yüzey sınırı
    TAKIPCI_MAX_DERINLIK = -100.0        # Takipçi için maksimum derinlik


class SimulasyonSabitleri:
    """
    Simülasyon ortamı oluşturma ve yerleştirme sabitleri.
    """
    # ROV yerleştirme
    ROV_YERLESTIRME_MAX_DENEME = 500     # ROV yerleştirme için maksimum deneme sayısı
    ROV_YERLESTIRME_DERINLIK_MIN = -20.0 # ROV yerleştirme minimum derinlik
    ROV_YERLESTIRME_DERINLIK_MAX = -5.0  # ROV yerleştirme maksimum derinlik
    
    # Ada ve güvenlik
    ADA_GUVENLIK_PAYI = 100.0            # Ada radyusuna ek güvenlik payı (birim)
    ADA_VARSAYILAN_RADIUS = 100.0        # Ada varsayılan radyusu (geriye uyumluluk)
    
    # Havuz sınırları (görünmez duvarlar)
    DUVAR_KALINLIGI = 1.0                # Havuz duvar kalınlığı
    DUVAR_YUKSEKLIGI = 500.0             # Havuz duvar yüksekliği
    
    # Görselleştirme
    KESIKLI_CIZGI_PARCA_UZUNLUGU = 2.0   # Kesikli çizgi parça uzunluğu (engel tespiti)
    KESIKLI_CIZGI_BOSLUK_UZUNLUGU = 1.0  # Kesikli çizgi boşluk uzunluğu (engel tespiti)
    ILETISIM_CIZGI_PARCA_UZUNLUGU = 1.5  # İletişim çizgisi parça uzunluğu
    ILETISIM_CIZGI_BOSLUK_UZUNLUGU = 0.8 # İletişim çizgisi boşluk uzunluğu
    
    # Sensör ayarları
    LIDAR_RAYCAST_SAYISI = 5             # Lidar için raycast sayısı (her yön için)
    ENGEL_TESPITI_MIN_MESAFE = 999.0    # Engel tespiti için başlangıç minimum mesafesi


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
    
    # Formasyon isimleri (20 tip)
    TIPLER = [
        "LINE",          # 0: Çizgi formasyonu (tek sıra)
        "V_SHAPE",       # 1: V şekli (uçan kazlar)
        "DIAMOND",       # 2: Elmas formasyonu
        "SQUARE",        # 3: Kare formasyonu
        "CIRCLE",        # 4: Daire formasyonu
        "ARROW",         # 5: Ok şekli
        "WEDGE",         # 6: Kama şekli
        "ECHELON",       # 7: Eşelon (çapraz sıra)
        "COLUMN",        # 8: Sütun (dikey sıra)
        "SPREAD",        # 9: Yayılım (geniş yayılım)
        "TRIANGLE",      # 10: Üçgen formasyonu
        "CROSS",         # 11: Haç formasyonu
        "STAGGERED",     # 12: Kademeli formasyon
        "WALL",          # 13: Duvar formasyonu
        "STAR",          # 14: Yıldız formasyonu
        "PHALANX",       # 15: Falanks (sıkı düzen, askeri formasyon)
        "RECTANGLE",     # 16: Dikdörtgen formasyonu
        "HEXAGON",       # 17: Altıgen formasyonu
        "WAVE",          # 18: Dalga formasyonu
        "SPIRAL"         # 19: Spiral formasyonu
    ]
    
    
    def pozisyonlar(self, tip, aralik=15.0, is_3d=False, lider_koordinat=None, yaw=None):
        """
        Liderin Yaw açısını dikkate alarak formasyon pozisyonlarını hesaplar.
        
        Args:
            tip (int veya str): Formasyon tipi (0-14 arası indeks veya isim)
            aralik (float): ROV'lar arası mesafe (varsayılan: 15.0)
            is_3d (bool): 3D formasyon modu (varsayılan: False - 2D)
                - True: ROV'lar 3D uzayda (x, y, z) dizilir
                - False: ROV'lar 2D düzlemde (x, y, z=0) dizilir
            lider_koordinat (tuple, optional): (x, y, z) - Lider koordinatı (varsayılan: None - gerçek pozisyon kullanılır)
                - Verilirse, lider bu koordinattaymış gibi pozisyonlar hesaplanır
            yaw (float, optional): Liderin yaw açısı (derece, varsayılan: None - Filo'dan alınır)
                - 0 derece = Kuzey (+Y yönü)
                - Pozitif değerler saat yönünde dönüş
        
        Returns:
            list: [(x0, y0, z0), (x1, y1, z1), ...] formatında pozisyon listesi
                  İlk eleman lider, diğerleri takipçiler için global pozisyonlar
                  x, y: 2D koordinatlar (yatay düzlem)
                  z: derinlik (dikey) - 3D modda değişken, 2D modda 0
        
        Örnek:
            pozisyonlar = Formasyon.pozisyonlar("V_SHAPE", aralik=20.0)
            # Liderin yaw açısına göre döndürülmüş pozisyonlar
            
            pozisyonlar = Formasyon.pozisyonlar("V_SHAPE", aralik=20.0, yaw=45)
            # Lider 45 derece döndüğünde formasyon pozisyonları
        """
        import math
        
        if isinstance(tip, str):
            tip = tip.upper()
            if tip in Formasyon.TIPLER:
                tip_index = Formasyon.TIPLER.index(tip)
            else:
                tip_index = 0  # Varsayılan: LINE
        else:
            tip_index = int(tip) % len(Formasyon.TIPLER)

        n_rovs = len(self.Filo.sistemler) if self.Filo else 1
        if n_rovs <= 1:
            return [(0.0, 0.0, 0.0)] * n_rovs
        
        # Yerel ofsetleri tutacak liste (lider merkezli: 0,0,0)
        yerel_ofsetler = [(0.0, 0.0, 0.0)] * n_rovs
        
        # 1. LİDER BİLGİLERİNİ AL (Global Pozisyon ve Yaw)
        lider_global_pos = (0.0, 0.0, 0.0)
        lider_id = 0
        
        if self.Filo is not None:
            # Gerçek lideri bul
            for i in range(n_rovs):
                if self.Filo.get(i, 'rol') == 1:
                    lider_id = i
                    break
            
            # Liderin Global GPS'ini al
            if lider_koordinat is not None:
                lider_global_pos = (float(lider_koordinat[0]), float(lider_koordinat[1]), float(lider_koordinat[2]))
            else:
                gps = self.Filo.get(lider_id, "gps")  # Tercüman sayesinde (x, y, z) geliyor
                if gps:
                    # Ursina formatından (x, y, z) -> Bizim format (x, y, z) çevir
                    # Ursina: (x, y, z) -> x: sağ-sol, y: derinlik, z: ileri-geri
                    # Bizim: (x, y, z) -> x: sağ-sol, y: ileri-geri, z: derinlik
                    lider_global_pos = (float(gps[0]), float(gps[1]), float(gps[2]))
            
            # Liderin Yaw açısını al
            if yaw is None:
                yaw = self.Filo.get(lider_id, "yaw")
        
        if yaw is None:
            yaw = 0.0
        
        # 2. YEREL (LOCAL) OFSETLERİ HESAPLA 
        # (Lider 0,0'daymış ve 0 dereceye bakıyormuş gibi)

        
  
        
        # 3D mod için derinlik hesaplama yardımcı fonksiyonu
        def hesapla_z_3d(index, formasyon_tipi=None):
            """3D modda z koordinatını hesapla (formasyon tipine göre optimize edilmiş)"""
            if not is_3d:
                return 0.0
            
            # Formasyon tipine göre özel 3D yerleşim
            if formasyon_tipi in [4, 17, 14]:  # CIRCLE, HEXAGON, STAR - Küresel dağılım
                # Küresel dağılım: hem yatay hem dikey açı
                total_rovs = len(takipci_listesi)
                if total_rovs > 0:
                    # Yatay açı (zaten hesaplanmış)
                    # Dikey açı (derinlik için)
                    vertical_angle = math.pi * (index % 3) / 3 - math.pi / 2  # -90° ile +90° arası
                    depth_range = aralik * 0.8
                    return depth_range * math.sin(vertical_angle)
                return -index * aralik * 0.3
            
            elif formasyon_tipi == 19:  # SPIRAL - 3D spiral
                # Spiral hem yatay hem dikey döner
                spiral_vertical = 2.0 * math.pi * index / max(len(takipci_listesi), 1)
                return -aralik * 0.4 * math.sin(spiral_vertical)
            
            elif formasyon_tipi == 18:  # WAVE - 3D dalga
                # Dalga hem yatay hem dikey
                wave_vertical = 2.0 * math.pi * index / max(len(takipci_listesi), 1)
                return -aralik * 0.3 * math.cos(wave_vertical)
            
            elif formasyon_tipi == 10:  # TRIANGLE - 3D piramit
                # Piramit şeklinde: üstte daha az, altta daha fazla derinlik
                satir_sayisi = int(math.ceil((-1 + math.sqrt(1 + 8 * len(takipci_listesi))) / 2))
                satir_no = 0
                temp_idx = 0
                for s in range(satir_sayisi):
                    if temp_idx + s + 1 > index:
                        satir_no = s
                        break
                    temp_idx += s + 1
                # Üst satırlar daha yukarıda, alt satırlar daha aşağıda
                return -(satir_no * aralik * 0.4)
            
            elif formasyon_tipi in [15, 16]:  # PHALANX, RECTANGLE - 3D katmanlar
                # Her satır farklı derinlikte
                genislik = min(len(takipci_listesi), 5) if formasyon_tipi == 15 else int(math.ceil(math.sqrt(len(takipci_listesi) * 2)))
                satir_no = index // genislik
                return -satir_no * aralik * 0.5
            
            else:
                # Varsayılan: Her 3-4 ROV bir katman oluşturur
                katman = index // 3
                return -katman * aralik * 0.5  # Negatif = su altı
        
        # Takipçi sayısı (lider hariç)
        takipci_listesi = [i for i in range(n_rovs) if i != lider_id]
        
        # Formasyon tipine göre yerel ofsetleri hesapla
        # Format: (x, y, z) - x,y: 2D koordinatlar, z: derinlik
        # Lider merkezli koordinat sistemi (lider 0,0,0'da ve kuzeye bakar)
        
        if tip_index == 0:  # LINE (Çizgi)
            for idx, i in enumerate(takipci_listesi):
                z_3d = hesapla_z_3d(i, tip_index)
                yerel_ofsetler[i] = (0.0, -aralik * (idx + 1), z_3d)  # (x, y, z)
                
        
        elif tip_index == 1:  # V_SHAPE (V şekli)
            for idx, i in enumerate(takipci_listesi):
                row = (idx + 2) // 2  # Satır numarası (1, 1, 2, 2, 3, 3, ...)
                # İlk takipçi sağda, ikinci solda, üçüncü sağda, dördüncü solda...
                side = 1 if (idx + 1) % 2 == 1 else -1  # Tek indeksler sağ, çift indeksler sol
                z_3d = hesapla_z_3d(i, tip_index)
                yerel_ofsetler[i] = (side * aralik * row, -aralik * row, z_3d)  # (x, y, z)
        
        elif tip_index == 2:  # DIAMOND (Elmas)
            # Elmas şekli: merkez lider, etrafında eşit dağılım
            for idx, i in enumerate(takipci_listesi):
                angle = 2 * math.pi * idx / len(takipci_listesi)
                radius = aralik * (1 + (idx // len(takipci_listesi)))
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                z_3d = hesapla_z_3d(i, tip_index)
                yerel_ofsetler[i] = (x, y, z_3d)  # (x, y, z)
        
        elif tip_index == 3:  # SQUARE (Kare)
            # Kare formasyonu: mümkün olduğunca kare şeklinde
            side_length = int(math.ceil(math.sqrt(len(takipci_listesi))))
            idx = 0
            for row in range(side_length):
                for col in range(side_length):
                    if idx < len(takipci_listesi):
                        i = takipci_listesi[idx]
                        x = (col - side_length / 2 + 0.5) * aralik
                        y = -row * aralik
                        z_3d = hesapla_z_3d(i, tip_index)
                        yerel_ofsetler[i] = (x, y, z_3d)  # (x, y, z)
                    idx += 1
        
        elif tip_index == 4:  # CIRCLE (Daire)
            # Dairesel formasyon: lider merkezde, takipçiler çember üzerinde (3D: küresel dağılım)
            for idx, i in enumerate(takipci_listesi):
                angle = 2 * math.pi * idx / len(takipci_listesi)
                radius = aralik * 1.5
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                z_3d = hesapla_z_3d(i, tip_index)
                yerel_ofsetler[i] = (x, y, z_3d)  # (x, y, z)
        
        elif tip_index == 5:  # ARROW (Ok)
            # Ok şekli: lider önde, takipçiler arkada ok şeklinde
            for idx, i in enumerate(takipci_listesi):
                row = idx // 3 + 1
                col = (idx % 3) - 1  # -1, 0, 1 (sol, orta, sağ)
                z_3d = hesapla_z_3d(i, tip_index)
                yerel_ofsetler[i] = (col * aralik * 0.8, -row * aralik * 1.2, z_3d)  # (x, y, z)
            
        elif tip_index == 6:  # WEDGE (Kama)
            # Kama şekli: V'ye benzer ama daha dar
            for idx, i in enumerate(takipci_listesi):
                row = (idx + 2) // 2  # Satır numarası (1, 1, 2, 2, ...)
                side = 1 if (idx + 1) % 2 == 1 else -1  # Tek indeksler sağ, çift indeksler sol
                z_3d = hesapla_z_3d(i, tip_index)
                yerel_ofsetler[i] = (side * aralik * row * 0.6, -aralik * row * 0.8, z_3d)  # (x, y, z)
            
        elif tip_index == 7:  # ECHELON (Eşelon)
            # Eşelon: çapraz sıra
            for idx, i in enumerate(takipci_listesi):
                z_3d = hesapla_z_3d(i, tip_index)
                yerel_ofsetler[i] = (aralik * (idx + 1) * 0.7, -aralik * (idx + 1) * 0.7, z_3d)  # (x, y, z)
        
        elif tip_index == 8:  # COLUMN (Sütun)
            # Sütun: dikey sıra (yan yana)
            for idx, i in enumerate(takipci_listesi):
                z_3d = hesapla_z_3d(i, tip_index)
                yerel_ofsetler[i] = (aralik * (idx + 1), 0.0, z_3d)  # (x, y, z)
        
        elif tip_index == 9:  # SPREAD (Yayılım)
            # Yayılım: geniş açılı dağılım
            for idx, i in enumerate(takipci_listesi):
                angle = math.pi * (idx + 1) / (len(takipci_listesi) + 1) - math.pi / 2
                radius = aralik * 2.0
                x = radius * math.sin(angle)
                y = -radius * math.cos(angle) * 0.5
                z_3d = hesapla_z_3d(i, tip_index)
                yerel_ofsetler[i] = (x, y, z_3d)  # (x, y, z)
        
        elif tip_index == 10:  # TRIANGLE (Üçgen)
            # Üçgen formasyonu: lider önde, takipçiler üçgen şeklinde (3D: piramit)
            takipci_sayisi = len(takipci_listesi)
            if takipci_sayisi > 0:
                # Üçgenin satır sayısını hesapla
                satir_sayisi = int(math.ceil((-1 + math.sqrt(1 + 8 * takipci_sayisi)) / 2))
                idx = 0
                for satir in range(satir_sayisi):
                    satir_eleman_sayisi = satir + 1
                    for pozisyon in range(satir_eleman_sayisi):
                        if idx < takipci_sayisi:
                            rov_idx = takipci_listesi[idx]
                            x_offset = (pozisyon - satir / 2) * aralik
                            y_offset = -(satir + 1) * aralik
                            z_3d = hesapla_z_3d(rov_idx, tip_index)
                            yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
                        idx += 1
        
        elif tip_index == 11:  # CROSS (Haç)
            # Haç formasyonu: lider merkezde, takipçiler dört yöne
            yonler = [(1, 0), (-1, 0), (0, -1), (0, 1)]  # Sağ, Sol, Geri, İleri
            idx = 0
            for yon_idx, (dx, dy) in enumerate(yonler):
                for kademe in range(1, (len(takipci_listesi) // 4) + 2):
                    if idx < len(takipci_listesi):
                        rov_idx = takipci_listesi[idx]
                        x_offset = dx * kademe * aralik
                        y_offset = dy * kademe * aralik
                        z_3d = hesapla_z_3d(rov_idx, tip_index)
                        yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
                        idx += 1
        
        elif tip_index == 12:  # STAGGERED (Kademeli)
            # Kademeli formasyon: her satır bir öncekinden kaydırılmış
            if len(takipci_listesi) > 0:
                satir_genisligi = int(math.ceil(math.sqrt(len(takipci_listesi))))
                idx = 0
                for satir in range(satir_genisligi):
                    for kol in range(satir_genisligi):
                        if idx < len(takipci_listesi):
                            rov_idx = takipci_listesi[idx]
                            # Her satır yarım aralık kaydırılmış
                            x_offset = (kol - satir_genisligi / 2 + 0.5) * aralik + (satir % 2) * aralik * 0.5
                            y_offset = -satir * aralik
                            z_3d = hesapla_z_3d(rov_idx, tip_index)
                            yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
                            idx += 1
        
        elif tip_index == 13:  # WALL (Duvar)
            # Duvar formasyonu: geniş bir duvar gibi yan yana
            if len(takipci_listesi) > 0:
                for idx, rov_idx in enumerate(takipci_listesi):
                    # Yan yana dizilim
                    x_offset = ((idx % 2) * 2 - 1) * ((idx // 2) + 1) * aralik * 0.5
                    y_offset = -(idx // 2) * aralik * 0.3
                    z_3d = hesapla_z_3d(rov_idx, tip_index)
                    yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
        
        elif tip_index == 14:  # STAR (Yıldız)
            # Yıldız formasyonu: lider merkezde, takipçiler yıldız kollarında (3D: küresel dağılım)
            if len(takipci_listesi) > 0:
                kol_sayisi = min(8, len(takipci_listesi))  # Maksimum 8 kol
                for idx, i in enumerate(takipci_listesi):
                    kol_no = idx % kol_sayisi
                    kademe = (idx // kol_sayisi) + 1
                    angle = 2 * math.pi * kol_no / kol_sayisi
                    radius = aralik * kademe * 1.2
                    x = radius * math.cos(angle)
                    y = radius * math.sin(angle)
                    z_3d = hesapla_z_3d(i, tip_index)
                    yerel_ofsetler[i] = (x, y, z_3d)  # (x, y, z)
        
        elif tip_index == 15:  # PHALANX (Falanks)
            # Falanks: Sıkı düzen, askeri formasyon (geniş ama derin değil) (3D: katmanlar)
            if len(takipci_listesi) > 0:
                # Genişlik hesapla (mümkün olduğunca geniş ama derin değil)
                genislik = min(len(takipci_listesi), 5)  # Maksimum 5 sütun
                derinlik = (len(takipci_listesi) + genislik - 1) // genislik
                idx = 0
                for satir in range(derinlik):
                    for kol in range(genislik):
                        if idx < len(takipci_listesi):
                            rov_idx = takipci_listesi[idx]
                            x_offset = (kol - genislik / 2 + 0.5) * aralik * 0.8
                            y_offset = -satir * aralik * 0.6
                            z_3d = hesapla_z_3d(rov_idx, tip_index)
                            yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
                        idx += 1
        
        elif tip_index == 16:  # RECTANGLE (Dikdörtgen)
            # Dikdörtgen formasyonu: geniş ve derin (3D: katmanlar)
            if len(takipci_listesi) > 0:
                # En-boy oranı yaklaşık 2:1 (genişlik 2 katı)
                genislik = int(math.ceil(math.sqrt(len(takipci_listesi) * 2)))
                derinlik = int(math.ceil(len(takipci_listesi) / genislik))
                idx = 0
                for satir in range(derinlik):
                    for kol in range(genislik):
                        if idx < len(takipci_listesi):
                            rov_idx = takipci_listesi[idx]
                            x_offset = (kol - genislik / 2 + 0.5) * aralik
                            y_offset = -satir * aralik
                            z_3d = hesapla_z_3d(rov_idx, tip_index)
                            yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
                        idx += 1
        
        elif tip_index == 17:  # HEXAGON (Altıgen)
            # Altıgen formasyonu: lider merkezde, takipçiler altıgen şeklinde (3D: küresel dağılım)
            if len(takipci_listesi) > 0:
                # Altıgen katmanları (katman 1'den başla, katman 0 lider)
                katman = 1
                idx = 0
                while idx < len(takipci_listesi):
                    # Her katmanda 6 * katman kadar ROV var (katman 1: 6, katman 2: 12, ...)
                    katman_rov_sayisi = 6 * katman
                    for pozisyon in range(katman_rov_sayisi):
                        if idx < len(takipci_listesi):
                            rov_idx = takipci_listesi[idx]
                            # Altıgen kenarları
                            angle = 2 * math.pi * pozisyon / katman_rov_sayisi
                            radius = aralik * katman * 1.2
                            x_offset = radius * math.cos(angle)
                            y_offset = radius * math.sin(angle)
                            z_3d = hesapla_z_3d(rov_idx, tip_index)
                            yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
                            idx += 1
                    katman += 1
        
        elif tip_index == 18:  # WAVE (Dalga)
            # Dalga formasyonu: sinüs dalgası şeklinde (3D: hem yatay hem dikey dalga)
            if len(takipci_listesi) > 0:
                for idx, i in enumerate(takipci_listesi):
                    # Sinüs dalgası şeklinde yerleştir
                    x_offset = (idx - len(takipci_listesi) / 2) * aralik * 0.8
                    wave_amplitude = aralik * 0.5
                    wave_frequency = 2.0 * math.pi / max(len(takipci_listesi), 1)
                    y_offset = wave_amplitude * math.sin(wave_frequency * idx) - aralik * 0.5
                    z_3d = hesapla_z_3d(i, tip_index)
                    yerel_ofsetler[i] = (x_offset, y_offset, z_3d)
        
        elif tip_index == 19:  # SPIRAL (Spiral)
            # Spiral formasyonu: lider merkezde, takipçiler spiral şeklinde (3D: hem yatay hem dikey spiral)
            if len(takipci_listesi) > 0:
                for idx, i in enumerate(takipci_listesi):
                    # Spiral açısı ve yarıçapı
                    spiral_turns = 2.0  # Spiral dönüş sayısı
                    angle = spiral_turns * 2 * math.pi * idx / len(takipci_listesi)
                    radius = aralik * (1.0 + idx * 0.3)  # Yarıçap artarak büyür
                    x_offset = radius * math.cos(angle)
                    y_offset = radius * math.sin(angle)
                    z_3d = hesapla_z_3d(i, tip_index)
                    yerel_ofsetler[i] = (x_offset, y_offset, z_3d)
            

        
        # 3. YAW AÇISINA GÖRE DÖNDÜR VE GLOBAL KOORDİNATA EKLE
        # Simülasyon sistemi: X=Sağ, Y=İleri. 
        # Yaw 0 = +Y yönü (Kuzey).
        angle_rad = math.radians(yaw)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        final_pozisyonlar = [(0.0, 0.0, 0.0)] * n_rovs
        
        for i in range(n_rovs):
            if i == lider_id:
                final_pozisyonlar[i] = lider_global_pos
                continue
            
            # Yerel koordinatlar
            lx, ly, lz = yerel_ofsetler[i]
            
            # 2D Rotasyon (X ve Y düzleminde)
            # x' = x cos θ + y sin θ
            # y' = -x sin θ + y cos θ
            # Standart sağ-el kuralına göre rotation:
            gx = lx * cos_a + ly * sin_a
            gy = -lx * sin_a + ly * cos_a
            
            # Global konuma ekle
            final_pozisyonlar[i] = (
                lider_global_pos[0] + gx,
                lider_global_pos[1] + gy,
                lider_global_pos[2] + lz
            )

        return final_pozisyonlar
    
    
    @staticmethod
    def formasyon_ismi(tip_index):
        """
        Formasyon indeksine göre isim döndürür.
        
        Args:
            tip_index (int): Formasyon indeksi (0-19)
        
        Returns:
            str: Formasyon ismi
        """
        return Formasyon.TIPLER[tip_index % len(Formasyon.TIPLER)]

