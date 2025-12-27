
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
    CARPISMA = 5.0    # Kod 2: Çarpışma riski mesafesi
    ENGEL = 20.0      # Kod 1: Engel yakınlığı mesafesi
    KOPMA = 60.0      # Kod 3: Bağlantı kopması mesafesi
    UZAK = 80.0       # Kod 5: Liderden uzaklık mesafesi
    
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
    Sensör ayarları - Fonksiyonlar ile dinamik değer döndürür.
    """
    @staticmethod
    def get(rol='varsayilan', parametre=None):
        """
        ROV rolüne göre sensör ayarlarını döndürür.
        
        Args:
            rol (str): 'lider', 'takipci' veya 'varsayilan'
            parametre (str, optional): Belirli bir parametre isteniyorsa ('engel_mesafesi', 'iletisim_menzili', vb.)
        
        Returns:
            dict veya float: Tüm ayarlar dict'i veya belirli parametre değeri
        """
        # Temel değerler (GAT limitlerinden türetilir)
        engel_mesafesi = GATLimitleri.ENGEL
        iletisim_menzili = GATLimitleri.KOPMA
        kacinma_mesafesi = GATLimitleri.CARPISMA
        
        # Rol bazlı farklılıklar
        if rol == 'lider':
            min_pil_uyarisi = 0.2
        elif rol == 'takipci':
            min_pil_uyarisi = 0.15
        else:  # varsayilan
            min_pil_uyarisi = 0.2
        
        ayarlar = {
            'engel_mesafesi': engel_mesafesi,
            'iletisim_menzili': iletisim_menzili,
            'min_pil_uyarisi': min_pil_uyarisi,
            'kacinma_mesafesi': kacinma_mesafesi
        }
        
        return ayarlar.get(parametre) if parametre else ayarlar
    
    # Geriye dönük uyumluluk için class property'ler
    @classmethod
    def LIDER(cls):
        return cls.get('lider')
    
    @classmethod
    def TAKIPCI(cls):
        return cls.get('takipci')
    
    @classmethod
    def VARSAYILAN(cls):
        return cls.get('varsayilan')


class HareketAyarlari:
    """
    Hareket ve formasyon ayarları - Minimum değişken, maksimum fonksiyon kullanımı.
    """
    # Temel katsayılar (birleştirilmiş)
    KATSAYI_HAREKET_ESIGI = 0.7      # İletişim menzilinin %70'i
    KATSAYI_HISTERESIS = 0.9         # Aktif-pasif mod geçişi
    KATSAYI_KACINMA_FALLBACK = 0.2   # Engel mesafesinin %20'si
    KATSAYI_UZAKLASMA = 0.2          # Uzaklaşma gücü (azaltıldı: 0.4 -> 0.2, daha yumuşak)
    KATSAYI_YUMUSAKLIK = 0.2         # Yumuşaklık çarpanı (azaltıldı: 0.4 -> 0.2, daha yumuşak)
    KATSAYI_MOTOR_GUC = 0.5          # Manuel hareket güç
    
    # Mesafe eşikleri (metre)
    MESAFE_HEDEF_TOLERANS_LIDER = 0.5
    MESAFE_HEDEF_TOLERANS_TAKIPCI = 2.0
    MESAFE_FORMAYON = 15.0
    MESAFE_MIN_KACINMA = 3.0         # Minimum kaçınma mesafesi (artırıldı: 2.0 -> 3.0, titreme önleme)
    MESAFE_YAKIN_ESIGI = 20.0
    MESAFE_MAX_ALGILAMA = 999.0
    MESAFE_PASIF_MOD_MIN = 5.0
    
    # Toleranslar (metre)
    TOLERANS_DIKEY_ENGEL = 10.0
    TOLERANS_DIKEY_ADA = 5.0
    
    # Güç seviyeleri (0.0-1.0)
    GUC_ENGEL = 0.6
    GUC_PASIF = 0.1
    GUC_UZAK = 1.5
    
    # Çarpışma parametreleri
    CARPISMA_TITREME_MESAFE = 5.0
    CARPISMA_TITREME_HIZ = 2.0
    CARPISMA_AYIRMA = 0.6
    CARPISMA_HIZ_YAVASLATMA = 0.7
    CARPISMA_MOMENTUM = 0.6
    CARPISMA_HIZ_ESIGI = 0.1
    
    # Fizik sabitleri
    FIZIK_SURTUNME = 0.95
    FIZIK_HIZLANMA = 30.0
    FIZIK_KALDIRMA = 2.0
    FIZIK_BATARYA = 0.001
    
    # Havuz sınırları
    HAVUZ_SINIR_KATSAYISI = 0.95
    HAVUZ_Y_UST = 0.3
    HAVUZ_Y_ALT = -95.0
    HAVUZ_Y_YUZEY_TOLERANS = 0.5
    HAVUZ_Y_MIN_DERINLIK = -2.0
    HAVUZ_GUVENLIK_PAYI = 50.0
    HAVUZ_VARSAYILAN_RADIUS = 50.0
    
    # Formasyon
    FORMASYON_DERINLIK = -10.0
    FORMASYON_ADIM = 1.0
    FORMASYON_MARGIN_MIN = 0.2
    FORMASYON_MARGIN_BASLANGIC = 0.6
    
    # ROV boyutları
    ROV_SCALE = (1.5, 0.8, 2.5)
    ROV_LABEL_Y = 3.0
    ROV_LABEL_SCALE = 20.0
    
    # Hareket
    HAREKET_DUR_YAVASLATMA = 0.7
    HAREKET_DUR_HIZ_ESIGI = 0.1
    HAREKET_YAW_HIZI = 90.0
    HAREKET_YAW_GUC_MIN = -1.0
    HAREKET_YAW_GUC_MAX = 1.0
    HAREKET_NORMAL_GUC_MIN = 0.0
    HAREKET_NORMAL_GUC_MAX = 1.0
    
    # Vektör birleştirme
    VEKTOR_NORMAL_KACINMA = 0.5
    VEKTOR_TAKIPCI_KACINMA = 0.8
    VEKTOR_TAKIPCI_HEDEF = 0.2
    
    # Görselleştirme
    GORSELLEŞTIRME_ENGEL_KIRMIZI = 5.0
    GORSELLEŞTIRME_ENGEL_TURUNCU = 10.0
    GORSELLEŞTIRME_ENGEL_PARCA = 2.0
    GORSELLEŞTIRME_ENGEL_BOSLUK = 1.0
    GORSELLEŞTIRME_ILETISIM_PARCA = 1.5
    GORSELLEŞTIRME_ILETISIM_BOSLUK = 0.8
    GORSELLEŞTIRME_ILETISIM_YAKIN = 0.3
    GORSELLEŞTIRME_ILETISIM_ORTA = 0.6
    
    # Hedef ve diğer
    HEDEF_X_BOYUTU = 10.0
    HEDEF_KALINLIK = 1.0
    RANDOM_HEDEF_HAVUZ_KATSAYISI = 0.7
    RANDOM_HEDEF_MIN_MESAFE_ADA = 30.0
    ADA_ISLAND_BOUNDARY_FARK = 5.0
    ILETISIM_VARSAYILAN_MENZIL = 35.0
    VELOCITY_THRESHOLD = 0.1
    
    # Formasyon şekil katsayıları (fonksiyon ile)
    @staticmethod
    def formasyon_katsayisi(tip='v'):
        """Formasyon tipine göre katsayıları döndürür."""
        katsayilar = {
            'v': {'x': 0.8, 'z': 0.6},
            'ok': {'x': 0.8, 'z': 1.5}
        }
        return katsayilar.get(tip.lower(), {'x': 0.8, 'z': 0.6})


class ModemAyarlari:
    """
    Modem ayarları - Fonksiyonlar ile dinamik değer döndürür.
    """
    @staticmethod
    def get(rol='lider', parametre=None):
        """
        ROV rolüne göre modem ayarlarını döndürür.
        
        Args:
            rol (str): 'lider' veya 'takipci'
            parametre (str, optional): Belirli bir parametre isteniyorsa
        
        Returns:
            dict veya float: Tüm ayarlar dict'i veya belirli parametre değeri
        """
        if rol == 'lider':
            ayarlar = {
                'gurultu_orani': 0.05,
                'kayip_orani': 0.1,
                'gecikme': 0.5
            }
        else:  # takipci
            ayarlar = {
                'gurultu_orani': 0.1,
                'kayip_orani': 0.1,
                'gecikme': 0.5
            }
        
        return ayarlar.get(parametre) if parametre else ayarlar
    
    # Geriye dönük uyumluluk için class method'lar
    @classmethod
    def LIDER(cls):
        return cls.get('lider')
    
    @classmethod
    def TAKIPCI(cls):
        return cls.get('takipci')


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
        def hesapla_z_3d(index):
            """3D modda z koordinatını hesapla (katmanlar halinde)"""
            if not is_3d:
                return 0.0
            # Her 3-4 ROV bir katman oluşturur
            katman = index // 3
            return -katman * aralik * 0.5  # Negatif = su altı
        
        # Takipçi sayısı (lider hariç)
        takipci_listesi = [i for i in range(n_rovs) if i != lider_id]
        
        # Formasyon tipine göre yerel ofsetleri hesapla
        # Format: (x, y, z) - x,y: 2D koordinatlar, z: derinlik
        # Lider merkezli koordinat sistemi (lider 0,0,0'da ve kuzeye bakar)
        
        if tip_index == 0:  # LINE (Çizgi)
            for idx, i in enumerate(takipci_listesi):
                z_3d = hesapla_z_3d(i)
                yerel_ofsetler[i] = (0.0, -aralik * (idx + 1), z_3d)  # (x, y, z)
                
        
        elif tip_index == 1:  # V_SHAPE (V şekli)
            for idx, i in enumerate(takipci_listesi):
                row = (idx + 2) // 2  # Satır numarası (1, 1, 2, 2, 3, 3, ...)
                # İlk takipçi sağda, ikinci solda, üçüncü sağda, dördüncü solda...
                side = 1 if (idx + 1) % 2 == 1 else -1  # Tek indeksler sağ, çift indeksler sol
                z_3d = hesapla_z_3d(i)
                yerel_ofsetler[i] = (side * aralik * row, -aralik * row, z_3d)  # (x, y, z)
        
        elif tip_index == 2:  # DIAMOND (Elmas)
            # Elmas şekli: merkez lider, etrafında eşit dağılım
            for idx, i in enumerate(takipci_listesi):
                angle = 2 * math.pi * idx / len(takipci_listesi)
                radius = aralik * (1 + (idx // len(takipci_listesi)))
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                z_3d = hesapla_z_3d(i)
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
                        z_3d = hesapla_z_3d(i)
                        yerel_ofsetler[i] = (x, y, z_3d)  # (x, y, z)
                    idx += 1
        
        elif tip_index == 4:  # CIRCLE (Daire)
            # Dairesel formasyon: lider merkezde, takipçiler çember üzerinde
            for idx, i in enumerate(takipci_listesi):
                angle = 2 * math.pi * idx / len(takipci_listesi)
                radius = aralik * 1.5
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                z_3d = hesapla_z_3d(i)
                yerel_ofsetler[i] = (x, y, z_3d)  # (x, y, z)
        
        elif tip_index == 5:  # ARROW (Ok)
            # Ok şekli: lider önde, takipçiler arkada ok şeklinde
            for idx, i in enumerate(takipci_listesi):
                row = idx // 3 + 1
                col = (idx % 3) - 1  # -1, 0, 1 (sol, orta, sağ)
                z_3d = hesapla_z_3d(i)
                yerel_ofsetler[i] = (col * aralik * 0.8, -row * aralik * 1.2, z_3d)  # (x, y, z)
            
        elif tip_index == 6:  # WEDGE (Kama)
            # Kama şekli: V'ye benzer ama daha dar
            for idx, i in enumerate(takipci_listesi):
                row = (idx + 2) // 2  # Satır numarası (1, 1, 2, 2, ...)
                side = 1 if (idx + 1) % 2 == 1 else -1  # Tek indeksler sağ, çift indeksler sol
                z_3d = hesapla_z_3d(i)
                yerel_ofsetler[i] = (side * aralik * row * 0.6, -aralik * row * 0.8, z_3d)  # (x, y, z)
            
        elif tip_index == 7:  # ECHELON (Eşelon)
            # Eşelon: çapraz sıra
            for idx, i in enumerate(takipci_listesi):
                z_3d = hesapla_z_3d(i)
                yerel_ofsetler[i] = (aralik * (idx + 1) * 0.7, -aralik * (idx + 1) * 0.7, z_3d)  # (x, y, z)
        
        elif tip_index == 8:  # COLUMN (Sütun)
            # Sütun: dikey sıra (yan yana)
            for idx, i in enumerate(takipci_listesi):
                z_3d = hesapla_z_3d(i)
                yerel_ofsetler[i] = (aralik * (idx + 1), 0.0, z_3d)  # (x, y, z)
        
        elif tip_index == 9:  # SPREAD (Yayılım)
            # Yayılım: geniş açılı dağılım
            for idx, i in enumerate(takipci_listesi):
                angle = math.pi * (idx + 1) / (len(takipci_listesi) + 1) - math.pi / 2
                radius = aralik * 2.0
                x = radius * math.sin(angle)
                y = -radius * math.cos(angle) * 0.5
                z_3d = hesapla_z_3d(i)
                yerel_ofsetler[i] = (x, y, z_3d)  # (x, y, z)
        
        elif tip_index == 10:  # TRIANGLE (Üçgen)
            # Üçgen formasyonu: lider önde, takipçiler üçgen şeklinde
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
                            z_3d = hesapla_z_3d(rov_idx)
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
                        z_3d = hesapla_z_3d(rov_idx)
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
                            z_3d = hesapla_z_3d(rov_idx)
                            yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
                            idx += 1
        
        elif tip_index == 13:  # WALL (Duvar)
            # Duvar formasyonu: geniş bir duvar gibi yan yana
            if len(takipci_listesi) > 0:
                for idx, rov_idx in enumerate(takipci_listesi):
                    # Yan yana dizilim
                    x_offset = ((idx % 2) * 2 - 1) * ((idx // 2) + 1) * aralik * 0.5
                    y_offset = -(idx // 2) * aralik * 0.3
                    z_3d = hesapla_z_3d(rov_idx)
                    yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
        
        elif tip_index == 14:  # STAR (Yıldız)
            # Yıldız formasyonu: lider merkezde, takipçiler yıldız kollarında
            if len(takipci_listesi) > 0:
                kol_sayisi = min(8, len(takipci_listesi))  # Maksimum 8 kol
                for idx, i in enumerate(takipci_listesi):
                    kol_no = idx % kol_sayisi
                    kademe = (idx // kol_sayisi) + 1
                    angle = 2 * math.pi * kol_no / kol_sayisi
                    radius = aralik * kademe * 1.2
                    x = radius * math.cos(angle)
                    y = radius * math.sin(angle)
                    z_3d = hesapla_z_3d(i)
                    yerel_ofsetler[i] = (x, y, z_3d)  # (x, y, z)
        
        elif tip_index == 15:  # PHALANX (Falanks)
            # Falanks: Sıkı düzen, askeri formasyon (geniş ama derin değil)
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
                            z_3d = hesapla_z_3d(rov_idx)
                            yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
                        idx += 1
        
        elif tip_index == 16:  # RECTANGLE (Dikdörtgen)
            # Dikdörtgen formasyonu: geniş ve derin
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
                            z_3d = hesapla_z_3d(rov_idx)
                            yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
                        idx += 1
        
        elif tip_index == 17:  # HEXAGON (Altıgen)
            # Altıgen formasyonu: lider merkezde, takipçiler altıgen şeklinde
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
                            z_3d = hesapla_z_3d(rov_idx)
                            yerel_ofsetler[rov_idx] = (x_offset, y_offset, z_3d)
                            idx += 1
                    katman += 1
        
        elif tip_index == 18:  # WAVE (Dalga)
            # Dalga formasyonu: sinüs dalgası şeklinde
            if len(takipci_listesi) > 0:
                for idx, i in enumerate(takipci_listesi):
                    # Sinüs dalgası şeklinde yerleştir
                    x_offset = (idx - len(takipci_listesi) / 2) * aralik * 0.8
                    wave_amplitude = aralik * 0.5
                    wave_frequency = 2.0 * math.pi / max(len(takipci_listesi), 1)
                    y_offset = wave_amplitude * math.sin(wave_frequency * idx) - aralik * 0.5
                    z_3d = hesapla_z_3d(i)
                    yerel_ofsetler[i] = (x_offset, y_offset, z_3d)
        
        elif tip_index == 19:  # SPIRAL (Spiral)
            # Spiral formasyonu: lider merkezde, takipçiler spiral şeklinde
            if len(takipci_listesi) > 0:
                for idx, i in enumerate(takipci_listesi):
                    # Spiral açısı ve yarıçapı
                    spiral_turns = 2.0  # Spiral dönüş sayısı
                    angle = spiral_turns * 2 * math.pi * idx / len(takipci_listesi)
                    radius = aralik * (1.0 + idx * 0.3)  # Yarıçap artarak büyür
                    x_offset = radius * math.cos(angle)
                    y_offset = radius * math.sin(angle)
                    z_3d = hesapla_z_3d(i)
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
