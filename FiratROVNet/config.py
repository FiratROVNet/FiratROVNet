
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
    ENGEL = 25.0      # Kod 1: Engel yakınlığı mesafesi
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
    MOTOR_GUC_KATSAYISI = 0.5              # Manuel hareket güç katsayısı              # Manuel hareket güç katsayısı


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
    
    # Sensör ayarları (sonar/lidar menzili ROV.sensor_config["engel_mesafesi"] ile, varsayılan 10 m)
    LIDAR_RAYCAST_SAYISI = 5             # Lidar için raycast sayısı (her yön için)
    LIDAR_GORUS_ACISI_DERECE = 60        # Lidar koni taraması açısı (derece)
    ENGEL_TESPITI_MIN_MESAFE = 999.0    # Engel tespiti için başlangıç minimum mesafesi


class MinimapAyarlari:
    """
    Minimap ROV ikon güncelleme ayarları — jitter azaltma ve takip hızı.
    """
    # Jitter önleme: Hedef ile ikon arası bu mesafeden küçükse (harita birimi) anında hizala
    JITTER_THRESHOLD = 0.0015   # ~0.6 m (400 m havuzda)
    # Lerp hızı: time.dt * LERP_SPEED — yüksek = daha hızlı takip (varsayılan 35)
    LERP_SPEED = 35.0


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


import math

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
        "SPIRAL",        # 19: Spiral formasyonu
        "TSHAPE"         # 20: T formasyonu (yeni)
    ]
    
    
   

    def pozisyonlar(self,tip, aralik=15.0, is_3d=False, lider_koordinat=None, yaw=None,g_id=0):
               # 1. GRUP VE ROV LİSTESİNİ AL
        # g_rovs[g_id] bize ROV entity listesini verir: [RovEntity1, RovEntity2...]
        grup_rov_listesi = self.Filo.g_rovs.get(g_id)
        
        if not grup_rov_listesi:
            return {}
            
        n_rovs = len(grup_rov_listesi)
        if n_rovs == 0:
            return {}

        # 2. FORMASYON TİPİNİ BELİRLE
        if isinstance(tip, str):
            tip = tip.upper()
            tip_index = self.TIPLER.index(tip) if tip in self.TIPLER else 0
        else:
            tip_index = int(tip) % len(self.TIPLER)

        # 3. LİDERİ VE REFERANS NOKTASINI BELİRLE
        # Varsayılan lider listenin ilk elemanıdır
        lider_entity = grup_rov_listesi[0]
        lider_id = lider_entity.id
        
        # Grupta 'rol' değeri 1 olan bir ROV var mı diye bak (Entity üzerinden veya Filo verisinden)
        for rov in grup_rov_listesi:
            # Not: Entity içinde .rol attribute'u varsa direkt if rov.rol == 1: kullanılabilir.
            # Biz mevcut yapıya sadık kalarak Filo.get kullanıyoruz:
            if self.Filo.get(rov.id, 'rol') == 1:
                lider_entity = rov
                lider_id = rov.id
                break
        
        # Lider Global Pozisyonu
        if lider_koordinat is not None:
            lider_pos = tuple(map(float, lider_koordinat))
        else:
            # Entity üzerinden gps çekilebiliyorsa: lider_entity.gps
            gps = self.Filo.get(lider_id, "gps")
            lider_pos = (float(gps[0]), float(gps[1]), float(gps[2])) if gps else (0.0, 0.0, 0.0)

        # Lider Yaw Açısı
        if yaw is None:
            yaw = self.Filo.get(lider_id, "yaw") or 0.0

        # 4. YEREL OFSETLERİN HESAPLANMASI
        # Takipçileri ayır (Lider hariç diğer entity'ler)
        takipciler = [rov for rov in grup_rov_listesi if rov.id != lider_id]
        
        # Sonuçları tutacak sözlük: {rov_id: (x, y, z)}
        yerel_ofsetler = {lider_id: (0.0, 0.0, 0.0)}
        
        for idx, rov in enumerate(takipciler):
            # 2D Ofset (x, y)
            lx, ly = self._yerel_xy_hesapla(tip_index, idx, aralik, len(takipciler))
            
            # 3D Ofset (z)
            lz = 0.0
            if is_3d:
                lz = self._yerel_z_hesapla(tip_index, idx, aralik, len(takipciler))
                
            yerel_ofsetler[rov.id] = (lx, ly, lz)

        # 5. GLOBAL KOORDİNATA DÖNÜŞTÜRME (ROTASYON)
        # Yaw açısına göre döndür ve liderin pozisyonuna ekle
        angle_rad = math.radians(yaw)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        global_pozisyonlar = {}
        final_list = []
        
        for rov in grup_rov_listesi:
            lx, ly, lz = yerel_ofsetler[rov.id]
            
            # Rotasyon (X=Sağ, Y=İleri eksenine göre)
            gx = lx * cos_a + ly * sin_a
            gy = -lx * sin_a + ly * cos_a
            
            global_pozisyonlar[rov.id] = (
                lider_pos[0] + gx,
                lider_pos[1] + gy,
                lider_pos[2] + lz
            )

            #Eski yöntem pozisyonları liste şeklinde döndür.
            global_pos = (
                lider_pos[0] + gx,
                lider_pos[1] + gy,
                lider_pos[2] + lz
            )
            final_list.append(global_pos)
            
        return global_pozisyonlar

    def _yerel_xy_hesapla(self, tip, idx, aralik, n_takipci):
        """Formasyon tipine göre X, Y ofsetlerini hesaplar."""
        # Ortak Değişkenler
        row = (idx // 2) + 1
        side = 1 if (idx + 1) % 2 != 0 else -1  # Tekler sağ(1), Çiftler sol(-1)

        if tip == 0:   # LINE
            return (0.0, -aralik * (idx + 1))
            
        elif tip == 1: # V_SHAPE
            row_v = (idx + 2) // 2
            return (side * aralik * row_v, -aralik * row_v)
            
        elif tip == 2: # DIAMOND
            angle = 2 * math.pi * idx / max(n_takipci, 1)
            radius = aralik * (1 + (idx // max(n_takipci, 1)))
            return (radius * math.cos(angle), radius * math.sin(angle))
            
        elif tip == 3: # SQUARE
            side_len = int(math.ceil(math.sqrt(n_takipci)))
            c_row = idx // side_len
            c_col = idx % side_len
            return ((c_col - side_len / 2 + 0.5) * aralik, -c_row * aralik)
            
        elif tip == 4: # CIRCLE
            angle = 2 * math.pi * idx / max(n_takipci, 1)
            radius = aralik * 1.5
            return (radius * math.cos(angle), radius * math.sin(angle))
            
        elif tip == 5: # ARROW
            row_a = idx // 3 + 1
            col_a = (idx % 3) - 1
            return (col_a * aralik * 0.8, -row_a * aralik * 1.2)

        elif tip == 8: # COLUMN
            return (aralik * (idx + 1), 0.0)
            
        elif tip == 10: # TRIANGLE
            satir = int(math.ceil((-1 + math.sqrt(1 + 8 * (idx + 1))) / 2)) - 1
            onceki_toplam = (satir * (satir + 1)) // 2
            pos_in_row = idx - onceki_toplam
            x = (pos_in_row - satir / 2.0) * aralik
            y = -(satir + 1) * aralik
            return (x, y)
            
        elif tip == 20: # TSHAPE
            split_idx = n_takipci // 2
            if idx < split_idx: # Gövde
                return (0.0, -aralik * (idx + 1))
            else: # Baş
                head_idx = idx - split_idx
                h_side = 1 if head_idx % 2 == 0 else -1
                dist = ((head_idx // 2) + 1) * aralik
                return (h_side * dist, 0.0)
                
        # Diğer formasyon tipleri buraya eklenebilir...
        # Fallback (Varsayılan): LINE
        return (0.0, -aralik * (idx + 1))

    def _yerel_z_hesapla(self, tip, idx, aralik, n_takipci):
        """Formasyon tipine göre Z (derinlik) ofsetlerini hesaplar."""
        # Küresel Dağılımlar (CIRCLE, HEXAGON, STAR)
        if tip in [4, 14, 17]:
            vert_angle = math.pi * (idx % 3) / 3 - math.pi / 2
            return aralik * 0.8 * math.sin(vert_angle)
            
        # SPIRAL
        elif tip == 19:
            vert = 2.0 * math.pi * idx / max(n_takipci, 1)
            return -aralik * 0.4 * math.sin(vert)
            
        # WAVE
        elif tip == 18:
            vert = 2.0 * math.pi * idx / max(n_takipci, 1)
            return -aralik * 0.3 * math.cos(vert)
            
        # TRIANGLE (Piramit yapısı)
        elif tip == 10:
            satir = int(math.ceil((-1 + math.sqrt(1 + 8 * (idx + 1))) / 2)) - 1
            return -(satir * aralik * 0.4)
            
        # Varsayılan (Kademeli derinlik)
        katman = idx // 3
        return -katman * aralik * 0.5