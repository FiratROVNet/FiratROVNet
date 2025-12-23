import numpy as np
from ursina import Vec3, time, distance
from .config import cfg, GATLimitleri, SensorAyarlari, ModemAyarlari, HareketAyarlari
from .iletisim import AkustikModem
import math
import random

# ==========================================
# 1. FİLO (ROV FİLO YÖNETİCİSİ)
# ==========================================
class Filo:
    def __init__(self):
        self.sistemler = [] 
        self.asil_hedef = None  # Asıl hedef (orijinal liderin hedefi)
        self.orijinal_lider_id = 0  # Orijinal lider ID
        self.ortam_ref = None  # Ortam referansı (hedef görselleştirme için)
        self.hedef_gorsel = None  # Hedef görsel Entity (Ursina'da X işareti)
        self.hedef_pozisyon = None  # Mevcut hedef pozisyonu (x, y, z)
    
    @property
    def rovs(self):
        """ROV entity listesini döndürür (sistemler üzerinden)."""
        return [s.rov for s in self.sistemler if hasattr(s, 'rov')]

    def ekle(self, gnc_objesi):
        self.sistemler.append(gnc_objesi)

    def rehber_dagit(self, modem_rehberi):
        if self.sistemler:
            for sistem in self.sistemler:
                # Tüm GNC sistemlerine rehber dağıt
                sistem.rehber_guncelle(modem_rehberi)

    def otomatik_kurulum(self, rovs, lider_id=0, modem_ayarlari=None, baslangic_hedefleri=None, sensor_ayarlari=None, ortam_ref=None):
        """
        ROV filo sistemini otomatik olarak kurar ve yapılandırır.
        
        Bu fonksiyon tüm ROV'lar için modem, GNC sistemi, sensör ayarları ve başlangıç hedeflerini
        otomatik olarak oluşturur. Manuel kurulum ihtiyacını ortadan kaldırır.
        
        Args:
            rovs: ROV entity listesi (Ortam.rovs)
            lider_id (int): Lider ROV'un ID'si (varsayılan: 0)
            modem_ayarlari (dict, optional): Modem parametreleri. Örnek:
                {
                    'lider': {'gurultu_orani': 0.05, 'kayip_orani': 0.1, 'gecikme': 0.5},
                    'takipci': {'gurultu_orani': 0.1, 'kayip_orani': 0.1, 'gecikme': 0.5}
                }
            baslangic_hedefleri (dict, optional): ROV ID'lerine göre başlangıç hedefleri. Örnek:
                {
                    0: (40, 0, 60),    # Lider: (x, y, z)
                    1: (35, -10, 50),  # Takipçi 1
                    2: (40, -10, 50),  # Takipçi 2
                    3: (45, -10, 50)   # Takipçi 3
                }
            sensor_ayarlari (dict, optional): Sensör ayarları. Üç format desteklenir:
                # Format 1: Tüm ROV'lar için ortak ayarlar
                {
                    'engel_mesafesi': 25.0,
                    'iletisim_menzili': 40.0,
                    'min_pil_uyarisi': 15.0
                }
                # Format 2: Lider ve takipçi için ayrı ayarlar
                {
                    'lider': {'engel_mesafesi': 30.0, 'iletisim_menzili': 50.0, 'min_pil_uyarisi': 20.0},
                    'takipci': {'engel_mesafesi': 25.0, 'iletisim_menzili': 40.0, 'min_pil_uyarisi': 15.0}
                }
                # Format 3: Her ROV için özel ayarlar (ROV ID ile)
                {
                    0: {'engel_mesafesi': 30.0, 'iletisim_menzili': 50.0},  # Lider
                    1: {'engel_mesafesi': 25.0, 'iletisim_menzili': 40.0},  # Takipçi 1
                    2: {'engel_mesafesi': 20.0, 'iletisim_menzili': 35.0}   # Takipçi 2
                }
        
        Returns:
            dict: Tüm modemlerin rehberi (rehber_dagit için kullanılabilir)
        
        Örnekler:
            # Basit kullanım (varsayılan ayarlar)
            filo = Filo()
            tum_modemler = filo.otomatik_kurulum(rovs=app.rovs)
            
            # Özel başlangıç hedefleri ile
            tum_modemler = filo.otomatik_kurulum(
                rovs=app.rovs,
                baslangic_hedefleri={
                    0: (40, 0, 60),    # Lider
                    1: (35, -10, 50),  # Takipçi 1
                    2: (40, -10, 50),  # Takipçi 2
                    3: (45, -10, 50)   # Takipçi 3
                }
            )
            
            # Özel modem ayarları ile
            tum_modemler = filo.otomatik_kurulum(
                rovs=app.rovs,
                modem_ayarlari={
                    'lider': {'gurultu_orani': 0.03, 'kayip_orani': 0.05, 'gecikme': 0.3},
                    'takipci': {'gurultu_orani': 0.15, 'kayip_orani': 0.2, 'gecikme': 0.6}
                }
            )
            
            # Tüm parametrelerle tam kontrol
            tum_modemler = filo.otomatik_kurulum(
                rovs=app.rovs,
                lider_id=0,
                modem_ayarlari={
                    'lider': {'gurultu_orani': 0.02, 'kayip_orani': 0.05, 'gecikme': 0.4},
                    'takipci': {'gurultu_orani': 0.12, 'kayip_orani': 0.15, 'gecikme': 0.5}
                },
                baslangic_hedefleri={
                    0: (40, 0, 60),
                    1: (35, -10, 50),
                    2: (40, -10, 50),
                    3: (45, -10, 50)
                },
                sensor_ayarlari={
                    'lider': {'engel_mesafesi': 30.0, 'iletisim_menzili': 50.0, 'min_pil_uyarisi': 20.0},
                    'takipci': {'engel_mesafesi': 25.0, 'iletisim_menzili': 40.0, 'min_pil_uyarisi': 15.0}
                }
            )
        """
        # Varsayılan modem ayarları (config.py'den alınır)
        if modem_ayarlari is None:
            modem_ayarlari = {
                'lider': ModemAyarlari.LIDER.copy(),
                'takipci': ModemAyarlari.TAKIPCI.copy()
            }
        
        # Varsayılan sensör ayarları (config.py'den alınır - GAT limitleri ile tutarlı)
        if sensor_ayarlari is None:
            sensor_ayarlari = {
                'lider': SensorAyarlari.LIDER.copy(),
                'takipci': SensorAyarlari.TAKIPCI.copy()
            }
        
        # Ortam referansını kaydet
        if ortam_ref is not None:
            self.ortam_ref = ortam_ref
        elif rovs and len(rovs) > 0 and hasattr(rovs[0], 'environment_ref'):
            # ROV'lardan ortam referansını al
            self.ortam_ref = rovs[0].environment_ref
        
        # Sensör ayarları için kontrol listesi (config.py'den alınır)
        varsayilan_sensor_ayarlari = SensorAyarlari.VARSAYILAN.copy()
        
        tum_modemler = {}
        lider_modem = None
        
        # Her ROV için işlem yap
        for i, rov in enumerate(rovs):
            # Sensör ayarlarını uygula
            if sensor_ayarlari:
                # Format 1: Tüm ROV'lar için ortak ayarlar (anahtar direkt sensör adı)
                if 'engel_mesafesi' in sensor_ayarlari or 'iletisim_menzili' in sensor_ayarlari or 'min_pil_uyarisi' in sensor_ayarlari:
                    for key, value in sensor_ayarlari.items():
                        if key in varsayilan_sensor_ayarlari:
                            rov.set(key, value)
                # Format 2: Lider ve takipçi için ayrı ayarlar
                elif 'lider' in sensor_ayarlari or 'takipci' in sensor_ayarlari:
                    if i == lider_id and 'lider' in sensor_ayarlari:
                        for key, value in sensor_ayarlari['lider'].items():
                            if key in varsayilan_sensor_ayarlari:
                                rov.set(key, value)
                    elif i != lider_id and 'takipci' in sensor_ayarlari:
                        for key, value in sensor_ayarlari['takipci'].items():
                            if key in varsayilan_sensor_ayarlari:
                                rov.set(key, value)
                # Format 3: Her ROV için özel ayarlar (ROV ID ile)
                elif isinstance(sensor_ayarlari, dict) and i in sensor_ayarlari:
                    for key, value in sensor_ayarlari[i].items():
                        if key in varsayilan_sensor_ayarlari:
                            rov.set(key, value)
            
            if i == lider_id:
                # Lider ROV
                rov.set("rol", 1)
                lider_modem = AkustikModem(
                    rov_id=i,
                    gurultu_orani=modem_ayarlari['lider'].get('gurultu_orani', 0.05),
                    kayip_orani=modem_ayarlari['lider'].get('kayip_orani', 0.1),
                    gecikme=modem_ayarlari['lider'].get('gecikme', 0.5)
                )
                rov.modem = lider_modem
                tum_modemler[i] = lider_modem
                
                # TemelGNC oluştur ve ekle (Lider için)
                gnc = TemelGNC(rov, lider_modem)
                self.ekle(gnc)
                
                # Başlangıç hedefi varsa ata (hedef_atama ile)
                if baslangic_hedefleri and i in baslangic_hedefleri:
                    hedef = baslangic_hedefleri[i]
                    # (x, y, z) formatında
                    if len(hedef) >= 3:
                        gnc.hedef_atama(hedef[0], hedef[1], hedef[2])
                    else:
                        # Geriye uyumluluk: (x, z, y) formatı
                        gnc.hedef_atama(hedef[0], hedef[2] if len(hedef) > 2 else 0, hedef[1] if len(hedef) > 1 else 0)
                elif baslangic_hedefleri is None:
                    # Varsayılan lider hedefi
                    gnc.hedef_atama(40, 0, 60)
            else:
                # Takipçi ROV
                rov.set("rol", 0)
                modem = AkustikModem(
                    rov_id=i,
                    gurultu_orani=modem_ayarlari['takipci'].get('gurultu_orani', 0.1),
                    kayip_orani=modem_ayarlari['takipci'].get('kayip_orani', 0.1),
                    gecikme=modem_ayarlari['takipci'].get('gecikme', 0.5)
                )
                rov.modem = modem
                tum_modemler[i] = modem
                
                # TemelGNC oluştur ve ekle (Takipçi için)
                gnc = TemelGNC(rov, modem)
                self.ekle(gnc)
                
                # Başlangıç hedefi varsa ata (hedef_atama ile)
                if baslangic_hedefleri and i in baslangic_hedefleri:
                    hedef = baslangic_hedefleri[i]
                    # (x, y, z) formatında
                    if len(hedef) >= 3:
                        gnc.hedef_atama(hedef[0], hedef[1], hedef[2])
                    else:
                        # Geriye uyumluluk: (x, z, y) formatı
                        gnc.hedef_atama(hedef[0], hedef[2] if len(hedef) > 2 else 0, hedef[1] if len(hedef) > 1 else 0)
                else:
                    # Takipçi için hedef yoksa
                    # Eğer baslangic_hedefleri boş dict ise (senaryo modülü için), formasyon hesaplama yapma
                    if baslangic_hedefleri == {}:
                        # Senaryo modülü: Hedef atama yapma, ROV pozisyonları korunsun
                        pass
                    else:
                        # Normal mod: Liderin hedefine göre otomatik belirle
                        lider_gnc = self.sistemler[lider_id] if lider_id < len(self.sistemler) else None
                        if lider_gnc and lider_gnc.hedef:
                            # Lider hedefine göre formasyon
                            self._takipci_hedefi_belirle(gnc, i, lider_gnc.hedef.x, lider_gnc.hedef.y, lider_gnc.hedef.z, lider_id)
                        else:
                            # Lider hedefi henüz yoksa, varsayılan takipçi hedefi (formasyon)
                            offset_x = 30 + (i * 5)
                            gnc.hedef_atama(offset_x, -10, 50)
        
        # Rehberi dağıt
        self.rehber_dagit(tum_modemler)
        
        # Asıl hedefi belirle (orijinal liderin hedefi)
        if lider_id < len(self.sistemler):
            lider_gnc = self.sistemler[lider_id]
            if lider_gnc.hedef:
                self.asil_hedef = lider_gnc.hedef
            elif baslangic_hedefleri and lider_id in baslangic_hedefleri:
                hedef = baslangic_hedefleri[lider_id]
                if len(hedef) >= 3:
                    self.asil_hedef = Vec3(hedef[0], hedef[1], hedef[2])
                else:
                    self.asil_hedef = Vec3(hedef[0], hedef[2] if len(hedef) > 2 else 0, hedef[1] if len(hedef) > 1 else 0)
            else:
                # Varsayılan hedef
                self.asil_hedef = Vec3(40, 0, 60)
        
        self.orijinal_lider_id = lider_id
        
        print(f"✅ GNC Sistemi Kuruldu: {len(rovs)} ROV (Lider: ROV-{lider_id})")
        
        return tum_modemler
    
    def manuel_kontrol_all(self, aktif=True):
        """
        Tüm ROV'ları manuel kontrol moduna alır veya otomatik moda geri döndürür.
        
        Args:
            aktif (bool): True ise tüm ROV'ları manuel kontrol moduna alır.
                         False ise otomatik moda geri döndürür.
        
        Örnek:
            # Tüm ROV'ları manuel kontrol moduna al
            filo.manuel_kontrol_all(True)
            
            # Otomatik moda geri döndür
            filo.manuel_kontrol_all(False)
        """
        for gnc in self.sistemler:
            gnc.manuel_kontrol = aktif
        
        if aktif:
            print(f"🔧 [FİLO] Tüm ROV'lar manuel kontrol moduna alındı.")
        else:
            print(f"🤖 [FİLO] Tüm ROV'lar otomatik moda döndürüldü.")

    def guncelle_hepsi(self, tahminler):
        # Lider ROV'u bul
        lider_rov_id = None
        lider_gnc = None
        lider_rov = None
        for i, gnc in enumerate(self.sistemler):
            if hasattr(gnc, 'rov') and gnc.rov.role == 1:
                lider_rov_id = i
                lider_gnc = gnc
                lider_rov = gnc.rov
                break
        
        # Lider her zaman hedefe gitmeli (hedef varsa)
        # Takipçiler sadece lider yeteri kadar uzaklaştığında hareket etmeli
        
        # Eğer lider varsa ve hedefi varsa
        if lider_rov_id is not None and lider_gnc and hasattr(lider_gnc, 'hedef') and lider_gnc.hedef:
            # Liderin hedefi (Ursina koordinat sistemi)
            lider_hedef_ursina = lider_gnc.hedef
            # Ursina koordinatları direkt kullan (takipçi hedefi belirleme fonksiyonu Ursina formatında çalışıyor)
            lider_hedef_x = lider_hedef_ursina.x      # Ursina X
            lider_hedef_y = lider_hedef_ursina.y      # Ursina Y (derinlik)
            lider_hedef_z = lider_hedef_ursina.z      # Ursina Z
            
            # Takipçilerin hedeflerini kontrol et (sadece lider yeteri kadar uzaklaştığında)
            for i, gnc in enumerate(self.sistemler):
                if i == lider_rov_id:
                    continue  # Lideri atla
                
                if hasattr(gnc, 'rov') and gnc.rov.role == 0:  # Takipçi ise
                    takipci_rov = gnc.rov
                    
                    # Lider ile takipçi arasındaki mesafeyi hesapla
                    lider_pos = lider_rov.position
                    takipci_pos = takipci_rov.position
                    mesafe = distance(lider_pos, takipci_pos)
                    
                    # İletişim menzilini al (takipçinin sensor_config'inden)
                    iletisim_menzili = takipci_rov.sensor_config.get("iletisim_menzili", SensorAyarlari.VARSAYILAN["iletisim_menzili"])
                    
                    # Hareket eşiği: Config'den alınan katsayı ile hesapla
                    hareket_esigi = iletisim_menzili * HareketAyarlari.HAREKET_ESIGI_KATSAYISI
                    
                    # Histerezis (gecikme) mekanizması: Pasif moddan aktif moda geçiş için
                    # Eğer zaten aktif moddaysa ve lider yaklaştıysa, biraz daha tolerans göster
                    if not gnc.pasif_mod and mesafe > hareket_esigi * HareketAyarlari.HISTERESIS_KATSAYISI:
                        # Aktif modda, lider hala uzakta, formasyon pozisyonuna git
                        self._takipci_hedefi_belirle(
                            gnc, i,
                            lider_hedef_x, lider_hedef_y, lider_hedef_z,
                            lider_rov_id
                        )
                    elif mesafe > hareket_esigi:
                        # Lider yeteri kadar uzaklaştı: Takipçi aktif modda, formasyon pozisyonuna gitmeli
                        gnc.pasif_mod = False
                        self._takipci_hedefi_belirle(
                            gnc, i,
                            lider_hedef_x, lider_hedef_y, lider_hedef_z,
                            lider_rov_id
                        )
                    else:
                        # Lider yakında: Takipçi pasif modda (hareket etmez)
                        # Hedefi None yap ki hareket etmesin (daha temiz)
                        if not gnc.pasif_mod:
                            gnc.pasif_mod = True
                            gnc.hedef = None
        
        # Tüm GNC sistemlerini güncelle
        for i, gnc in enumerate(self.sistemler):
            if i < len(tahminler):
                gnc.guncelle(tahminler[i])
    
    
    def _takipci_hedefi_belirle(self, takipci_gnc, takipci_rov_id, lider_x, lider_y, lider_z, lider_rov_id):
        """
        Tek bir takipçi ROV için hedef belirler (liderin hedefine göre formasyon pozisyonu).
        
        Args:
            takipci_gnc: Takipçi GNC objesi
            takipci_rov_id: Takipçi ROV'un ID'si
            lider_x: Lider hedef X koordinatı (Ursina formatında)
            lider_y: Lider hedef Y koordinatı (Ursina formatında - derinlik)
            lider_z: Lider hedef Z koordinatı (Ursina formatında)
            lider_rov_id: Lider ROV'un ID'si
        """
        formasyon_mesafesi = 15.0  # Formasyon mesafesi (metre)
        
        # Formasyon offset'leri (Ursina koordinat sisteminde: X ve Z'ye offset)
        # Basit formasyon: Lider merkezde, takipçiler çevresinde
        formasyon_offsetleri = [
            (-formasyon_mesafesi, -formasyon_mesafesi),  # Takipçi 1: Sol-Alt (X-, Z-)
            (formasyon_mesafesi, -formasyon_mesafesi),   # Takipçi 2: Sağ-Alt (X+, Z-)
            (-formasyon_mesafesi, formasyon_mesafesi),   # Takipçi 3: Sol-Üst (X-, Z+)
            (formasyon_mesafesi, formasyon_mesafesi),   # Takipçi 4: Sağ-Üst (X+, Z+)
            (0, -formasyon_mesafesi),                    # Takipçi 5: Alt (Z-)
            (0, formasyon_mesafesi),                     # Takipçi 6: Üst (Z+)
            (-formasyon_mesafesi, 0),                    # Takipçi 7: Sol (X-)
            (formasyon_mesafesi, 0),                     # Takipçi 8: Sağ (X+)
        ]
        
        # Takipçi index'i: Lider hariç, takipçilerin sırası
        takipci_index = 0
        for i, gnc in enumerate(self.sistemler):
            if i == lider_rov_id:
                continue
            if i == takipci_rov_id:
                break
            if gnc.rov.role == 0:  # Takipçi ise
                takipci_index += 1
        
        # Formasyon offset'ini al (eğer takipçi sayısı offset sayısından fazlaysa, tekrar kullan)
        offset_x, offset_z = formasyon_offsetleri[takipci_index % len(formasyon_offsetleri)]
        
        # Takipçi hedefi: Lider hedefi + offset (Ursina koordinat sisteminde)
        takipci_x_ursina = lider_x + offset_x  # Ursina X + offset X
        takipci_z_ursina = lider_z + offset_z  # Ursina Z + offset Z
        takipci_y_ursina = lider_y  # Derinlik aynı kalır
        
        # Eğer lider yüzeydeyse (y >= 0), takipçiler su altında olmalı
        if lider_y >= 0:
            takipci_y_ursina = -10.0  # Su altı derinliği
        
        # Hedef atama (Ursina koordinat sisteminde)
        try:
            takipci_gnc.hedef_atama(takipci_x_ursina, takipci_y_ursina, takipci_z_ursina)
        except Exception as e:
            print(f"⚠️ [UYARI] ROV-{takipci_rov_id} hedefi belirlenirken hata: {e}")
    
    def _takipci_hedeflerini_guncelle(self, lider_rov_id, lider_x, lider_y, lider_z):
        """
        Lider ROV'un hedefi değiştiğinde, tüm takipçi ROV'ların hedeflerini
        liderin hedefine göre +-10 metre mesafede formasyon şeklinde günceller.
        
        Args:
            lider_rov_id: Lider ROV'un ID'si
            lider_x: Lider hedef X koordinatı
            lider_y: Lider hedef Y koordinatı (derinlik)
            lider_z: Lider hedef Z koordinatı
        """
        for i, gnc in enumerate(self.sistemler):
            # Lider ROV'u atla
            if i == lider_rov_id:
                continue
            
            # Sadece takipçi ROV'lar için hedef güncelle
            if gnc.rov.role == 0:  # Takipçi ise
                self._takipci_hedefi_belirle(gnc, i, lider_x, lider_y, lider_z, lider_rov_id)
                print(f"✅ [FİLO] ROV-{i} hedefi otomatik güncellendi: Lider hedefine göre formasyon")
    
    
    def set(self, rov_id, ayar_adi, deger):
        """
        ROV ayarlarını değiştirir.
        
        Args:
            rov_id: ROV ID (0, 1, 2, ...)
            ayar_adi: Ayar adı ('rol', 'renk', 'engel_mesafesi', 'iletisim_menzili', 'min_pil_uyarisi', 'kacinma_mesafesi')
            deger: Ayar değeri
        
        Örnekler:
            filo.set(0, 'rol', 1)  # ROV-0'ı lider yap
            filo.set(1, 'renk', (255, 0, 0))  # ROV-1'i kırmızı yap
            filo.set(2, 'engel_mesafesi', 30.0)  # ROV-2'nin engel mesafesini ayarla
        """
        # Sistemler listesi boş mu kontrol et
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            print(f"   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return False
        
        # ROV ID geçerliliği kontrolü
        if not isinstance(rov_id, int) or rov_id < 0:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return False
        
        if rov_id >= len(self.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return False
        
        try:
            rov = self.sistemler[rov_id].rov
            rov.set(ayar_adi, deger)
            print(f"✅ [FİLO] ROV-{rov_id} ayarı güncellendi: {ayar_adi} = {deger}")
            return True
        except Exception as e:
            print(f"❌ [HATA] Ayar güncelleme sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return False

    def get(self, rov_id, veri_tipi):
        """
        ROV bilgilerini alır.
        
        Args:
            rov_id: ROV ID (0, 1, 2, ...)
            veri_tipi: Veri tipi ('gps', 'hiz', 'batarya', 'rol', 'renk', 'sensör', 
                                  'engel_mesafesi', 'iletisim_menzili', 'min_pil_uyarisi', 'kacinma_mesafesi', 'sonar')
        
        Returns:
            İstenen veri tipine göre değer
        
        Örnekler:
            pozisyon = filo.get(0, 'gps')
            rol = filo.get(1, 'rol')
            sensörler = filo.get(2, 'sensör')
            batarya = filo.get(0, 'batarya')
        """
        # Sistemler listesi boş mu kontrol et
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            return None
        
        # ROV ID geçerliliği kontrolü
        if not isinstance(rov_id, int) or rov_id < 0:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return None
        
        if rov_id >= len(self.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return None
        
        try:
            rov = self.sistemler[rov_id].rov
            deger = rov.get(veri_tipi)
            if deger is None:
                print(f"⚠️ [UYARI] ROV-{rov_id} için '{veri_tipi}' veri tipi bulunamadı")
            return deger
        except Exception as e:
            print(f"❌ [HATA] Veri alma sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return None

    def formasyon(self, tip="KAMA", aralik=15):
        """
        Filoyu belirtilen formasyona sokar.
        
        Önce liderleri denetler, fazlalıkları "takipçi" yapar ve ardından formasyonu kurar.
        
        Args:
            tip (str): Formasyon tipi (varsayılan: "KAMA")
                - "KAMA": V şekli formasyon (kanatlı)
                - "SAF": Yan yana formasyon
                - "DAIRE": Çember formasyonu
                - "CIZGI" veya "LINE": Arka arkaya çizgi formasyonu
                - "V" veya "V_SEKLI": V şekli formasyon
                - "KARE" veya "SQUARE": Kare formasyonu
                - "OK" veya "ARROW": Ok şekli formasyon
                - "ELMAS" veya "DIAMOND": Elmas formasyonu
            aralik (float): ROV'lar arası mesafe (varsayılan: 15)
        
        Örnekler:
            filo.formasyon()  # Varsayılan KAMA formasyonu
            filo.formasyon("SAF", aralik=20)  # Yan yana formasyon, 20 birim aralık
            filo.formasyon("DAIRE", aralik=25)  # Çember formasyonu, 25 birim aralık
            filo.formasyon("CIZGI", aralik=15)  # Çizgi formasyonu
            filo.formasyon("KARE", aralik=20)  # Kare formasyonu
            filo.formasyon("OK", aralik=18)  # Ok formasyonu
            filo.formasyon("ELMAS", aralik=22)  # Elmas formasyonu
        """
        # 1. ADIM: Otorite Denetimi (Lowest-ID Authority)
        liderler = [r for r in self.rovs if r.role == 1]
        
        if not liderler:
            print("❌ [FORMASYON] Kritik Hata: Filoda hiç lider yok!")
            return
        
        # En düşük ID'li olanı asıl lider seç
        asil_lider = min(liderler, key=lambda r: r.id)
        
        # DİĞER LİDERLERİ AZLET: Asıl lider dışındaki herkesi takipçi yap
        for r in liderler:
            if r.id != asil_lider.id:
                print(f"⚠️ [FORMASYON] Sistem Uyarısı: Otorite Çatışması! ROV-{r.id} takipçi yapıldı. Asıl Lider: ROV-{asil_lider.id}")
                # filo.set kullanarak rolü 0 (Takipçi) olarak güncelliyoruz
                self.set(r.id, "rol", 0)
        
        # 2. ADIM: Takipçileri Hazırla
        # Artık sistemde tek lider olduğundan emin olduğumuz için geri kalanları ID sırasına diziyoruz
        takipciler = sorted([r for r in self.rovs if r.id != asil_lider.id], key=lambda r: r.id)
        toplam_n = len(self.rovs)
        
        # 3. ADIM: Liderin Mevcut Konum ve Yön Verileri
        # Ursina koordinatlarını simülasyon mantığına alalım: (x, z, y_depth)
        lider_pos = (asil_lider.x, asil_lider.z, asil_lider.y)
        lider_hiz = (asil_lider.velocity.x, asil_lider.velocity.z)
        
        # 4. ADIM: Slot Atamaları
        for i, rov in enumerate(takipciler):
            # i = 0, 1, 2... (Lider hariç takipçi indeksi)
            
            # Dinamik ofset hesabı (Formasyon Motoru üzerinden)
            tip_upper = tip.upper()
            if tip_upper == "KAMA":
                offset = FormasyonMotoru.kama_hesapla(i + 1, aralik)
            elif tip_upper == "SAF":
                offset = FormasyonMotoru.saf_hesapla(i + 1, aralik)
            elif tip_upper == "DAIRE":
                offset = FormasyonMotoru.daire_hesapla(i, toplam_n, aralik)
            elif tip_upper == "CIZGI" or tip_upper == "LINE":
                offset = FormasyonMotoru.cizgi_hesapla(i + 1, aralik)
            elif tip_upper == "V" or tip_upper == "V_SEKLI":
                offset = FormasyonMotoru.v_hesapla(i + 1, aralik)
            elif tip_upper == "KARE" or tip_upper == "SQUARE":
                offset = FormasyonMotoru.kare_hesapla(i, toplam_n, aralik)
            elif tip_upper == "OK" or tip_upper == "ARROW":
                offset = FormasyonMotoru.ok_hesapla(i + 1, aralik)
            elif tip_upper == "ELMAS" or tip_upper == "DIAMOND":
                offset = FormasyonMotoru.elmas_hesapla(i, toplam_n, aralik)
            else:
                offset = (0, -10 * (i+1), 0)  # Varsayılan: Arka arkaya sıra
                print(f"⚠️ [FORMASYON] Bilinmeyen formasyon tipi: {tip}, varsayılan formasyon kullanılıyor")
            
            # Ofseti Liderin baktığı yöne göre Dünya Koordinatlarına çevir
            hedef_dunya = lokal_to_global(lider_pos, lider_hiz, offset)
            
            # ROV'un GNC sistemine hedefi ver
            # formasyon_hedefi özelliğini kontrol et ve ayarla
            if not hasattr(rov, 'formasyon_hedefi'):
                rov.formasyon_hedefi = None
            
            rov.formasyon_hedefi = hedef_dunya
            
            # Eğer GNC sistemi varsa, hedefi git() ile ayarla
            try:
                # ROV'un hangi GNC sistemine ait olduğunu bul
                for gnc_idx, gnc_sistem in enumerate(self.sistemler):
                    if hasattr(gnc_sistem, 'rov') and gnc_sistem.rov.id == rov.id:
                        # GNC sistemine hedefi ver
                        hedef_x, hedef_y, hedef_z = hedef_dunya
                        # Ursina koordinat sistemine dönüştür: (x, z, y) -> (x, y, z)
                        self.git(gnc_idx, hedef_x, hedef_z, y=hedef_y, ai=True)
                        break
            except Exception as e:
                print(f"⚠️ [FORMASYON] ROV-{rov.id} için hedef ayarlanırken hata: {e}")
        
        print(f"✅ [FORMASYON] Formasyon kuruldu: Tip={tip}, Aralık={aralik}, Lider=ROV-{asil_lider.id}, Takipçi Sayısı={len(takipciler)}")
    
    def hedef(self, x=None, y=None, z=None):
        """
        Liderin hedefini ayarlar ve takipçilerin formasyon pozisyonlarını otomatik hesaplar.
        Lider hedefe gider, takipçiler formasyonlarını koruyarak lideri takip eder.
        Hedef görsel olarak (büyük X işareti) gösterilir ve haritaya eklenir.
        Derinlik her zaman 0 (su üstünde) olarak ayarlanır.
        
        Parametre verilmezse mevcut hedef koordinatlarını döndürür.
        Parametre verilirse hedefi günceller ve yeni koordinatları döndürür.
        
        Args:
            x (float, optional): X koordinatı (yatay düzlem). None ise mevcut hedef döndürülür.
            y (float, optional): Y koordinatı (yatay düzlem). None ise mevcut hedef döndürülür.
            z (float, optional): İGNORED - Her zaman 0 (su üstünde) kullanılır
        
        Returns:
            tuple: (x, y, z) - Hedef koordinatları (z her zaman 0)
        
        Örnekler:
            filo.hedef(50, 60)  # Lider (50, 60, 0) hedefine gider, takipçiler formasyonla takip eder
            filo.hedef(40, 50)  # Lider (40, 50, 0) hedefine gider, takipçiler formasyonla takip eder
            filo.hedef()  # Mevcut hedef koordinatlarını döndürür: (x, y, 0) veya None
        """
        # Parametre verilmediyse mevcut hedefi döndür
        if x is None or y is None:
            if self.hedef_pozisyon:
                return self.hedef_pozisyon
            else:
                return None
        
        # Derinlik her zaman 0 (su üstünde)
        z = 0
        
        # Hedef pozisyonunu kaydet (z her zaman 0 - su üstünde)
        self.hedef_pozisyon = (x, y, 0)
        
        # Lider ROV'u bul
        lider_rov_id = None
        for i, sistem in enumerate(self.sistemler):
            if hasattr(sistem, 'rov') and sistem.rov.role == 1:
                lider_rov_id = i
                break
        
        if lider_rov_id is None:
            print("❌ [HEDEF] Lider ROV bulunamadı!")
            return None
        
        # Sadece liderin hedefini güncelle
        # filo.hedef() simülasyon koordinat sisteminde (x, y, 0) alıyor
        # filo.git() Ursina koordinat sisteminde (x, z, y) alıyor
        # Dönüşüm: Simülasyon (x, y, 0) -> Ursina (x, 0, y)
        ursina_x = x      # X aynı kalır
        ursina_z = y      # Simülasyon Y -> Ursina Z
        ursina_y = 0      # Derinlik her zaman 0 (su üstünde)
        self.git(lider_rov_id, ursina_x, ursina_z, y=ursina_y, ai=True)
        
        # Takipçilerin hedeflerini liderin hedefine göre formasyon pozisyonları olarak güncelle
        # Liderin hedefi: (x, y, 0) - simülasyon koordinat sistemi
        # Ursina koordinat sistemi: (x, 0, y)
        lider_x = x  # Simülasyon X
        lider_y = 0  # Derinlik (su üstünde)
        lider_z = y  # Simülasyon Y (Ursina Z)
        
        # Tüm takipçiler için formasyon hedeflerini hesapla
        for i, sistem in enumerate(self.sistemler):
            if i == lider_rov_id:
                continue  # Lideri atla
            
            if hasattr(sistem, 'rov') and sistem.rov.role == 0:  # Takipçi ise
                # Liderin hedefine göre takipçi hedefini belirle
                self._takipci_hedefi_belirle(
                    sistem, i, 
                    lider_x, lider_y, lider_z,  # Lider hedefi (simülasyon koordinat sistemi)
                    lider_rov_id
                )
        
        # Hedef görselini oluştur/güncelle (z her zaman 0 - su üstünde)
        self._hedef_gorsel_olustur(x, y, 0)
        
        # Haritaya hedefi ekle
        if self.ortam_ref and hasattr(self.ortam_ref, 'harita'):
            self.ortam_ref.harita.hedef_pozisyon = (x, y)
        
        print(f"✅ [HEDEF] Lider hedefi güncellendi: ({x:.2f}, {y:.2f}, 0) - Su üstünde. Takipçiler formasyonla takip ediyor.")
        
        # Hedef koordinatlarını döndür
        return (x, y, 0)
    
    def _hedef_gorsel_olustur(self, x, y, z):
        """
        Hedef pozisyonunu Ursina'da büyük X işareti olarak gösterir.
        """
        if not self.ortam_ref:
            return
        
        # Eski görseli kaldır
        if self.hedef_gorsel:
            try:
                from ursina import destroy
                destroy(self.hedef_gorsel)
            except:
                pass
        
        # Ursina koordinat sistemine dönüştür: (x_2d, y_2d, z_depth) -> (x, z, y)
        ursina_pos = (x, z, y)
        
        # Büyük X işareti oluştur (iki çapraz çizgi)
        from ursina import Entity, destroy, color
        
        # X işareti için parent entity
        self.hedef_gorsel = Entity()
        self.hedef_gorsel.position = ursina_pos
        
        # X işareti boyutu (Config'den alınan değerler)
        x_boyutu = HareketAyarlari.HEDEF_X_BOYUTU
        kalinlik = HareketAyarlari.HEDEF_KALINLIK
        
        # İlk çapraz çizgi (sol üst -> sağ alt)
        Entity(
            model='cube',
            position=(0, 0, 0),
            rotation=(0, 0, 45),  # 45 derece döndür
            scale=(x_boyutu, kalinlik, kalinlik),
            color=color.red,
            parent=self.hedef_gorsel,
            unlit=True,
            billboard=False
        )
        
        # İkinci çapraz çizgi (sağ üst -> sol alt)
        Entity(
            model='cube',
            position=(0, 0, 0),
            rotation=(0, 0, -45),  # -45 derece döndür
            scale=(x_boyutu, kalinlik, kalinlik),
            color=color.red,
            parent=self.hedef_gorsel,
            unlit=True,
            billboard=False
        )
        
        # Merkez nokta (daha belirgin olsun)
        Entity(
            model='sphere',
            position=(0, 0, 0),
            scale=(2, 2, 2),
            color=color.red,
            parent=self.hedef_gorsel,
            unlit=True
        )
    

    def git(self, rov_id, x, z, y=None, ai=True):
        """
        ROV'a hedef koordinatı atar ve otomatik moda geçirir.

        Args:
            rov_id: ROV ID (0, 1, 2, ...)
            x: X koordinatı (yatay düzlem)
            z: Z koordinatı (yatay düzlem)
            y: Y koordinatı (derinlik, negatif = su altı, opsiyonel)
                - None ise mevcut derinlik korunur
            ai: AI aktif/pasif (varsayılan: True)
                - True: Zeki Mod (GAT tahminleri kullanılır)
                - False: Kör Mod (GAT tahminleri görmezden gelinir)

        Örnekler:
            filo.git(0, 40, 60, 0)           # ROV-0: (40, 0, 60), AI açık
            filo.git(1, 50, 50, -10, ai=False)  # ROV-1: (50, -10, 50), AI kapalı
            filo.git(2, 30, 40)               # ROV-2: (30, mevcut_y, 40), AI açık
        """
        # Sistemler listesi boş mu kontrol et
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            print(f"   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return
        
        # ROV ID geçerliliği kontrolü
        if not isinstance(rov_id, int) or rov_id < 0:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return
        
        if rov_id >= len(self.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            print(f"   💡 Çözüm: filo.ekle() ile daha fazla GNC sistemi ekleyin")
            return
        
        # Manuel modu kapat, otopilotu aç
        self.sistemler[rov_id].manuel_kontrol = False
        
        # AI Durumunu Ayarla
        self.sistemler[rov_id].ai_aktif = ai
        
        # Hedef Ata
        # Eğer y belirtilmemişse, mevcut derinliği koru
        if y is None:
            hedef_y = self.sistemler[rov_id].rov.y
        else:
            hedef_y = y
        
        # Bilgilendirme mesajı
        ai_durum = "AÇIK" if ai else "KAPALI (Kör Mod)"
        print(f"🔵 [FİLO] ROV-{rov_id} Rota: ({x}, {hedef_y}, {z}) | AI: {ai_durum}")
        
        # Hedef atama (x, y, z formatında)
        try:
            self.sistemler[rov_id].hedef_atama(x, hedef_y, z)
            print(f"✅ [FİLO] ROV-{rov_id} hedefi başarıyla atandı")
            
            # Eğer lider ROV'a hedef verildiyse, takipçilerin hedeflerini otomatik güncelle
            if self.sistemler[rov_id].rov.role == 1:  # Lider ise
                self._takipci_hedeflerini_guncelle(rov_id, x, hedef_y, z)
        except Exception as e:
            print(f"❌ [HATA] Hedef atama sırasında hata: {e}")
            import traceback
            traceback.print_exc()

    def move(self, rov_id, yon, guc=1.0):
        """
        ROV'a güç bazlı hareket komutu verir (gerçek dünya gibi, gerçekçi fizik ile).
        
        Args:
            rov_id: ROV ID
            yon: Hareket yönü ('ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur')
            guc: Motor gücü (0.0 - 1.0 arası, varsayılan: 1.0)
                - 1.0 = %100 güç (maksimum hız)
                - 0.5 = %50 güç (yarı hız)
                - 0.0 = %0 güç (dur)
        
        Örnekler:
            filo.move(0, 'ileri', 1.0)   # ROV-0 %100 güçle ileri
            filo.move(1, 'sag', 0.5)     # ROV-1 %50 güçle sağa
            filo.move(2, 'cik', 0.3)      # ROV-2 %30 güçle yukarı
            filo.move(3, 'dur', 0.0)      # ROV-3 dur (güç=0)
            filo.move(0, 'ileri')         # ROV-0 %100 güçle ileri (varsayılan)
        """
        # Sistemler listesi boş mu kontrol et
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            print(f"   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return
        
        # ROV ID geçerliliği kontrolü
        if not isinstance(rov_id, int) or rov_id < 0:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return
        
        if rov_id >= len(self.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            print(f"   💡 Çözüm: filo.ekle() ile daha fazla GNC sistemi ekleyin")
            return
        
        # Yön geçerliliği kontrolü
        gecerli_yonler = ['ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur']
        if yon not in gecerli_yonler:
            print(f"❌ [HATA] Geçersiz hareket yönü: '{yon}'")
            print(f"   Geçerli yönler: {', '.join(gecerli_yonler)}")
            return
        
        # Güç değerini kontrol et (0.0 - 1.0 arası)
        if not isinstance(guc, (int, float)):
            print(f"❌ [HATA] Güç değeri sayı olmalı: {guc}")
            return
        
        guc = max(0.0, min(1.0, float(guc)))
        
        try:
            # Manuel kontrolü aç
            self.sistemler[rov_id].manuel_kontrol = True
            gnc = self.sistemler[rov_id]
            rov = gnc.rov
            
            # 'dur' komutu özel durum
            if yon == 'dur' or guc == 0.0:
                if hasattr(rov, 'manuel_hareket'):
                    rov.manuel_hareket['yon'] = None
                    rov.manuel_hareket['guc'] = 0.0
                rov.velocity *= 0.9  # Yavaşça dur (momentum korunumu)
                print(f"🛑 [FİLO] ROV-{rov_id} durduruluyor")
                return
            
            # Lider ROV batırılamaz kontrolü
            if yon == 'bat' and rov.role == 1:
                print(f"⚠️ [FİLO] ROV-{rov_id} lider, batırılamaz!")
                return
            
            # Havuz sınır kontrolü (hareket öncesi)
            # Sınırlar: +-havuz_genisligi (yani +-200 birim)
            if hasattr(rov, 'environment_ref') and rov.environment_ref:
                havuz_genisligi = getattr(rov.environment_ref, 'havuz_genisligi', 200)
                havuz_sinir = havuz_genisligi  # +-havuz_genisligi
                
                # Sınırda mı kontrol et
                sinirda_x = abs(rov.x) >= havuz_sinir * 0.95
                sinirda_z = abs(rov.z) >= havuz_sinir * 0.95
                sinirda_y_ust = rov.y >= 0.3
                sinirda_y_alt = rov.y <= -95
                
                # Sınırda ise o yöne hareketi engelle
                if sinirda_x and ((yon == 'sag' and rov.x > 0) or (yon == 'sol' and rov.x < 0)):
                    print(f"⚠️ [FİLO] ROV-{rov_id} havuz sınırında (X), {yon} yönünde hareket engellendi")
                    return
                
                if sinirda_z and ((yon == 'ileri' and rov.z > 0) or (yon == 'geri' and rov.z < 0)):
                    print(f"⚠️ [FİLO] ROV-{rov_id} havuz sınırında (Z), {yon} yönünde hareket engellendi")
                    return
                
                if sinirda_y_ust and yon == 'cik':
                    print(f"⚠️ [FİLO] ROV-{rov_id} su yüzeyinde, yukarı hareket engellendi")
                    return
                
                if sinirda_y_alt and yon == 'bat':
                    print(f"⚠️ [FİLO] ROV-{rov_id} deniz tabanında, aşağı hareket engellendi")
                    return
            
            # Manuel hareket modunu ayarla (sürekli hareket için)
            if hasattr(rov, 'manuel_hareket'):
                rov.manuel_hareket['yon'] = yon
                rov.manuel_hareket['guc'] = guc
                guc_yuzdesi = int(guc * 100)
                print(f"🔵 [FİLO] ROV-{rov_id} {yon} yönünde %{guc_yuzdesi} güçle hareket ediyor (sürekli mod)")
                return
            
            # Alternatif: ROV'un move metodunu kullan (manuel_hareket yoksa)
            if hasattr(rov, 'move'):
                try:
                    rov.move(yon, guc)
                    guc_yuzdesi = int(guc * 100)
                    print(f"🔵 [FİLO] ROV-{rov_id} {yon} yönünde %{guc_yuzdesi} güçle hareket ediyor")
                    return
                except Exception as e:
                    # ROV.move() başarısız oldu, alternatif yöntem kullan
                    pass
            
            # Son alternatif: Direkt velocity kullan
            hareket_vektoru = Vec3(0, 0, 0)
            if yon == 'ileri': hareket_vektoru.z = 1.0
            elif yon == 'geri': hareket_vektoru.z = -1.0
            elif yon == 'sag': hareket_vektoru.x = 1.0
            elif yon == 'sol': hareket_vektoru.x = -1.0
            elif yon == 'cik': hareket_vektoru.y = 1.0
            elif yon == 'bat' and rov.role != 1: hareket_vektoru.y = -1.0
            
            # Hız uygula
            max_guc = 100.0 * guc
            if hareket_vektoru.length() > 0:
                # Manuel hareket güç katsayısı (Config'den)
                rov.velocity += hareket_vektoru.normalized() * max_guc * time.dt * HareketAyarlari.MOTOR_GUC_KATSAYISI
                
                # Hız limiti
                if rov.velocity.length() > max_guc:
                    rov.velocity = rov.velocity.normalized() * max_guc
            
            guc_yuzdesi = int(guc * 100)
            print(f"🔵 [FİLO] ROV-{rov_id} {yon} yönünde %{guc_yuzdesi} güçle hareket ediyor")
            
        except AttributeError as e:
            print(f"❌ [HATA] ROV-{rov_id} için gerekli özellik bulunamadı: {e}")
            print(f"   💡 Debug: GNC sistemi tipi: {type(self.sistemler[rov_id])}")
            import traceback
            traceback.print_exc()
        except Exception as e:
            print(f"❌ [HATA] Hareket komutu sırasında hata: {e}")
            print(f"   💡 Debug: ROV ID: {rov_id}, Yön: {yon}, Güç: {guc}")
            import traceback
            traceback.print_exc()

# ==========================================
# 2. TEMEL GNC SINIFI
# ==========================================
class TemelGNC:
    def __init__(self, rov_entity, modem):
        self.rov = rov_entity
        self.modem = modem
        self.hedef = None 
        self.hiz_limiti = 100.0 
        self.manuel_kontrol = False
        self.pasif_mod = False  # Takipçiler için: Lider yakındayken pasif mod (hareket etmez)
        
        # YENİ: Bireysel AI Anahtarı
        self.ai_aktif = True 

    def hedef_atama(self, x, y, z):
        self.hedef = Vec3(x, y, z)

    def rehber_guncelle(self, rehber):
        if self.modem: self.modem.rehber_guncelle(rehber)
    
    def guncelle(self, gat_kodu):
        """
        GNC güncelleme metodu - ROV'u hedefe yönlendirir ve GAT kodlarına göre tepki verir.
        Lider ve takipçi ROV'lar için ortak kullanılır.
        """
        # Erken çıkış kontrolleri
        if self.manuel_kontrol and self.hedef is None:
            return
        if self.hedef is None:
            return
        
        # Takipçiler için pasif mod kontrolü: Lider yakındayken hareket etme
        if self.rov.role == 0 and self.pasif_mod:
            # Pasif modda hareket etme, ama hedef varsa ve çok uzaktaysa minimal hareket yap
            if self.hedef is not None:
                fark = self.hedef - self.rov.position
                if fark.length() > 5.0:  # 5 metreden fazla uzaktaysa minimal hareket
                    # Minimal hareket (çok yavaş)
                    hedef_vektoru = fark.normalized() if hasattr(fark, 'normalized') else Vec3(0, 0, 0)
                    self.vektor_to_motor(hedef_vektoru, guc_carpani=0.1)  # %10 güç
            return  # Pasif modda normal hareket yok
        
        # AI kapalıysa uyarıları görmezden gel
        if not self.ai_aktif:
            gat_kodu = 0
        
        # Hedefe mesafe kontrolü
        fark = self.hedef - self.rov.position
        yatay_fark = Vec3(fark.x, 0, fark.z) if hasattr(fark, 'x') else Vec3(0, 0, 0)
        # Takipçiler için daha esnek tolerans (formasyon korunması için)
        # Config'den alınan tolerans değerleri
        tolerans = HareketAyarlari.HEDEF_TOLERANS_LIDER if self.rov.role == 1 else HareketAyarlari.HEDEF_TOLERANS_TAKIPCI
        if yatay_fark.length() < tolerans:
            # Takipçiler için: Hedefe ulaşıldıysa bile lideri takip etmeye devam et
            if self.rov.role == 0:  # Takipçi ise
                # Lideri takip etmeye devam et (hedef güncellenecek)
                pass
            else:
                return  # Lider için: Hedefe ulaşıldı
        
        # Lider için su yüzeyi kontrolü
        if self.rov.role == 1 and self.hedef.y < 0:
            self.hedef.y = 0
        
        # Hedef vektörü hesapla
        hedef_vektoru = fark.normalized() if hasattr(fark, 'normalized') else Vec3(0, 0, 0)
        
        # Kaçınma vektörü hesapla
        kacinma_vektoru = self._yaklasma_onleme_vektoru(gat_kodu, hedef_vektoru)
        
        # GAT koduna göre kaçınma vektörünü ayarla
        kacinma_vektoru = self._gat_kod_tepkisi(gat_kodu, kacinma_vektoru, hedef_vektoru)
        
        # Nihai hareket vektörünü hesapla
        nihai_vektor = self._vektor_birlestir(gat_kodu, hedef_vektoru, kacinma_vektoru)
        
        # Güç ayarı
        guc = self._guc_hesapla(gat_kodu)
        
        # Motorlara uygula
        self.vektor_to_motor(nihai_vektor, guc_carpani=guc)
    
    def _gat_kod_tepkisi(self, gat_kodu, kacinma_vektoru, hedef_vektoru):
        """GAT koduna göre kaçınma vektörünü ayarlar."""
        is_lider = (self.rov.role == 1)
        
        if gat_kodu == 1:  # ENGEL
            if kacinma_vektoru.length() > 0:
                kacinma_vektoru.y += 0.3
                return kacinma_vektoru.normalized()
            else:
                # Kaçınma vektörü yoksa varsayılan yön
                return Vec3(1, 0, 0) if is_lider else Vec3(0, 1.0, 0) + (hedef_vektoru * -0.5)
        
        elif gat_kodu == 2:  # CARPISMA
            # En uygun rota zaten hesaplandı, değişiklik yok
            return kacinma_vektoru
        
        elif gat_kodu == 3:  # KOPUK
            if kacinma_vektoru.length() > 0:
                kacinma_vektoru.y += 0.2
                return kacinma_vektoru.normalized()
            else:
                return Vec3(0, 0.2, 0)
        
        elif gat_kodu == 5:  # UZAK
            # Normal hareket, kaçınma yok
            return kacinma_vektoru
        
        else:  # gat_kodu == 0 (OK)
            return kacinma_vektoru
    
    def _vektor_birlestir(self, gat_kodu, hedef_vektoru, kacinma_vektoru):
        """Hedef ve kaçınma vektörlerini birleştirir."""
        if gat_kodu == 2:  # ÇARPISMA: Kaçınma öncelikli
            return kacinma_vektoru if kacinma_vektoru.length() > 0 else Vec3(0, 0, 0)
        
        elif gat_kodu != 0:  # Diğer tehlikeler: Kaçınma + hedef
            if kacinma_vektoru.length() > 0:
                # Config'den alınan katsayılar
                kacinma_agirlik = HareketAyarlari.VEKTOR_BIRLESTIRME_TAKIPCI_KACINMA
                hedef_agirlik = HareketAyarlari.VEKTOR_BIRLESTIRME_TAKIPCI_HEDEF
                return (kacinma_vektoru * kacinma_agirlik + hedef_vektoru * hedef_agirlik).normalized()
            else:
                return hedef_vektoru
        
        else:  # Normal durum: Hedef + kaçınma (varsa)
            if kacinma_vektoru.length() > 0:
                # Config'den alınan katsayı
                return (hedef_vektoru + kacinma_vektoru * HareketAyarlari.VEKTOR_BIRLESTIRME_NORMAL_KACINMA).normalized()
            else:
                return hedef_vektoru
    
    def _guc_hesapla(self, gat_kodu):
        """GAT koduna göre motor gücünü hesaplar."""
        is_lider = (self.rov.role == 1)
        
        if is_lider:
            return 1.0  # Lider için sabit güç
        
        # Takipçi için özel güç ayarları
        if gat_kodu == 5:  # UZAK: Daha hızlı git
            return 1.5
        elif gat_kodu == 1:  # ENGEL: Yavaşla
            return 0.5
        else:
            return 1.0

    def vektor_to_motor(self, vektor, guc_carpani=1.0):
        if vektor.length() == 0: return

        # Güç çarpanını normalize et (0.0-1.0 arası)
        guc_carpani = max(0.0, min(1.0, guc_carpani))
        
        # Vektörü normalize et (eğer normalize edilmemişse)
        vektor_magnitude = vektor.length()
        if vektor_magnitude > 0:
            vektor_normalized = vektor / vektor_magnitude
        else:
            return

        # Her bileşen için güç hesapla
        # Normalize edilmiş vektör bileşenleri zaten 0.0-1.0 arası
        # Ama diagonal hareketlerde bileşenler küçük olabilir (örn: 0.707)
        # Bu yüzden vektörün büyüklüğünü de dikkate alıyoruz
        if vektor_normalized.x > 0.1: 
            # Bileşen değerini direkt kullan (zaten normalize edilmiş)
            guc_x = abs(vektor_normalized.x) * guc_carpani
            self.rov.move("sag", guc_x)
        elif vektor_normalized.x < -0.1: 
            guc_x = abs(vektor_normalized.x) * guc_carpani
            self.rov.move("sol", guc_x)

        if vektor_normalized.y > 0.1: 
            guc_y = abs(vektor_normalized.y) * guc_carpani
            self.rov.move("cik", guc_y)
        elif vektor_normalized.y < -0.1: 
            guc_y = abs(vektor_normalized.y) * guc_carpani
            self.rov.move("bat", guc_y)

        if vektor_normalized.z > 0.1: 
            guc_z = abs(vektor_normalized.z) * guc_carpani
            self.rov.move("ileri", guc_z)
        elif vektor_normalized.z < -0.1: 
            guc_z = abs(vektor_normalized.z) * guc_carpani
            self.rov.move("geri", guc_z)
    
    def _yaklasma_onleme_vektoru(self, gat_kodu=0, hedef_vektoru=None):
        """
        Sensör mesafesine göre ROV'lar ve engellerden uzaklaşma vektörü.
        GAT kodlarına göre en uygun rotayı hesaplar.
        
        Args:
            gat_kodu: GAT kod (0=OK, 1=ENGEL, 2=CARPISMA, 3=KOPUK, 5=UZAK)
            hedef_vektoru: Hedef yönü vektörü (opsiyonel)
        
        Returns:
            Vec3: Kaçınma vektörü (en uygun rota)
        """
        if not hasattr(self.rov, 'environment_ref') or not self.rov.environment_ref:
            return Vec3(0, 0, 0)
        
        # Kaçınma mesafesini sensör ayarlarından al (veya engel_mesafesi kullan)
        kacinma_mesafesi = self.rov.sensor_config.get("kacinma_mesafesi", None)
        if kacinma_mesafesi is None:
            # Eğer kacinma_mesafesi yoksa, engel_mesafesi'nin bir kısmını kullan (Config'den katsayı)
            engel_mesafesi = self.rov.sensor_config.get("engel_mesafesi", SensorAyarlari.VARSAYILAN["engel_mesafesi"])
            kacinma_mesafesi = engel_mesafesi * HareketAyarlari.KACINMA_MESAFESI_FALLBACK_KATSAYISI
        
        # Hedef vektörü hesapla
        if hedef_vektoru is None:
            if self.hedef:
                hedef_vektoru = (self.hedef - self.rov.position)
                if hedef_vektoru.length() > 0:
                    hedef_vektoru = hedef_vektoru.normalized()
                else:
                    hedef_vektoru = Vec3(0, 0, 0)
            else:
                hedef_vektoru = Vec3(0, 0, 0)
        
        # Tehlikeli nesneleri tespit et (ROV'lar ve engeller)
        tehlikeli_nesneler = []
        
        # Diğer ROV'lar
        # ÖNEMLİ: Lider takipçilerden uzaklaşmaz - hedefe gitmek için sürüden ayrılabilir
        is_lider = (self.rov.role == 1)
        if not is_lider:  # Sadece takipçiler diğer ROV'lardan uzaklaşır
            for diger_rov in self.rov.environment_ref.rovs:
                if diger_rov.id == self.rov.id:
                    continue
                mesafe = distance(self.rov.position, diger_rov.position)
                if mesafe <= kacinma_mesafesi and mesafe > 0:
                    tehlikeli_nesneler.append({
                        'pozisyon': diger_rov.position,
                        'mesafe': mesafe,
                        'tip': 'rov'
                    })
        
        # Engeller
        for engel in self.rov.environment_ref.engeller:
            mesafe = distance(self.rov.position, engel.position)
            engel_yari_cap = max(engel.scale_x, engel.scale_y, engel.scale_z) / 2
            gercek_mesafe = mesafe - engel_yari_cap
            if gercek_mesafe <= kacinma_mesafesi and gercek_mesafe > 0:
                tehlikeli_nesneler.append({
                    'pozisyon': engel.position,
                    'mesafe': gercek_mesafe,
                    'tip': 'engel'
                })
        
        # Eğer tehlikeli nesne yoksa, boş vektör döndür
        if len(tehlikeli_nesneler) == 0:
            return Vec3(0, 0, 0)
        
        # GAT KOD 2 (ÇARPISMA): En uygun rotayı bul (yukarı çıkmak yerine)
        if gat_kodu == 2:
            return self._en_uygun_rota_bul(tehlikeli_nesneler, hedef_vektoru, kacinma_mesafesi)
        
        # GAT KOD 1 (ENGEL): Engelden uzaklaş + hedefe doğru yönel
        if gat_kodu == 1:
            uzaklasma_vektoru = Vec3(0, 0, 0)
            for nesne in tehlikeli_nesneler:
                uzaklasma_yonu = (self.rov.position - nesne['pozisyon']).normalized()
                uzaklasma_gucu = (kacinma_mesafesi - nesne['mesafe']) / kacinma_mesafesi
                uzaklasma_vektoru += uzaklasma_yonu * uzaklasma_gucu
            
            # Hedef yönüne de yönel
            if hedef_vektoru.length() > 0:
                uzaklasma_vektoru = uzaklasma_vektoru + hedef_vektoru * 0.3
            
            if uzaklasma_vektoru.length() > 0:
                return uzaklasma_vektoru.normalized()
            return Vec3(0, 0, 0)
        
        # Normal kaçınma: Tehlikeli nesnelerden uzaklaş
        uzaklasma_vektoru = Vec3(0, 0, 0)
        for nesne in tehlikeli_nesneler:
            uzaklasma_yonu = (self.rov.position - nesne['pozisyon']).normalized()
            uzaklasma_gucu = (kacinma_mesafesi - nesne['mesafe']) / kacinma_mesafesi
            uzaklasma_vektoru += uzaklasma_yonu * uzaklasma_gucu
        
        # Hedef yönüne de yönel (kaçınma ile birleştir)
        if hedef_vektoru.length() > 0 and uzaklasma_vektoru.length() > 0:
            # Kaçınma vektörünü normalize et
            uzaklasma_vektoru = uzaklasma_vektoru.normalized()
            # Hedef yönünü ekle (daha az ağırlıkla)
            nihai_vektor = uzaklasma_vektoru * 0.7 + hedef_vektoru * 0.3
            return nihai_vektor.normalized()
        
        if uzaklasma_vektoru.length() > 0:
            return uzaklasma_vektoru.normalized()
        
        return Vec3(0, 0, 0)
    
    def _en_uygun_rota_bul(self, tehlikeli_nesneler, hedef_vektoru, kacinma_mesafesi):
        """
        GAT kod 2 (çarpışma) için en uygun rotayı bulur.
        Yukarı çıkmak yerine, engeller ve ROV'lar arasından en güvenli yolu seçer.
        """
        # Farklı yönleri test et (8 yön: ileri, geri, sağ, sol, çaprazlar)
        test_yonleri = [
            Vec3(1, 0, 0),   # Sağ
            Vec3(-1, 0, 0),  # Sol
            Vec3(0, 0, 1),   # İleri
            Vec3(0, 0, -1),  # Geri
            Vec3(1, 0, 1).normalized(),   # Sağ-İleri
            Vec3(-1, 0, 1).normalized(),  # Sol-İleri
            Vec3(1, 0, -1).normalized(),  # Sağ-Geri
            Vec3(-1, 0, -1).normalized(), # Sol-Geri
        ]
        
        # Hedef yönünü de ekle (eğer varsa)
        if hedef_vektoru.length() > 0:
            # Yatay düzlemde (y=0)
            hedef_yatay = Vec3(hedef_vektoru.x, 0, hedef_vektoru.z)
            if hedef_yatay.length() > 0:
                test_yonleri.append(hedef_yatay.normalized())
        
        en_iyi_yon = None
        en_iyi_skor = float('-inf')
        
        for yon in test_yonleri:
            # Bu yönde ne kadar güvenli?
            skor = 0.0
            
            # Tehlikeli nesnelerden uzaklık kontrolü
            for nesne in tehlikeli_nesneler:
                # Bu yönde ilerlersek nesneye ne kadar yaklaşırız?
                nesne_yonu = (nesne['pozisyon'] - self.rov.position).normalized()
                yon_nesne_aci = yon.dot(nesne_yonu)
                
                # Eğer bu yöne doğru gidersek nesneye yaklaşırsak, skor düşer
                if yon_nesne_aci > 0:  # Aynı yöne
                    uzaklik_skoru = (kacinma_mesafesi - nesne['mesafe']) / kacinma_mesafesi
                    skor -= uzaklik_skoru * 2.0  # Tehlikeli nesneye yaklaşma cezası
                else:  # Uzaklaşma
                    skor += abs(yon_nesne_aci) * 1.0  # Uzaklaşma bonusu
            
            # Hedef yönüne yakınlık bonusu
            if hedef_vektoru.length() > 0:
                hedef_yatay = Vec3(hedef_vektoru.x, 0, hedef_vektoru.z)
                if hedef_yatay.length() > 0:
                    hedef_yatay = hedef_yatay.normalized()
                    hedef_benzerligi = yon.dot(hedef_yatay)
                    if hedef_benzerligi > 0:
                        skor += hedef_benzerligi * 0.5  # Hedefe yakınlık bonusu
            
            if skor > en_iyi_skor:
                en_iyi_skor = skor
                en_iyi_yon = yon
        
        # Eğer hiç güvenli yön bulunamazsa, en az tehlikeli olanı seç
        if en_iyi_yon is None:
            # Tüm tehlikeli nesnelerden uzaklaş
            uzaklasma_vektoru = Vec3(0, 0, 0)
            for nesne in tehlikeli_nesneler:
                uzaklasma_yonu = (self.rov.position - nesne['pozisyon']).normalized()
                uzaklasma_gucu = (kacinma_mesafesi - nesne['mesafe']) / kacinma_mesafesi
                uzaklasma_vektoru += uzaklasma_yonu * uzaklasma_gucu
            
            if uzaklasma_vektoru.length() > 0:
                return uzaklasma_vektoru.normalized()
            return Vec3(0, 1, 0)  # Son çare: yukarı
        
        return en_iyi_yon


# ==========================================
# FORMASYON MOTORU (Matematiksel Ofsetler)
# ==========================================
class FormasyonMotoru:
    """Formasyon tipleri için matematiksel ofset hesaplamaları."""
    
    @staticmethod
    def kama_hesapla(idx, aralik):
        """
        Dinamik Kama (V) formasyonu: idx 1->Sol, 2->Sağ, 3->Uzak Sol...
        
        Args:
            idx: Takipçi indeksi (1'den başlar, lider hariç)
            aralik: ROV'lar arası mesafe
        
        Returns:
            (x_offset, y_offset, z_offset): Formasyon ofseti
        """
        taraf = -1 if idx % 2 != 0 else 1
        derinlik_sirasi = (idx + 1) // 2
        return (taraf * derinlik_sirasi * aralik, -derinlik_sirasi * aralik, 0)
    
    @staticmethod
    def saf_hesapla(idx, aralik):
        """
        Dinamik Yan Yana formasyonu: idx 1->Sol, 2->Sağ...
        
        Args:
            idx: Takipçi indeksi (1'den başlar, lider hariç)
            aralik: ROV'lar arası mesafe
        
        Returns:
            (x_offset, y_offset, z_offset): Formasyon ofseti
        """
        taraf = -1 if idx % 2 != 0 else 1
        yan_sira = (idx + 1) // 2
        return (taraf * yan_sira * aralik, 0, 0)
    
    @staticmethod
    def daire_hesapla(i, n, aralik):
        """
        Lider etrafında çember formasyonu.
        
        Args:
            i: Takipçi indeksi (0'dan başlar, lider hariç)
            n: Toplam ROV sayısı
            aralik: ROV'lar arası mesafe
        
        Returns:
            (x_offset, y_offset, z_offset): Formasyon ofseti
        """
        radius = aralik * (n / 4)  # Araç sayısı arttıkça çemberi genişlet
        aci = (2 * math.pi * i) / (n - 1) if n > 1 else 0
        return (math.cos(aci) * radius, math.sin(aci) * radius, 0)
    
    @staticmethod
    def cizgi_hesapla(idx, aralik):
        """
        Çizgi (LINE) formasyonu: Arka arkaya tek sıra.
        
        Args:
            idx: Takipçi indeksi (1'den başlar, lider hariç)
            aralik: ROV'lar arası mesafe
        
        Returns:
            (x_offset, y_offset, z_offset): Formasyon ofseti
        """
        # Arka arkaya sıralama: Her takipçi bir öncekinin arkasında
        return (0, -idx * aralik, 0)
    
    @staticmethod
    def v_hesapla(idx, aralik):
        """
        V şekli formasyonu: Lider önde, takipçiler V şeklinde dağılır.
        
        Args:
            idx: Takipçi indeksi (1'den başlar, lider hariç)
            aralik: ROV'lar arası mesafe
        
        Returns:
            (x_offset, y_offset, z_offset): Formasyon ofseti
        """
        # V şekli: Sol ve sağ kanatlar - Config'den katsayılar
        taraf = -1 if idx % 2 != 0 else 1  # Tek sayılar sol, çift sayılar sağ
        kanat_sirasi = (idx + 1) // 2  # Kanat içindeki sıra
        x_offset = taraf * kanat_sirasi * aralik * HareketAyarlari.V_FORMASYON_X_KATSAYISI
        z_offset = -kanat_sirasi * aralik * HareketAyarlari.V_FORMASYON_Z_KATSAYISI
        return (x_offset, z_offset, 0)
    
    @staticmethod
    def kare_hesapla(i, n, aralik):
        """
        Kare formasyonu: Lider merkezde, takipçiler kare köşelerinde.
        
        Args:
            i: Takipçi indeksi (0'dan başlar, lider hariç)
            n: Toplam ROV sayısı
            aralik: ROV'lar arası mesafe
        
        Returns:
            (x_offset, y_offset, z_offset): Formasyon ofseti
        """
        # Kare köşeleri: 4 köşe + kenarlar
        kare_boyutu = aralik * 2  # Kare kenar uzunluğu
        
        # Köşeler ve kenarlar için pozisyonlar
        if i < 4:
            # 4 köşe
            if i == 0:  # Sol-Alt
                return (-kare_boyutu, -kare_boyutu, 0)
            elif i == 1:  # Sağ-Alt
                return (kare_boyutu, -kare_boyutu, 0)
            elif i == 2:  # Sol-Üst
                return (-kare_boyutu, kare_boyutu, 0)
            elif i == 3:  # Sağ-Üst
                return (kare_boyutu, kare_boyutu, 0)
        else:
            # Kenarlarda: Fazla takipçiler kenarlara yerleşir
            kenar_index = (i - 4) % 4
            kenar_pozisyon = (i - 4) // 4 + 1  # Hangi kenar pozisyonu
            
            if kenar_index == 0:  # Alt kenar
                return (-kare_boyutu + kenar_pozisyon * aralik, -kare_boyutu, 0)
            elif kenar_index == 1:  # Sağ kenar
                return (kare_boyutu, -kare_boyutu + kenar_pozisyon * aralik, 0)
            elif kenar_index == 2:  # Üst kenar
                return (kare_boyutu - kenar_pozisyon * aralik, kare_boyutu, 0)
            else:  # Sol kenar
                return (-kare_boyutu, kare_boyutu - kenar_pozisyon * aralik, 0)
        
        return (0, 0, 0)
    
    @staticmethod
    def ok_hesapla(idx, aralik):
        """
        Ok (ARROW) formasyonu: Lider önde, takipçiler ok şeklinde.
        
        Args:
            idx: Takipçi indeksi (1'den başlar, lider hariç)
            aralik: ROV'lar arası mesafe
        
        Returns:
            (x_offset, y_offset, z_offset): Formasyon ofseti
        """
        # Ok şekli: Merkez çizgi + kanatlar
        if idx == 1:
            # İlk takipçi: Merkez çizgide
            return (0, -aralik, 0)
        elif idx == 2:
            # İkinci takipçi: Sol kanat - Config'den katsayılar
            return (-aralik * HareketAyarlari.OK_FORMASYON_X_KATSAYISI, 
                   -aralik * HareketAyarlari.OK_FORMASYON_Z_KATSAYISI, 0)
        elif idx == 3:
            # Üçüncü takipçi: Sağ kanat - Config'den katsayılar
            return (aralik * HareketAyarlari.OK_FORMASYON_X_KATSAYISI, 
                   -aralik * HareketAyarlari.OK_FORMASYON_Z_KATSAYISI, 0)
        else:
            # Diğer takipçiler: Merkez çizgide devam eder
            merkez_sira = (idx - 1) // 3 + 1
            return (0, -aralik * (merkez_sira + 1), 0)
    
    @staticmethod
    def elmas_hesapla(i, n, aralik):
        """
        Elmas (DIAMOND) formasyonu: Lider merkezde, takipçiler elmas şeklinde.
        
        Args:
            i: Takipçi indeksi (0'dan başlar, lider hariç)
            n: Toplam ROV sayısı
            aralik: ROV'lar arası mesafe
        
        Returns:
            (x_offset, y_offset, z_offset): Formasyon ofseti
        """
        # Elmas şekli: 4 köşe + kenarlar
        elmas_boyutu = aralik * 1.5
        
        if i == 0:
            # Üst köşe
            return (0, elmas_boyutu, 0)
        elif i == 1:
            # Sağ köşe
            return (elmas_boyutu, 0, 0)
        elif i == 2:
            # Alt köşe
            return (0, -elmas_boyutu, 0)
        elif i == 3:
            # Sol köşe
            return (-elmas_boyutu, 0, 0)
        else:
            # Fazla takipçiler: Köşeler arası kenarlara yerleşir
            kenar_index = (i - 4) % 4
            kenar_pozisyon = (i - 4) // 4 + 1
            
            if kenar_index == 0:  # Üst-Sağ kenar
                return (elmas_boyutu * kenar_pozisyon / 2, elmas_boyutu * (1 - kenar_pozisyon / 2), 0)
            elif kenar_index == 1:  # Sağ-Alt kenar
                return (elmas_boyutu * (1 - kenar_pozisyon / 2), -elmas_boyutu * kenar_pozisyon / 2, 0)
            elif kenar_index == 2:  # Alt-Sol kenar
                return (-elmas_boyutu * kenar_pozisyon / 2, -elmas_boyutu * (1 - kenar_pozisyon / 2), 0)
            else:  # Sol-Üst kenar
                return (-elmas_boyutu * (1 - kenar_pozisyon / 2), elmas_boyutu * kenar_pozisyon / 2, 0)


# ==========================================
# ROTASYON MANTIĞI (Liderle Birlikte Dönme)
# ==========================================
def lokal_to_global(lider_pos, lider_hiz, offset):
    """
    Liderin baktığı yöne göre formasyon ofsetini dünya koordinatlarına çevirir.
    
    Args:
        lider_pos: Lider pozisyonu (x_2d, y_2d, z_depth) formatında
        lider_hiz: Lider hızı (velocity_x, velocity_z) formatında
        offset: Formasyon ofseti (x_offset, y_offset, z_offset)
    
    Returns:
        (x, y, z): Dünya koordinatlarındaki hedef pozisyon
    """
    lx, lz, ly = lider_pos  # x_2d, y_2d, z_depth
    dx, dz, dy = offset      # Formasyon ofsetleri
    
    # Liderin hareket açısı (atan2: velocity x ve z kullanır)
    # Eğer lider duruyorsa (hız yoksa), varsayılan olarak ileriye (Z+) baksın
    if math.sqrt(lider_hiz[0]**2 + lider_hiz[1]**2) < 0.1:
        aci = 0
    else:
        aci = math.atan2(lider_hiz[0], lider_hiz[1])
    
    # Rotasyon Matrisi (Z ekseni etrafında döndürme mantığı)
    # Liderin baktığı yönü 'İleri' kabul eder
    rotated_x = dx * math.cos(aci) + dz * math.sin(aci)
    rotated_z = -dx * math.sin(aci) + dz * math.cos(aci)
    
    # Dünya koordinatlarına ekle
    return (lx + rotated_x, ly + dy, lz + rotated_z)

