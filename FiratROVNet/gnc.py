import numpy as np
from ursina import Vec3, time, distance
from .config import cfg, GATLimitleri, SensorAyarlari, ModemAyarlari, HareketAyarlari, Formasyon
from .iletisim import AkustikModem
import math
import random

# Convex Hull için scipy import
try:
    from scipy.spatial import ConvexHull
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("⚠️ [UYARI] scipy.spatial.ConvexHull bulunamadı. ConvexHull fonksiyonu çalışmayacak.")

# ==========================================
# 0. KOORDİNAT TERCÜMANI
# ==========================================
class Koordinator:
    """
    Simülasyon ve Ursina koordinat sistemleri arasında dönüşüm yapar.
    
    Simülasyon Sistemi:
    - X: Sağ-Sol (horizontal)
    - Y: İleri-Geri (forward-backward)
    - Z: Derinlik (depth, pozitif = derin)
    
    Ursina Sistemi:
    - X: Sağ-Sol (horizontal, aynı)
    - Y: Yukarı-Aşağı (vertical, derinlik)
    - Z: İleri-Geri (forward-backward)
    """
    @staticmethod
    def sim_to_ursina(sim_x, sim_y, sim_z):
        """
        Sim (X:Sağ, Y:İleri, Z:Derinlik) -> Ursina (X, Y:Yukarı, Z:İleri)
        
        Args:
            sim_x: Sağ-Sol koordinatı
            sim_y: İleri-Geri koordinatı
            sim_z: Derinlik koordinatı
        
        Returns:
            tuple: (ursina_x, ursina_y, ursina_z)
        """
        return (sim_x, sim_z, sim_y)
    
    @staticmethod
    def ursina_to_sim(u_x, u_y, u_z):
        """
        Ursina (X, Y:Yukarı, Z:İleri) -> Sim (X, Y:İleri, Z:Derinlik)
        
        Args:
            u_x: Ursina X (sağ-sol)
            u_y: Ursina Y (yukarı-aşağı, derinlik)
            u_z: Ursina Z (ileri-geri)
        
        Returns:
            tuple: (sim_x, sim_y, sim_z)
        """
        return (u_x, u_z, u_y)

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
                    # Takipçiler için otomatik hedef belirleme yok
                    # Sadece baslangic_hedefleri içinde belirtilen hedefler atanır
                    # Eğer baslangic_hedefleri boş dict ise (senaryo modülü için), hedef atama yapma
                    if baslangic_hedefleri and baslangic_hedefleri != {} and i in baslangic_hedefleri:
                        # Manuel olarak belirtilen hedef varsa, onu kullan
                        hedef = baslangic_hedefleri[i]
                        if len(hedef) >= 3:
                            gnc.hedef_atama(hedef[0], hedef[1], hedef[2])
                        elif len(hedef) >= 2:
                            gnc.hedef_atama(hedef[0], hedef[1], 0)
                        # Eğer hedef belirtilmemişse, takipçi olduğu yerde bekler (hedef atama yapılmaz)
        
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
        
        # Tüm GNC sistemlerini güncelle
        for i, gnc in enumerate(self.sistemler):
            if i < len(tahminler):
                gnc.guncelle(tahminler[i])
    
    def set(self, rov_id, ayar_adi, deger):
        """
        ROV ayarlarını değiştirir.
        
        Args:
            rov_id: ROV ID (0, 1, 2, ...)
            ayar_adi: Ayar adı ('rol', 'renk', 'engel_mesafesi', 'iletisim_menzili', 'min_pil_uyarisi', 'kacinma_mesafesi', 'yaw')
            deger: Ayar değeri
                - 'yaw' için: Derece (0-360 arası, otomatik normalize edilir)
        
        Örnekler:
            filo.set(0, 'rol', 1)  # ROV-0'ı lider yap
            filo.set(1, 'renk', (255, 0, 0))  # ROV-1'i kırmızı yap
            filo.set(2, 'engel_mesafesi', 30.0)  # ROV-2'nin engel mesafesini ayarla
            filo.set(0, 'yaw', 90.0)  # ROV-0'ı 90 dereceye döndür
            filo.set(1, 'yaw', 180)  # ROV-1'i 180 dereceye döndür
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

    def get(self, rov_id=None, veri_tipi=None, taraf=None):
        """
        ROV bilgilerini alır.
        
        Args:
            rov_id: ROV ID (0, 1, 2, ...) veya None (tüm ROV'lar için)
            veri_tipi: Veri tipi ('gps', 'hiz', 'batarya', 'rol', 'renk', 'sensör', 
                                  'engel_mesafesi', 'iletisim_menzili', 'min_pil_uyarisi', 
                                  'kacinma_mesafesi', 'sonar', 'lidar', 'yaw', 'engels')
                                  veya None (tüm ROV'ların GPS koordinatları)
            taraf: Lidar için yön parametresi (sadece 'lidar' için geçerli)
                - 0: Ön (lidarx)
                - 1: Sağ (lidary)
                - 2: Sol (lidary1)
                - None: Tüm yönlerden en yakın engel mesafesi
        
        Returns:
            İstenen veri tipine göre değer veya tüm ROV'ların koordinatları
        
        Örnekler:
            # Tüm ROV'ların koordinatlarını al
            tum_rovlar = filo.get()  # {0: (x, y, z), 1: (x, y, z), ...}
            
            # Tek bir ROV için
            pozisyon = filo.get(0, 'gps')
            rol = filo.get(1, 'rol')
            sensörler = filo.get(2, 'sensör')
            batarya = filo.get(0, 'batarya')
            yaw_acisi = filo.get(0, 'yaw')  # Yaw açısı (derece)
            on_lidar = filo.get(0, 'lidar', 0)  # Ön lidar
            sag_lidar = filo.get(0, 'lidar', 1)  # Sağ lidar
            sol_lidar = filo.get(0, 'lidar', 2)  # Sol lidar
            en_yakin = filo.get(0, 'lidar')  # Tüm yönlerden en yakın
            engeller = filo.get(0, 'engels')  # Tüm tespit edilen engellerin koordinatları [(x,y,z), ...]
        """
        # Parametre verilmediyse tüm ROV'ların koordinatlarını döndür
        if rov_id is None and veri_tipi is None:
            return self._get_all_rovs_positions()
        
        # Sistemler listesi boş mu kontrol et
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            return None
        
        # ROV ID geçerliliği kontrolü
        if rov_id is not None and (not isinstance(rov_id, int) or rov_id < 0):
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return None
        
        if rov_id is not None and rov_id >= len(self.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return None
        
        try:
            # rov_id None kontrolü
            if rov_id is None:
                print(f"❌ [HATA] ROV ID belirtilmedi!")
                return None
            
            rov = self.sistemler[rov_id].rov
            # Lidar için özel işleme
            if veri_tipi == "lidar":
                deger = rov.get(veri_tipi, taraf=taraf)
            elif veri_tipi == "gps":
                # GPS'i Simülasyon formatına dönüştür
                ursina_gps = rov.get("gps")
                if ursina_gps is not None:
                    if isinstance(ursina_gps, np.ndarray):
                        ursina_gps = tuple(ursina_gps.tolist())
                    elif isinstance(ursina_gps, (tuple, list)):
                        ursina_gps = tuple(ursina_gps)
                    deger = Koordinator.ursina_to_sim(*ursina_gps)
                else:
                    deger = None
            elif veri_tipi == "engels":
                # Tüm lidar sensörlerinden engel koordinatlarını hesapla
                deger = self._compute_obstacle_positions(rov_id)
            else:
                deger = rov.get(veri_tipi)
            if deger is None:
                print(f"⚠️ [UYARI] ROV-{rov_id} için '{veri_tipi}' veri tipi bulunamadı")
            return deger
        except Exception as e:
            print(f"❌ [HATA] Veri alma sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _get_all_rovs_positions(self):
        """
        Tüm ROV'ların 3D koordinatlarını Simülasyon formatında döndürür.
        
        Returns:
            dict: {rov_id: (x, y, z), ...} - Tüm ROV'ların GPS koordinatları (Sim formatı)
                x: Sağ-Sol, y: İleri-Geri, z: Derinlik
        """
        all_positions = {}
        
        try:
            for i in range(len(self.sistemler)):
                if i < len(self.sistemler):
                    rov = self.sistemler[i].rov
                    # Ursina koordinatlarını al
                    ursina_pos = (rov.x, rov.y, rov.z)
                    # Simülasyon formatına dönüştür
                    sim_pos = Koordinator.ursina_to_sim(*ursina_pos)
                    all_positions[i] = sim_pos
        except Exception as e:
            print(f"❌ [HATA] Tüm ROV koordinatları alınırken hata: {e}")
            import traceback
            traceback.print_exc()
        
        return all_positions
    
    def points(self):
        """
        Tüm ROV koordinatlarını ve tüm engel koordinatlarını birleştirip döndürür.
        
        Returns:
            list: [(x, y, z), ...] - Tüm ROV koordinatları + tüm engel koordinatları birleşik liste
        
        Örnekler:
            tum_noktalar = filo.points()
            # Çıktı: [(x1, y1, z1), (x2, y2, z2), ...]  # ROV'lar + engeller
            
            # Convex Hull için kullanım
            points = filo.points()
            result = filo.ConvexHull(points, test_point, margin=0.2)
        """
        all_points = []
        
        try:
            # 1. Tüm ROV koordinatlarını al
            rovs_positions = self._get_all_rovs_positions()
            for rov_id, position in rovs_positions.items():
                if position is not None:
                    all_points.append(position)
            
            # 2. Her ROV için engel koordinatlarını al ve ekle
            for rov_id in rovs_positions.keys():
                engels = self._compute_obstacle_positions(rov_id)
                if engels:
                    all_points.extend(engels)
        
        except Exception as e:
            print(f"❌ [HATA] Points hesaplanırken hata: {e}")
            import traceback
            traceback.print_exc()
        
        return all_points
    
    def _compute_obstacle_positions(self, rov_id):
        """
        ROV'un tüm lidar sensörlerinden engel koordinatlarını hesaplar.
        Simülasyon formatında (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik) çalışır.
        
        Args:
            rov_id: ROV ID
        
        Returns:
            list: [(x, y, z), ...] - Tespit edilen engellerin koordinatları (Sim formatı)
        """
        # Lidar açısal offset'ler
        LIDAR_OFFSETS = {
            0: 0,     # ön
            1: -90,   # sağ
            2: 90     # sol
        }
        
        obstacles = []
        
        try:
            # ROV pozisyonu (Sim formatında)
            gps = self.get(rov_id, "gps")
            if gps is None:
                return []
            
            x0, y0, z0 = gps[0], gps[1], gps[2]  # Sim formatı: x=sağ, y=ileri, z=derinlik
            
            # ROV yaw açısı (derece) - Ursina'da 0 derece = İleri (Sim Y+)
            yaw_deg = self.get(rov_id, "yaw")
            if yaw_deg is None:
                yaw_deg = 0.0
            
            # Her lidar sensörü için kontrol et
            for lidar_indis in [0, 1, 2]:
                # Lidar mesafesi
                distance = self.get(rov_id, "lidar", lidar_indis)
                
                # Eğer engel tespit edilmişse (mesafe -1 değilse)
                if distance is not None and distance > 0 and distance != -1:
                    # Lidar açısal offset
                    offset = LIDAR_OFFSETS[lidar_indis]
                    
                    # Global açı
                    theta_deg = yaw_deg + offset
                    theta_rad = math.radians(theta_deg)
                    
                    # Engel koordinatı (Simülasyon formatında)
                    # Simülasyon Y ekseni İLERİ ise:
                    # x = merkez_x + mesafe * sin(theta)  (sağ-sol)
                    # y = merkez_y + mesafe * cos(theta)  (ileri-geri)
                    # z = merkez_z (derinlik aynı)
                    ox = x0 + distance * math.sin(theta_rad)
                    oy = y0 + distance * math.cos(theta_rad)
                    oz = z0  # Derinlik aynı
                    
                    obstacles.append((ox, oy, oz))
        
        except Exception as e:
            print(f"❌ [HATA] Engel koordinatları hesaplanırken hata: {e}")
            import traceback
            traceback.print_exc()
        
        return obstacles

    def formasyon(self, formasyon_id="LINE", aralik=15, is_3d=False, lider_koordinat=None):
        """
        Filoyu belirtilen formasyona sokar.
        Formasyon.pozisyonlar() ile pozisyonları alır ve filo.git() ile uygular.
        
        Args:
            formasyon_id (str veya int): Formasyon tipi (varsayılan: "LINE")
                - Config.py'deki Formasyon.TIPLER listesindeki tiplerden biri
                - Veya 0-14 arası indeks
            aralik (float): ROV'lar arası mesafe (varsayılan: 15)
            is_3d (bool): 3D formasyon modu (varsayılan: False - 2D)
                - True: ROV'lar 3D uzayda (farklı derinliklerde) dizilir
                - False: ROV'lar 2D düzlemde (aynı derinlikte) dizilir
            lider_koordinat (tuple, optional): (x, y, z) - Lider koordinatı (varsayılan: None)
                - Verilirse, lider bu koordinattaymış gibi pozisyonlar hesaplanır
                - Format: (x, y, z) - x,y: 2D koordinatlar, z: derinlik
                - None ise liderin gerçek pozisyonu kullanılır ve ROV'lar hareket eder
                - Verilirse, sadece pozisyonlar hesaplanır ve döndürülür (ROV'lar hareket etmez)
        
        Returns:
            None: lider_koordinat verilmediğinde (ROV'lar hareket eder)
            list: lider_koordinat verildiğinde - [(x, z, y), ...] Ursina formatında pozisyonlar
        
        Örnekler:
            filo.formasyon()  # Varsayılan LINE formasyonu (2D), ROV'lar hareket eder
            filo.formasyon("V_SHAPE", aralik=20)  # V şekli formasyon, 20 birim aralık (2D)
            filo.formasyon("DIAMOND", aralik=25, is_3d=True)  # Elmas formasyonu, 3D mod
            filo.formasyon(1, aralik=20, is_3d=True)  # İndeks ile: V_SHAPE, 3D mod
            
            # Sadece pozisyonları hesapla (ROV'ları hareket ettirme)
            pozisyonlar = filo.formasyon("V_SHAPE", aralik=20, lider_koordinat=(10, 20, -5))
            # Çıktı: [(x1, z1, y1), (x2, z2, y2), ...] - Ursina formatında
        """
        # 1. ADIM: Formasyon.pozisyonlar() ile pozisyonları al
        formasyon_obj = Formasyon(self)
        pozisyonlar = formasyon_obj.pozisyonlar(formasyon_id, aralik, is_3d=is_3d, lider_koordinat=lider_koordinat)
        
        if not pozisyonlar or len(pozisyonlar) == 0:
            print("❌ [FORMASYON] Pozisyonlar alınamadı!")
            return None if lider_koordinat is not None else None
        
        if len(pozisyonlar) != len(self.sistemler):
            print(f"⚠️ [FORMASYON] Uyarı: Pozisyon sayısı ({len(pozisyonlar)}) ROV sayısı ({len(self.sistemler)}) ile eşleşmiyor!")
        
        # Eğer lider_koordinat verilmişse, sadece pozisyonları döndür (ROV'ları hareket ettirme)
        if lider_koordinat is not None:
            # Pozisyonları Ursina formatına dönüştür ve döndür
            ursina_positions = []
            for pozisyon in pozisyonlar:
                config_x, config_y, config_z = pozisyon
                # Config (x, y, z) -> Ursina (x, z, y)
                ursina_x = config_x  # x: sağ-sol (aynı)
                ursina_z = config_y  # Config'deki y -> Ursina'da z (ileri-geri)
                ursina_y = config_z  # Config'deki z -> Ursina'da y (derinlik)
                
                # lider_koordinat verildiğinde yüzey kontrolü yapma, koordinatı olduğu gibi kullan
                ursina_positions.append((ursina_x, ursina_z, ursina_y))
            
            print(f"✅ [FORMASYON] Pozisyonlar hesaplandı: Tip={formasyon_id}, Aralık={aralik}, ROV Sayısı={len(pozisyonlar)}")
            return ursina_positions
        
        # 2. ADIM: Her ROV için pozisyonu filo.git() ile uygula (lider_koordinat verilmemişse)
        # Formasyon.pozisyonlar() zaten mutlak pozisyonları döndürüyor (lider pozisyonu + offset'ler)
        # Format: (x, y, z) - x,y: 2D koordinatlar, z: derinlik (Config formatı = Sim formatı)
        # filo.git() artık Sim formatında çalışıyor: (x, y, z) - x: sağ-sol, y: ileri-geri, z: derinlik
        for i, pozisyon in enumerate(pozisyonlar):
            if i >= len(self.sistemler):
                break
            
            # Config formatı = Sim formatı: (x, y, z)
            sim_x, sim_y, sim_z = pozisyon
            # x: sağ-sol (aynı)
            # y: ileri-geri (aynı)
            # z: derinlik (aynı)
            
            # Eğer yüzeydeyse (z >= 0), su altına gönder
            if sim_z >= 0:
                sim_z = -10.0
            
            # filo.git() ile hedefi uygula (Sim formatında)
            try:
                self.git(i, sim_x, sim_y, sim_z, ai=True)
                print(f"✅ [FORMASYON] ROV-{i} hedefi ayarlandı: ({sim_x:.2f}, {sim_y:.2f}, {sim_z:.2f})")
            except Exception as e:
                print(f"⚠️ [FORMASYON] ROV-{i} için hedef ayarlanırken hata: {e}")
        
        print(f"✅ [FORMASYON] Formasyon kuruldu: Tip={formasyon_id}, Aralık={aralik}, ROV Sayısı={len(pozisyonlar)}")
        return None
    def formasyon_sec(self, margin=30, is_3d=False, offset=20.0):
        """
        Convex hull kullanarak en uygun formasyonu seçer.

        KESİN KURALLAR:
        - Güvenlik hull (sanal + gerçek engeller) SADECE 1 KEZ hesaplanır (sabit hull)
        - Margin sadece formasyon_aralik için kullanılır (ROV'lar arası mesafe)
        - Hull içinde kalma kontrolü margin olmadan yapılır
        - İlk geçerli formasyon bulunduğunda DERHAL döner

        Args:
            margin (float): Formasyon aralığı için kullanılır (varsayılan: 30)
                - formasyon_aralik = margin * 0.6 (ROV'lar arası mesafe)
            is_3d (bool): 3D formasyon modu (varsayılan: False)
            offset (float): ROV hull genişletme mesafesi (varsayılan: 20.0)

        Returns:
            int | None: Seçilen formasyon ID'si veya None (uygun formasyon bulunamazsa)
        """
        try:
            # 1. Güvenlik hull'u oluştur (sanal + gerçek engeller, SADECE 1 KEZ)
            guvenlik_hull_dict = self.guvenlik_hull_olustur(offset=offset)

            hull = guvenlik_hull_dict.get("hull")
            merkez = guvenlik_hull_dict.get("center")

            if hull is None or merkez is None:
                return None

            # 2. Formasyon aralığı parametreleri
            min_aralik = margin * 0.2
            baslangic_aralik = margin * 0.6
            adim = 1.0  # metre

            # 3. Formasyon tiplerini sırayla dene
            for i, formasyon_tipi in enumerate(Formasyon.TIPLER):
                aralik = baslangic_aralik

                while aralik >= min_aralik:
                    test_points = self.formasyon(
                        i,
                        aralik=aralik,
                        is_3d=is_3d,
                        lider_koordinat=merkez
                    )

                    if (
                        test_points
                        and self._formasyon_gecerli_mi(
                            test_points,
                            hull,
                            aralik
                        )
                    ):
                        print(
                            f"✅ [FORMASYON_SEC] Formasyon seçildi: {formasyon_tipi} "
                            f"(ID={i}, aralık={aralik:.1f}m)"
                        )

                        # Formasyon pozisyonlarını al (Ursina formatında)
                        ursina_positions = self.formasyon(
                            i,
                            aralik=aralik,
                            is_3d=is_3d,
                            lider_koordinat=merkez
                        )
                        
                        if not ursina_positions:
                            print("⚠️ [FORMASYON_SEC] Formasyon pozisyonları alınamadı!")
                            return None
                        
                        # Lider ROV'u merkeze gönder
                        lider_rov_id = None
                        for rov_id in range(len(self.sistemler)):
                            if self.get(rov_id, "rol") == 1:
                                lider_rov_id = rov_id
                                self.git(
                                    rov_id,
                                    merkez[0],
                                    merkez[1],
                                    merkez[2]
                                )
                                break
                        
                        # Takipçi ROV'ları formasyon pozisyonlarına gönder
                        for rov_id, ursina_pos in enumerate(ursina_positions):
                            if rov_id >= len(self.sistemler):
                                break
                            
                            # Lider'i atla (zaten merkeze gönderildi)
                            if rov_id == lider_rov_id:
                                continue
                            
                            # Ursina formatından (x, z, y) -> Sim formatına (x, y, z) dönüştür
                            sim_pos = tuple(ursina_pos) # (x, z, y) -> (x, y, z)
                            sim_x, sim_y, sim_z = sim_pos
                            
                            # Eğer yüzeydeyse (z >= 0), su altına gönder
                            if sim_z >= 0:
                                sim_z = -10.0
                            
                            # Takipçi ROV'u formasyon pozisyonuna gönder
                            self.git(rov_id, sim_x, sim_y, sim_z, ai=True)
                            print(f"✅ [FORMASYON_SEC] ROV-{rov_id} formasyon pozisyonuna gönderildi: ({sim_x:.2f}, {sim_y:.2f}, {sim_z:.2f})")

                        return i

                    aralik -= adim

            # Hiçbir formasyon geçerli değil
            return None

        except Exception as e:
            print(f"❌ [HATA] Formasyon seçimi sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return None

    
    def _formasyon_gecerli_mi(self, test_points, hull, formasyon_aralik):
        """
        Formasyon pozisyonlarının geçerli olup olmadığını kontrol eder.
        
        Args:
            test_points: list - [(x, z, y), ...] Ursina formatında formasyon pozisyonları
            hull: ConvexHull - Güvenlik hull (Simülasyon formatında)
            formasyon_aralik: float - ROV'lar arası minimum mesafe
        
        Returns:
            bool: True if formasyon geçerli, False otherwise
        """
        if hull is None or test_points is None or len(test_points) == 0:
            return False
        
        try:
            # 1. Tüm pozisyonlar hull içinde mi?
            for test_point in test_points:
                # Ursina formatından (x, z, y) -> Simülasyon formatına (x, y, z) dönüştür
                #sim_point = Koordinator.ursina_to_sim(*test_point)
                #point_3d = np.array(sim_point)
                point_3d = np.array(test_point)
                
                # _is_point_inside_hull hem 2D hem 3D hull'u destekler
                if not self._is_point_inside_hull(point_3d, hull):
                    #print(f"❌ [FORMASYON_GECERLI_MI] Pozisyon hull dışında: {test_point}")
                    return False
            
            # 2. ROV'lar arası mesafe kontrolü (Simülasyon formatında)
            for i in range(len(test_points)):
                for j in range(i + 1, len(test_points)):
                    # Ursina formatından Sim formatına dönüştür
                    p1_sim = Koordinator.ursina_to_sim(*test_points[i])
                    p2_sim = Koordinator.ursina_to_sim(*test_points[j])
                    p1 = np.array(p1_sim)
                    p2 = np.array(p2_sim)
                    mesafe = np.linalg.norm(p1 - p2)
                    
                    if mesafe < formasyon_aralik:
                        return False
            
            return True
            
        except Exception as e:
            print(f"❌ [HATA] Formasyon geçerliliği kontrolü sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def hedef(self, x=None, y=None, z=None):
        """
        Sadece lider ROV'un hedefini ayarlar. Takipçiler bu komuttan etkilenmez.
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
            filo.hedef(50, 60)  # Sadece lider (50, 60, 0) hedefine gider
            filo.hedef(40, 50)  # Sadece lider (40, 50, 0) hedefine gider
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
        
        # Sadece liderin hedefini güncelle (Sim formatında)
        # Sadece liderin hedefini güncelle (Sim formatında)
        # filo.git() artık Sim formatında çalışıyor: (x, y, z)
        self.git(lider_rov_id, x, y, z, ai=True)
        
        # Hedef görselini oluştur/güncelle (Ursina formatına dönüştür)
        ursina_pos = Koordinator.sim_to_ursina(x, y, z)
        self._hedef_gorsel_olustur(*ursina_pos)
        
        # Haritaya hedefi ekle
        if self.ortam_ref and hasattr(self.ortam_ref, 'harita'):
            self.ortam_ref.harita.hedef_pozisyon = (x, y)
        
        print(f"✅ [HEDEF] Lider hedefi güncellendi: ({x:.2f}, {y:.2f}, 0) - Su üstünde. Takipçiler de aynı hedefe gidiyor.")
        
        # Hedef koordinatlarını döndür
        return (x, y, 0)
    
    def ConvexHull(self, points, test_point, margin=0.0):
        """
        3D Convex Hull oluşturur ve test noktasının hull içinde olup olmadığını kontrol eder.
        
        Args:
            points: Nx3 numpy array veya liste - Convex hull oluşturmak için kullanılacak noktalar
            test_point: (x, y, z) tuple veya liste - Test edilecek nokta
            margin: float - Minimum mesafe (hull yüzeyinden ne kadar uzakta olmalı)
        
        Returns:
            dict: {
                'inside': bool - Test noktası hull içinde mi? (margin ile)
                'center': (x, y, z) - Convex hull'un merkezi (3D koordinat)
                'hull': ConvexHull objesi (None if scipy not available)
            }
        
        Örnekler:
            points = np.array([[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0], [0, 0, 2], [2, 2, 2]])
            test_point = [1, 1, 1]
            result = filo.ConvexHull(points, test_point, margin=0.2)
            print(f"İçinde mi: {result['inside']}, Merkez: {result['center']}")
        """
        if not SCIPY_AVAILABLE:
            print("❌ [HATA] scipy.spatial.ConvexHull bulunamadı!")
            return {
                'inside': False,
                'center': None,
                'hull': None
            }
        
        try:
            # Points'i numpy array'e çevir
            points = np.asarray(points)
            if points.ndim != 2 or points.shape[1] != 3:
                print(f"❌ [HATA] Points Nx3 formatında olmalı! Alınan shape: {points.shape}")
                return {
                    'inside': False,
                    'center': None,
                    'hull': None
                }
            
            # Test point'i numpy array'e çevir
            test_point = np.asarray(test_point)
            if test_point.shape != (3,):
                print(f"❌ [HATA] Test point (x, y, z) formatında olmalı! Alınan shape: {test_point.shape}")
                return {
                    'inside': False,
                    'center': None,
                    'hull': None
                }
            
            # En az 4 nokta gerekli (3D convex hull için)
            if len(points) < 4:
                print(f"⚠️ [UYARI] 3D Convex Hull için en az 4 nokta gerekli! Alınan: {len(points)}")
                # Yeterli nokta yoksa, merkezi hesapla ve inside=False döndür
                center = np.mean(points, axis=0)
                return {
                    'inside': False,
                    'center': tuple(center),
                    'hull': None
                }
            
            # Convex Hull oluştur
            hull = ConvexHull(points)
            
            # Hull merkezini hesapla (tüm noktaların ortalaması)
            center = np.mean(points, axis=0)
            
            # Test noktasının hull içinde olup olmadığını kontrol et
            inside = self._is_point_inside_hull(test_point, hull)
            
            return {
                'inside': inside,
                'center': tuple(center),
                'hull': hull
            }
            
        except Exception as e:
            print(f"❌ [HATA] ConvexHull hesaplama sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return {
                'inside': False,
                'center': None,
                'hull': None
            }
    
    def _is_point_inside_hull(self, point, hull):
        """
        Noktanın convex hull içinde olup olmadığını kontrol eder.
        2D veya 3D hull destekler.
        
        Mantık: Scipy hull.equations içindeki her bir denklem için 
        np.dot(normal, point) + d <= 0 ise nokta içeridedir.
        Tek bir denklem bile > 0 sonucunu verirse nokta dışarıdadır.
        
        Args:
            point: (x, y, z) numpy array veya (x, y) numpy array (2D için)
            hull: scipy.spatial.ConvexHull (2D veya 3D)
        
        Returns:
            bool: True if point is inside hull, False otherwise
        """
        point = np.asarray(point)
        
        # Hull'un boyutunu kontrol et (2D veya 3D)
        hull_dim = hull.points.shape[1]
        point_dim = point.shape[0]
        
        # Eğer hull 2D ise, point'i de 2D'ye çevir
        if hull_dim == 2:
            if point_dim == 3:
                # 3D point'ten 2D'ye çevir (x, y, z) -> (x, y)
                point_2d = np.array([point[0], point[1]])
            else:
                point_2d = point
        else:
            # 3D hull
            point_2d = point
        
        # Hull equations'ları kullanarak kontrol et
        for eq in hull.equations:
            normal = eq[:-1]
            d = eq[-1]
            
            # np.dot(normal, point) + d <= 0 ise nokta içeridedir
            # Tek bir denklem bile > 0 sonucunu verirse nokta dışarıdadır
            if np.dot(normal, point_2d) + d > 0:
                return False
        
        return True

    
    def genisletilmis_rov_hull_olustur(self, offset=20.0):
        """
        ROV'ların convex hull'unu alır ve kenarlarına dik yönde offset uygular.
        Sistem 2D (x-z düzlemi) çalışır, derinlik sabit kabul edilir.
        
        Args:
            offset (float): Hull kenarlarından dışarı offset mesafesi (metre, varsayılan: 20.0)
        
        Returns:
            list: [(x, y, z), ...] - Genişletilmiş sanal engel noktaları
        """
        if not SCIPY_AVAILABLE:
            print("❌ [HATA] scipy.spatial.ConvexHull bulunamadı!")
            return []
        
        try:
            # 1. Tüm ROV koordinatlarını al (2D: x-z düzlemi)
            rovs_positions = self._get_all_rovs_positions()
            if len(rovs_positions) < 3:
                # En az 3 ROV gerekli (2D convex hull için)
                print(f"⚠️ [UYARI] 2D Convex Hull için en az 3 ROV gerekli! Mevcut: {len(rovs_positions)}")
                return []
            
            # 2. ROV pozisyonlarını 2D (x, y) formatına çevir (z derinlik sabit)
            # x, y: 2D düzlem koordinatları
            # z: derinlik
            rov_2d_points = []
            ortalama_z = 0.0
            for rov_id, position in rovs_positions.items():
                if position is not None:
                    x, y, z = position
                    rov_2d_points.append([x, y])  # 2D: x-y düzlemi
                    ortalama_z += z
            
            ortalama_z /= len(rovs_positions) if len(rovs_positions) > 0 else 1
            rov_2d_points = np.array(rov_2d_points)
            
            # 3. ROV'ların 2D convex hull'unu hesapla
            if len(rov_2d_points) < 3:
                return []
            
            rov_hull_2d = ConvexHull(rov_2d_points)
            
            # 4. Hull kenarlarına dik yönde offset uygula
            genisletilmis_noktalar = []
            
            # Her hull kenarı için
            for i in range(len(rov_hull_2d.vertices)):
                # Kenar başlangıç ve bitiş noktaları
                v1_idx = rov_hull_2d.vertices[i]
                v2_idx = rov_hull_2d.vertices[(i + 1) % len(rov_hull_2d.vertices)]
                
                p1 = rov_2d_points[v1_idx]  # [x, z]
                p2 = rov_2d_points[v2_idx]  # [x, z]
                
                # Kenar vektörü
                edge_vec = p2 - p1
                edge_length = np.linalg.norm(edge_vec)
                
                if edge_length < 1e-6:
                    continue
                
                # Kenar vektörünü normalize et
                edge_vec_normalized = edge_vec / edge_length
                
                # Dik vektör (saat yönünün tersine 90 derece döndür)
                # 2D'de: (x, z) -> (-z, x) saat yönünün tersine
                normal_vec = np.array([-edge_vec_normalized[1], edge_vec_normalized[0]])
                
                # Offset uygula: her iki noktayı da normal yönde offset kadar kaydır
                p1_offset = p1 + normal_vec * offset
                p2_offset = p2 + normal_vec * offset
                
                # 3D formatına çevir (z derinlik sabit)
                genisletilmis_noktalar.append((p1_offset[0], p1_offset[1], ortalama_z))  # (x, y, z)
                genisletilmis_noktalar.append((p2_offset[0], p2_offset[1], ortalama_z))  # (x, y, z)
            
            return genisletilmis_noktalar
            
        except Exception as e:
            print(f"❌ [HATA] Genişletilmiş ROV hull oluşturulurken hata: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    def lidar_engel_noktalari(self):
        """
        Tüm ROV'lardan lidar ile tespit edilen gerçek engel koordinatlarını toplar.
        
        Returns:
            list: [(x, y, z), ...] - Tüm tespit edilen engellerin koordinatları
        """
        tum_engeller = []
        
        try:
            # Tüm ROV'lar için
            for rov_id in range(len(self.sistemler)):
                engels = self._compute_obstacle_positions(rov_id)
                if engels:
                    tum_engeller.extend(engels)
        
        except Exception as e:
            print(f"❌ [HATA] Lidar engel noktaları toplanırken hata: {e}")
            import traceback
            traceback.print_exc()
        
        return tum_engeller
    
    def guvenlik_hull_olustur(self, offset=20.0):
        """
        Sanal engel noktaları (genişletilmiş ROV hull) + gerçek engel noktalarını birleştirir
        ve Convex Hull hesaplar.
        
        Args:
            offset (float): ROV hull genişletme mesafesi (varsayılan: 20.0)
        
        Returns:
            dict: {
                'hull': ConvexHull objesi veya None,
                'points': numpy array - Hull hesaplamasında kullanılan noktalar,
                'center': (x, y, z) - Hull merkezi veya None
            }
        """
        if not SCIPY_AVAILABLE:
            print("❌ [HATA] scipy.spatial.ConvexHull bulunamadı!")
            hull_data = {
                'hull': None,
                'points': None,
                'center': None
            }
            
            # Hull bilgisini haritaya aktar (eğer harita varsa)
            if self.ortam_ref and hasattr(self.ortam_ref, 'harita') and self.ortam_ref.harita:
                self.ortam_ref.harita.convex_hull_data = hull_data
            
            return hull_data
        
        try:
            # 1. Genişletilmiş ROV hull noktalarını al (sanal engeller)
            sanal_engeller = self.genisletilmis_rov_hull_olustur(offset=offset)
            
            # 2. Gerçek engel noktalarını al (lidar)
            gercek_engeller = self.lidar_engel_noktalari()
            
            # 3. ROV pozisyonlarını da ekle (tüm ROV'ların hull içinde kalması için)
            rovs_positions = self._get_all_rovs_positions()
            rov_pozisyonlari = []
            for rov_id, position in rovs_positions.items():
                if position is not None:
                    rov_pozisyonlari.append(position)
            
            # 4. Birleştir: Genişletilmiş ROV hull + Gerçek engeller + ROV pozisyonları
            tum_noktalar = sanal_engeller + gercek_engeller + rov_pozisyonlari
            
            if len(tum_noktalar) == 0:
                print("⚠️ [UYARI] Güvenlik hull için nokta bulunamadı!")
                hull_data = {
                    'hull': None,
                    'points': None,
                    'center': None
                }
                
                # Hull bilgisini haritaya aktar (eğer harita varsa)
                if self.ortam_ref and hasattr(self.ortam_ref, 'harita') and self.ortam_ref.harita:
                    self.ortam_ref.harita.convex_hull_data = hull_data
                
                return hull_data
            
            # 4. Noktaları 2D (x-y düzlemi) formatına çevir ve derinlik bilgisini sakla
            # x, y: 2D düzlem koordinatları
            # z: derinlik
            points_2d = []
            z_degerleri = []
            
            for nokta in tum_noktalar:
                x, y, z = nokta
                points_2d.append([x, y])  # 2D: x-y düzlemi
                z_degerleri.append(z)
            
            points_2d = np.array(points_2d)
            ortalama_z = np.mean(z_degerleri) if z_degerleri else 0.0
            
            # 5. En az 3 nokta gerekli (2D convex hull için)
            if len(points_2d) < 3:
                print(f"⚠️ [UYARI] 2D Convex Hull için en az 3 nokta gerekli! Mevcut: {len(points_2d)}")
                # Yeterli nokta yoksa, merkezi hesapla
                if len(points_2d) > 0:
                    center_2d = np.mean(points_2d, axis=0)
                    center = (center_2d[0], center_2d[1], ortalama_z)  # (x, y, z)
                else:
                    center = (0.0, 0.0, ortalama_z)
                hull_data = {
                    'hull': None,
                    'points': np.array(tum_noktalar) if tum_noktalar else None,
                    'center': center
                }
                
                # Hull bilgisini haritaya aktar (eğer harita varsa)
                if self.ortam_ref and hasattr(self.ortam_ref, 'harita') and self.ortam_ref.harita:
                    self.ortam_ref.harita.convex_hull_data = hull_data
                
                return hull_data
            
            # 6. 2D Convex Hull hesapla (TEK KEZ)
            try:
                hull_2d = ConvexHull(points_2d)
            except Exception as e:
                print(f"⚠️ [UYARI] 2D Convex Hull hesaplanamadı: {e}")
                # QJ (joggle) seçeneği ile tekrar dene
                try:
                    hull_2d = ConvexHull(points_2d, qhull_options='QJ')
                except Exception as e2:
                    print(f"❌ [HATA] 2D Convex Hull (QJ ile) hesaplanamadı: {e2}")
                    center_2d = np.mean(points_2d, axis=0)
                    center = (center_2d[0], center_2d[1], ortalama_z)  # (x, y, z)
                    hull_data = {
                        'hull': None,
                        'points': np.array(tum_noktalar),
                        'center': center
                    }
                    
                    # Hull bilgisini haritaya aktar (eğer harita varsa)
                    if self.ortam_ref and hasattr(self.ortam_ref, 'harita') and self.ortam_ref.harita:
                        self.ortam_ref.harita.convex_hull_data = hull_data
                    
                    return hull_data
            
            # 7. 2D hull'u 3D'ye çevir (derinlik bilgisini ekle)
            # Hull'u 3D noktalara çevir (z derinlik sabit)
            hull_3d_points = []
            for vertex_idx in hull_2d.vertices:
                x, y = points_2d[vertex_idx]
                hull_3d_points.append([x, y, ortalama_z])  # (x, y, z)
            
            hull_3d_points = np.array(hull_3d_points)
            
            # 8. 3D hull oluştur (eğer yeterli nokta varsa)
            hull = None
            if len(hull_3d_points) >= 4:
                try:
                    hull = ConvexHull(hull_3d_points, qhull_options='QJ')
                except Exception:
                    # 3D hull oluşturulamazsa, 2D hull'u kullan
                    # Bu durumda _is_point_inside_hull fonksiyonu 2D'yi destekliyor
                    # Şimdilik 2D hull'u saklayalım
                    pass
            
            # 9. Hull merkezini hesapla
            center_2d = np.mean(points_2d, axis=0)
            center = (center_2d[0], center_2d[1], ortalama_z)  # (x, y, z)
            
            # Eğer 3D hull oluşturulamadıysa, 2D hull'u kullan
            if hull is None:
                # 2D hull'u kullan (zaten _is_point_inside_hull 2D'yi destekliyor)
                hull = hull_2d
                # 2D hull için points_2d'yi sakla
                points_for_hull = points_2d
            else:
                # 3D hull için points_3d'yi sakla
                points_for_hull = np.array(tum_noktalar)
            
            hull_data = {
                'hull': hull,
                'points': points_for_hull,
                'center': tuple(center)
            }
            
            # Hull bilgisini haritaya aktar (eğer harita varsa)
            if self.ortam_ref and hasattr(self.ortam_ref, 'harita') and self.ortam_ref.harita:
                self.ortam_ref.harita.convex_hull_data = hull_data
            
            return hull_data
            
        except Exception as e:
            print(f"❌ [HATA] Güvenlik hull oluşturulurken hata: {e}")
            import traceback
            traceback.print_exc()
            return {
                'hull': None,
                'points': None,
                'center': None
            }
    
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
    

    def git(self, rov_id, x, y, z=None, ai=True):
        """
        ROV'a hedef koordinatı atar ve otomatik moda geçirir.
        Tüm girişler Simülasyon formatındadır: (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik)

        Args:
            rov_id: ROV ID (0, 1, 2, ...)
            x: X koordinatı (Sağ-Sol)
            y: Y koordinatı (İleri-Geri)
            z: Z koordinatı (Derinlik, opsiyonel)
                - None ise mevcut derinlik korunur
            ai: AI aktif/pasif (varsayılan: True)
                - True: Zeki Mod (GAT tahminleri kullanılır)
                - False: Kör Mod (GAT tahminleri görmezden gelinir)

        Örnekler:
            filo.git(0, 40, 60, 20)           # ROV-0: X=40 (sağ), Y=60 (ileri), Z=20 (derinlik), AI açık
            filo.git(1, 50, 50, -10, ai=False)  # ROV-1: X=50, Y=50, Z=-10, AI kapalı
            filo.git(2, 30, 40)               # ROV-2: X=30, Y=40, mevcut derinlik, AI açık
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
        
        # Eğer Z (derinlik) verilmemişse mevcut derinliği koru
        if z is None:
            current_sim_pos = Koordinator.ursina_to_sim(
                self.sistemler[rov_id].rov.x,
                self.sistemler[rov_id].rov.y,
                self.sistemler[rov_id].rov.z
            )
            z = current_sim_pos[2]
        
        # GNC'ye hedefi SİMÜLASYON formatında veriyoruz
        try:
            self.sistemler[rov_id].hedef_atama(x, y, z)
            ai_durum = "AÇIK" if ai else "KAPALI (Kör Mod)"
            print(f"✅ [FİLO] ROV-{rov_id} Hedef: X:{x}, Y:{y}, Z:{z} (Sim Formatı) | AI: {ai_durum}")
        except Exception as e:
            print(f"❌ [HATA] Hedef atama sırasında hata: {e}")
            import traceback
            traceback.print_exc()

    def move(self, rov_id, yon, guc=1.0):
        """
        ROV'a güç bazlı hareket komutu verir (gerçek dünya gibi, gerçekçi fizik ile).
        
        Args:
            rov_id: ROV ID
            yon: Hareket yönü ('ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur', 'yaw')
            guc: Motor gücü (0.0 - 1.0 arası, varsayılan: 1.0)
                - Normal hareket için: 0.0 - 1.0 arası
                - Yaw rotasyonu için: -1.0 ile 1.0 arası
                    - 1.0 = Saat yönünün tersine döndürme (pozitif yaw)
                    - -1.0 = Saat yönünde döndürme (negatif yaw)
        
        Örnekler:
            filo.move(0, 'ileri', 1.0)   # ROV-0 %100 güçle ileri
            filo.move(1, 'sag', 0.5)     # ROV-1 %50 güçle sağa
            filo.move(2, 'cik', 0.3)      # ROV-2 %30 güçle yukarı
            filo.move(3, 'dur', 0.0)      # ROV-3 dur (güç=0)
            filo.move(0, 'ileri')         # ROV-0 %100 güçle ileri (varsayılan)
            filo.move(0, 'yaw', 1.0)     # ROV-0 saat yönünün tersine döndürme
            filo.move(0, 'yaw', -1.0)    # ROV-0 saat yönünde döndürme
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
        gecerli_yonler = ['ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur', 'yaw']
        if yon not in gecerli_yonler:
            print(f"❌ [HATA] Geçersiz hareket yönü: '{yon}'")
            print(f"   Geçerli yönler: {', '.join(gecerli_yonler)}")
            return
        
        # Güç değerini kontrol et
        if not isinstance(guc, (int, float)):
            print(f"❌ [HATA] Güç değeri sayı olmalı: {guc}")
            return
        
        # Yaw rotasyonu için özel güç kontrolü (-1.0 ile 1.0 arası)
        if yon == 'yaw':
            guc = max(-1.0, min(1.0, float(guc)))
        else:
            # Normal hareket için güç kontrolü (0.0 - 1.0 arası)
            guc = max(0.0, min(1.0, float(guc)))
        
        try:
            # Manuel kontrolü aç
            self.sistemler[rov_id].manuel_kontrol = True
            gnc = self.sistemler[rov_id]
            rov = gnc.rov
            
            # Yaw rotasyonu özel durum
            if yon == 'yaw':
                # Yaw rotasyonu için rotation.y güncelle
                # Güç değeri: 1.0 = saat yönünün tersine, -1.0 = saat yönünde
                # Maksimum dönüş hızı: 90 derece/saniye (config'den alınabilir)
                from .config import HareketAyarlari
                yaw_hizi = abs(guc) * 90.0  # Derece/saniye (maksimum 90 derece/saniye)
                yaw_delta = yaw_hizi * time.dt  # Bu frame'de döndürülecek açı (küçük adım)
                
                # Mevcut rotation değerini al ve Vec3 olarak ayarla
                if not hasattr(rov, 'rotation') or rov.rotation is None:
                    rov.rotation = Vec3(0, 0, 0)
                elif not isinstance(rov.rotation, Vec3):
                    # Tuple veya list ise Vec3'e dönüştür
                    if isinstance(rov.rotation, (tuple, list)) and len(rov.rotation) >= 3:
                        rov.rotation = Vec3(rov.rotation[0], rov.rotation[1], rov.rotation[2])
                    else:
                        rov.rotation = Vec3(0, 0, 0)
                
                # Mevcut rotation değerlerini al
                current_x = rov.rotation.x if isinstance(rov.rotation, Vec3) else 0
                current_y = rov.rotation.y if isinstance(rov.rotation, Vec3) else 0
                current_z = rov.rotation.z if isinstance(rov.rotation, Vec3) else 0
                
                # Y ekseni etrafında döndür (yaw) - küçük adımlarla
                if guc > 0:
                    # Pozitif güç: saat yönünün tersine (pozitif yaw)
                    new_y = current_y + yaw_delta
                elif guc < 0:
                    # Negatif güç: saat yönünde (negatif yaw)
                    new_y = current_y - yaw_delta
                else:
                    new_y = current_y
                
                # Rotation'ı normalize et (0-360 arası tutmak için)
                while new_y >= 360:
                    new_y -= 360
                while new_y < 0:
                    new_y += 360
                
                # Rotation'ı yeni Vec3 olarak atama (küçük adımlarla güncelleme)
                rov.rotation = Vec3(current_x, new_y, current_z)
                
                # Manuel hareket modunu ayarla (sürekli yaw için)
                if hasattr(rov, 'manuel_hareket'):
                    rov.manuel_hareket['yon'] = 'yaw'
                    rov.manuel_hareket['guc'] = guc
                
                guc_yuzdesi = int(abs(guc) * 100)
                yon_metni = "saat yönünün tersine" if guc > 0 else "saat yönünde"
                print(f"🔄 [FİLO] ROV-{rov_id} {yon_metni} %{guc_yuzdesi} güçle döndürülüyor (yaw)")
                return
            
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
            # ROV'un yaw rotasyonunu al (Y ekseni etrafında dönme açısı - derece)
            yaw_acisi = 0.0
            if hasattr(rov, 'rotation') and rov.rotation is not None:
                if isinstance(rov.rotation, Vec3):
                    yaw_acisi = rov.rotation.y
                elif isinstance(rov.rotation, (tuple, list)) and len(rov.rotation) >= 2:
                    yaw_acisi = rov.rotation[1]
            
            # Yaw açısını radyana çevir
            from math import sin, cos, radians
            yaw_radyan = radians(yaw_acisi)
            
            hareket_vektoru = Vec3(0, 0, 0)
            if yon == 'ileri':
                # İleri: ROV'un baktığı yön (Z ekseni pozitif yönü, yaw açısına göre döndürülmüş)
                hareket_vektoru.x = sin(yaw_radyan)
                hareket_vektoru.z = cos(yaw_radyan)
            elif yon == 'geri':
                # Geri: ROV'un arkası (Z ekseni negatif yönü, yaw açısına göre döndürülmüş)
                hareket_vektoru.x = -sin(yaw_radyan)
                hareket_vektoru.z = -cos(yaw_radyan)
            elif yon == 'sag':
                # Sağ: ROV'un sağ tarafı (X ekseni pozitif yönü, yaw açısına göre döndürülmüş)
                hareket_vektoru.x = cos(yaw_radyan)
                hareket_vektoru.z = -sin(yaw_radyan)
            elif yon == 'sol':
                # Sol: ROV'un sol tarafı (X ekseni negatif yönü, yaw açısına göre döndürülmüş)
                hareket_vektoru.x = -cos(yaw_radyan)
                hareket_vektoru.z = sin(yaw_radyan)
            elif yon == 'cik': 
                hareket_vektoru.y = 1.0
            elif yon == 'bat' and rov.role != 1: 
                hareket_vektoru.y = -1.0
            
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
        
        # YENİ: Bireysel AI Anahtarı
        self.ai_aktif = True 

    def hedef_atama(self, x, y, z):
        self.hedef = Vec3(x, y, z)

    def rehber_guncelle(self, rehber):
        if self.modem: self.modem.rehber_guncelle(rehber)
    
    def guncelle(self, gat_kodu):
        """
        GNC Güncelleme: Hedef varsa ve manuel kontrol kapalıysa hedefe git.
        - Rol ayrımı gözetmeksizin, tüm ROV'lar hedef varsa hedefe gider.
        - Hedefe yaklaşma toleransı: 0.1 metre
        - Hedefe ulaşıldığında veya hedef yoksa motorları durdur.
        """
        # Manuel kontrol durumunda hareket koduna girmeden çık
        if self.manuel_kontrol:
            return

        # Hedef yoksa işlem yapma
        if self.hedef is None:
            # Hedef yoksa motorları durdur
            if self.rov.velocity.length() > 0.1:
                self.rov.velocity *= 0.8  # Momentumu yumuşatarak durdur
            return
        
        # 1. Mevcut pozisyonu Ursina'dan alıp Simülasyona çevir
        current_sim_pos = Vec3(*Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z))
        
        # 2. Farkı Simülasyon dünyasında hesapla
        fark = self.hedef - current_sim_pos
        mevcut_mesafe = fark.length()

        # HEDEF KONTROLÜ: Hedefe ulaşıldıysa dur
        if mevcut_mesafe <= 0.1:
            # Hedefe ulaşıldı, dur
            if self.rov.velocity.length() > 0.1:
                self.rov.velocity *= 0.8  # Momentumu yumuşatarak durdur
            return

        # 3. Hareket vektörünü normalize et
        if mevcut_mesafe > 0.01:
            hareket_vektoru = fark / mevcut_mesafe
        else:
            hareket_vektoru = Vec3(0, 0, 0)
        
        # 4. Hareket vektörünü motor komutlarına haritala
        # hareket_vektoru.x -> Sağ/Sol
        # hareket_vektoru.y -> İleri/Geri (Simülasyonda Y ileridir)
        # hareket_vektoru.z -> Çık/Bat (Simülasyonda Z derinliktir)
        # Hızı 0.5 ile çarp (yarı hız)
        self.vektor_to_motor_sim(hareket_vektoru, guc=0.5)

    def vektor_to_motor_sim(self, v_sim, guc=1.0):
        """
        Vektörü Simülasyon eksenlerinden Ursina motor komutlarına çevirir.
        Global koordinatlara göre direkt hareket eder (yaw açısından bağımsız).
        
        Args:
            v_sim: Simülasyon formatında vektör (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik)
            guc: Güç çarpanı (varsayılan: 1.0)
        """
        if v_sim.length() < 0.01:
            return
        
        # Güç çarpanını normalize et
        guc = max(0.0, min(2.0, guc))
        
        # Vektörü normalize et
        v = v_sim.normalized()
        
        # Direkt global koordinatlara göre velocity ayarla (yaw açısından bağımsız)
        # Sim formatından Ursina formatına dönüştür
        from .config import HareketAyarlari
        from ursina import time
        
        # Hız çarpanı
        max_guc = 100.0 * guc
        thrust = max_guc * time.dt * HareketAyarlari.MOTOR_GUC_KATSAYISI
        
        # X: Sağ-Sol (Sim ve Ursina'da aynı)
        if abs(v.x) > 0.01:
            self.rov.velocity.x += v.x * thrust
        
        # Y: İleri-Geri (Simülasyon Y = Ursina Z)
        if abs(v.y) > 0.01:
            self.rov.velocity.z += v.y * thrust
            
        # Z: Derinlik (Simülasyon Z = Ursina Y)
        # Ursina'da Y yukarı (+), Simülasyonda Z derinlik (+) ise:
        # v_sim.z > 0 (daha derine git) -> Ursina Y negatif
        if abs(v.z) > 0.01:
            self.rov.velocity.y += v.z * thrust  # Sim Z+ (derinlik) -> Ursina Y+ (yukarı)
        
        # Hız limiti
        if self.rov.velocity.length() > max_guc:
            self.rov.velocity = self.rov.velocity.normalized() * max_guc


