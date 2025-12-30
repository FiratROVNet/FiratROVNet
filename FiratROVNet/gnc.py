import numpy as np
from ursina import Vec3, time, distance
from .config import cfg, GATLimitleri, SensorAyarlari, ModemAyarlari, HareketAyarlari, Formasyon
from .iletisim import AkustikModem
from .hull import HullManager
import math
import random
import threading
import queue

# Alpha Shape ve Shapely için import (kontur hesaplama için)
try:
    import alphashape
    ALPHASHAPE_AVAILABLE = True
except ImportError:
    ALPHASHAPE_AVAILABLE = False
    print("⚠️ [UYARI] alphashape bulunamadı. yeniden_ciz() fonksiyonu çalışmayacak.")

try:
    from shapely.geometry import Point, LineString, Polygon, MultiPolygon
    from shapely.ops import unary_union, nearest_points
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False
    print("⚠️ [UYARI] shapely bulunamadı. yeniden_ciz() fonksiyonu çalışmayacak.")

# Convex Hull için scipy import (geriye dönük uyumluluk için)
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
        self.hull_manager = HullManager(self)  # Convex Hull yönetimi
        self._command_queue = queue.Queue()  # Thread-safe komut kuyruğu
        self._main_thread_id = threading.get_ident()  # Ana thread ID'si
        # Formasyon ID shuffle mekanizması
        self._formasyon_id_pool = []  # Shuffle edilmiş formasyon ID'leri
        self._formasyon_id_pool_olustur()  # İlk pool'u oluştur
        # Formasyon hedef takibi (ROV ID -> {'pozisyon': (x, y, z), 'hedef_yaw': float})
        self._formasyon_hedefleri = {}  # Takipçi ROV'ların formasyon hedefleri ve hedef yaw açıları
        self._formasyon_yaw_senkronizasyon_mesafesi = 5.0  # Yaw senkronizasyonu için mesafe eşiği (metre)
        self._maksimum_yaw_donme_hizi = 60.0  # Maksimum yaw dönme hızı (derece/saniye) - Formasyon için
        # git() hedef takibi (ROV ID -> hedef_yaw açısı)
        self._git_hedef_yaw = {}  # git() ile gönderilen ROV'ların hedef yaw açıları (kademeli dönüş için)
        self._git_maksimum_yaw_donme_hizi = 90.0  # git() için maksimum yaw dönme hızı (derece/saniye)
        
        # Çoklu nokta takibi (ROV ID -> nokta listesi ve mevcut indeks)
        self._git_nokta_listesi = {}  # {rov_id: [[x1, y1], [x2, y2], ...], ...}
        self._git_mevcut_nokta_indeksi = {}  # {rov_id: 0, ...} - Hangi noktaya gidiyor
        self._git_hedef_mesafe_toleransi = 2.0  # Hedefe ulaşma toleransı (metre)
    
    def _is_main_thread(self):
        """Şu anki thread'in ana thread olup olmadığını kontrol eder."""
        try:
            # threading.main_thread() Python 3.4+ için
            return threading.current_thread() is threading.main_thread()
        except AttributeError:
            # Geriye dönük uyumluluk için eski yöntem
            return threading.get_ident() == self._main_thread_id
    
    def _process_command_queue(self):
        """Ana thread'de çağrılmalı: Queue'daki komutları işler."""
        try:
            # Her frame'de maksimum 1 komut işle (arka plan işlemleri için)
            # Bu sayede konsolu rahatsız etmeden her frame'de bir işlem yapılır
            max_commands = 1
            processed = 0
            while not self._command_queue.empty() and processed < max_commands:
                cmd_type, args, kwargs = self._command_queue.get_nowait()
                if cmd_type == 'git':
                    self._git_impl(*args, **kwargs)
                elif cmd_type == 'hull':
                    self._guvenlik_hull_olustur_impl(*args, **kwargs)
                elif cmd_type == 'formasyon_sec':
                    self._formasyon_sec_impl(*args, **kwargs)
                elif cmd_type == 'set':
                    self._set_impl(*args, **kwargs)
                elif cmd_type == 'hedef':
                    self._hedef_impl(*args, **kwargs)
                else:
                    # Genel fonksiyon çağrısı
                    if isinstance(args, tuple) and len(args) > 0 and callable(args[0]):
                        func, func_args, func_kwargs = args[0], args[1:], kwargs
                        func(*func_args, **func_kwargs)
                processed += 1
        except queue.Empty:
            pass
        except Exception as e:
            print(f"⚠️ [UYARI] Komut kuyruğu işlenirken hata: {e}")
            import traceback
            traceback.print_exc()
    
    def execute_queued_commands(self):
        """
        Ana thread'de çağrılmalı: Queue'daki tüm komutları işler.
        main.py'deki update() fonksiyonunun başına eklenmelidir.
        """
        self._process_command_queue()
    
    def _formasyon_id_pool_olustur(self):
        """Formasyon ID pool'unu oluşturur ve shuffle eder."""
        # Tüm formasyon ID'lerini al (0'dan len(Formasyon.TIPLER)-1'e kadar)
        self._formasyon_id_pool = list(range(len(Formasyon.TIPLER)))
        # Random shuffle et
        random.shuffle(self._formasyon_id_pool)
    
    def _formasyon_id_al(self):
        """Formasyon ID pool'undan bir ID alır. Pool boşalırsa yeniden doldurur."""
        if len(self._formasyon_id_pool) == 0:
            # Pool boşaldı, yeniden doldur ve shuffle et
            self._formasyon_id_pool_olustur()
        # Pool'dan bir ID pop et
        return self._formasyon_id_pool.pop(0)
    
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
                gnc = TemelGNC(rov, lider_modem, filo_ref=self)
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
                gnc = TemelGNC(rov, modem, filo_ref=self)
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
        # Ana thread'de queue'daki komutları işle (thread-safe)
        self._process_command_queue()
        
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
        
        # Formasyon yaw senkronizasyonu: Takipçi ROV'lar hedefe yaklaştığında liderin yaw açısına göre yönlenir
        # Kademeli dönme: Maksimum 90 derece/saniye dönme hızı ile yumuşak dönüş
        if lider_rov_id is not None and len(self._formasyon_hedefleri) > 0:
            lider_yaw = self.get(lider_rov_id, 'yaw')
            if lider_yaw is not None:
                # Frame süresini al (saniye cinsinden)
                dt = time.dt if hasattr(time, 'dt') else 0.016  # Varsayılan: 60 FPS
                
                for rov_id, hedef_bilgisi in list(self._formasyon_hedefleri.items()):
                    # Sadece takipçi ROV'lar için kontrol et
                    if rov_id >= len(self.sistemler) or rov_id == lider_rov_id:
                        continue
                    
                    if hasattr(self.sistemler[rov_id], 'rov'):
                        takipci_rov = self.sistemler[rov_id].rov
                        # Takipçi ROV'un mevcut pozisyonunu al (Sim formatında)
                        mevcut_sim_pos = Koordinator.ursina_to_sim(
                            takipci_rov.x,
                            takipci_rov.y,
                            takipci_rov.z
                        )
                        mevcut_x, mevcut_y, mevcut_z = mevcut_sim_pos
                        
                        # Hedef bilgisini al (dict formatında)
                        if isinstance(hedef_bilgisi, dict):
                            hedef_pozisyon = hedef_bilgisi.get('pozisyon')
                            hedef_yaw = hedef_bilgisi.get('hedef_yaw', lider_yaw)
                        else:
                            # Geriye dönük uyumluluk: Eski format (sadece pozisyon tuple'ı)
                            hedef_pozisyon = hedef_bilgisi
                            hedef_yaw = lider_yaw
                            # Yeni formata dönüştür
                            self._formasyon_hedefleri[rov_id] = {
                                'pozisyon': hedef_pozisyon,
                                'hedef_yaw': hedef_yaw
                            }
                        
                        if hedef_pozisyon is None:
                            continue
                        
                        hedef_x, hedef_y, hedef_z = hedef_pozisyon
                        
                        # 2D mesafe hesapla (X-Y düzleminde, Z'yi yok say)
                        dx = hedef_x - mevcut_x
                        dy = hedef_y - mevcut_y
                        mesafe_2d = math.sqrt(dx**2 + dy**2)
                        
                        # Eğer hedefe yaklaştıysa (mesafe eşiğinin altındaysa), liderin yaw açısına göre yönlen
                        if mesafe_2d <= self._formasyon_yaw_senkronizasyon_mesafesi:
                            # Takipçinin mevcut yaw açısını al
                            mevcut_yaw = self.get(rov_id, 'yaw')
                            if mevcut_yaw is not None:
                                # Hedef yaw açısını güncelle (liderin yaw açısı değişmiş olabilir)
                                hedef_yaw = lider_yaw
                                self._formasyon_hedefleri[rov_id]['hedef_yaw'] = hedef_yaw
                                
                                # Yaw açıları arasındaki farkı hesapla
                                yaw_farki = hedef_yaw - mevcut_yaw
                                # Açı farkını -180 ile +180 arasına normalize et
                                while yaw_farki > 180:
                                    yaw_farki -= 360
                                while yaw_farki < -180:
                                    yaw_farki += 360
                                
                                # Eğer açı farkı önemliyse (1 dereceden fazla), kademeli olarak döndür
                                if abs(yaw_farki) > 1.0:
                                    # Maksimum dönme hızına göre bu frame'de döndürülecek açı
                                    maksimum_donme_acisi = self._maksimum_yaw_donme_hizi * dt
                                    
                                    # Eğer kalan açı farkı maksimum dönme açısından küçükse, direkt hedefe git
                                    if abs(yaw_farki) <= maksimum_donme_acisi:
                                        yeni_yaw = hedef_yaw
                                        # Hedefi kaldır (artık yaw senkronize edildi)
                                        if rov_id in self._formasyon_hedefleri:
                                            del self._formasyon_hedefleri[rov_id]
                                    else:
                                        # Kademeli dönme: Maksimum dönme hızına göre döndür
                                        donme_yonu = 1 if yaw_farki > 0 else -1
                                        yeni_yaw = mevcut_yaw + (donme_yonu * maksimum_donme_acisi)
                                        # Yaw açısını 0-360 aralığına normalize et
                                        while yeni_yaw >= 360:
                                            yeni_yaw -= 360
                                        while yeni_yaw < 0:
                                            yeni_yaw += 360
                                    
                                    # Yaw açısını güncelle
                                    self.set(rov_id, 'yaw', yeni_yaw)
        
        # git() yaw senkronizasyonu: git() ile gönderilen ROV'ların yaw açıları kademeli olarak güncellenir
        if len(self._git_hedef_yaw) > 0:
            # Frame süresini al (saniye cinsinden)
            dt = time.dt if hasattr(time, 'dt') else 0.016  # Varsayılan: 60 FPS
            
            for rov_id, hedef_yaw in list(self._git_hedef_yaw.items()):
                # ROV ID geçerliliği kontrolü
                if rov_id >= len(self.sistemler):
                    if rov_id in self._git_hedef_yaw:
                        del self._git_hedef_yaw[rov_id]
                    continue
                
                if hasattr(self.sistemler[rov_id], 'rov'):
                    # Mevcut yaw açısını al
                    mevcut_yaw = self.get(rov_id, 'yaw')
                    if mevcut_yaw is not None:
                        # Yaw açıları arasındaki farkı hesapla
                        yaw_farki = hedef_yaw - mevcut_yaw
                        # Açı farkını -180 ile +180 arasına normalize et
                        while yaw_farki > 180:
                            yaw_farki -= 360
                        while yaw_farki < -180:
                            yaw_farki += 360
                        
                        # Eğer açı farkı önemliyse (1 dereceden fazla), kademeli olarak döndür
                        if abs(yaw_farki) > 1.0:
                            # git() için maksimum dönme hızına göre bu frame'de döndürülecek açı (90 derece/saniye)
                            maksimum_donme_acisi = self._git_maksimum_yaw_donme_hizi * dt
                            
                            # Eğer kalan açı farkı maksimum dönme açısından küçükse, direkt hedefe git
                            if abs(yaw_farki) <= maksimum_donme_acisi:
                                yeni_yaw = hedef_yaw
                                # Hedefi kaldır (artık yaw hedefine ulaşıldı)
                                if rov_id in self._git_hedef_yaw:
                                    del self._git_hedef_yaw[rov_id]
                            else:
                                # Kademeli dönme: Maksimum dönme hızına göre döndür
                                donme_yonu = 1 if yaw_farki > 0 else -1
                                yeni_yaw = mevcut_yaw + (donme_yonu * maksimum_donme_acisi)
                                # Yaw açısını 0-360 aralığına normalize et
                                while yeni_yaw >= 360:
                                    yeni_yaw -= 360
                                while yeni_yaw < 0:
                                    yeni_yaw += 360
                            
                            # Yaw açısını güncelle
                            self.set(rov_id, 'yaw', yeni_yaw)
                        else:
                            # Açı farkı çok küçük, hedefe ulaşıldı
                            if rov_id in self._git_hedef_yaw:
                                del self._git_hedef_yaw[rov_id]
    
    def set(self, rov_id, ayar_adi, deger):
        """
        ROV ayarlarını değiştirir (Thread-safe).
        
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
        # Thread-safe çağrı: Ana thread'de değilse queue'ya ekle
        if not self._is_main_thread():
            self._command_queue.put(('set', (rov_id, ayar_adi, deger), {}))
            return True  # Queue'ya eklendi, başarılı kabul et
        
        # Ana thread'deyiz, direkt çalıştır
        return self._set_impl(rov_id, ayar_adi, deger)
    
    def _set_impl(self, rov_id, ayar_adi, deger):
        """set() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır)."""
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
            # Yaw güncellemeleri için log yazma (çok sık çağrılıyor, ekranı dolduruyor)
            if ayar_adi != 'yaw':
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
            
            # ROV yaw açısı (derece) - Ursina Y-rotation
            # Ursina'da rotation_y (Yaw) 0 iken ROV ileri (+Z) bakar. Bu bizim Simülasyon sistemimizde +Y'dir.
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
                    
                    # Ursina Yaw sisteminde: 0 derece -> +Z (Sim Y), 90 derece -> +X (Sim X)
                    theta_rad = math.radians(yaw_deg + offset)
                    
                    # Engel koordinatı (Simülasyon formatında)
                    # X = x0 + d*sin(theta), Y = y0 + d*cos(theta)
                    ox = x0 + distance * math.sin(theta_rad)
                    oy = y0 + distance * math.cos(theta_rad)
                    oz = z0  # Derinlik
                    
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
    def formasyon_sec(self, margin=30, is_3d=False, offset=20.0, harita=False, yaw_senkronizasyon_mesafesi=5.0, maksimum_yaw_donme_hizi=90.0):
        """
        Convex hull kullanarak en uygun formasyonu seçer (Thread-safe).

        KESİN KURALLAR:
        - Güvenlik hull (sanal + gerçek engeller) SADECE 1 KEZ hesaplanır (sabit hull)
        - Margin sadece formasyon_aralik için kullanılır (ROV'lar arası mesafe)
        - Hull içinde kalma kontrolü margin olmadan yapılır
        - İlk geçerli formasyon bulunduğunda DERHAL döner
        - Takipçi ROV'lar hedef pozisyonlarına yaklaştığında (yaw_senkronizasyon_mesafesi metre), 
          liderin yaw açısına göre otomatik olarak yönlenirler
        - Yaw dönüşü kademeli olarak yapılır (maksimum_yaw_donme_hizi derece/saniye)

        Args:
            margin (float): Formasyon aralığı için kullanılır (varsayılan: 30)
                - formasyon_aralik = margin * 0.6 (ROV'lar arası mesafe)
            is_3d (bool): 3D formasyon modu (varsayılan: False)
            offset (float): ROV hull genişletme mesafesi (varsayılan: 20.0)
            harita (bool): Harita görüntülemeyi aç/kapat (varsayılan: False)
            yaw_senkronizasyon_mesafesi (float): Takipçi ROV'ların hedefe yaklaştığında liderin yaw açısına 
                göre yönlenmesi için mesafe eşiği (metre, varsayılan: 5.0)
            maksimum_yaw_donme_hizi (float): Maksimum yaw dönme hızı (derece/saniye, varsayılan: 60.0)

        Returns:
            tuple | None: Seçilen formasyon bilgileri (formasyon_id, aralik, yaw, koordinat) veya None (uygun formasyon bulunamazsa)
                - formasyon_id (int): Formasyon tipi ID'si (0-19)
                - aralik (float): ROV'lar arası mesafe (metre)
                - yaw (float): Liderin yaw açısı (derece)
                - koordinat (tuple): Seçilen formasyon koordinatı (x, y, z) - Lider pozisyonu
        """
        if harita:
            if self.ortam_ref and hasattr(self.ortam_ref, 'harita') and self.ortam_ref.harita:
                self.ortam_ref.harita.goster(True, True)
        
        # Yaw senkronizasyon parametrelerini ayarla
        self._formasyon_yaw_senkronizasyon_mesafesi = yaw_senkronizasyon_mesafesi
        self._maksimum_yaw_donme_hizi = maksimum_yaw_donme_hizi
        
        # Thread-safe çağrı: Ana thread'de değilse queue'ya ekle
        if not self._is_main_thread():
            try:
                # Ursina'nın invoke mekanizmasını kullan (varsa)
                from ursina import invoke
                result = [None]  # Mutable container for return value
                def wrapper():
                    result[0] = self._formasyon_sec_impl(margin, is_3d, offset)
                invoke(wrapper)
                return result[0]
            except (ImportError, AttributeError):
                # Ursina invoke yoksa, queue kullan
                self._command_queue.put(('formasyon_sec', (margin, is_3d, offset), {}))
                # Queue'dan dönen değer beklenemez, None döndür
                return None
        
        # Ana thread'deyiz, direkt çalıştır
        return self._formasyon_sec_impl(margin, is_3d, offset)
    
    def _formasyon_sec_impl(self, margin=30, is_3d=False, offset=20.0):
        """
        formasyon_sec() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır).
        
        Hiyerarşik Arama Stratejisi:
        - Adım A: Lider ROV'un GPS koordinatını merkez kabul et, tüm formasyon tiplerini ve aralıklarını dene
        - Adım B: Eğer mevcut açıyla sığmıyorsa, liderin yaw açısını 90, 180, 270 derece döndürerek tekrar dene
        - Adım C: Eğer liderin olduğu yerde hiçbir açıda uygun formasyon bulunamazsa, Hull Merkezi koordinatına geç
        
        Returns:
            tuple | None: (formasyon_id, aralik, yaw, koordinat) veya None
                - formasyon_id (int): Formasyon tipi ID'si (0-19)
                - aralik (float): ROV'lar arası mesafe (metre)
                - yaw (float): Liderin yaw açısı (derece)
                - koordinat (tuple): Seçilen formasyon koordinatı (x, y, z) - Lider pozisyonu
        """
        try:
            # Eski formasyon hedeflerini temizle (yeni formasyon için)
            self._formasyon_hedefleri.clear()
            # 1. Ada çevre noktalarını al (yasaklı noktalar olarak kullanılacak)
            ada_cevre_noktalari = self.ada_cevre()
            
            # Ada çevre noktalarını 2D formatına çevir (sadece x, y)
            yasakli_noktalar = []
            if ada_cevre_noktalari:
                for nokta in ada_cevre_noktalari:
                    if len(nokta) >= 2:
                        yasakli_noktalar.append([float(nokta[0]), float(nokta[1])])
            
            # 2. Yeni hull oluştur (yasaklı noktaları çıkararak)
            guvenlik_hull_dict = self.yeni_hull(
                yasakli_noktalar=yasakli_noktalar,
                offset=offset,
                alpha=2.0,
                buffer_radius=10.0,  # Ada çevresinden 15 metre güvenli mesafe
                channel_width=10.0   # Kanal genişliği 10 metre
            )

            hull = guvenlik_hull_dict.get("hull")
            hull_merkez = guvenlik_hull_dict.get("center")

            if hull is None or hull_merkez is None:
                return None

            # Hull merkezini Sim formatına dönüştür (z=0 yap)
            hull_merkez_liste = list(hull_merkez)
            hull_merkez_liste[2] = 0
            hull_merkez = tuple(hull_merkez_liste)

            # 2. Lider ROV'u bul ve GPS koordinatını al
            lider_rov_id = None
            lider_gps = None
            for rov_id in range(len(self.sistemler)):
                if self.get(rov_id, "rol") == 1:
                    lider_rov_id = rov_id
                    gps = self.get(rov_id, "gps")
                    if gps:
                        # GPS koordinatını Sim formatında al (Config.py'deki değişikliğe uygun)
                        lider_gps = (float(gps[0]), float(gps[1]), float(gps[2]))
                    break

            if lider_rov_id is None:
                return None

            if lider_gps is None:
                lider_gps = hull_merkez

            # 3. Formasyon aralığı parametreleri
            min_aralik = margin * 0.2
            baslangic_aralik = margin * 0.6
            adim = 1.0  # metre

            # 4. Yaw açıları (0, 90, 180, 270 derece)
            yaw_acilari = [0, 90, 180, 270]

            # 5. HİYERARŞİK ARAMA: Nokta Döngüsü -> Yaw Döngüsü -> Formasyon Tipi Döngüsü -> Aralık Döngüsü
            # Adım A: Lider GPS koordinatı
            # Adım B: Lider GPS'ten Hull Merkezi'ne kadar 20 metre dilimlerle ara noktalar
            # Adım C: Hull Merkezi (eğer lider GPS'te bulunamazsa)
            arama_noktalari = [("Lider GPS", lider_gps)]
            
            # Lider GPS'ten Hull Merkezi'ne kadar 20 metre dilimlerle ara noktalar oluştur
            lider_x, lider_y, lider_z = lider_gps
            hull_x, hull_y, hull_z = hull_merkez
            
            # 2D mesafe hesapla (X-Y düzleminde, Z'yi yok say)
            dx = hull_x - lider_x
            dy = hull_y - lider_y
            mesafe_2d = math.sqrt(dx**2 + dy**2)
            
            # Eğer mesafe 20 metreden fazlaysa, ara noktalar oluştur
            if mesafe_2d > 10.0:
                # Normalize edilmiş yön vektörü
                if mesafe_2d > 0.001:  # Sıfıra bölme kontrolü
                    yon_x = dx / mesafe_2d
                    yon_y = dy / mesafe_2d
                    
                    # 20 metre dilimlerle ara noktalar oluştur
                    dilim_boyutu = 10.0
                    mevcut_mesafe = dilim_boyutu
                    
                    while mevcut_mesafe < mesafe_2d:
                        # Ara nokta koordinatları
                        ara_x = lider_x + (yon_x * mevcut_mesafe)
                        ara_y = lider_y + (yon_y * mevcut_mesafe)
                        ara_z = lider_z  # Z koordinatını lider ile aynı tut
                        
                        # Ara noktayı listeye ekle
                        arama_noktalari.append((f"Ara Nokta ({mevcut_mesafe:.1f}m)", (ara_x, ara_y, ara_z)))
                        
                        mevcut_mesafe += dilim_boyutu
            
            # Hull Merkezi'ni en sona ekle
            arama_noktalari.append(("Hull Merkezi", hull_merkez))

            for nokta_adi, merkez_koordinat in arama_noktalari:
                # Yaw Döngüsü: 0, 90, 180, 270 derece
                for deneme_yaw in yaw_acilari:

                    # Formasyon Tipi Döngüsü - Pool'dan random ID'leri sırayla dene
                    # Bu arama için tüm formasyon ID'lerini random sırayla al
                    # (Pool'dan çıkarılmadan önce kopyala)
                    denenecek_formasyon_idleri = []
                    # Pool'dan mevcut ID'leri al
                    pool_kopyasi = self._formasyon_id_pool.copy()
                    while len(denenecek_formasyon_idleri) < len(Formasyon.TIPLER) and len(pool_kopyasi) > 0:
                        denenecek_formasyon_idleri.append(pool_kopyasi.pop(0))
                    # Eğer pool boşaldıysa, kalan ID'leri ekle ve shuffle et
                    if len(denenecek_formasyon_idleri) < len(Formasyon.TIPLER):
                        kalan_idler = [i for i in range(len(Formasyon.TIPLER)) if i not in denenecek_formasyon_idleri]
                        random.shuffle(kalan_idler)
                        denenecek_formasyon_idleri.extend(kalan_idler)
                    
                    for i in denenecek_formasyon_idleri:
                        formasyon_tipi = Formasyon.TIPLER[i]
                        aralik = baslangic_aralik

                        # Aralık Döngüsü
                        while aralik >= min_aralik:
                            # Formasyon pozisyonlarını hesapla (yaw açısı ile)
                            formasyon_obj = Formasyon(self)
                            pozisyonlar = formasyon_obj.pozisyonlar(
                                i,
                                aralik=aralik,
                                is_3d=is_3d,
                                lider_koordinat=merkez_koordinat,
                                yaw=deneme_yaw
                            )

                            if not pozisyonlar:
                                aralik -= adim
                                continue

                            # Pozisyonları Ursina formatına dönüştür (test için)
                            ursina_positions = []
                            for pozisyon in pozisyonlar:
                                config_x, config_y, config_z = pozisyon
                                # Config (x, y, z) -> Ursina (x, z, y)
                                ursina_x = config_x
                                ursina_z = config_y
                                ursina_y = config_z
                                ursina_positions.append((ursina_x, ursina_z, ursina_y))

                            # Formasyon geçerliliğini kontrol et
                            if self._formasyon_gecerli_mi(ursina_positions, hull, aralik):
                                # Başarılı formasyon bulundu! Uygula
                                
                                # Liderin yaw açısını set et
                                self.set(lider_rov_id, 'yaw', float(deneme_yaw))

                                # Eğer formasyon Lider GPS dışında bir noktada bulunduysa (ara nokta veya Hull Merkezi), lideri oraya gönder
                                if nokta_adi != "Lider GPS":
                                    self.git(
                                        lider_rov_id,
                                        merkez_koordinat[0],
                                        merkez_koordinat[1],
                                        merkez_koordinat[2],
                                        ai=True
                                    )
                        
                                # Takipçi ROV'ları formasyon pozisyonlarına gönder
                                for rov_id, pozisyon in enumerate(pozisyonlar):
                                    if rov_id >= len(self.sistemler):
                                        break
                                    
                                    # Lider'i atla (zaten işlendi)
                                    if rov_id == lider_rov_id:
                                        continue
                                    
                                    # Config formatı = Sim formatı: (x, y, z)
                                    sim_x, sim_y, sim_z = pozisyon
                                    
                                    # Eğer yüzeydeyse (z >= 0), su altına gönder
                                    if sim_z >= 0:
                                        sim_z = -10.0
                                    
                                    # Takipçi ROV'un formasyon hedefini kaydet (yaw senkronizasyonu için)
                                    # Liderin yaw açısını hedef yaw olarak kaydet
                                    self._formasyon_hedefleri[rov_id] = {
                                        'pozisyon': (sim_x, sim_y, sim_z),
                                        'hedef_yaw': deneme_yaw  # Liderin yaw açısı
                                    }
                                    
                                    # Takipçi ROV'u formasyon pozisyonuna gönder
                                    self.git(rov_id, sim_x, sim_y, sim_z, ai=True)

                                # Formasyon bulundu, pool'dan bu ID'yi çıkar
                                if i in self._formasyon_id_pool:
                                    self._formasyon_id_pool.remove(i)
                                
                                # Seçilen formasyon koordinatı (lider pozisyonu)
                                secilen_koordinat = merkez_koordinat
                                
                                # Formasyon bilgilerini döndür: (formasyon_id, aralik, yaw, koordinat)
                                return (i, aralik, deneme_yaw, secilen_koordinat)

                            aralik -= adim

            # Hiçbir formasyon geçerli değil
            return None

        except Exception as e:
            import traceback
            traceback.print_exc()
            return None

    

    def hedef(self, x=None, y=None, z=None):
        """
        Sadece lider ROV'un hedefini ayarlar (Thread-safe). Takipçiler bu komuttan etkilenmez.
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
        # Parametre verilmediyse mevcut hedefi döndür (thread-safe değil, sadece okuma)
        if x is None or y is None:
            if self.hedef_pozisyon:
                return self.hedef_pozisyon
            else:
                return None
        
        # Thread-safe çağrı: Ana thread'de değilse queue'ya ekle
        if not self._is_main_thread():
            self._command_queue.put(('hedef', (x, y, z), {}))
            # Queue'ya eklendi, hedef pozisyonunu kaydet (okuma için)
            self.hedef_pozisyon = (x, y, 0)
            return (x, y, 0)
        
        # Ana thread'deyiz, direkt çalıştır
        return self._hedef_impl(x, y, z)
    
    def _hedef_impl(self, x, y, z):
        """hedef() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır)."""
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
        # filo.git() artık Sim formatında çalışıyor: (x, y, z)
        self.git(lider_rov_id, x, y, z, ai=True)
        
        # Hedef görselini oluştur/güncelle (Ursina formatına dönüştür)
        ursina_pos = (x, y, z)
        self._hedef_gorsel_olustur(*ursina_pos)
        
        # Haritaya hedefi ekle (Matplotlib - ana thread'de olmalı)
        if self.ortam_ref and hasattr(self.ortam_ref, 'harita'):
            self.ortam_ref.harita.hedef_pozisyon = (x, y)
        
        print(f"✅ [HEDEF] Lider hedefi güncellendi: ({x:.2f}, {y:.2f}, 0) - Su üstünde. Takipçiler de aynı hedefe gidiyor.")
        
        # Hedef koordinatlarını döndür
        return (x, y, 0)

    def _formasyon_gecerli_mi(self, test_points, hull, formasyon_aralik):
        """Wrapper: HullManager'a yönlendirir (geriye dönük uyumluluk için)."""
        return self.hull_manager.formasyon_gecerli_mi(test_points, hull, formasyon_aralik)
    
    
    def ConvexHull(self, points, test_point, margin=0.0):
        """Wrapper: HullManager'a yönlendirir (geriye dönük uyumluluk için)."""
        return self.hull_manager.convex_hull_3d(points, test_point, margin=margin)
    
    def _is_point_inside_hull(self, point, hull):
        """
        Noktanın convex hull içinde olup olmadığını kontrol eder (wrapper).
        Geriye dönük uyumluluk için bırakılmıştır.
        """
        return self.hull_manager.is_point_inside_hull(point, hull)
    
    def genisletilmis_rov_hull_olustur(self, offset=20.0):
        """Wrapper: HullManager'a yönlendirir (geriye dönük uyumluluk için)."""
        return self.hull_manager.genisletilmis_rov_hull_olustur(offset=offset)
    
    def lidar_engel_noktalari(self):
        """Wrapper: HullManager'a yönlendirir (geriye dönük uyumluluk için)."""
        return self.hull_manager.lidar_engel_noktalari()
    
    def ada_engel_noktalari(self, yakinlik_siniri=200.0):
        """Wrapper: HullManager'a yönlendirir (geriye dönük uyumluluk için)."""
        return self.hull_manager.ada_engel_noktalari(yakinlik_siniri=yakinlik_siniri)
    
    def ada_engel_noktalari_pro(self, yakinlik_siniri=100.0, offset=20.0):
        """Wrapper: HullManager'a yönlendirir (geriye dönük uyumluluk için)."""
        return self.hull_manager.ada_engel_noktalari_pro(yakinlik_siniri=yakinlik_siniri, offset=offset)
    
    def hull(self, offset=40.0):
        """
        Güvenlik hull oluşturur (Thread-safe).
        Ana thread'de değilse, komutu queue'ya ekler.
        """
        # Thread-safe çağrı: Ana thread'de değilse queue'ya ekle
        if not self._is_main_thread():
            try:
                # Ursina'nın invoke mekanizmasını kullan (varsa)
                from ursina import invoke
                result = [None]  # Mutable container for return value
                def wrapper():
                    result[0] = self._guvenlik_hull_olustur_impl(offset)
                invoke(wrapper)
                return result[0] if result[0] is not None else {'hull': None, 'points': None, 'center': None}
            except (ImportError, AttributeError):
                # Ursina invoke yoksa, queue kullan
                self._command_queue.put(('hull', (offset,), {}))
                # Queue'dan dönen değer beklenemez, None döndür
                return {'hull': None, 'points': None, 'center': None}
        
        # Ana thread'deyiz, direkt çalıştır
        return self._guvenlik_hull_olustur_impl(offset)
    
    def _guvenlik_hull_olustur_impl(self, offset=20.0):
        """guvenlik_hull_olustur() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır)."""
        return self.hull_manager.hull(offset=offset)
    
    def ada_cevre(self, offset=15.0):
        """
        Simülasyondaki adaları tespit edip her ada için eşit çevrede 12 nokta döndürür.
        
        Her ada için 12 nokta hesaplanır (30° aralıklarla: 0°, 30°, 60°, ..., 330°).
        Noktalar ada yarıçapından belirli bir mesafe uzakta olur (offset parametresi).
        
        Args:
            offset (float): Ada yarıçapından uzaklık (metre, varsayılan: 10.0)
                - Noktalar ada merkezinden (radius + offset) mesafede olur
        
        Returns:
            list: [(x1, y1, z1), (x2, y2, z2), ...] - Ada çevresi noktaları (Simülasyon formatı)
                - 1 ada varsa: 12 nokta
                - 2 ada varsa: 24 nokta
                - 3 ada varsa: 36 nokta
                - Format: (x, y, z) - x: sağ-sol, y: ileri-geri, z: derinlik
        
        Örnekler:
            # Tüm adaların çevre noktalarını al
            noktalar = filo.ada_cevre()
            # Çıktı: [(x1, y1, z1), (x2, y2, z2), ...] - Her ada için 12 nokta
            
            # Özel offset ile
            noktalar = filo.ada_cevre(offset=15.0)  # Ada yarıçapından 15 metre uzakta
        """
        if not self.ortam_ref:
            print("⚠️ [UYARI] Ortam referansı bulunamadı!")
            return []
        
        # Ada pozisyonlarını al
        if not hasattr(self.ortam_ref, 'island_positions') or not self.ortam_ref.island_positions:
            print("⚠️ [UYARI] Simülasyonda ada bulunamadı!")
            return []
        
        tum_noktalar = []
        
        # Her ada için 12 nokta hesapla
        for island_data in self.ortam_ref.island_positions:
            if len(island_data) < 3:
                continue
            
            # Ada bilgileri: (island_x, island_z, island_radius)
            island_x = float(island_data[0])  # X koordinatı (sağ-sol)
            island_z = float(island_data[1])  # Z koordinatı (ileri-geri) - Simülasyon formatında Y
            island_radius = float(island_data[2])  # Ada yarıçapı
            
            # Çevre mesafesi: Ada yarıçapı + offset
            cevre_mesafesi = island_radius + offset
            
            # 12 nokta hesapla (30° aralıklarla: 0°, 30°, 60°, 90°, 120°, 150°, 180°, 210°, 240°, 270°, 300°, 330°)
            # Simülasyon sistemi: X=Sağ-Sol, Y=İleri-Geri
            # 0° = Kuzey (+Y), 90° = Doğu (+X), 180° = Güney (-Y), 270° = Batı (-X)
            acilar = [i * 30 for i in range(12)]  # 0°, 30°, 60°, ..., 330° (12 nokta)
            
            for aci in acilar:
                # Açıyı radyana çevir
                aci_rad = math.radians(aci)
                
                # Nokta koordinatları (Simülasyon formatı)
                # X = island_x + mesafe * sin(aci)
                # Y = island_z + mesafe * cos(aci)
                # Z = 0 (yüzey, derinlik yok)
                nokta_x = island_x + cevre_mesafesi * math.sin(aci_rad)
                nokta_y = island_z + cevre_mesafesi * math.cos(aci_rad)
                nokta_z = 0.0  # Yüzey (derinlik yok)
                
                tum_noktalar.append((nokta_x, nokta_y, nokta_z))
        
        print(f"✅ [ADA_CEVRE] {len(self.ortam_ref.island_positions)} ada için {len(tum_noktalar)} nokta hesaplandı (offset={offset}m)")
        return tum_noktalar
    
    def yeni_hull(self, yasakli_noktalar, offset=40.0, alpha=2.0, buffer_radius=20.0, channel_width=15.0):
            """
            Mevcut hull noktalarını alır, yasaklı bölgeleri kesip çıkarır.
            Hem harita çizimi hem de 'is_point_inside' kontrolü için uyumlu nesne döndürür.
            """
            try:
                # 1. Kütüphane kontrolü
                if not SHAPELY_AVAILABLE:
                    return {'hull': None, 'points': None, 'center': None}
                    
                from shapely.geometry import Point, Polygon
                
                # --- 1. Mevcut Hull'ı Al ---
                guvenlik_hull_dict = self.hull_manager.hull(offset=offset)
                hull_noktalari = guvenlik_hull_dict.get("points")
                eski_hull_merkez = guvenlik_hull_dict.get("center")
                
                if hull_noktalari is None:
                    return {'hull': None, 'points': None, 'center': None}
                
                # --- 2. Noktaları Hazırla ---
                hull_noktalari_2d = []
                if isinstance(hull_noktalari, np.ndarray):
                    hull_noktalari_2d = [[float(p[0]), float(p[1])] for p in hull_noktalari]
                else:
                    hull_noktalari_2d = [[float(p[0]), float(p[1])] for p in hull_noktalari if len(p) >= 2]
                
                yasakli_noktalar_2d = []
                if yasakli_noktalar:
                    for nokta in yasakli_noktalar:
                        if len(nokta) >= 2:
                            yasakli_noktalar_2d.append([float(nokta[0]), float(nokta[1])])
                
                # --- 3. Yeniden Çiz ---
                if yasakli_noktalar_2d:
                    yeni_kontur_noktalari = self.yeniden_ciz(
                        noktalar=hull_noktalari_2d,
                        yasakli_noktalar=yasakli_noktalar_2d,
                        alpha=alpha,
                        buffer_radius=buffer_radius,
                        channel_width=channel_width
                    )
                else:
                    yeni_kontur_noktalari = hull_noktalari_2d

                # --- 4. Sonuçları Paketle ---
                if yeni_kontur_noktalari and len(yeni_kontur_noktalari) >= 3:
                    kontur_noktalari_np = np.array(yeni_kontur_noktalari)
                    
                    # Polygon nesnesi oluştur (Geometrik kontrol için şart)
                    yeni_poly = Polygon(yeni_kontur_noktalari)
                    if not yeni_poly.is_valid:
                        yeni_poly = yeni_poly.buffer(0)
                    
                    # Merkez hesapla (Eski merkez güvenli mi?)
                    eski_merkez_2d = (eski_hull_merkez[0], eski_hull_merkez[1])
                    if yeni_poly.contains(Point(eski_merkez_2d)):
                        final_merkez_2d = eski_merkez_2d
                    else:
                        guvenli_nokta = yeni_poly.representative_point()
                        final_merkez_2d = (guvenli_nokta.x, guvenli_nokta.y)

                    eski_z = eski_hull_merkez[2] if eski_hull_merkez and len(eski_hull_merkez) >= 3 else 0.0
                    yeni_hull_merkez = (float(final_merkez_2d[0]), float(final_merkez_2d[1]), float(eski_z))
                    
                    # --- SAHTE HULL (GÜNCELLENDİ) ---
                    class SahteHull:
                        def __init__(self, points, polygon_obj):
                            self.points = points
                            self.polygon = polygon_obj  # <-- KRİTİK EKLEME: Polygon nesnesini sakla
                            self.vertices = np.arange(len(points))
                            self.simplices = []
                            for i in range(len(points)):
                                self.simplices.append([i, (i + 1) % len(points)])
                            self.simplices = np.array(self.simplices)
                            # equations özelliği YOK, bu yüzden hull.py'de bunu kontrol edeceğiz

                    custom_hull = SahteHull(kontur_noktalari_np, yeni_poly)
                    
                    # Haritaya gönder
                    if self.ortam_ref and hasattr(self.ortam_ref, 'harita') and self.ortam_ref.harita:
                        hull_data = {
                            'hull': custom_hull,
                            'points': kontur_noktalari_np,
                            'center': yeni_hull_merkez
                        }
                        self.ortam_ref.harita.convex_hull_data = hull_data
                        self.ortam_ref.harita.goster(True, True)
                    
                    return {
                        'hull': custom_hull,
                        'points': kontur_noktalari_np,
                        'center': yeni_hull_merkez
                    }
                else:
                    return {'hull': None, 'points': None, 'center': None}
            
            except Exception as e:
                print(f"❌ [HATA] Yeni hull oluşturulurken hata: {e}")
                import traceback
                traceback.print_exc()
                return {'hull': None, 'points': None, 'center': None}
    
    def yeniden_ciz(self, noktalar, yasakli_noktalar, alpha=2.0, buffer_radius=15.0, channel_width=10.0):
            """
            Verilen nokta kümesini saran, ancak yasaklı noktaları dışarıda bırakacak şekilde
            içeri bükülmüş sınırın koordinatlarını döndürür.
            """
            # 1. Kütüphane kontrolü
            if not SHAPELY_AVAILABLE:
                print("❌ [HATA] shapely kütüphanesi bulunamadı!")
                return []
                
            # Global importları kullan
            try:
                from shapely.geometry import Point, LineString, Polygon, MultiPolygon
                from shapely.ops import unary_union, nearest_points
                from scipy.spatial import ConvexHull
            except ImportError as e:
                print(f"❌ [HATA] Gerekli kütüphaneler eksik: {e}")
                return []

            try:
                # 2. Giriş verisini düzenle
                points_cloud = []
                for p in noktalar:
                    if len(p) >= 2:
                        points_cloud.append((float(p[0]), float(p[1])))
                
                if len(points_cloud) < 3:
                    print("⚠️ [UYARI] Yeterli nokta yok (en az 3 nokta gerekli)")
                    return []
                
                # ==========================================================
                # ADIM A: TEMEL ŞEKLİ (CONVEX HULL) OLUŞTUR
                # ==========================================================
                # Alpha shape yerine ConvexHull kullanıyoruz. 
                # Çünkü "Güvenlik Hull"ı her zaman en dıştan sarmalıdır.
                try:
                    points_np = np.array(points_cloud)
                    hull = ConvexHull(points_np) 
                    # Convex Hull noktalarını sıraya diz (önemli!)
                    hull_points = points_np[hull.vertices]
                    base_shape = Polygon(hull_points)
                except Exception as e:
                    print(f"❌ [HATA] Başlangıç Hull oluşturulamadı: {e}")
                    return []

                # Şekil temizliği
                if not base_shape.is_valid:
                    base_shape = base_shape.buffer(0)

                final_shape = base_shape
                kesilen_nokta_sayisi = 0

                # ==========================================================
                # ADIM B: YASAKLI NOKTALARI KESİP ÇIKAR
                # ==========================================================
                if yasakli_noktalar:
                    print(f"🔍 [YENIDEN_CIZ] Kontrol edilecek yasaklı nokta: {len(yasakli_noktalar)}")
                    
                    for i, fp in enumerate(yasakli_noktalar):
                        if len(fp) < 2: continue
                        
                        p_obj = Point(float(fp[0]), float(fp[1]))
                        
                        # Eğer nokta zaten şeklin dışındaysa işlem yapma
                        if not final_shape.contains(p_obj):
                            # print(f"   -> Nokta {i} zaten dışarıda.")
                            continue
                        
                        # Buraya geldiyse nokta içeride demektir, kesip atacağız
                        kesilen_nokta_sayisi += 1
                        # print(f"   ✂️  Nokta {i} ({fp[0]:.1f}, {fp[1]:.1f}) içeride! Kesiliyor...")
                        
                        # 1. Yasaklı Bölge (Güvenlik Çemberi)
                        forbidden_zone = p_obj.buffer(buffer_radius)
                        
                        # 2. Kanal Açma (En kısa yoldan dışarı tünel)
                        exterior_line = final_shape.exterior
                        p1, p2 = nearest_points(forbidden_zone, exterior_line)
                        
                        channel_line = LineString([p_obj, p2])
                        # Kanal genişliği en az buffer kadar olmalı ki darboğaz olmasın
                        channel_poly = channel_line.buffer(max(channel_width, buffer_radius * 0.5))
                        
                        # 3. Kesme işlemi
                        cut_area = unary_union([forbidden_zone, channel_poly])
                        final_shape = final_shape.difference(cut_area)
                        
                        # 4. Parçalanma kontrolü
                        if isinstance(final_shape, MultiPolygon):
                            if not final_shape.is_empty:
                                final_shape = max(final_shape.geoms, key=lambda a: a.area)
                            else:
                                final_shape = base_shape # Hata durumunda geri al

                print(f"✅ [YENIDEN_CIZ] İşlem tamam. Kesilen engel sayısı: {kesilen_nokta_sayisi}")

                # ==========================================================
                # ADIM C: SONUÇ KOORDİNATLARINI DÖNDÜR
                # ==========================================================
                if isinstance(final_shape, Polygon):
                    return list(final_shape.exterior.coords)
                else:
                    print("⚠️ [UYARI] Sonuç bir Polygon değil.")
                    return []
            
            except Exception as e:
                print(f"❌ [HATA] Kontur hesaplama genel hatası: {e}")
                import traceback
                traceback.print_exc()
                return []
    
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
            rotation=(90, 0, 45),  # 45 derece döndür
            scale=(x_boyutu, kalinlik, kalinlik),
            color=color.rgba(255, 0, 0, 0.5),
            parent=self.hedef_gorsel,
            unlit=True,
            billboard=False
        )
        
        # İkinci çapraz çizgi (sağ üst -> sol alt)
        Entity(
            model='cube',
            position=(0, 0, 0),
            rotation=(90, 0, -45),  # -45 derece döndür
            scale=(x_boyutu, kalinlik, kalinlik),
            color=color.rgba(255, 0, 0, 0.5),
            parent=self.hedef_gorsel,
            unlit=True,
            billboard=False
        )


        
        # Merkez nokta (daha belirgin olsun)
        Entity(
            model='sphere',
            position=(0, 0, 0),
            scale=(2, 2, 2),
            color=color.rgba(255, 0, 0, 0.5),
            parent=self.hedef_gorsel,
            unlit=True
        )
                    # Dış çember
        hedef_rengi = color.rgb(0, 255, 120)

        # Ring (içi boş çember)
        Entity(
            model='circle',
            position=(0, 0, 0),
            rotation=(90, 0, 0),
            scale=(x_boyutu * 1.5, x_boyutu * 1.5, 1),
            color=hedef_rengi,
            parent=self.hedef_gorsel,
            unlit=True,
            wireframe=True
        )

    def git(self, rov_id, x, y=None, z=None, ai=True):
        """
        ROV'a hedef koordinatı atar ve otomatik moda geçirir (Thread-safe).
        Tüm girişler Simülasyon formatındadır: (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik)
        
        Çoklu nokta desteği: Eğer x bir liste ise, ROV bu noktaları sırayla ziyaret eder.

        Args:
            rov_id: ROV ID (0, 1, 2, ...)
            x: X koordinatı (Sağ-Sol) veya nokta listesi [[x1, y1], [x2, y2], ...]
            y: Y koordinatı (İleri-Geri) - x liste ise kullanılmaz
            z: Z koordinatı (Derinlik, opsiyonel)
                - None ise mevcut derinlik korunur
            ai: AI aktif/pasif (varsayılan: True)
                - True: Zeki Mod (GAT tahminleri kullanılır)
                - False: Kör Mod (GAT tahminleri görmezden gelinir)

        Örnekler:
            # Tek nokta
            filo.git(0, 40, 60, 20)           # ROV-0: X=40 (sağ), Y=60 (ileri), Z=20 (derinlik), AI açık
            filo.git(1, 50, 50, -10, ai=False)  # ROV-1: X=50, Y=50, Z=-10, AI kapalı
            filo.git(2, 30, 40)               # ROV-2: X=30, Y=40, mevcut derinlik, AI açık
            
            # Çoklu nokta (gidilecek_noktalar listesi)
            gidilecek_n = [[150.5, 10.5], [142.5, 2.5], [134.5, -5.5]]
            filo.git(0, gidilecek_n)  # ROV-0 bu noktaları sırayla ziyaret eder
        """
        # Eğer x bir liste ise, çoklu nokta modu
        if isinstance(x, (list, tuple)) and len(x) > 0:
            # İlk elemanın formatını kontrol et
            if isinstance(x[0], (list, tuple)) and len(x[0]) >= 2:
                # Çoklu nokta listesi: [[x1, y1], [x2, y2], ...]
                nokta_listesi = [[float(n[0]), float(n[1])] for n in x if len(n) >= 2]
                
                if len(nokta_listesi) == 0:
                    print(f"❌ [FİLO] Geçersiz nokta listesi: {x}")
                    return
                
                # Nokta listesini kaydet
                self._git_nokta_listesi[rov_id] = nokta_listesi
                self._git_mevcut_nokta_indeksi[rov_id] = 0
                
                # İlk noktaya git (arka plan işlemi - konsolu rahatsız etme)
                ilk_nokta = nokta_listesi[0]
                # Print'i kaldır - arka plan işlemi
                
                # Thread-safe çağrı - her frame'de bir işlem için queue kullan
                # Ana thread'de olsak bile queue'ya ekle ki her frame'de bir işlem yapılsın
                self._command_queue.put(('git', (rov_id, ilk_nokta[0], ilk_nokta[1], z, ai), {}))
                return
            else:
                # Tek nokta ama tuple/list formatında: (x, y) veya [x, y]
                if len(x) >= 2:
                    x_val, y_val = float(x[0]), float(x[1])
                    z_val = float(x[2]) if len(x) >= 3 else z
                else:
                    print(f"❌ [FİLO] Geçersiz koordinat formatı: {x}")
                    return
        else:
            # Normal tek nokta modu
            x_val, y_val = float(x), float(y) if y is not None else None
            z_val = z
        
        # Y parametresi kontrolü
        if y_val is None:
            print(f"❌ [FİLO] Y koordinatı gerekli! (x liste değilse)")
            return
        
        # Thread-safe çağrı: Ana thread'de değilse queue'ya ekle
        if not self._is_main_thread():
            try:
                # Ursina'nın invoke mekanizmasını kullan (varsa)
                from ursina import invoke
                invoke(self._git_impl, rov_id, x_val, y_val, z_val, ai)
                return
            except (ImportError, AttributeError):
                # Ursina invoke yoksa, queue kullan
                self._command_queue.put(('git', (rov_id, x_val, y_val, z_val, ai), {}))
                return
        
        # Ana thread'deyiz, direkt çalıştır
        self._git_impl(rov_id, x_val, y_val, z_val, ai)

    def git_path(self, rov_id, hedef, ai=True):
        """
        ROV'a bir yol atar ve otomatik moda geçirir (Thread-safe).
        """
        path=self.a_star(rov_id,hedef)
        if not isinstance(path, list) or len(path) == 0:
            print(f"❌ [FİLO] Geçersiz yol listesi: {path}")
            return
        
        
        gidilecek_n=self.gidilecek_noktalar(path)
        self.git(rov_id,gidilecek_n,ai)
    
    def _git_impl(self, rov_id, x, y, z=None, ai=True):
        """git() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır)."""
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
        
        # Mevcut pozisyonu al (Sim formatında)
        current_sim_pos = Koordinator.ursina_to_sim(
            self.sistemler[rov_id].rov.x,
            self.sistemler[rov_id].rov.y,
            self.sistemler[rov_id].rov.z
        )
        current_x, current_y, current_z = current_sim_pos
        
        # Eğer Z (derinlik) verilmemişse mevcut derinliği koru
        if z is None:
            z = current_z
        
        # Yaw açısını hesapla (hedef yönüne doğru)
        # Sim formatında: X=Sağ-Sol, Y=İleri-Geri
        # Yaw açısı: atan2(dx, dy) - Y eksenine göre açı
        dx = x - current_x
        dy = y - current_y
        
        # Mesafe kontrolü (çok yakınsa yaw açısını değiştirme)
        mesafe = math.sqrt(dx**2 + dy**2)
        if mesafe > 0.1:  # 10 cm'den fazla mesafe varsa yaw açısını ayarla
            # Yaw açısını hesapla (derece)
            # atan2(dx, dy) -> Y eksenine göre açı (0 derece = +Y yönü)
            yaw_rad = math.atan2(dx, dy)
            yaw_deg = math.degrees(yaw_rad)
            
            # Yaw açısını normalize et (0-360 arası)
            while yaw_deg >= 360:
                yaw_deg -= 360
            while yaw_deg < 0:
                yaw_deg += 360
            
            # Hedef yaw açısını kaydet (kademeli dönüş için, direkt set etme)
            self._git_hedef_yaw[rov_id] = yaw_deg
        
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
            # 10 metre güvenlik mesafesi: ROV'lar sınırlardan 10 metre içeride kalmalı
            HAVUZ_GUVENLIK_MESAFESI = 10.0  # Metre cinsinden güvenlik mesafesi
            if hasattr(rov, 'environment_ref') and rov.environment_ref:
                havuz_genisligi = getattr(rov.environment_ref, 'havuz_genisligi', 200)
                havuz_sinir = havuz_genisligi  # +-havuz_genisligi
                guvenli_sinir = havuz_sinir - HAVUZ_GUVENLIK_MESAFESI  # 10 metre içerideki sınır
                
                # Güvenlik sınırında mı kontrol et (10 metre içeride)
                sinirda_x = abs(rov.x) >= guvenli_sinir * 0.95
                sinirda_z = abs(rov.z) >= guvenli_sinir * 0.95
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

    def harita(self, goster=True, convex=True, a_star=True):
        """Harita penceresini açar, kapatır veya görünürlük ayarlarını yapar."""
        if self.ortam_ref and hasattr(self.ortam_ref, 'harita') and self.ortam_ref.harita:
            self.ortam_ref.harita.goster(goster, convex, a_star)
    
    def minimap(self, durum=True, convex=True, a_star=True):
        """
        Minimap'i açar, kapatır veya durumunu döndürür.
        Harita fonksiyonunun tüm işlevlerine sahiptir.
        
        Args:
            durum: True/False - Minimap'i aç/kapat (None ise toggle)
            convex: True/False - Convex hull'u göster/gizle
            a_star: True/False - A* yolunu göster/gizle
        
        Örnekler:
            filo.minimap()  # Toggle (aç/kapat)
            filo.minimap(True)  # Aç
            filo.minimap(False)  # Kapat
            filo.minimap(True, convex=True, a_star=True)  # Aç ve her şeyi göster
        """
        if self.ortam_ref and hasattr(self.ortam_ref, 'minimap') and self.ortam_ref.minimap:
            # Filo referansını minimap'e ver
            if not hasattr(self.ortam_ref.minimap, 'filo_ref') or self.ortam_ref.minimap.filo_ref != self:
                self.ortam_ref.minimap.filo_ref = self
            
            if durum is None:
                # Toggle
                self.ortam_ref.minimap.visible = not self.ortam_ref.minimap.visible
                status = "AÇIK" if self.ortam_ref.minimap.visible else "KAPALI"
                print(f"🗺️ [MİNİMAP] Minimap şu an {status}")
            else:
                # Görünürlük ve ayarları güncelle
                self.ortam_ref.minimap.goster(durum, convex, a_star)
        else:
            print("❌ [MİNİMAP] Minimap sistemi bulunamadı!") 

    def a_star(self, start=None, goal=None, safety_margin=20.0, **kwargs):
        """
        A* algoritması kullanarak başlangıçtan hedefe yol hesaplar.
        
        Args:
            start: (x, y) başlangıç koordinatları (metre), ROV ID (int), veya kwargs'tan alınır
                - Eğer int ise: ROV ID olarak yorumlanır ve GPS bilgisi çekilir
                - Eğer tuple/list ise: Doğrudan (x, y) koordinatları olarak kullanılır
            goal: (x, y) hedef koordinatları (metre) veya kwargs'tan alınır
            safety_margin: Engel etrafında güvenlik mesafesi (metre, varsayılan: 8.0)
            **kwargs: Alternatif parametre geçişi için
        
        Returns:
            Optional[List[Tuple[float, float]]]: Bulunan yol [(x1, y1), (x2, y2), ...] veya None
        
        Örnekler:
            # ROV ID ile başlangıç
            yol = filo.a_star(start=0, goal=(100, 100))  # ROV-0'ın GPS'inden başla
            
            # Doğrudan koordinatlar
            yol = filo.a_star(start=(-100, -100), goal=(100, 100), safety_margin=2.0)
            
            # kwargs ile
            yol = filo.a_star(start=(-100, -100), goal=(100, 100))
        """
        # kwargs'tan parametreleri al (eğer doğrudan verilmemişse)
        if start is None:
            start = kwargs.get('start')
        if goal is None:
            goal = kwargs.get('goal')
        if safety_margin == 8.0:  # Varsayılan değer, kwargs'tan kontrol et
            safety_margin = kwargs.get('safety_margin', 8.0)
        
        # Start parametresi ROV ID ise GPS bilgisini çek
        if isinstance(start, int):
            rov_id = start  # ROV ID'yi sakla
            try:
                gps_bilgisi = self.get(rov_id, 'gps')
                if gps_bilgisi is None:
                    print(f"❌ [FİLO] ROV-{rov_id} için GPS bilgisi alınamadı!")
                    return None
                
                # GPS formatı: (x, y, z) -> (x, y) olarak al
                if isinstance(gps_bilgisi, (tuple, list)) and len(gps_bilgisi) >= 2:
                    start = (float(gps_bilgisi[0]), float(gps_bilgisi[1]))
                    print(f"✅ [FİLO] ROV-{rov_id}'ın GPS'inden başlangıç: {start}")
                else:
                    print(f"❌ [FİLO] ROV-{rov_id} için geçersiz GPS formatı: {gps_bilgisi}")
                    return None
            except Exception as e:
                print(f"❌ [FİLO] ROV-{rov_id} GPS bilgisi alınırken hata: {e}")
                return None
        
        # Parametre kontrolü
        if start is None or goal is None:
            print("❌ [FİLO] A* için start ve goal parametreleri gerekli!")
            print("   Kullanım: filo.a_star(start=(x1, y1), goal=(x2, y2), safety_margin=2.0)")
            print("   veya: filo.a_star(start=rov_id, goal=(x2, y2))  # ROV ID ile başlangıç")
            return None
        
        # Start'ın tuple/list formatında olduğunu kontrol et
        if not isinstance(start, (tuple, list)) or len(start) < 2:
            print(f"❌ [FİLO] Start parametresi geçersiz format: {start}")
            print("   Format: (x, y) tuple veya [x, y] list olmalı")
            return None
        
        # Harita referansını kontrol et
        if not self.ortam_ref or not hasattr(self.ortam_ref, 'harita') or self.ortam_ref.harita is None:
            print("❌ [FİLO] Harita sistemi bulunamadı!")
            return None
        
        # Harita'nın a_star_yolu_hesapla metodunu çağır
        try:
            return self.ortam_ref.harita.a_star_yolu_hesapla(
                start=start,
                goal=goal,
                safety_margin=safety_margin
            )
        except Exception as e:
            print(f"❌ [FİLO] A* yolu hesaplanırken hata: {e}")
            import traceback
            traceback.print_exc()
            return None

    def gidilecek_noktalar(self, path=None, r=10, derece_threshold=15):
        """
        A* yolu üzerinden gidilecek noktaları filtreler.
        Mesafe ve eğim açısına göre gereksiz noktaları çıkarır.

        Args:
            path: [(x1, y1), (x2, y2), ...] şeklindeki orijinal yol
                (None ise haritadaki A* yolunu kullanır)
            r: Örnekleme mesafesi (yarıçap, metre, varsayılan: 10)
            derece_threshold: Kabul edilen minimum eğim açısı
                            (derece, varsayılan: 15)

        Returns:
            List[List[float, float]]: [[x, y], [x, y], ...]
            şeklinde filtrelenmiş koordinat dizisi
        """

        # Eğer path verilmemişse, haritadaki A* yolunu kullan
        if path is None:
            if not self.ortam_ref or not hasattr(self.ortam_ref, 'harita') or self.ortam_ref.harita is None:
                print("❌ [FİLO] Harita sistemi bulunamadı!")
                return []

            if not hasattr(self.ortam_ref.harita, 'a_star_yolu') or self.ortam_ref.harita.a_star_yolu is None:
                print("⚠️ [FİLO] A* yolu henüz hesaplanmamış!")
                print("   Önce filo.a_star(start=(x1, y1), goal=(x2, y2)) çağırın.")
                return []

            path = self.ortam_ref.harita.a_star_yolu

        # Path boşsa boş liste döndür
        if len(path) == 0:
            return []

        gidilecek_noktalar = []

        # Başlangıç referans noktası
        x_baslangic, y_baslangic = path[0]

        # İlk noktayı ekle (başlangıç noktası)
        gidilecek_noktalar.append([x_baslangic, y_baslangic])

        aci_radyan = np.arctan2(y_baslangic, x_baslangic)
        ilk_derece = np.degrees(aci_radyan)

        for i in range(1, len(path)):
            x_son, y_son = path[i]

            # İki nokta arasındaki mesafe hesabı
            mesafe = np.sqrt(
                (x_son - x_baslangic) ** 2 +
                (y_son - y_baslangic) ** 2
            )

            if mesafe >= r:
                # arctan2 kullanarak eğim açısını (radyan) hesapla
                aci_radyan = np.arctan2(
                    y_son - y_baslangic,
                    x_son - x_baslangic
                )
                son_derece = np.degrees(aci_radyan)

                fark = ilk_derece - son_derece

                # Eğim açısı eşik değeri geçiyorsa ekle
                if abs(fark) >= derece_threshold:
                    ilk_derece = son_derece
                    gidilecek_noktalar.append([x_son, y_son])

                    # Referans noktasını güncelle
                    x_baslangic, y_baslangic = x_son, y_son

        # Son noktayı da ekle (hedef)
        if len(path) > 1:
            son_nokta = path[-1]
            if son_nokta not in gidilecek_noktalar:
                gidilecek_noktalar.append([son_nokta[0], son_nokta[1]])

        return gidilecek_noktalar

# ==========================================
# 2. TEMEL GNC SINIFI
# ==========================================
class TemelGNC:
    def __init__(self, rov_entity, modem, filo_ref=None):
        self.rov = rov_entity
        self.modem = modem
        self.filo_ref = filo_ref  # Filo referansı (çoklu nokta takibi için)
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
            if self.rov.velocity.length() > 1:
                self.rov.velocity *= 0.4  # Momentumu yumuşatarak durdur
            return
        
        # 1. Mevcut pozisyonu Ursina'dan alıp Simülasyona çevir
        current_sim_pos = Vec3(*Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z))
        
        # 2. Farkı Simülasyon dünyasında hesapla
        fark = self.hedef - current_sim_pos
        mevcut_mesafe = fark.length()

        # HEDEF KONTROLÜ: Hedefe ulaşıldıysa dur veya sonraki noktaya geç
        # Salınım önleme: Hedefe çok yakınsa (0.5m) motorları durdur ve yaw açısını hedefe doğru ayarla
        if mevcut_mesafe <= 0.5:
            # Hedefe çok yakın - motorları durdur ve yaw açısını hedefe doğru ayarla
            self.rov.velocity *= 0.1  # Çok agresif durdurma (salınım önleme)
            
            # Yaw açısını hedefe doğru ayarla (stabilite için)
            if hasattr(self, 'filo_ref') and self.filo_ref:
                rov_id = None
                for idx, gnc in enumerate(self.filo_ref.sistemler):
                    if gnc == self:
                        rov_id = idx
                        break
                
                if rov_id is not None:
                    # Hedefe doğru yaw açısını hesapla
                    dx = fark.x
                    dy = fark.y
                    if abs(dx) > 0.01 or abs(dy) > 0.01:
                        yaw_rad = math.atan2(dx, dy)
                        yaw_deg = math.degrees(yaw_rad)
                        # Normalize et
                        while yaw_deg >= 360:
                            yaw_deg -= 360
                        while yaw_deg < 0:
                            yaw_deg += 360
                        # Yaw açısını direkt ayarla (kademeli değil, hedefe doğru dön)
                        if hasattr(self.rov, 'rotation_y'):
                            # Kademeli dönüş yerine direkt hedefe dön (daha stabil)
                            mevcut_yaw = self.rov.rotation_y
                            yaw_farki = yaw_deg - mevcut_yaw
                            # En kısa yolu bul (-180 ile 180 arası)
                            if yaw_farki > 180:
                                yaw_farki -= 360
                            elif yaw_farki < -180:
                                yaw_farki += 360
                            # Hızlı dönüş (her frame'de maksimum 5 derece)
                            max_donme = 5.0
                            if abs(yaw_farki) > max_donme:
                                yaw_farki = max_donme if yaw_farki > 0 else -max_donme
                            self.rov.rotation_y = mevcut_yaw + yaw_farki
            
            # Çoklu nokta takibi: Sonraki noktaya geç (0.5m tolerans ile)
            if mevcut_mesafe <= 0.5 and hasattr(self, 'filo_ref') and self.filo_ref:
                rov_id = None
                # ROV ID'yi bul
                for idx, gnc in enumerate(self.filo_ref.sistemler):
                    if gnc == self:
                        rov_id = idx
                        break
                
                if rov_id is not None and rov_id in self.filo_ref._git_nokta_listesi:
                    nokta_listesi = self.filo_ref._git_nokta_listesi[rov_id]
                    mevcut_indeks = self.filo_ref._git_mevcut_nokta_indeksi.get(rov_id, 0)
                    
                    # Sonraki noktaya geç
                    if mevcut_indeks + 1 < len(nokta_listesi):
                        sonraki_nokta = nokta_listesi[mevcut_indeks + 1]
                        self.filo_ref._git_mevcut_nokta_indeksi[rov_id] = mevcut_indeks + 1
                        
                        # Sonraki noktayı hedef olarak ata
                        self.hedef = Vec3(sonraki_nokta[0], sonraki_nokta[1], self.hedef.z)
                        # Konsolu rahatsız etmemek için print'i kaldır (arka plan işlemi)
                    else:
                        # Tüm noktalar tamamlandı
                        # Listeyi temizle
                        if rov_id in self.filo_ref._git_nokta_listesi:
                            del self.filo_ref._git_nokta_listesi[rov_id]
                        if rov_id in self.filo_ref._git_mevcut_nokta_indeksi:
                            del self.filo_ref._git_mevcut_nokta_indeksi[rov_id]
            
            return
        
        # Hedefe yaklaşırken hızı azalt (salınım önleme)
        if mevcut_mesafe < 2.0:
            # Hedefe yaklaşırken hızı azalt
            hiz_carpani = mevcut_mesafe / 2.0  # 2m'de 1.0, 0.5m'de 0.25
            hiz_carpani = max(0.2, min(1.0, hiz_carpani))  # Minimum 0.2, maksimum 1.0
        else:
            hiz_carpani = 1.0

        # 3. Hareket vektörünü normalize et
        if mevcut_mesafe > 0.01:
            hareket_vektoru = fark / mevcut_mesafe
        else:
            hareket_vektoru = Vec3(0, 0, 0)
        
        # 4. Yaw açısını hedefe doğru ayarla (her zaman hedefe dön)
        if hasattr(self, 'filo_ref') and self.filo_ref:
            rov_id = None
            for idx, gnc in enumerate(self.filo_ref.sistemler):
                if gnc == self:
                    rov_id = idx
                    break
            
            if rov_id is not None:
                # Hedefe doğru yaw açısını hesapla
                dx = fark.x
                dy = fark.y
                if abs(dx) > 0.01 or abs(dy) > 0.01:
                    yaw_rad = math.atan2(dx, dy)
                    yaw_deg = math.degrees(yaw_rad)
                    # Normalize et
                    while yaw_deg >= 360:
                        yaw_deg -= 360
                    while yaw_deg < 0:
                        yaw_deg += 360
                    
                    # Yaw açısını direkt ayarla (her zaman hedefe dön)
                    if hasattr(self.rov, 'rotation_y'):
                        mevcut_yaw = self.rov.rotation_y
                        yaw_farki = yaw_deg - mevcut_yaw
                        # En kısa yolu bul (-180 ile 180 arası)
                        if yaw_farki > 180:
                            yaw_farki -= 360
                        elif yaw_farki < -180:
                            yaw_farki += 360
                        # Hızlı dönüş (her frame'de maksimum 5 derece)
                        max_donme = 5.0
                        if abs(yaw_farki) > max_donme:
                            yaw_farki = max_donme if yaw_farki > 0 else -max_donme
                        self.rov.rotation_y = mevcut_yaw + yaw_farki
        
        # 5. Hareket vektörünü motor komutlarına haritala
        # hareket_vektoru.x -> Sağ/Sol
        # hareket_vektoru.y -> İleri/Geri (Simülasyonda Y ileridir)
        # hareket_vektoru.z -> Çık/Bat (Simülasyonda Z derinliktir)
        # Hızı hiz_carpani ile çarp (yaklaşırken yavaşla)
        guc_degeri = 0.4 * hiz_carpani if 'hiz_carpani' in locals() else 0.4
        self.vektor_to_motor_sim(hareket_vektoru, guc=guc_degeri)

    def vektor_to_motor_sim(self, v_sim, guc=0.4):
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


