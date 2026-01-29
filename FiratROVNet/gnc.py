from typing_extensions import Self
import numpy as np
from ursina import Vec3, time, distance
from .config import cfg, GATLimitleri, SensorAyarlari, ModemAyarlari, HareketAyarlari, Formasyon
from .iletisim import AkustikModem
from .hull import HullManager
from FiratROVNet.kutuphane.helper.gnc_helper import FiloHelper, TemelGNCHelper
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
    """
    ROV filo yönetim sistemi.
    Thread-safe komut işleme, formasyon yönetimi ve hareket kontrolü sağlar.
    """
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
        
        # Helper instance for complex calculations
        self.helper = FiloHelper(self)
    
    # ============================================================
    # THREAD MANAGEMENT
    # ============================================================
    
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
    
    # ============================================================
    # SYSTEM MANAGEMENT
    # ============================================================
    
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
                # Lider ROVa
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
        
        if getattr(self.ortam_ref, "verbose", False):
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

    # ============================================================
    # MOVEMENT LOGIC
    # ============================================================

    def guncelle_hepsi(self, tahminler):
        """Tüm GNC sistemlerini günceller ve yaw senkronizasyonu yapar."""
        # Ana thread'de queue'daki komutları işle (thread-safe)
        self._process_command_queue()
        
        # Lider ROV'u bul
        lider_rov_id, lider_gnc, lider_rov = self._find_leader()
        
        # Tüm GNC sistemlerini güncelle
        for i, gnc in enumerate(self.sistemler):
            if i < len(tahminler):
                gnc.guncelle(tahminler[i])
        
        # Formasyon yaw senkronizasyonu
        if lider_rov_id is not None and len(self._formasyon_hedefleri) > 0:
            lider_yaw = self.get(lider_rov_id, 'yaw')
            if lider_yaw is not None:
                self._formasyon_yaw_senkronizasyonu(lider_rov_id, lider_yaw)
        
        # git() yaw senkronizasyonu
        if len(self._git_hedef_yaw) > 0:
            self._git_yaw_senkronizasyonu()
    
    def _find_leader(self) -> tuple:
        """Lider ROV'u bulur ve bilgilerini döndürür."""
        for i, gnc in enumerate(self.sistemler):
            if hasattr(gnc, 'rov') and gnc.rov.role == 1:
                return i, gnc, gnc.rov
        return None, None, None
    
    def _formasyon_yaw_senkronizasyonu(self, lider_rov_id: int, lider_yaw: float) -> None:
        """Formasyon yaw senkronizasyonu: Takipçi ROV'lar hedefe yaklaştığında liderin yaw açısına göre yönlenir."""
        dt = time.dt if hasattr(time, 'dt') else 0.016
        
        for rov_id, hedef_bilgisi in list(self._formasyon_hedefleri.items()):
            if rov_id >= len(self.sistemler) or rov_id == lider_rov_id:
                continue
            
            if not hasattr(self.sistemler[rov_id], 'rov'):
                continue
            
            takipci_rov = self.sistemler[rov_id].rov
            mevcut_sim_pos = Koordinator.ursina_to_sim(takipci_rov.x, takipci_rov.y, takipci_rov.z)
            mevcut_x, mevcut_y, mevcut_z = mevcut_sim_pos
            
            # Hedef bilgisini al ve normalize et
            if isinstance(hedef_bilgisi, dict):
                hedef_pozisyon = hedef_bilgisi.get('pozisyon')
                hedef_yaw = hedef_bilgisi.get('hedef_yaw', lider_yaw)
            else:
                hedef_pozisyon = hedef_bilgisi
                hedef_yaw = lider_yaw
                self._formasyon_hedefleri[rov_id] = {
                    'pozisyon': hedef_pozisyon,
                    'hedef_yaw': hedef_yaw
                }
            
            if hedef_pozisyon is None:
                continue
            
            hedef_x, hedef_y, hedef_z = hedef_pozisyon
            dx = hedef_x - mevcut_x
            dy = hedef_y - mevcut_y
            mesafe_2d = math.sqrt(dx**2 + dy**2)
            
            # Eğer hedefe yaklaştıysa, liderin yaw açısına göre yönlen
            if mesafe_2d <= self._formasyon_yaw_senkronizasyon_mesafesi:
                hedef_yaw = lider_yaw
                self._formasyon_hedefleri[rov_id]['hedef_yaw'] = hedef_yaw
                
            # Yaw senkronizasyonu yap
            if self._yaw_senkronizasyon(rov_id, hedef_yaw, self._maksimum_yaw_donme_hizi, dt):
                # Hedefi kaldır (artık yaw senkronize edildi)
                if rov_id in self._formasyon_hedefleri:
                    del self._formasyon_hedefleri[rov_id]
    
    def _git_yaw_senkronizasyonu(self) -> None:
        """git() yaw senkronizasyonu: git() ile gönderilen ROV'ların yaw açıları kademeli olarak güncellenir."""
        dt = time.dt if hasattr(time, 'dt') else 0.016
        
        for rov_id, hedef_yaw in list(self._git_hedef_yaw.items()):
            if rov_id >= len(self.sistemler):
                if rov_id in self._git_hedef_yaw:
                    del self._git_hedef_yaw[rov_id]
                continue
            
            if not hasattr(self.sistemler[rov_id], 'rov'):
                continue
            
            # Yaw senkronizasyonu yap
            if self._yaw_senkronizasyon(rov_id, hedef_yaw, self._git_maksimum_yaw_donme_hizi, dt):
                # Hedefi kaldır (artık yaw hedefine ulaşıldı)
                if rov_id in self._git_hedef_yaw:
                    del self._git_hedef_yaw[rov_id]
    
    def _yaw_senkronizasyon(self, rov_id: int, hedef_yaw: float, maksimum_donme_hizi: float, dt: float) -> bool:
        """
        Yaw senkronizasyonu yapar (ortak helper method).
        
        Args:
            rov_id: ROV ID
            hedef_yaw: Hedef yaw açısı (derece)
            maksimum_donme_hizi: Maksimum dönme hızı (derece/saniye)
            dt: Frame süresi (saniye)
        
        Returns:
            bool: True eğer hedefe ulaşıldıysa (hedef kaldırılabilir), False aksi halde
        """
        mevcut_yaw = self.get(rov_id, 'yaw')
        if mevcut_yaw is None:
            return False
        
        # Yaw açıları arasındaki farkı hesapla ve normalize et
        yaw_farki = hedef_yaw - mevcut_yaw
        while yaw_farki > 180:
            yaw_farki -= 360
        while yaw_farki < -180:
            yaw_farki += 360
        
        # Eğer açı farkı çok küçükse (1 dereceden az), hedefe ulaşıldı
        if abs(yaw_farki) <= 1.0:
            return True
        
        # Kademeli dönme
        maksimum_donme_acisi = maksimum_donme_hizi * dt
        
        if abs(yaw_farki) <= maksimum_donme_acisi:
            # Direkt hedefe git
            yeni_yaw = hedef_yaw
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
        return False
    
    def set(self, rov_id: int, ayar_adi: str, deger) -> bool:
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
        # ============================================================
        # THREAD MANAGEMENT
        # ============================================================
        if not self._is_main_thread():
            self._command_queue.put(('set', (rov_id, ayar_adi, deger), {}))
            return True
        
        return self._set_impl(rov_id, ayar_adi, deger)
    
    def _set_impl(self, rov_id: int, ayar_adi: str, deger) -> bool:
        """set() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır)."""
        # ============================================================
        # GUARD CLAUSES - Erken Çıkışlar
        # ============================================================
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            print(f"   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return False
        
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

    def get(self, rov_id: int = None, veri_tipi: str = None, taraf: int = None, sessiz: bool = False):
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
            sessiz: Hata mesajlarını bastırır (RL eğitimi için)
        
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
        return self.helper.get(rov_id=rov_id, veri_tipi=veri_tipi, taraf=taraf, koordinator=Koordinator, sessiz=sessiz)
    
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
                # None kontrolü (çıkarılmış ROV'lar için sistem yoksa None olabilir)
                if i < len(self.sistemler) and self.sistemler[i] is not None:
                    rov = self.sistemler[i].rov
                    if rov is not None:
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
        return self.helper.points()
    
    def _compute_obstacle_positions(self, rov_id):
        """
        ROV'un tüm lidar sensörlerinden engel koordinatlarını hesaplar.
        Simülasyon formatında (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik) çalışır.
        
        Args:
            rov_id: ROV ID
        
        Returns:
            list: [(x, y, z), ...] - Tespit edilen engellerin koordinatları (Sim formatı)
        """
        return self.helper.compute_obstacle_positions(rov_id)

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
        return self.helper.formasyon(formasyon_id=formasyon_id, aralik=aralik, is_3d=is_3d, lider_koordinat=lider_koordinat)
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
        return self.helper.formasyon_sec(
            margin=margin,
            is_3d=is_3d,
            offset=offset,
            harita=harita,
            yaw_senkronizasyon_mesafesi=yaw_senkronizasyon_mesafesi,
            maksimum_yaw_donme_hizi=maksimum_yaw_donme_hizi,
        )
    

    def get_100_samples(self, hull_output=None, sample_count=100):
        """
        yeni_hull çıktısındaki noktaları alır ve çevre uzunluğu üzerinden 
        sabit sayıda (sample_count) örnek nokta döndürür.
        
        Args:
            hull_output (dict): {'hull': ..., 'points': np.array, 'center': ...}
            sample_count (int): Hedeflenen sabit nokta sayısı (Varsayılan: 100)
            
        Returns:
            np.ndarray: (sample_count, 2) boyutunda örneklenmiş noktalar

        """
        return self.helper.get_100_samples(hull_output=hull_output, sample_count=sample_count)

    # --- KULLANIM ÖRNEĞİ ---
    # hull_sonuc = filo.yeni_hull(filo.ada_cevre())
    # sabit_100_nokta = get_100_samples(hull_sonuc, 100)

    # Artık 'sabit_100_nokta' değişkenini RL modeline input olarak verebilirsin.
    # Boyutu her zaman (100, 2) olacaktır.
    # ============================================================
    # FORMATION LOGIC - Helper Methods
    # ============================================================

    def uret_rl_egitim_verisi(self):
            """
            RL eğitimi için hızlı senaryo üretir ve sabit boyutlu verileri döner.
            
            Dönüş Formatı:
            - lider_pozisyon: [x, y, z] (3,)
            - lider_yaw: float (1,)
            - rov_filo_gps: 8 adet ROV için [x, y, z] (8, 3) - Olmayanlar 400.0
            - hull_merkez: [x, y] (2,) - Yoksa 400.0
            - hull_noktalar: 100 adet [x, y] (100, 2) - Yoksa 400.0
            """
            return self.helper.uret_rl_egitim_verisi()
    # ============================================================
    # FORMATION LOGIC - Helper Methods
    # ============================================================

    def lider_sec_veri_uret(self):
            """
            RL eğitimi için lider seçim verisi üretir.
            Matematiksel liderlik formülünü 'Label' olarak kullanır.
            """
            return self.helper.lider_sec_veri_uret(asil_hedef=self.asil_hedef)
    
    def gat_veri_uret(self):
        """
        GAT eğitimi için senaryo verisi üretir.
        Senaryo.py kullanarak 8, 6, 4 rastgele ROV ve 2-5 arası ada ile ortam oluşturur.
        
        Returns:
            dict: {
                'senaryo': Senaryo instance,
                'filo': Filo instance,
                'ortam': Ortam instance,
                'n_rovs': int,
                'n_adalar': int,
                'n_engels': int
            } veya None (hata durumunda)
        """
        return self.helper.gat_veri_uret()
    
    def _prepare_forbidden_points(self) -> list:
        """Ada çevre noktalarını yasaklı nokta listesine dönüştürür."""
        return self.helper.prepare_forbidden_points()
    
    def _normalize_hull_center(self, hull_merkez) -> tuple:
        """Hull merkezini Sim formatına dönüştürür (z=0 yapar)."""
        return self.helper.normalize_hull_center(hull_merkez)
    
    def _find_leader_info(self, sessiz: bool = False) -> tuple:
        """Lider ROV ID ve GPS koordinatını bulur."""
        return self.helper.find_leader_info(sessiz=sessiz)
    
    def _generate_search_points(self, lider_gps: tuple, hull_merkez: tuple) -> list:
        """Lider GPS'ten hull merkezine kadar ara noktalar oluşturur."""
        return self.helper.generate_search_points(lider_gps, hull_merkez)
    
    def _get_formation_ids_to_try(self) -> list:
        """Denenecek formasyon ID'lerini pool'dan alır."""
        return self.helper.get_formation_ids_to_try()
    
    def _try_formation_fit(self, formasyon_id: int, aralik: float, is_3d: bool, 
                          merkez_koordinat: tuple, deneme_yaw: float, hull, 
                          lider_rov_id: int, nokta_adi: str, sessiz: bool = False) -> bool:
        """Formasyonun geçerli olup olmadığını kontrol eder ve uygular."""
        return self.helper.try_formation_fit(formasyon_id, aralik, is_3d,
                                            merkez_koordinat, deneme_yaw, hull,
                                            lider_rov_id, nokta_adi, sessiz=sessiz)

    

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
        return self.helper.hull(offset=offset)
    
    def _guvenlik_hull_olustur_impl(self, offset=20.0):
        """guvenlik_hull_olustur() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır)."""
        return self.hull_manager.hull(offset=offset)
    
    def ada_cevre(self, offset=15.0, sessiz: bool = False):
        """
        Simülasyondaki adaları tespit edip her ada için eşit çevrede 12 nokta döndürür.
        
        Her ada için 12 nokta hesaplanır (30° aralıklarla: 0°, 30°, 60°, ..., 330°).
        Noktalar ada yarıçapından belirli bir mesafe uzakta olur (offset parametresi).
        
        Args:
            offset (float): Ada yarıçapından uzaklık (metre, varsayılan: 15.0)
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
        return self.helper.ada_cevre(offset=offset, sessiz=sessiz)
    
    def yeni_hull(self, yasakli_noktalar, offset=40.0, alpha=2.0, buffer_radius=20.0, channel_width=15.0):
        """
        Mevcut hull noktalarını alır, yasaklı bölgeleri kesip çıkarır.
        Hem harita çizimi hem de 'is_point_inside' kontrolü için uyumlu nesne döndürür.
        """
        return self.helper.yeni_hull(yasakli_noktalar, offset, alpha, buffer_radius, channel_width)
    def yeniden_ciz(self, noktalar, yasakli_noktalar, alpha=2.0, buffer_radius=15.0, channel_width=10.0):
        """
        Verilen nokta kümesini saran, ancak yasaklı noktaları dışarıda bırakacak şekilde
        içeri bükülmüş sınırın koordinatlarını döndürür.
        """
        return self.helper.yeniden_ciz(noktalar, yasakli_noktalar, alpha, buffer_radius, channel_width)
    
    def _hedef_gorsel_olustur(self, x, y, z):
        """
        Hedef pozisyonunu Ursina'da büyük X işareti olarak gösterir.
        """
        return self.helper.hedef_gorsel_olustur(x, y, z)

    def git(self, rov_id: int, x, y: float = None, z: float = None, ai: bool = True, sessiz: bool = False) -> None:
        """
        ROV'a hedef koordinatı atar ve otomatik moda geçirir (Thread-safe).
        Tüm girişler Simülasyon formatındadır: (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik)
        
        Çoklu nokta desteği: Eğer x bir liste ise, ROV bu noktaları sırayla ziyaret eder.
        
        Args:
            sessiz: Log mesajlarını kapatır (RL eğitimi için)

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
        return self.helper.git(rov_id=rov_id, x=x, y=y, z=z, ai=ai, sessiz=sessiz)

    def git_path(self, rov_id, hedef, ai=True):
        """
        ROV'a bir yol atar ve otomatik moda geçirir (Thread-safe).
        """
        return self.helper.git_path(rov_id, hedef, ai=ai)
    
    def _git_impl(self, rov_id: int, x: float, y: float, z: float = None, ai: bool = True, sessiz: bool = False) -> None:
        """git() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır)."""
        return self.helper._git_impl(rov_id, x, y, z, ai, sessiz=sessiz)

    def move(self, rov_id: int, yon: str, guc: float = 1.0) -> None:
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
        return self.helper.move(rov_id=rov_id, yon=yon, guc=guc)
    
    def rov(self, rov_id: int, komut: str, konum=None):
        """
        ROV ekleme ve çıkarma işlemleri için metod.
        Arka planda ROV sınıfının ekle() ve cikar() metotlarını kullanır.
        
        Args:
            rov_id: ROV ID'si
            komut: "ekle" veya "cikar"
            konum: (x, y, z) tuple veya None (sadece "ekle" komutu için)
        
        Returns:
            bool: İşlem başarılı ise True, aksi halde False
        
        Örnekler:
            # ROV ekle (varsayılan pozisyon)
            filo.rov(5, "ekle")
            
            # ROV ekle (belirtilen pozisyon)
            filo.rov(6, "ekle", (50, -10, 30))
            
            # ROV çıkar
            filo.rov(5, "cikar")
        """
        # Ortam referansı kontrolü
        if not hasattr(self, 'ortam_ref') or self.ortam_ref is None:
            print(f"⚠️ ROV-{rov_id} işlemi başarısız: ortam_ref bulunamadı")
            return False
        
        ortam = self.ortam_ref
        
        # ROV listesi kontrolü
        if not hasattr(ortam, 'rovs'):
            ortam.rovs = []
        
        # ROV sınıfını import et (lazy import)
        from .simulasyon import ROV
        
        if komut == "ekle":
            # Eğer bu ID'de zaten ROV varsa, ekleme yapma
            # Sessiz mod kontrolü (GAT eğitimi için)
            if rov_id < len(ortam.rovs) and ortam.rovs[rov_id] is not None:
                # Sessiz mod aktif değilse uyarı göster
                if not getattr(self, '_sessiz_mod', False):
                    print(f"⚠️ ROV-{rov_id} zaten mevcut. Önce çıkarmak için: filo.rov({rov_id}, 'cikar')")
                return False
            
            # Yeni ROV oluştur
            if konum is not None and isinstance(konum, (tuple, list)) and len(konum) == 3:
                x, y, z = konum
                rov = ROV(rov_id, position=(x, y, z))
            else:
                rov = ROV(rov_id)
            
            # Environment referansını ayarla (ekle() çağrılmadan önce)
            if hasattr(self, 'ortam_ref') and self.ortam_ref is not None:
                rov.environment_ref = self.ortam_ref
            elif hasattr(self, 'environment_ref') and self.environment_ref is not None:
                rov.environment_ref = self.environment_ref
            
            # ROV'u ekle
            success = rov.ekle(konum)
            
            # Başarılıysa filo sistemine ekle
            if success:
                # Eğer filo sisteminde bu ROV için GNC sistemi yoksa, otomatik kurulum yapılabilir
                # Ama şimdilik sadece ROV'u ekliyoruz
                pass
            
            return success
        
        elif komut == "cikar":
            # ROV'u bul
            # Sessiz mod kontrolü (GAT eğitimi için)
            if rov_id >= len(ortam.rovs) or ortam.rovs[rov_id] is None:
                # Sessiz mod aktif değilse uyarı göster
                if not getattr(self, '_sessiz_mod', False):
                    print(f"⚠️ ROV-{rov_id} bulunamadı")
                return False
            
            rov = ortam.rovs[rov_id]
            
            # ROV'u çıkar
            success = rov.cikar()
            
            # Başarılıysa filo sisteminden de çıkar (eğer varsa)
            if success:
                # Sistemler listesinden bu ROV'un GNC sistemini çıkar
                self.sistemler = [s for s in self.sistemler if not (hasattr(s, 'rov') and hasattr(s.rov, 'id') and s.rov.id == rov_id)]
            
            return success
        
        else:
            print(f"⚠️ Geçersiz komut: '{komut}'. 'ekle' veya 'cikar' kullanın.")
            return False

    def harita(self, goster=True, convex=True, a_star=True):
        """Harita penceresini açar, kapatır veya görünürlük ayarlarını yapar."""
        return self.helper.harita(goster=goster, convex=convex, a_star=a_star)
    
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
        return self.helper.minimap(durum=durum, convex=convex, a_star=a_star)

    def a_star(self, start=None, goal=None, safety_margin=15.0, **kwargs):
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
        return self.helper.a_star(start=start, goal=goal, safety_margin=safety_margin, **kwargs)
    
    
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

        return self.helper.gidilecek_noktalar(path=path, r=r, derece_threshold=derece_threshold)



# ==========================================
# 2. TEMEL GNC SINIFI
# ==========================================
# ==========================================
# 2. TEMEL GNC SINIFI
# ==========================================
class TemelGNC:
    """
    ROV için Temel Güdüm, Navigasyon ve Kontrol (GNC) sınıfı.
    Modernize edilmiş yapı.
    """
    # Sabitler (Eski kodun davranışını koruyacak şekilde)
    HEDEF_TOLERANSI = 0.5
    YAVASLAMA_MESAFESI = 2.0
    
    def __init__(self, rov_entity, modem, filo_ref=None):
        self.rov = rov_entity
        self.modem = modem
        self.filo_ref = filo_ref
        self.hedef = None 
        self.hiz_limiti = 100.0 
        self.manuel_kontrol = False
        self.ai_aktif = True 
        
        # Helper instance for complex calculations
        self.helper = TemelGNCHelper(rov_entity, filo_ref, self)
        
        # GAT Manevra Yöneticisi - Filo referansı ile initialize et
        # ROV ID dinamik olarak guncelle() içinde bulunacak
        self.gat_manevra = GATManevraYoneticisi(filo_ref, None) if filo_ref else None 

    def hedef_atama(self, x, y, z):
        self.hedef = Vec3(x, y, z)

    def rehber_guncelle(self, rehber):
        if self.modem: 
            self.modem.rehber_guncelle(rehber)
    
    def guncelle(self, gat_kodu=None):
        """
        GNC Güncelleme: Hedef varsa ve manuel kontrol kapalıysa hedefe git.
        GAT kodlarına göre manevra yapılır.
        Sensör verilerine göre GAT kodu otomatik belirlenir.
        Modernize edilmiş versiyon.
        """
        return self.helper.guncelle(gat_kodu=gat_kodu)

    def _hedefe_varis_islemleri(self, fark):
        """Hedefe ulaşıldığında yapılacak işlemler."""
        self.rov.velocity *= 0.1
        self.helper.yaw_ayarla(fark, ani=True)  # Son düzeltme
        
        # Çoklu nokta geçiş mantığı (Filo ref üzerinden)
        if self.filo_ref:
            self._siradaki_noktaya_gec()

    def _siradaki_noktaya_gec(self):
        """Çoklu nokta takibinde sonraki noktaya geçer."""
        try:
            my_id = self.filo_ref.sistemler.index(self)
            nokta_listesi = self.filo_ref._git_nokta_listesi.get(my_id)
            mevcut_indeks = self.filo_ref._git_mevcut_nokta_indeksi.get(my_id, 0)
            
            if nokta_listesi and mevcut_indeks + 1 < len(nokta_listesi):
                yeni_indeks = mevcut_indeks + 1
                nxt = nokta_listesi[yeni_indeks]
                self.filo_ref._git_mevcut_nokta_indeksi[my_id] = yeni_indeks
                self.hedef = Vec3(nxt[0], nxt[1], self.hedef.z)
            elif nokta_listesi:
                # Liste bitti
                self.filo_ref._git_nokta_listesi.pop(my_id, None)
        except:
            pass

    def _hiz_hesapla(self, mesafe: float) -> float:
        """Hedefe yaklaşırken hızı azaltır (wrapper for helper)."""
        return self.helper.hiz_hesapla(mesafe)

    def _yaw_ayarla(self, fark_vektoru: Vec3, ani: bool = False):
        """Yaw açısını hedefe doğru ayarlar (wrapper for helper)."""
        self.helper.yaw_ayarla(fark_vektoru, ani=ani)

    def vektor_to_motor_sim(self, v_sim: Vec3, guc: float = 0.4):
        """
        Vektörü Simülasyon eksenlerinden Ursina motor komutlarına çevirir.
        Global koordinatlara göre direkt hareket eder (yaw açısından bağımsız).
        
        Args:
            v_sim: Simülasyon formatında vektör (X: Sağ-Sol, Y: İleri-Geri, Z: Derinlik)
            guc: Güç çarpanı (varsayılan: 0.4)
        """
        self.helper.vektor_to_motor_sim(v_sim, guc=guc)


# ==========================================
# YENİ SINIF: GAT MANEVRA YÖNETİCİSİ (KACIN)
# ==========================================
class GATManevraYoneticisi:
    """
    GAT (AI) tahminlerine göre özel kaçınma manevraları ve vektörleri üretir.
    
    GAT Kodları:
    0: OK -> Normal Seyir (Hedefe doğru normal hızda git)
    1: ENGEL -> Yumuşak Kaçınma (Hızı azalt, yanlamasına git, lidar'a göre yön seç)
    2: CARPISMA -> Acil Durum (Tam geri, sert dönüş, rastgele sağ/sol kırma)
    3: KOPUK -> İletişim Kopması (Dur, yukarı çık, iletişimi yeniden kurmaya çalış)
    4: UZAK -> Liderden Uzaklaşma (Hızı artır, lideri yakalamaya çalış)
    """
    def __init__(self, filo_ref, rov_id):
        """
        Initialize GAT Manevra Yöneticisi.
        
        Args:
            filo_ref: Filo referansı (lidar verilerine erişim için)
            rov_id: ROV ID (lidar verilerini almak için)
        """
        self.filo_ref = filo_ref
        self.rov_id = rov_id
    
    def manevra_hesapla(self, gat_kodu, hedef_vektoru):
        """
        GAT koduna göre nihai hareket vektörünü ve hız çarpanını döndürür.
        
        Args:
            gat_kodu: GAT tahmin kodu (0=OK, 1=ENGEL, 2=CARPISMA, 3=KOPUK, 4=UZAK)
            hedef_vektoru: Hedefe doğru hareket vektörü (Sim formatında)
        
        Returns:
            tuple: (final_vektor, hiz_carpani, manevra_adi)
        """
        # Varsayılan değerler (Normal Seyir - Kod 0)
        final_vektor = hedef_vektoru
        hiz_carpani = 1.0
        manevra_adi = "NORMAL"

        # GAT Kodu 0: OK (Normal Seyir)
        if gat_kodu == 0:
            manevra_adi = "NORMAL"
            hiz_carpani = 1.0
            final_vektor = hedef_vektoru

        # GAT Kodu 1: ENGEL (Yakınlarda engel var, dikkatli ol)
        elif gat_kodu == 10:
            manevra_adi = "YUMUSAK_KACIS"
            hiz_carpani = 0.6  # Hızı %60'a düşür
            
            # Lidar verilerine bakıp boş tarafa yönelme
            if self.filo_ref and self.rov_id is not None:
                lidar_sag = self.filo_ref.get(self.rov_id, 'lidar', taraf=1) or 100
                lidar_sol = self.filo_ref.get(self.rov_id, 'lidar', taraf=2) or 100
            else:
                lidar_sag = 100
                lidar_sol = 100
            
            # Boş olan tarafa ek vektör ekle (sağ=+X, sol=-X)
            kacis_yonu = 1 if lidar_sol > lidar_sag else -1
            ek_vektor = Vec3(kacis_yonu * 1.5, 0, 0)  # Yana doğru it
            
            # Hedefle kaçışı harmanla (%40 hedef, %60 kaçış)
            final_vektor = (hedef_vektoru * 0.4) + (ek_vektor * 0.6)
            
            # Biraz yukarı çık (engelden uzaklaşmak için)
            # Sim formatında: Z=derinlik, yukarı çıkmak için Z'yi azalt (negatif)
            final_vektor.z -= 0.3
            final_vektor = final_vektor.normalized() if final_vektor.length() > 0.01 else hedef_vektoru

        # GAT Kodu 2: CARPISMA (Çok kritik, hemen uzaklaş)
        elif gat_kodu == 20:
            manevra_adi = "ACIL_GERI"
            hiz_carpani = 0.8  # Kaçarken hızlı olmalı ama kontrollü
            
            # Tam geri vektörü (Y ekseni tersi - Sim formatında Y=ileri, -Y=geri)
            geri_vektor = Vec3(0, -2.0, 0) 
            
            # Rastgele sağ/sol kırarak gerile (Sıkışmayı önler)
            kirma = Vec3(random.uniform(-1, 1), 0, 0)
            
            # Yukarı çık (engelden uzaklaşmak için)
            # Sim formatında: Z=derinlik, yukarı çıkmak için Z'yi azalt (negatif)
            yukari_vektor = Vec3(0, 0, -0.5)  # Sim formatında Z=derinlik, yukarı için negatif
            
            # Hedefi tamamen yok say, sadece kaç
            final_vektor = geri_vektor + kirma + yukari_vektor
            final_vektor = final_vektor.normalized() if final_vektor.length() > 0.01 else geri_vektor

        # GAT Kodu 4: UZAK (Liderden uzaklaşma, hızı artır)
        elif gat_kodu == 40:
            manevra_adi = "UZAK_HIZLI"
            hiz_carpani = 1.3  # Hızı %30 artır (lideri yakalamak için)
            
            # Hedefe doğru daha hızlı git
            final_vektor = hedef_vektoru
            
            # Eğer lider varsa ve çok uzaksa, direkt lider yönüne git
            if self.filo_ref and self.rov_id is not None:
                try:
                    # Lider ROV'u bul
                    lider_id = None
                    for i, gnc in enumerate(self.filo_ref.sistemler):
                        if hasattr(gnc, 'rov') and gnc.rov.role == 1:
                            lider_id = i
                            break
                    
                    if lider_id is not None and lider_id != self.rov_id:
                        lider_gps = self.filo_ref.get(lider_id, 'gps')
                        mevcut_gps = self.filo_ref.get(self.rov_id, 'gps')
                        
                        if lider_gps is not None and mevcut_gps is not None:
                            # GPS koordinatları Ursina formatında, Sim formatına dönüştür
                            lider_sim = Koordinator.ursina_to_sim(lider_gps[0], lider_gps[1], lider_gps[2])
                            mevcut_sim = Koordinator.ursina_to_sim(mevcut_gps[0], mevcut_gps[1], mevcut_gps[2])
                            
                            # Lider yönüne doğru vektör hesapla (Sim formatında)
                            lider_vektor_sim = Vec3(
                                lider_sim[0] - mevcut_sim[0],
                                lider_sim[1] - mevcut_sim[1],
                                lider_sim[2] - mevcut_sim[2]
                            )
                            
                            if lider_vektor_sim.length() > 0.01:
                                # Lider yönüne öncelik ver (%70 lider, %30 hedef)
                                final_vektor = (lider_vektor_sim.normalized() * 0.7) + (hedef_vektoru * 0.3)
                                final_vektor = final_vektor.normalized() if final_vektor.length() > 0.01 else hedef_vektoru
                except Exception:
                    # Hata durumunda normal hedef vektörünü kullan
                    pass

        # Bilinmeyen kod (varsayılan davranış)
        else:
            manevra_adi = "BILINMEYEN"
            hiz_carpani = 1.0
            final_vektor = hedef_vektoru

        return final_vektor, hiz_carpani, manevra_adi



