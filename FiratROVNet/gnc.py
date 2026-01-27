from typing_extensions import Self
import numpy as np
from ursina import Vec3, time, distance
from .config import cfg, GATLimitleri, SensorAyarlari, ModemAyarlari, HareketAyarlari, Formasyon
from .iletisim import AkustikModem
from .hull import HullManager
from helper.gnc_helper import FiloHelper, TemelGNCHelper
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

    def get(self, rov_id: int = None, veri_tipi: str = None, taraf: int = None):
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
        # ============================================================
        # GUARD CLAUSES - Erken Çıkışlar
        # ============================================================
        if rov_id is None and veri_tipi is None:
            return self._get_all_rovs_positions()
        
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            return None
        
        if rov_id is not None and (not isinstance(rov_id, int) or rov_id < 0):
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return None
        
        if rov_id is not None and rov_id >= len(self.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return None
        
            if rov_id is None:
                print(f"❌ [HATA] ROV ID belirtilmedi!")
                return None
        
        try:
            
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
    
    def _formasyon_sec_impl(self, margin: float = 30, is_3d: bool = False, offset: float = 20.0):
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
            # ============================================================
            # FORMATION LOGIC - Hazırlık
            # ============================================================
            self._formasyon_hedefleri.clear()
            
            # Güvenlik hull oluştur
            yasakli_noktalar = self._prepare_forbidden_points()
            guvenlik_hull_dict = self.yeni_hull(
                yasakli_noktalar=yasakli_noktalar,
                offset=offset,
                alpha=2.0,
                buffer_radius=10.0,
                channel_width=10.0
            )

            hull = guvenlik_hull_dict.get("hull")
            hull_merkez = guvenlik_hull_dict.get("center")

            if hull is None or hull_merkez is None:
                return None

            hull_merkez = self._normalize_hull_center(hull_merkez)
            
            # Lider bilgilerini al
            lider_rov_id, lider_gps = self._find_leader_info()
            if lider_rov_id is None:
                return None

            if lider_gps is None:
                lider_gps = hull_merkez

            # Arama parametrelerini hazırla
            min_aralik = margin * 0.2
            baslangic_aralik = margin * 0.6
            adim = 1.0
            yaw_acilari = [0, 90, 180, 270]

            # Arama noktalarını oluştur
            arama_noktalari = self._generate_search_points(lider_gps, hull_merkez)

            # ============================================================
            # FORMATION LOGIC - Hiyerarşik Arama
            # ============================================================
            for nokta_adi, merkez_koordinat in arama_noktalari:
                for deneme_yaw in yaw_acilari:
                    denenecek_formasyon_idleri = self._get_formation_ids_to_try()
                    
                    for i in denenecek_formasyon_idleri:
                        aralik = baslangic_aralik

                        while aralik >= min_aralik:
                            if self._try_formation_fit(i, aralik, is_3d, merkez_koordinat, 
                                                      deneme_yaw, hull, lider_rov_id, nokta_adi):
                                # Formasyon bulundu, pool'dan bu ID'yi çıkar
                                if i in self._formasyon_id_pool:
                                    self._formasyon_id_pool.remove(i)
                                
                                return (i, aralik, deneme_yaw, merkez_koordinat)

                            aralik -= adim

            return None

        except Exception as e:
            import traceback
            traceback.print_exc()
            return None

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
        if hull_output is None:
            hull_output = self.yeni_hull(self.ada_cevre())
            
        points = hull_output.get('points')
        
        if points is None or len(points) < 2:
            print("⚠️ [SAMPLED] Örnekleme için yetersiz nokta!")
            return None

        # 1. Noktaların kapalı bir döngü olduğundan emin ol (ilk ve son nokta aynı olmalı)
        if not np.allclose(points[0], points[-1]):
            points = np.vstack([points, points[0]])

        # 2. Her segmentin uzunluğunu hesapla
        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt((diffs**2).sum(axis=1))
        
        # 3. Kümülatif (birikimli) mesafeyi hesapla (Çevre üzerindeki konumlar)
        cumulative_dist = np.concatenate(([0], np.cumsum(segment_lengths)))
        total_perimeter = cumulative_dist[-1]
        
        if total_perimeter == 0:
            return np.tile(points[0], (sample_count, 1))

        # 4. Sabit adımlarla hedef mesafeleri belirle (adım = çevre / örneklem_sayısı)
        # endpoint=False yapıyoruz çünkü kapalı döngüde son nokta ilk noktanın aynısıdır.
        target_dists = np.linspace(0, total_perimeter, sample_count, endpoint=False)

        # 5. X ve Y koordinatları için ayrı ayrı doğrusal interpolasyon yap
        new_x = np.interp(target_dists, cumulative_dist, points[:, 0])
        new_y = np.interp(target_dists, cumulative_dist, points[:, 1])

        # 6. Noktaları birleştir (100, 2)
        sampled_points = np.column_stack((new_x, new_y))
        
        # print(f"✅ [SAMPLED] {len(points)} noktadan {sample_count} sabit örnek üretildi. Çevre: {total_perimeter:.2f}m")
        
        return sampled_points

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
            from . import senaryo # Fonksiyon içinde import ederek dairesel bağımlılığı önleriz
            import random
            import numpy as np

            try:
                # 1. Senaryo Parametrelerini Hazırla
                n_rov_secenekleri = [4, 6, 8]
                secilen_n = random.choice(n_rov_secenekleri)
                n_engels = random.randint(12, 22)
                
                # 2. Senaryoyu Üret (Headless/Hızlı)
                # Not: senaryo.uret içindeki Safe Position algoritması adaların çakışmamasını sağlar.
                senaryo.uret(n_rovs=secilen_n, n_engels=n_engels, havuz_genisligi=200)
                
                # Senaryo sonrası oluşan filo referansına eriş
                aktif_filo = senaryo.filo 
                if not aktif_filo:
                    return None

                # 3. Lider Bilgilerini Al (ID: 0 her zaman lider kabul edilir)
                lider_id = 0
                # senaryo.get simülasyon formatında (x, y, z) döner
                lider_gps = senaryo.get(lider_id, "gps") 
                lider_yaw = senaryo.get(lider_id, "yaw")
                
                if lider_gps is None: lider_gps = np.array([400.0, 400.0, 400.0])
                if lider_yaw is None: lider_yaw = 0.0

                # 4. Tüm ROV Koordinatlarını Topla (Sabit 8 slot)
                rov_filo_gps = []
                for i in range(8):
                    if i < secilen_n:
                        pos = senaryo.get(i, "gps")
                        rov_filo_gps.append(pos if pos is not None else [400.0, 400.0, 400.0])
                    else:
                        # Olmayan ROV'lar için absürt değer (Maskeleme)
                        rov_filo_gps.append([400.0, 400.0, 400.0])
                
                rov_filo_gps = np.array(rov_filo_gps) # Shape: (8, 3)

                # 5. Convex Hull Verilerini Hazırla (Sabit 100 Nokta)
                hull_merkez = np.array([400.0, 400.0])
                hull_noktalar = np.full((100, 2), 400.0)

                # Ada çevrelerinden Hull hesapla
                ada_cevreleri = aktif_filo.ada_cevre(offset=15.0)
                hull_dict = aktif_filo.yeni_hull(ada_cevreleri)

                if hull_dict and hull_dict.get('points') is not None:
                    # Merkez koordinatları (x, y)
                    center = hull_dict.get('center')
                    if center:
                        hull_merkez = np.array([center[0], center[1]])
                    
                    # 100 Nokta Örneklemesi (Sınıf içindeki metodunuzu çağırır)
                    samples = self.get_100_samples(hull_dict, 100)
                    if samples is not None:
                        hull_noktalar = samples

                # 6. Temizlik (Bir sonraki iterasyon için)
                senaryo.temizle()

                # 7. RL Veri Paketini Döndür
                return {
                    "n_rovs": secilen_n,
                    "lider_pozisyon": lider_gps,   # (3,)
                    "lider_yaw": lider_yaw,        # float
                    "rov_filo_gps": rov_filo_gps,  # (8, 3)
                    "hull_merkez": hull_merkez,    # (2,)
                    "hull_noktalar": hull_noktalar # (100, 2)
                }

            except Exception as e:
                print(f"❌ [RL_DATA] Veri üretimi sırasında hata: {e}")
                import traceback
                traceback.print_exc()
                senaryo.temizle()
                return None
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
            from . import senaryo # Fonksiyon içinde import ederek dairesel bağımlılığı önleriz
            import random
            import numpy as np

            try:
                # 1. Senaryo Parametrelerini Hazırla
                n_rov_secenekleri = [4, 6, 8]
                secilen_n = random.choice(n_rov_secenekleri)
                n_engels = random.randint(12, 22)
                
                # 2. Senaryoyu Üret (Headless/Hızlı)
                # Not: senaryo.uret içindeki Safe Position algoritması adaların çakışmamasını sağlar.
                senaryo.uret(n_rovs=secilen_n, n_engels=n_engels, havuz_genisligi=200)
                
                # Senaryo sonrası oluşan filo referansına eriş
                aktif_filo = senaryo.filo 
                if not aktif_filo:
                    return None

                # 3. Lider Bilgilerini Al (ID: 0 her zaman lider kabul edilir)
                lider_id = 0
                # senaryo.get simülasyon formatında (x, y, z) döner
                lider_gps = senaryo.get(lider_id, "gps") 
                lider_yaw = senaryo.get(lider_id, "yaw")
                
                if lider_gps is None: lider_gps = np.array([400.0, 400.0, 400.0])
                if lider_yaw is None: lider_yaw = 0.0

                # 4. Tüm ROV Koordinatlarını Topla (Sabit 8 slot)
                rov_filo_gps = []
                for i in range(8):
                    if i < secilen_n:
                        pos = senaryo.get(i, "gps")
                        rov_filo_gps.append(pos if pos is not None else [400.0, 400.0, 400.0])
                    else:
                        # Olmayan ROV'lar için absürt değer (Maskeleme)
                        rov_filo_gps.append([400.0, 400.0, 400.0])
                
                rov_filo_gps = np.array(rov_filo_gps) # Shape: (8, 3)

                # 5. Convex Hull Verilerini Hazırla (Sabit 100 Nokta)
                hull_merkez = np.array([400.0, 400.0])
                hull_noktalar = np.full((100, 2), 400.0)

                # Ada çevrelerinden Hull hesapla
                ada_cevreleri = aktif_filo.ada_cevre(offset=15.0)
                hull_dict = aktif_filo.yeni_hull(ada_cevreleri)

                if hull_dict and hull_dict.get('points') is not None:
                    # Merkez koordinatları (x, y)
                    center = hull_dict.get('center')
                    if center:
                        hull_merkez = np.array([center[0], center[1]])
                    
                    # 100 Nokta Örneklemesi (Sınıf içindeki metodunuzu çağırır)
                    samples = self.get_100_samples(hull_dict, 100)
                    if samples is not None:
                        hull_noktalar = samples

                # 6. Temizlik (Bir sonraki iterasyon için)
                senaryo.temizle()

                # 7. RL Veri Paketini Döndür
                return {
                    "n_rovs": secilen_n,
                    "lider_pozisyon": lider_gps,   # (3,)
                    "lider_yaw": lider_yaw,        # float
                    "rov_filo_gps": rov_filo_gps,  # (8, 3)
                    "hull_merkez": hull_merkez,    # (2,)
                    "hull_noktalar": hull_noktalar # (100, 2)
                }

            except Exception as e:
                print(f"❌ [RL_DATA] Veri üretimi sırasında hata: {e}")
                import traceback
                traceback.print_exc()
                senaryo.temizle()
                return None
    
    def _prepare_forbidden_points(self) -> list:
        """Ada çevre noktalarını yasaklı nokta listesine dönüştürür."""
        return self.helper.prepare_forbidden_points()
    
    def _normalize_hull_center(self, hull_merkez) -> tuple:
        """Hull merkezini Sim formatına dönüştürür (z=0 yapar)."""
        return self.helper.normalize_hull_center(hull_merkez)
    
    def _find_leader_info(self) -> tuple:
        """Lider ROV ID ve GPS koordinatını bulur."""
        return self.helper.find_leader_info()
    
    def _generate_search_points(self, lider_gps: tuple, hull_merkez: tuple) -> list:
        """Lider GPS'ten hull merkezine kadar ara noktalar oluşturur."""
        return self.helper.generate_search_points(lider_gps, hull_merkez)
    
    def _get_formation_ids_to_try(self) -> list:
        """Denenecek formasyon ID'lerini pool'dan alır."""
        return self.helper.get_formation_ids_to_try()
    
    def _try_formation_fit(self, formasyon_id: int, aralik: float, is_3d: bool, 
                          merkez_koordinat: tuple, deneme_yaw: float, hull, 
                          lider_rov_id: int, nokta_adi: str) -> bool:
        """Formasyonun geçerli olup olmadığını kontrol eder ve uygular."""
        return self.helper.try_formation_fit(formasyon_id, aralik, is_3d,
                                            merkez_koordinat, deneme_yaw, hull,
                                            lider_rov_id, nokta_adi)

    

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
        return self.helper.ada_cevre(offset)
    
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

    def git(self, rov_id: int, x, y: float = None, z: float = None, ai: bool = True) -> None:
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
        # ============================================================
        # ÇOKLU NOKTA MODU
        # ============================================================
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
        
        # ============================================================
        # GUARD CLAUSES - Erken Çıkışlar
        # ============================================================
        if y_val is None:
            print(f"❌ [FİLO] Y koordinatı gerekli! (x liste değilse)")
            return
        
        # ============================================================
        # THREAD MANAGEMENT
        # ============================================================
        if not self._is_main_thread():
            try:
                from ursina import invoke
                invoke(self._git_impl, rov_id, x_val, y_val, z_val, ai)
                return
            except (ImportError, AttributeError):
                self._command_queue.put(('git', (rov_id, x_val, y_val, z_val, ai), {}))
                return
        
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
    
    def _git_impl(self, rov_id: int, x: float, y: float, z: float = None, ai: bool = True) -> None:
        """git() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır)."""
        # ============================================================
        # GUARD CLAUSES - Erken Çıkışlar
        # ============================================================
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            print(f"   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return
        
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
        # ============================================================
        # GUARD CLAUSES - Erken Çıkışlar
        # ============================================================
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            print(f"   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return
        
        if not isinstance(rov_id, int) or rov_id < 0:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return
        
        if rov_id >= len(self.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            print(f"   💡 Çözüm: filo.ekle() ile daha fazla GNC sistemi ekleyin")
            return
        
        gecerli_yonler = ['ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur', 'yaw']
        if yon not in gecerli_yonler:
            print(f"❌ [HATA] Geçersiz hareket yönü: '{yon}'")
            print(f"   Geçerli yönler: {', '.join(gecerli_yonler)}")
            return
        
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
        self.helper = TemelGNCHelper(rov_entity, filo_ref)
        
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
        if self.manuel_kontrol:
            return

        if self.hedef is None:
            if self.rov.velocity.length() > 1: 
                self.rov.velocity *= 0.4
            return
        
        # Koordinat Dönüşümü
        current_sim_pos = Vec3(*Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z))
        fark = self.hedef - current_sim_pos
        mesafe = fark.length()

        # Varış Kontrolü
        if mesafe <= self.HEDEF_TOLERANSI:
            self._hedefe_varis_islemleri(fark)
            return

        # Hareket Mantığı - Temel hedef vektörü
        hiz_carpani = self.helper.hiz_hesapla(mesafe)
        hareket_vektoru = fark / mesafe if mesafe > 0.01 else Vec3(0, 0, 0)
        
        # GAT Manevra Yöneticisi - GAT koduna göre manevra hesapla (TÜM kodlar için)
        # GAT kodu None ise 0 (OK) olarak kabul et
        if gat_kodu is None:
            gat_kodu = 0
        
        if self.gat_manevra:
            # ROV ID'yi dinamik olarak bul ve güncelle
            if self.filo_ref and self.gat_manevra.rov_id is None:
                try:
                    self.gat_manevra.rov_id = self.filo_ref.sistemler.index(self)
                except (ValueError, AttributeError):
                    pass
            
            # Tüm GAT kodları için manevra hesapla (ROV ID None olsa bile kod 0, 1, 2, 3 için çalışır)
            # Kod 4 için lider takibi gerektiğinden ROV ID kontrolü yapılır
            if gat_kodu != 4 or self.gat_manevra.rov_id is not None:
                final_vektor, gat_hiz_carpani, manevra_adi = self.gat_manevra.manevra_hesapla(gat_kodu, hareket_vektoru)
                hareket_vektoru = final_vektor
                hiz_carpani *= gat_hiz_carpani  # GAT hız çarpanını temel hız çarpanıyla çarp
        
        # Yaw Ayarı (hedefe doğru, GAT manevrasından sonra)
        self.helper.yaw_ayarla(hareket_vektoru, ani=False)
        
        # Motor Sürüşü
        guc = 0.4 * hiz_carpani
        self.helper.vektor_to_motor_sim(hareket_vektoru, guc=guc)

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



