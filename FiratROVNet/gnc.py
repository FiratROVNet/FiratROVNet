import builtins

from typing_extensions import Self
import numpy as np
from ursina import Vec3, time, distance
import ursina # base'e ursina.base olarak erişmek için ekleyelim
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
        """Sim (X:Sağ, Y:İleri, Z:Derinlik) -> Ursina (X, Y:Yukarı, Z:İleri)."""
        from FiratROVNet.kutuphane.helper.simulasyon_helper import sim_to_ursina as _stou
        return _stou(sim_x, sim_y, sim_z)

    @staticmethod
    def ursina_to_sim(u_x, u_y, u_z):
        """Ursina (X, Y:Yukarı, Z:İleri) -> Sim (X, Y:İleri, Z:Derinlik)."""
        from FiratROVNet.kutuphane.helper.simulasyon_helper import ursina_to_sim as _utot
        return _utot(u_x, u_y, u_z)


# ==========================================
# 0.5 DEBUG (APF / GNC Yardımcı Fonksiyonları)
# ==========================================
class Debug:
    """
    APF ve GNC yardımcı fonksiyonlarına debug erişimi.
    filo.helper üzerindeki apf, engel_vektor, rov_vektor, hedef_vektor, vektor,
    vektor_normalize, apf_temizle fonksiyonlarını sarmalar.

    Kullanım:
        debug = Debug(filo)
        debug.list()           # Tüm fonksiyon isimlerini listeler
        debug.apf()            # Kullanım bilgisi döner (parametresiz)
        debug.apf(0)           # ROV-0 için APF hesaplar
    """
    _FONKSIYONLAR = [
        'apf', 'engel_vektor', 'rov_vektor', 'hedef_vektor',
        'vektor', 'vektor_normalize', 'apf_temizle'
    ]

    def __init__(self, filo_ref):
        self._filo = filo_ref
        self._helper = getattr(filo_ref, 'helper', None)

    def list(self):
        """Tüm debug fonksiyon isimlerini döndürür ve yazdırır."""
        names = self._FONKSIYONLAR
        print("🔧 [DEBUG] Mevcut fonksiyonlar:", ", ".join(names))
        return names

    def _usage(self, name: str):
        """Fonksiyon kullanım bilgisini döndürür."""
        method = getattr(self, name, None)
        doc = getattr(method, '__doc__', 'Bilgi yok') if method else 'Bilgi yok'
        print(doc)
        return doc

    def apf(self, rov_id=None):
        """
        APF (Artificial Potential Field) hesaplaması.
        Parametreler: rov_id (int)
        Örnek: debug.apf(0) -> dict; filo.apf_birim_vektor(0) -> (ux, uz) sadece birim vektör
        """
        if rov_id is None:
            return self._usage('apf')
        if self._helper is None:
            print("❌ [DEBUG] filo.helper bulunamadı.")
            return None
        return self._helper.apf(rov_id)

    def engel_vektor(self, rov_id=None, menzil=None):
        """
        ROV'un engellere olan vektör bilgilerini döndürür.
        Parametreler: rov_id (int), menzil=10.0
        Örnek: debug.engel_vektor(0) veya debug.engel_vektor(0, menzil=30)
        """
        if rov_id is None:
            return self._usage('engel_vektor')
        if self._helper is None:
            print("❌ [DEBUG] filo.helper bulunamadı.")
            return []
        return self._helper.engel_vektor(rov_id, menzil)

    def rov_vektor(self, rov_id=None, menzil=None):
        """
        ROV'un diğer ROV'lara olan vektör bilgilerini döndürür.
        Parametreler: rov_id (int), menzil=10.0
        Örnek: debug.rov_vektor(0) veya debug.rov_vektor(1, menzil=25)
        """
        if rov_id is None:
            return self._usage('rov_vektor')
        if self._helper is None:
            print("❌ [DEBUG] filo.helper bulunamadı.")
            return []
        return self._helper.rov_vektor(rov_id, menzil)

    def hedef_vektor(self, rov_id=None, menzil=None):
        """
        ROV'un hedefine olan vektör bilgisini döndürür.
        Parametreler: rov_id (int), menzil=10.0
        Örnek: debug.hedef_vektor(0) veya debug.hedef_vektor(0, menzil=30)
        """
        if rov_id is None:
            return self._usage('hedef_vektor')
        if self._helper is None:
            print("❌ [DEBUG] filo.helper bulunamadı.")
            return None
        return self._helper.hedef_vektor(rov_id, menzil)

    def vektor(self, ilk=None, ikinci=None, **kwargs):
        """
        Minimap üzerinde vektör çizer. 
        Tüm parametreler (rov_id_ilk, rov_id_ikinci, baslangic_noktasi, bitis_noktasi, 
        vektor, renk, uzunluk, reverse, debug, ciz, is_3d) **kwargs üzerinden alınır.
        """
        # 1. Kontrol edilecek anahtar kelimeler (Kullanım yardımı için)
        print("kwargs:", kwargs)
        check_keys = [
            'rov_id_ilk', 'rov_id_ikinci', 'baslangic_noktasi', 
            'bitis_noktasi', 'vektor'
        ]

        # Eğer hiçbir ana parametre gelmemişse yardım mesajını göster
        if (ilk is None and ikinci is None and not any(k in kwargs for k in check_keys)):
            return self._usage('vektor')

        if self._helper is None:
            print("❌ [DEBUG] filo.helper bulunamadı.")
            return None

        # 2. Varsayılan Değerleri Belirle (Eğer kwargs içinde yoksa)
        # Bu sayede helper'a her zaman tam veri gider
        params = {
            'renk': 'm',
            'uzunluk': 20,
            'reverse': False,
            'debug': False,
            'ciz': True,
        }
        
        # Kullanıcının gönderdiği değerleri varsayılanların üzerine yaz
        params.update(kwargs)

        # 3. Tüm paketi Helper'a pasla
        # ilk ve ikinci positional olarak, diğerleri paketlenmiş (unpack) olarak gider
        return self._helper.vektor(
            ilk=ilk, 
            ikinci=ikinci, 
            **params
        )

    def vektor_normalize(self, ux=None, uz=None, uy=None, max_mag=1.0, vektor=None):
        """
        Vektörü normalize eder ve max_mag ile sınırlar.
        Parametreler: ux, uz (2D) veya ux, uy, uz (3D), max_mag=1.0, vektor=(ux,uz) veya (ux,uy,uz)
        Örnek: debug.vektor_normalize(3, 4) veya debug.vektor_normalize(vektor=(3, 4))
        """
        if ux is None and uz is None and vektor is None:
            return self._usage('vektor_normalize')
        if self._helper is None:
            print("❌ [DEBUG] filo.helper bulunamadı.")
            return None
        return self._helper.vektor_normalize(ux, uz, uy, max_mag, vektor)

    def apf_temizle(self, rov_id=None):
        """
        APF vektörlerini temizler. rov_id verilirse sadece o ROV'un vektörlerini siler;
        boş bırakılırsa hepsini temizler. Örnek: debug.apf_temizle() veya debug.apf_temizle(0)
        """
        if self._helper is None:
            print("❌ [DEBUG] filo.helper bulunamadı.")
            return
        self._helper.apf_temizle(rov_id=rov_id)


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
        self.hull_manager = HullManager()  # Convex Hull yönetimi
        self._command_queue = queue.Queue()  # Thread-safe komut kuyruğu
        self._main_thread_id = threading.get_ident()  # Ana thread ID'si
        # Formasyon ID shuffle mekanizması
        self._formasyon_id_pool = []  # Shuffle edilmiş formasyon ID'leri
        self._formasyon_id_pool_olustur()  # İlk pool'u oluştur
        # Formasyon parametreleri (aktif formasyon takibi için)
        self.aktif_formasyon = None  # {'id': str/int, 'aralik': float, 'is_3d': bool}
        # Formasyon hedef takibi (ROV ID -> {'pozisyon': (x, y, z), 'hedef_yaw': float})
        self._formasyon_hedefleri = {}  # Takipçi ROV'ların formasyon hedefleri ve hedef yaw açıları
        self._formasyon_yaw_senkronizasyon_mesafesi = 5.0  # Yaw senkronizasyonu için mesafe eşiği (metre)
        self._maksimum_yaw_donme_hizi = 30.0  # Maksimum yaw dönme hızı (derece/saniye) - Formasyon için (2x yavaşlatıldı)
        # git() hedef takibi (ROV ID -> hedef_yaw açısı)
        self._git_hedef_yaw = {}  # git() ile gönderilen ROV'ların hedef yaw açıları (kademeli dönüş için)
        self._git_maksimum_yaw_donme_hizi = 45.0  # git() için maksimum yaw dönme hızı (derece/saniye) (2x yavaşlatıldı)
        
        # Çoklu nokta takibi (ROV ID -> nokta listesi ve mevcut indeks)
        self._git_nokta_listesi = {}  # {rov_id: [[x1, y1], [x2, y2], ...], ...}
        self._git_mevcut_nokta_indeksi = {}  # {rov_id: 0, ...} - Hangi noktaya gidiyor
        self._git_isaret = {}  # {rov_id: bool} - git_path(isaret=True) ile bir sonraki nokta minimapte gösterilir
        self._git_hedef_mesafe_toleransi = 2.0  # Hedefe ulaşma toleransı (metre)
        # Her ROV için ayrı hedef takibi (ROV ID -> (x, y, z))
        self._rov_hedefleri = {}  # {rov_id: (x, y, z), ...} - Her ROV'un hedef koordinatları
        
        # engel_bul(debug=True) için görsel debug noktaları (kırmızı küreler)
        self._debug_noktalari = []
        # engel_bul konsol thread'den çağrıldığında uyarıyı sadece bir kez bas
        self._engel_bul_console_warned = False
        
        # Helper instance for complex calculations
        self.helper = FiloHelper(self)
        self.aktif_kameralar = {}
    
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
            # git/git_path gibi kritik komutlar hemen işlensin (bazen ilk çağrı atlanabiliyordu)
            max_commands = 5
            processed = 0
            while not self._command_queue.empty() and processed < max_commands:
                cmd_type, args, kwargs = self._command_queue.get_nowait()
                if cmd_type == 'git':
                    self.helper._git_impl(*args, **kwargs)
                elif cmd_type == 'git_path':
                    self.helper._git_path_impl(*args, **kwargs)
                elif cmd_type == 'hull':
                    self._guvenlik_hull_olustur_impl(*args, **kwargs)
                elif cmd_type == 'formasyon_sec':
                    # formasyon selection live implementation lives in helper
                    # delegate to helper to avoid AttributeError when running from separate thread
                    self.helper._formasyon_sec_impl(*args, **kwargs)
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
        gnc_objesi.filo_ref = self
        if hasattr(gnc_objesi, 'helper') and gnc_objesi.helper is not None:
            gnc_objesi.helper.filo_ref = self
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
        #print(f"🔄 [FİLO] GNC sistemleri güncelleniyor... (tahminler: {tahminler})")
        
        # Tüm GNC sistemlerini güncelle (doğrudan helper.guncelle çağrısı)
        for i, gnc in enumerate(self.sistemler):
            if hasattr(gnc, 'helper') and gnc.helper is not None:
                # GAT kodu varsa kullan, yoksa None (tüm ROV'lar güncellenir)
                gat_kodu = tahminler[i] if i < len(tahminler) else None
                gnc.helper.guncelle(gat_kodu=gat_kodu)
        

        # guncelle_hepsi metodunun içindeki o satırı şununla değiştir:
        if self.ortam_ref and hasattr(self.ortam_ref, 'minimap') and self.ortam_ref.minimap:
            try:
                self.ortam_ref.minimap.gorsel_guncelle()
            except Exception as e:
                if self.verbose: print(f"⚠️ Minimap güncellenemedi: {e}")

        

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

    def engel_bul(self, rov_id: int, menzil: float = None, debug: bool = False) -> list:
        """
        Belirtilen ROV için çevresel tarama yapar (sonar/lidar benzeri).
        İleri, sağ, sol, sağ-çapraz, sol-çapraz, yukarı, aşağı yönlerinde raycast atar;
        tespit edilen engellerin dünya koordinatlarını döndürür.
        
        Args:
            rov_id (int): ROV ID.
            menzil (float): Tarama menzili (metre, varsayılan GATLimitleri.ENGEL).
            debug (bool): True ise çarpışma noktalarında kırmızı küre gösterir.
        
        Returns:
            list: [{'koordinat': Vec3(x,y,z), 'mesafe': float, 'vektor': Vec3}, ...]; engel yoksa [].
        """
        return self.helper.engel_bul(rov_id=rov_id, menzil=menzil, debug=debug)

    def yakinlastir(self, rov_id1: int, rov_id2: int, mesafe: float):
        """
        rov_id1'i rov_id2'ye yatay düzlemde (X,Z) mesafe kadar yaklaştırır.
        Sadece rov_id1 hareket eder; hedef konum git() ile atanır (ROV o noktaya gider).
        Derinlik (Y) rov_id1 için korunur; rov_id2 sabit kalır.

        Args:
            rov_id1 (int): Hareket edecek ROV ID.
            rov_id2 (int): Hedef ROV ID (konumu hesaplanır, kendisi hareket etmez).
            mesafe (float): Yaklaşma miktarı (metre). Aralarındaki mesafe bu kadar azalır.

        Returns:
            bool: Başarılı ise True, geçersiz ID veya aynı konumda ise False.
        """
        return self.helper.yakinlastir(rov_id1=rov_id1, rov_id2=rov_id2, mesafe=mesafe)

    def vektor(self, ilk=None, ikinci=None,
               rov_id_ilk=None, rov_id_ikinci=None,
               baslangic_noktasi=None, bitis_noktasi=None, vektor=None,
               renk='m', uzunluk=10, reverse=False, debug=False, ciz=True):
        """
        Minimap üzerinde vektör çizer. Keyword: rov_id_ilk, rov_id_ikinci, baslangic_noktasi, bitis_noktasi, vektor=().
        Örnek: filo.vektor(rov_id_ilk=2, rov_id_ikinci=5)
               filo.vektor(rov_id_ilk=5, vektor=(0.76,-0.65), uzunluk=20)  # ROV-5'ten birim vektör yönünde
               filo.vektor(0, 1)  # Eski API
        """
        return self.helper.vektor(
            ilk=ilk, ikinci=ikinci,
            rov_id_ilk=rov_id_ilk, rov_id_ikinci=rov_id_ikinci,
            baslangic_noktasi=baslangic_noktasi, bitis_noktasi=bitis_noktasi, vektor=vektor,
            renk=renk, uzunluk=uzunluk, reverse=reverse, debug=debug, ciz=ciz
        )

    def apf(self, rov_id: int):
        """
        ROV'un mevcut konumundan hedefine doğru birim vektör hesaplar.

        Args:
            rov_id (int): ROV ID (0, 1, 2, ...)

        Returns:
            dict: rov_konum, engeller, rovlar, hedef, toplam_vektor (toplam_vektor['birim_vektor'] = (ux, uz))
        """
        return self.helper.apf(rov_id=rov_id)

    def apf_birim_vektor(self, rov_id: int):
        """
        Sadece birim vektör döndürür: (ux, uz) veya None.
        Örnek: birim_vektor = filo.apf_birim_vektor(0)
        """
        return self.helper.apf_birim_vektor(rov_id)

    def apf_temizle(self, rov_id=None) -> None:
        """APF vektörlerini temizler. rov_id verilirse sadece o ROV'un vektörlerini siler; boş bırakılırsa hepsini temizler."""
        self.helper.apf_temizle(rov_id=rov_id)

    def apf_guncelle_tum(self) -> None:
        """Tüm ROV'lar için APF vektörlerini günceller (engel/rov/hedef okları). filo.apf(0), apf(1) ile eklenenler güncellenir."""
        self.helper.apf_guncelle_tum()

    def hedef_vektor(self, rov_id: int, menzil: float = None):
        """
        ROV'un hedefine olan vektör bilgisini döndürür (çizim yapılmaz).
        
        Args:
            rov_id (int): ROV ID (0, 1, 2, ...)
            menzil (float): Yatay düzlemde (X,Z) menzil (metre). Varsayılan GATLimitleri.ENGEL.
        
        Returns:
            dict | None: Vektör bilgisi (baslangic, bitis, birim_vektor, uzunluk) veya None.
                Hedef yoksa veya ROV konumu bulunamazsa None döner.
        
        Örnek:
            vektor_bilgi = filo.hedef_vektor(0)  # ROV-0'un hedefine olan vektör bilgisi
        """
        return self.helper.hedef_vektor(rov_id=rov_id, menzil=menzil)

    def rov_vektor(self, rov_id: int, menzil: float = None):
        """
        ROV'un diğer ROV'lara olan vektör bilgilerini liste olarak döndürür (çizim yapılmaz).
        
        Args:
            rov_id (int): ROV ID (0, 1, 2, ...)
            menzil (float): Yatay düzlemde (X,Z) menzil (metre). Varsayılan GATLimitleri.CARPISMA.
        
        Returns:
            list: [{'rov_id': int, 'koordinat': (x, z), 'vektor_bilgi': {...}, 'mesafe': float}, ...]
                Her diğer ROV için vektör bilgisi içeren dict listesi.
        
        Örnek:
            rov_vektorler = filo.rov_vektor(0)  # ROV-0'un diğer ROV'lara olan vektörleri
        """
        return self.helper.rov_vektor(rov_id=rov_id, menzil=menzil)

    def engel_vektor(self, rov_id: int, menzil: float = None):
        """
        ROV'un engellere olan vektör bilgilerini liste olarak döndürür (çizim yapılmaz).
        
        Args:
            rov_id (int): ROV ID (0, 1, 2, ...)
            menzil (float): Yatay düzlemde (X,Z) menzil (metre). Varsayılan GATLimitleri.ENGEL.
        
        Returns:
            list: [{'koordinat': (x, z), 'vektor_bilgi': {...}, 'mesafe': float, 'radius': float}, ...]
                Her engel için vektör bilgisi ve yarıçap (metre) içeren dict listesi.
        
        Örnek:
            engel_vektorler = filo.engel_vektor(0)  # ROV-0'un engellere olan vektörleri
        """
        return self.helper.engel_vektor(rov_id=rov_id, menzil=menzil)

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

    def formasyon(self, formasyon_id="LINE", aralik=None, is_3d=False, lider_koordinat=None, dinamik=False):
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
            dinamik (bool): Formasyonun lideri dinamik olarak takip edip etmeyeceği (varsayılan: True)
                - True: Formasyon liderin hareketine göre sürekli güncellenir
                - False: Formasyon o anki konuma göre bir kez hesaplanır ve sabit kalır
        
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
        return self.helper.formasyon(formasyon_id=formasyon_id, aralik=aralik, is_3d=is_3d, lider_koordinat=lider_koordinat, dinamik=dinamik)
    

    def formasyon_sec_yedek(self, margin=None, is_3d=False, offset=None, dinamik=False,tekrar=10):
        """
        Convex hull kullanarak en uygun formasyonu seçer (Thread-safe).

        KESİN KURALLAR:
        - Güvenlik hull (sanal + gerçek engeller) SADECE 1 KEZ hesaplanır (sabit hull)
        - Margin = formasyon aralığı (ROV'lar arası mesafe, varsayılan 10m)
        - Hull içinde kalma kontrolü offset ile yapılır
        - İlk geçerli formasyon bulunduğunda DERHAL döner
        - Takipçi ROV'lar hedef pozisyonlarına yaklaştığında (yaw_senkronizasyon_mesafesi metre), 
          liderin yaw açısına göre otomatik olarak yönlenirler
        - Yaw dönüşü kademeli olarak yapılır (maksimum_yaw_donme_hizi derece/saniye)

        Args:
            margin (float): Formasyon aralığı (ROV'lar arası mesafe, varsayılan: 10)
            is_3d (bool): 3D formasyon modu (varsayılan: False)
            offset (float): ROV hull genişletme mesafesi (varsayılan: 10.0)
            harita (bool): Harita görüntülemeyi aç/kapat (varsayılan: False)
            yaw_senkronizasyon_mesafesi (float): Takipçi ROV'ların hedefe yaklaştığında liderin yaw açısına 
                göre yönlenmesi için mesafe eşiği (metre, varsayılan: 5.0)
            maksimum_yaw_donme_hizi (float): Maksimum yaw dönme hızı (derece/saniye, varsayılan: 60.0)
            dinamik (bool): Formasyonun lideri dinamik olarak takip edip etmeyeceği (varsayılan: True)

        Returns:
            tuple | None: Seçilen formasyon bilgileri (formasyon_id, aralik, yaw, koordinat) veya None (uygun formasyon bulunamazsa)
                - formasyon_id (int): Formasyon tipi ID'si (0-19)
                - aralik (float): ROV'lar arası mesafe (metre)
                - yaw (float): Liderin yaw açısı (derece)
                - koordinat (tuple): Seçilen formasyon koordinatı (x, y, z) - Lider pozisyonu
        """

        baslangic_zamnai = time.time()
        for i in range(tekrar):
            degerler = self.helper.formasyon_sec(
                margin=margin,
                is_3d=is_3d,
                offset=offset,
                dinamik=dinamik
            )

        bitis_zamani = time.time()
        gecen_sure = bitis_zamani - baslangic_zamnai
        print(f"⏱️ formasyon_sec süresi: {gecen_sure:.4f} saniye")

        
        return degerler
    


    def formasyon_sec(self, margin=None, is_3d=False, offset=None, dinamik=False):
            """
            Formasyon seçimini tek bir ayrı thread üzerinde çalıştırır ve sonucu döndürür.
            """
            import concurrent.futures

            # Hesaplama parametrelerini hazırla
            kwargs = {
                'margin': margin,
                'is_3d': is_3d,
                'offset': offset,
                'dinamik': dinamik
            }

            degerler = None

            # max_workers=1 ile tek bir thread havuzu oluşturuyoruz
            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                # DİKKAT: helper.formasyon_sec yerine doğrudan _formasyon_sec_impl çağırıyoruz
                # Çünkü ana fonksiyon thread kontrolü yapıp işlemi durdurabilir.
                future = executor.submit(self.helper._formasyon_sec_impl, **kwargs)
                
                try:
                    # Sonucun tamamlanmasını bekle ve al
                    degerler = future.result()
                except Exception as exc:
                    print(f'❌ Formasyon hesaplama thread hatası: {exc}')
                    import traceback
                    traceback.print_exc()
                    degerler = None

            return degerler

    # Kamera nesnelerini takip etmek için bir sözlük (Dictionary)


    def kamera_ayarla(self, rov_id=0, mesafe=(0, -40, 120), aci=(0, 0, 0), fov=75, bölge=(0.02, 0.20, 0.80, 0.98)):
            """
            Sol,Sağ,Alt,Üst sırasıyla bölge parametresi (0-1 arası) - örn: (0.02, 0.30, 0.70, 0.98)
            ROV'a dinamik bir FPV kamera bağlar.
            """
            # Simülasyonun çalışıp çalışmadığını kontrol et
            if not hasattr(builtins, 'base'):
                print("❌ HATA: Simülasyon henüz başlatılmadığı için kamera oluşturulamaz.")
                return None
            
            b = builtins.base # Panda3D ana nesnesi

            # 1. Eğer bu ROV için zaten bir kamera varsa temizle
            if hasattr(self, 'aktif_kameralar') and rov_id in self.aktif_kameralar:
                try:
                    eski_cam = self.aktif_kameralar[rov_id]
                    b.win.removeDisplayRegion(eski_cam.node().getDisplayRegion(0))
                    eski_cam.removeNode()
                except:
                    pass
            
            if not hasattr(self, 'aktif_kameralar'):
                self.aktif_kameralar = {}

            # 2. Yeni Kamera Oluştur
            cam_np = b.makeCamera(b.win)
            cam_node = cam_np.node()
            
            # 3. Kamerayı ROV'a Bağla
            # (Önemli: ortam_ref ve rovs listesinin dolu olduğundan emin olun)
            try:
                target_rov = self.ortam_ref.rovs[rov_id]
                cam_np.reparentTo(target_rov)
            except Exception as e:
                print(f"❌ HATA: ROV-{rov_id} nesnesine ulaşılamadı: {e}")
                return None
            
            # 4. Konum ve Açı (Panda3D: X sağ, Y ileri, Z yukarı)
            cam_np.setPos(mesafe[0], mesafe[1], mesafe[2])
            cam_np.setHpr(aci[0], aci[1], aci[2])
            
            # 5. Lens ve Ekran Bölgesi
            cam_node.getLens().setFov(fov)
            region = cam_node.get_display_region(0)
            region.set_dimensions(bölge[0], bölge[1], bölge[2], bölge[3])
            region.set_sort(10) # En üstte çizilmesi için
            
            # Minimap ve UI'yı bu kameradan gizle (isteğe bağlı)
            cam_node.set_camera_mask(1) 

            self.aktif_kameralar[rov_id] = cam_np
            print(f"🎥 ROV-{rov_id} FPV Kamera Aktif (Bölge: {bölge})")
            return cam_np
    



    def lidere_don(self, rov_id=None, sessiz=True):
        """
        Lider hariç tüm ROV'ları (veya sadece belirtilen ROV'u) lidere doğru döndürür (hareket ettirmez).
        Takipçiler lidere baksın; hedef atanmaz, sadece yaw ayarlanır.

        Args:
            rov_id (int, optional): Verilirse sadece bu ROV lidere döner (lider değilse).
                None ise tüm takipçiler lidere döner.
            sessiz (bool): True ise mesaj yazdırmaz (varsayılan: True).

        Örnek:
            filo.lidere_don()           # Tüm takipçiler, sessiz
            filo.lidere_don(2)          # Sadece ROV-2, sessiz
            filo.lidere_don(2, sessiz=False)  # Mesajlı
        """
        lider_rov_id, _, _ = self._find_leader()
        if lider_rov_id is None:
            if not sessiz:
                print("⚠️ [FİLO] Lider bulunamadı. lidere_don() iptal.")
            return

        lider_gps = self.get(lider_rov_id, "gps")
        if lider_gps is None or len(lider_gps) < 2:
            if not sessiz:
                print("⚠️ [FİLO] Lider konumu alınamadı. lidere_don() iptal.")
            return

        lx, ly = float(lider_gps[0]), float(lider_gps[1])

        if rov_id is not None:
            if rov_id < 0 or rov_id >= len(self.sistemler):
                if not sessiz:
                    print(f"⚠️ [FİLO] Geçersiz rov_id={rov_id}. lidere_don() iptal.")
                return
            if rov_id == lider_rov_id:
                if not sessiz:
                    print(f"⚠️ [FİLO] ROV-{rov_id} lider, lidere dönmesine gerek yok.")
                return
            rov_listesi = [rov_id]
        else:
            rov_listesi = [i for i in range(len(self.sistemler)) if i != lider_rov_id]

        for i in rov_listesi:
            rov_gps = self.get(i, "gps")
            if rov_gps is None or len(rov_gps) < 2:
                continue
            fx, fy = float(rov_gps[0]), float(rov_gps[1])
            dx, dy = lx - fx, ly - fy
            hedef_yaw = math.degrees(math.atan2(dx, dy))
            while hedef_yaw >= 360:
                hedef_yaw -= 360
            while hedef_yaw < 0:
                hedef_yaw += 360
            self.set(i, "yaw", hedef_yaw)

        if not sessiz:
            if rov_id is not None:
                print(f"✅ [FİLO] lidere_don: ROV-{rov_id} lidere (ROV-{lider_rov_id}) doğru döndürüldü.")
            else:
                print(f"✅ [FİLO] lidere_don: Tüm takipçiler lidere (ROV-{lider_rov_id}) doğru döndürüldü (hareket yok).")

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
    

    def hedef(self, koordinat=None, rov_id=None, ciz=True): # ciz parametresi eklendi
            """
            Hedef konumu atar veya okur (Thread-safe).
            ... (mevcut docstring) ...
            """
            # Okuma modu: koordinat None ise
            if koordinat is None:
                # ... (mevcut okuma mantığı aynen kalıyor) ...
                if rov_id is not None:
                    if rov_id < 0 or rov_id >= len(self.sistemler):
                        return None
                    return self._rov_hedefleri.get(rov_id)
                result = {}
                for i in range(len(self.sistemler)):
                    result[i] = self._rov_hedefleri.get(i)
                return result
            
            # Atama modu: koordinat verilmişse, rov_id zorunlu
            if rov_id is None:
                print("❌ [HEDEF] Hedef atama için rov_id parametresi gereklidir!")
                return None
            
            # ... (koordinat doğrulama mantığı aynen kalıyor) ...
            if not isinstance(koordinat, (tuple, list)) or len(koordinat) < 2:
                return None
            
            x = koordinat[0]
            y = koordinat[1]
            z = koordinat[2] if len(koordinat) > 2 else 0
            
            # Thread-safe çağrı: ciz parametresini kwargs içine ekliyoruz
            if not self._is_main_thread():
                self._command_queue.put(('hedef', (x, y, z), {'rov_id': rov_id, 'ciz': ciz}))
                return (x, y, z)
            
            # Ana thread'deyiz, ciz parametresiyle birlikte impl'e gönderiyoruz
            return self._hedef_impl(x, y, z, rov_id=rov_id, ciz=ciz)
    
    def _hedef_impl(self, x, y, z, rov_id=None, ciz=True): # ciz parametresi eklendi
            """hedef() fonksiyonunun gerçek implementasyonu (ana thread'de çalışır)."""
            if rov_id is None:
                return None
            
            if rov_id < 0 or rov_id >= len(self.sistemler):
                return None
            
            # Hedefi kaydet ve git komutunu ver
            self._rov_hedefleri[rov_id] = (x, y, z)
            self.git(rov_id, x, y, z, ai=True)
            
            # Lider tespiti
            lider_rov_id = None
            for i, sistem in enumerate(self.sistemler):
                if hasattr(sistem, 'rov') and sistem.rov.role == 1:
                    lider_rov_id = i
                    break
            
            # GÖRSELLEŞTİRME KONTROLÜ
            if lider_rov_id == rov_id:
                self.hedef_pozisyon = (x, y, z)
                
                # Eğer ciz True ise Ursina dünyasında X işaretini oluştur/güncelle
                if ciz:
                    self._hedef_gorsel_olustur(x, y, z)
                else:
                    # Opsiyonel: Eğer ciz False ise ve ekranda eski bir hedef görseli varsa silebilirsin
                    if self.hedef_gorsel:
                        from ursina import destroy
                        destroy(self.hedef_gorsel)
                        self.hedef_gorsel = None

                # Ortam referansı güncellemesi (Harita/Minimap verisi için)
                if self.ortam_ref:
                    try:
                        self.ortam_ref.hedef_pozisyon = (x, y)
                    except Exception:
                        pass
            
            return (x, y, z)

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
    
    def ada_cevre(self, offset=0.0, sessiz: bool = False):
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
    
    def _hedef_gorsel_olustur(self, x, y, z, id=None, debug=True):
        """
        Hedef pozisyonunu Ursina'da büyük X işareti olarak gösterir.
        """
        return self.helper.hedef_gorsel_olustur(x, y, z, id=id, debug=debug)
    
    def hedef_sil(self, id=None):
        """
        Hedef görselini siler.
        """
        return self.helper.hedef_sil(id=id)
    
    def debug_hedefleri_temizle(self):
        """
        Tüm hedef görsellerini temizler (debug amaçlı).
        """
        return self.helper.debug_hedefleri_temizle()
    def git(self, rov_id: int, x, y: float = None, z: float = None, ai: bool = True, sessiz: bool = True) -> None:
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

    def git_path(self, rov_id, hedef, ai=True, isaret=False):
        """
        ROV'a bir yol atar ve otomatik moda geçirir (Thread-safe).
        isaret=True ise bir sonraki waypoint minimapte uygun renkle gösterilir.
        """
        return self.helper.git_path(rov_id, hedef, ai=ai, isaret=isaret)


    def move(self, rov_id: int, yon: str, guc: float = 1.0, sessiz: bool = True) -> None:
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
            sessiz: True (varsayılan) ise log yazılmaz; False ise bilgi/hata mesajları yazdırılır.
        
        Örnekler:
            filo.move(0, 'ileri', 1.0)   # ROV-0 %100 güçle ileri
            filo.move(1, 'sag', 0.5)     # ROV-1 %50 güçle sağa
            filo.move(2, 'cik', 0.3)      # ROV-2 %30 güçle yukarı
            filo.move(3, 'dur', 0.0)      # ROV-3 dur (güç=0)
            filo.move(0, 'ileri')         # ROV-0 %100 güçle ileri (varsayılan)
            filo.move(0, 'yaw', 1.0)     # ROV-0 saat yönünün tersine döndürme
            filo.move(0, 'yaw', -1.0)    # ROV-0 saat yönünde döndürme
            filo.move(0, 'ileri', 1.0, sessiz=False)  # Loglarla birlikte
        """
        return self.helper.move(rov_id=rov_id, yon=yon, guc=guc, sessiz=sessiz)
    
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
    
    def minimap(self, durum=True, convex=True, a_star=True, scale=None, grid=None, *args, **kwargs):
        """
        Minimap'i açar, kapatır veya durumunu döndürür.
        Harita fonksiyonunun tüm işlevlerine sahiptir.

        Args:
            durum: True/False - Minimap'i aç/kapat (None ise toggle)
            convex: True/False - Convex hull'u göster/gizle
            a_star: True/False - A* yolunu göster/gizle
            scale: Çarpan (1=taban 0.45, 2=2 katı, 0.1=4.5 vb.); verilirse boyut dinamik güncellenir
            grid: Grid sayısı (None=varsayılan GRID_UNIT m; N=toplam N aralık, 1 grid=(2*havuz)/N m).
                  Minimap üzerinde "1 grid=X m" ve ölçek bilgisi yazılır.

        Örnekler:
            filo.minimap()  # Toggle (aç/kapat), varsayılan grid
            filo.minimap(True)  # Aç
            filo.minimap(grid=10)  # 10 aralık, 1 grid = (2*havuz)/10 m
            filo.minimap(scale=2, grid=8)  # 2 kat büyük, 8 aralık
            filo.minimap("ekle", filo.ada_cevre())  # Ada çevre noktalarını minimapte turuncu-kahverengi çizgi olarak göster
        """
        return self.helper.minimap(durum=durum, convex=convex, a_star=a_star, scale=scale, grid=grid, *args, **kwargs)

    
    
    def gidilecek_noktalar_n(self, path=None, n=10):
        """
        A* yolu üzerinde başlangıçtan itibaren her n metre (adım) sonra bir rota noktası alır.
        Başlangıç noktası hariç; ilk nokta n m, ikinci 2n m, ... sonra gelir.

        Args:
            path: A* yolu — [(x, y), ...]. None ise filo.ortam_ref.harita.a_star_yolu kullanılır.
            n: Adım uzunluğu (metre, varsayılan 10). Her n metre sonra bir waypoint eklenir.

        Returns:
            list: [[x1, y1], [x2, y2], ...] — rota noktaları (başlangıç hariç; bitiş dahil).
        """
        return self.helper.gidilecek_noktalar_n(path=path, n=n)
    def rota_bitir(self, rov_id: int):
            """
            ROV rotadaki son noktaya ulaştığında çağrılır.
            Navigasyon verilerini temizler, ROV'u durdurur ve formasyon moduna sokar.
            """
            # 1. Navigasyon Listelerinden Kaydı Sil
            # .pop(key, None) kullanıyoruz ki eğer key yoksa hata vermesin.
            self._git_nokta_listesi.pop(rov_id, None)
            self._git_mevcut_nokta_indeksi.pop(rov_id, None)

            # 2. Hedef Verisini Sıfırla
            if hasattr(self, '_rov_hedefleri') and self._rov_hedefleri:
                self._rov_hedefleri[rov_id] = None

            # 3. ROV Sistemini Durdur ve Sıfırla
            if hasattr(self, 'sistemler') and 0 <= rov_id < len(self.sistemler):
                sistem = self.sistemler[rov_id]
                if sistem:
                    # Fiziksel hızı sıfırla (Ursina Vec3)
                    if hasattr(sistem, 'rov'):
                        sistem.rov.velocity = Vec3(0, 0, 0)
                    
                    # Sistemin iç hedef değişkenini temizle (Varsa)
                    # (Helper sınıfındaki hedef çizimlerini kapatmak için)
                    if hasattr(sistem, 'hedef'):
                        sistem.hedef = None

            # 4. Formasyon Moduna Geri Dön
            # Rota bittiğine göre artık lideri takip etmeli veya formasyona girmeli
            self.lidere_don(rov_id=rov_id)
            
            # Bilgi mesajı (İsteğe bağlı)
            # print(f"[BİLGİ] ROV-{rov_id} rotasını tamamladı.")

    def hedef_guncelle(self, rov_id: int, koordinat: tuple):
            """
            ROV'un anlık hedefini günceller, filo hafızasına kaydeder 
            ve ilgili ROV sistemine bildirir.
            
            Args:
                rov_id (int): Hedefi değişecek ROV ID
                koordinat (tuple): (x, y, z) formatında yeni hedef noktası
            """
            # 1. Koordinat Güvenliği (Z ekseni eksik gelirse tamamla)
            if len(koordinat) == 3:
                x, y, z = koordinat
            elif len(koordinat) == 2:
                x, y = koordinat
                z = 0.0 # Z verilmezse 0 kabul et
            else:
                return # Geçersiz veri

            # 2. Filo Hafızasını Güncelle (Helper sınıfının okuduğu yer)
            if hasattr(self, '_rov_hedefleri'):
                self._rov_hedefleri[rov_id] = (x, y, z)

            # 3. ROV Nesnesine Bildir (Görsel çizimler ve iç mantık için)
            if hasattr(self, 'sistemler') and 0 <= rov_id < len(self.sistemler):
                sistem = self.sistemler[rov_id]
                if sistem is not None:
                    # Sistem sınıfında hedef_atama metodu varsa çağır
                    if hasattr(sistem, 'hedef_atama'):
                        sistem.hedef_atama(x, y, z)
                    # Alternatif: Direkt hedef değişkenini set et
                    elif hasattr(sistem, 'hedef'):
                        from ursina import Vec3 # Gerekirse import
                        sistem.hedef = Vec3(x, y, z)

            # 4. Vektörleri Anlık Olarak Güncelle
            # Hedef değiştiği için APF'nin hemen yeni rota çizmesi gerekir
            if hasattr(self, 'formasyon_sec'):
                self.formasyon_sec(dinamik=True)

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

    def hedef_atama(self, x, y, z):
        self.hedef = Vec3(x, y, z)

    def rehber_guncelle(self, rehber):
        if self.modem: 
            self.modem.rehber_guncelle(rehber)
    
    def guncelle(self, gat_kodu=None):
        """
        GNC Güncelleme: Hedef varsa ve manuel kontrol kapalıysa hedefe git.
        Doğrudan helper.guncelle() çağrılır (wrapper katmanı kaldırıldı).
        """
        if hasattr(self, 'helper') and self.helper is not None:
            return self.helper.guncelle(gat_kodu=gat_kodu)

    def _hedefe_varis_islemleri(self, fark):
        """Hedefe ulaşıldığında yapılacak işlemler. Sonraki nokta yoksa hedefi temizler, hızı sıfırlar ve log basar."""
        # Çoklu nokta geçiş mantığı: sonraki waypoint varsa ona geç, bu frame'de durma
        if self.filo_ref:
            had_next = self._siradaki_noktaya_gec()
            if had_next:
                return
        # Tek nokta veya son nokta: hedefi kaldır, dur, otonom sürüşü sonlandır
        rov_id = self.filo_ref.sistemler.index(self) if self.filo_ref else None
        self.hedef = None
        self.rov.velocity = Vec3(0, 0, 0)
        self.ai_aktif = False

    def _siradaki_noktaya_gec(self):
        """Çoklu nokta takibinde sonraki noktaya geçer. Sonraki nokta atandıysa True, yoksa False döner."""
        try:
            my_id = self.filo_ref.sistemler.index(self)
            nokta_listesi = self.filo_ref._git_nokta_listesi.get(my_id)
            mevcut_indeks = self.filo_ref._git_mevcut_nokta_indeksi.get(my_id, 0)
            
            if nokta_listesi and mevcut_indeks + 1 < len(nokta_listesi):
                yeni_indeks = mevcut_indeks + 1
                nxt = nokta_listesi[yeni_indeks]
                self.filo_ref._git_mevcut_nokta_indeksi[my_id] = yeni_indeks
                # Mevcut derinliği koru (hedef varsa onun z'sini kullan, yoksa ROV'un mevcut z'sini)
                if self.hedef is not None:
                    current_z = self.hedef.z
                else:
                    from FiratROVNet.gnc import Koordinator
                    current_sim_pos = Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z)
                    current_z = current_sim_pos[2]
                self.hedef = Vec3(nxt[0], nxt[1], current_z)
                return True
            elif nokta_listesi:
                # Liste bitti
                self.filo_ref._git_nokta_listesi.pop(my_id, None)
        except Exception:
            pass
        return False

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


