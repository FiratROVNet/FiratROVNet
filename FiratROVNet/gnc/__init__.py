"""
GNC Module
The main file for Mathematical calculations, geometric operations, and complex logic for Filo and TemelGNC classes.
"""

import builtins
import queue
import threading
import math
import random
import numpy as np
from ursina import *

# Yerel modül importları
from ..config import cfg, GATLimitleri, SensorAyarlari, HareketAyarlari, FizikSabitleri, Hidrodinamik, BasitKalmanFiltresi
from ..kutuphane.helper.gnc_helper.mixins.formation import Formasyon
from ..iletisim import AkustikModem
from ..hull import HullManager
from FiratROVNet.kutuphane.helper.gnc_helper import FiloHelper, TemelGNCHelper
import concurrent.futures
from FiratROVNet.lider_sec import liderlik_secimini_baslat
# Lazy import: FiratAnalizci circular import problemini önlemek için _basla_gat_modeli içinde import edilir

# Modüler yapı - GNC subpackage
from .koordinator import Koordinator, SafeDict
from .damage_system import DamageSystem
from .logs import LogSystem
from ..animations import GelismisParcacik, entity_patlat
from ..camera_manager import CameraManager
from ..lider_sec import LeaderManager

import inspect
import os
import logging

# ==========================================
# 1. FİLO (ROV FİLO YÖNETİCİSİ)
# ==========================================
class Filo:
    def __init__(self, ortam_ref=None):
        # Temel Referanslar
        self.ortam_ref = ortam_ref
        self.hull_manager = HullManager(self)
        self._command_queue = queue.Queue()
        self._main_thread_id = threading.get_ident()
        self.helper = FiloHelper(self)
        
        # Sorumlu Sistemler (ModülerYapı)
        self.damage_system = DamageSystem(filo_ref=self)
        self.camera_manager = CameraManager(filo_ref=self)
        self.leader_manager = LeaderManager(filo_ref=self)
        self.log_system = LogSystem()
        
        # Hedef ve Formasyon Durumu
        self.asil_hedef = None
        self.hedef_gorsel = None
        self.hedef_pozisyon = None
        
        # Formasyon Yönetimi
        self.aktif_formasyon = {}
        self._formasyon_id_pool = list(range(len(Formasyon.TIPLER)))
        random.shuffle(self._formasyon_id_pool)
        self._formasyon_hedefleri = {}
        
        # Navigasyon Verileri
        self._git_nokta_listesi = {}      # {rov_id: [[x,y], ...]}
        self._git_mevcut_nokta_indeksi = {}
        self._git_isaret = {}             # {rov_id: bool}
        self._rov_hedefleri = {}          # {rov_id: (x, y, z)}

        # Ayarlar
        self._git_hedef_mesafe_toleransi = 2.0
        self._maksimum_yaw_donme_hizi = 30.0
        self._git_maksimum_yaw_donme_hizi = 45.0
        self._formasyon_yaw_senkronizasyon_mesafesi = 5.0

        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.yeni_pozisyonlar = None
        
        # === GAT Modeli ve Navigasyon Kuyruğu ===
        self._basla_gat_modeli()
        self.nav_queue = {}               # {g_id: [{'pos': (x,y,z), 'id': n}, ...]}
        self.current_target_id = {}       # {g_id: id}
        self.target_counter = 0           # Her tıklamada artan benzersiz ID sayacı
        
        # Ortam referansını ayarla ve başlat
        if self.ortam_ref:
            self.ortam_ref.filo = self
            self._baslatma_tamamla()
    

    # ============================================================
    # KURULUM VE SİSTEM YÖNETİMİ (SADELEŞTİRİLMİŞ)
    # ============================================================
    
    def _baslatma_tamamla(self):
        """Filo ve ortam ilk kurulumunu tamamla."""
        if not self.ortam_ref:
            return
        
        # ROV'lara GNC örnekleri ekle
        for rov in self.ortam_ref.rovs:
            rov.gnc = TemelGNC(rov, self)
        
        # Minimap başlat
        self.minimap(scale=1.0)

    def _basla_gat_modeli(self):
        """GAT modelini yükle ve başlat. Başarısız olursa disable et."""
        try:
            # Lazy import: Circular import sorununu önle
            from GAT.gat_test import FiratAnalizci
            self.gat = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
            print("✅ GAT modeli yüklendi.")
        except Exception as e:
            print(f"⚠️ GAT modeli yüklenemedi, AI devre dışı: {e}")
            self.gat = None

    def guncelle_navigasyon_kuyrugu(self):
        """Navigasyon kuyruğu ve varış yönetimi (grup bazlı)."""
        for g_id, grup_rovs in self.g_rovs.items():
            lider_id, _ = self.find_leader_info(g_id=g_id)
            if lider_id is None:
                continue

            aktif_rota = self._git_nokta_listesi.get(lider_id)
            mevcut_hedef_id = self.current_target_id.get(g_id)

            # DURUM A: Hedefe Varildi mi?
            if mevcut_hedef_id is not None and not aktif_rota:
                print(f"✅ [NAV] Grup-{g_id} hedef {mevcut_hedef_id} noktasina varildi.")
                self.hedef_sil(mevcut_hedef_id)
                self.current_target_id[g_id] = None

            # DURUM B: Yeni hedefe basla mi?
            grup_kuyruk = self.nav_queue.get(g_id, [])
            if not aktif_rota and grup_kuyruk:
                next_data = grup_kuyruk.pop(0)
                target_pos = next_data['pos']
                self.current_target_id[g_id] = next_data['id']

                print(f"🚀 [NAV] Grup-{g_id} siradaki hedefe geciliyor: {self.current_target_id[g_id]}")
                self.git_path(lider_id, target_pos, isaret=True)

    def guncelle_gat_analizi(self, tahminler):
        """GAT modelinden tahmin alıp ROV'lara ata."""
        try:
            if not self.ortam_ref:
                return
            
            veri = self.ortam_ref.simden_veriye()
            ai_aktif = getattr(cfg, 'ai_aktif', True)
            
            active_rovs = [r for r in self.ortam_ref.rovs if r and not (hasattr(r, 'is_destroyed') and r.is_destroyed)]

            if ai_aktif and self.gat:
                try:
                    tahminler_yeni, _, _ = self.gat.analiz_et(veri)
                    
                    # GAT predictions'ı doğru indekslere ata
                    # tahminler_yeni active_rovs sırasında predictions döndürür
                    active_idx = 0
                    for all_idx, rov in enumerate(self.ortam_ref.rovs):
                        # Destroyed/None ROV'ları atla
                        if not rov or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
                            continue
                        
                        # Active index bounds check
                        if active_idx < len(tahminler_yeni) and all_idx < len(tahminler):
                            tahminler[all_idx] = tahminler_yeni[active_idx]
                        active_idx += 1
                except Exception as e:
                    print(f"⚠️ GAT analiz hatası: {e}")
            
            # Tahmin boyutunu ROV sayısına göre ayarla
            if len(tahminler) < len(active_rovs):
                tahminler.extend(np.zeros(len(active_rovs) - len(tahminler), dtype=int))
                
        except Exception as e:
            print(f"❌ GAT güncelleme hatası: {e}")

    def guncelle_gorseller_ve_renkler(self, tahminler):
        """ROV renkleri ve label'larını GAT koduna göre güncelle."""
        if not self.ortam_ref:
            return
        
        kod_renkleri = {
            0: color.orange, 1: color.red, 2: color.black, 3: color.yellow, 4: color.magenta
        }
        durum_txts = ["OK", "ENGEL", "CARPISMA", "KOPUK", "UZAK"]
        
        # app.rovs içinde her ROV'un indeksini al ve tahminler'den eşleştir
        for idx, rov in enumerate(self.ortam_ref.rovs):
            # Destroyed veya None ROV'ları atla
            if not rov or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
                continue
            
            # Tahminler indeksi bounds check yap
            if idx >= len(tahminler):
                continue
            
            gat_kodu = tahminler[idx]
            rov.gat_kodu = gat_kodu
            
            # Renk ayarı (Lider sabit kırmızı, diğerleri GAT'a göre)
            if rov.role == 1:
                rov.color = color.red
            else:
                rov.color = kod_renkleri.get(gat_kodu, color.white)
            
            # Label (Etiket) ayarları
            rov.label.color = rov.color
            durum_metni = durum_txts[gat_kodu] if 0 <= gat_kodu < len(durum_txts) else f"GAT:{gat_kodu}"
            rov.label.text = f"{durum_metni}{rov.id}"

    @property
    def rovs(self):
        """
        self.sistemler yerine doğrudan ortamdaki canlı ROV'ları döndürür.
        """
        if not self.ortam_ref or not hasattr(self.ortam_ref, 'rovs'):
            return []
        return [r for r in self.ortam_ref.rovs if r and not (hasattr(r, 'is_destroyed') and r.is_destroyed)]

    @property
    def g_rovs(self):
        return SafeDict(self.ortam_ref.g_rovs)


    def find_rov_by_id(self, rov_id):
        """ID'si verilen ROV'u tüm gruplar içerisinde arayıp bulur."""
        for g_id, grup in self.g_rovs.items():
            for rov in grup:
                if rov and rov.id == rov_id:
                    if not (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
                        return rov
        return None
    
    def _get_all_rovs_positions(self):
        """Tüm ROV'ların güncel konumlarını döner. {rov_id: (x, y, z)} formatında."""
        if not self.ortam_ref or not hasattr(self.ortam_ref, 'rovs'):
            return {}
        positions = {}
        for rov in self.ortam_ref.rovs:
            if rov and not (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
                positions[rov.id] = self.get(rov.id, 'gps')
        return positions
    
    def kamera_ayarla(self, *args, **kwargs):
        """Kamera yönetimini camera_manager'a yönlendir."""
        return self.camera_manager.kamera_ayarla(*args, **kwargs)
    
    def kamera_kaldir(self, rov_id):
        """Kamera kaldırma işlemini camera_manager'a yönlendir."""
        return self.camera_manager.kamera_kaldir(rov_id)
            
    def guncelle_hepsi(self, tahminler):
        """
        Tüm GNC sistemlerini koordineli şekilde günceller.
        
        Operasyon Sırası (Önem Sırasına Göre):
        1. Sistem Hazırlığı    → Command queue işle
        2. Lider Yönetimi      → Yeni lider seç & değişim yap
        3. ROV Başına İşlemler → Hasar, GNC, Motor komutları
        4. Sistem Güncellemeleri → Minimap, engel bulut
        """
        
        # ============================================================
        # 1. SİSTEM HAZIRLIGI
        # ============================================================
        self._process_command_queue()
        
        if not self.ortam_ref:
            return

        # ============================================================
        # 2. LİDER YÖNETİMİ (Grup bazlı lider seçim ve role transfer)
        # ============================================================
        yeni_lider_id, skor = liderlik_secimini_baslat(self, self.asil_hedef)
        self.leader_manager.guncelle_liderler(yeni_lider_id)

        # ============================================================
        # 3. ROV BAŞINA İŞLEMLER
        # ============================================================
        # Canlı ROV'ların kopyasını al (Döngü sırasında silinirse çökmemesi için)
        mevcut_rovlar = [r for r in list(self.ortam_ref.rovs) 
                        if r and not (hasattr(r, 'is_destroyed') and r.is_destroyed)]
        
        for i, rov in enumerate(mevcut_rovlar):
            # ROV'un GNC sistemi kontrolü
            if not hasattr(rov, 'gnc') or rov.gnc is None:
                continue

            # --- 3A. GAT Tahmini Ata ---
            gat_kodu = tahminler[i] if tahminler is not None and i < len(tahminler) else None

            # --- 3B. Hasar Kontrol (Öncelikli - Patlama Check) ---
            joule_esigi = 10.0  # Joule cinsinden hasar eşiği
            if self.damage_system.rov_hasar_kontrol_direct(rov, joule_esigi=joule_esigi):
                # ROV patladı - Patlama efekti ve limbo
                self.entity_patlat(rov, parca_sayisi=80)
                continue  # Bu ROV için güncelleme yapma

            # --- 3C. GNC Sistem Güncelleme ---
            try:
                rov.gnc.guncelle(gat_kodu=gat_kodu)
            except Exception as e:
                # Patlama anındaki hataları sessizce geç
                if "!is_empty()" not in str(e):
                    print(f"⚠️ [FİLO] ROV-{rov.id} GNC Hatası: {e}")
                LogSystem.log_exception(e)

        # ============================================================
        # 4. SİSTEM GÜNCELLEMELERİ
        # ============================================================
        # --- 4A. Engel Bulut Güncelleme ---
        if self.ortam_ref.minimap:
            try:
                self.ortam_ref.minimap._engel_bulutu_guncelle()
            except Exception as e:
                LogSystem.log_exception(e)

        # --- 4B. Minimap Görsel Güncelleme ---
        if self.ortam_ref.minimap:
            try:
                self.ortam_ref.minimap.gorsel_guncelle()
            except Exception as e:
                LogSystem.log_exception(e)

    # ============================================================
    # YOL BULMA İŞLEMLERİ
    # ============================================================

    def bul_guvenli_yol(self, baslangic, hedef, yasak_bolge_radius=15.0):
        """
        Başlangıç ve hedef arasında engelleri aşan güvenli yol bulur.
        
        Args:
            baslangic (tuple): (x, y, z) başlangıç konumu
            hedef (tuple): (x, y, z) hedef konumu
            yasak_bolge_radius (float): Engelden uzak kalınacak mesafe (meter)
        
        Returns:
            list: Waypoint'ler [(x1,y1), (x2,y2), ..., (xf,yf)] veya None
        """
        try:
            if not baslangic or not hedef:
                return None
            
            # Direkt yol kontrolü - engel var mı?
            if self._direkt_yol_gecerli_mi(baslangic, hedef, yasak_bolge_radius):
                # Direkt gidiş aman
                return [(baslangic[0], baslangic[1]), (hedef[0], hedef[1])]
            
            # Engel varsa A* pathfinding kullan
            yol = self._a_star_pathfinding(baslangic, hedef, yasak_bolge_radius)
            return yol
            
        except Exception as e:
            print(f"⚠️ Yol bulma hatası: {e}")
            LogSystem.log_exception(e)
            return None

    def _direkt_yol_gecerli_mi(self, baslangic, hedef, min_mesafe=15.0):
        """
        Başlangıçtan hedefe direkt gidişin engel ile çarpışıp çarpışmadığını kontrol et.
        
        Args:
            baslangic (tuple): (x, y, z) başlangıç
            hedef (tuple): (x, y, z) hedef
            min_mesafe (float): Engelden minimum uzak kalma mesafesi
        
        Returns:
            bool: True = güvenli direkt yol, False = engel var
        """
        try:
            # Helper sistemini kullan
            engeller = self.helper.engel_bul(baslangic, hedef)
            
            for engel_pos in engeller:
                # Engel ile baslangic-hedef hattı arasındaki mesafe
                dist = self._nokta_hat_mesafesi(engel_pos, baslangic, hedef)
                if dist < min_mesafe:
                    return False  # Engel çok yakın, direkt yol uygun değil
            
            return True  # Engel yok, direkt gitmek aman
            
        except Exception as e:
            print(f"⚠️ Direkt yol kontrolü hatası: {e}")
            return False

    def _a_star_pathfinding(self, baslangic, hedef, yasak_bolge_radius=15.0):
        """
        A* algoritması kullanarak engelleri aşan optimal yol bulur.
        
        Args:
            baslangic (tuple): (x, y, z) başlangıç
            hedef (tuple): (x, y, z) hedef
            yasak_bolge_radius (float): Engel civarındaki yasak bölge yarıçapı
        
        Returns:
            list: Waypoint'ler listesi [(x1,y1), (x2,y2), ..., (xf,yf)]
        """
        try:
            # Burada A* implementasyonu yapılacak
            # Şimdilik placeholder: 2-3 waypoint ile basit obstrüksyon kaçış
            
            if not self.ortam_ref:
                return None
            
            # Engel konumlarını al
            engeller = []
            if hasattr(self.ortam_ref, 'island_entities'):
                for ada in self.ortam_ref.island_entities:
                    if ada and hasattr(ada, 'position'):
                        engeller.append((ada.position.x, ada.position.z))  # (X, Z) 2D
            
            # Basit sidestepping: Engelin kenarından dolaş
            waypoints = self._engel_kenarindan_dolas(baslangic, hedef, engeller, yasak_bolge_radius)
            return waypoints
            
        except Exception as e:
            print(f"⚠️ A* pathfinding hatası: {e}")
            return None

    def _engel_kenarindan_dolas(self, baslangic, hedef, engeller, min_dist=15.0):
        """
        Engellerin kenarından dolaşarak waypoint'ler oluştur.
        
        Returns:
            list: Waypoint'ler [(x1,y1), (x2,y2), ..., (xf,yf)]
        """
        try:
            if not engeller:
                # Engel yoksa direkt git
                return [(baslangic[0], baslangic[1]), (hedef[0], hedef[1])]
            
            # Engel ile başlangıç/hedef arasındaki mesafeyi kontrol et
            waypoints = [(baslangic[0], baslangic[1])]
            
            # Her engel için bypass waypoint oluştur
            for engel_x, engel_z in engeller:
                hat_sonunda_mi = \
                    abs(engel_x - hedef[0]) < min_dist * 2 or \
                    abs(engel_z - hedef[1]) < min_dist * 2
                
                if hat_sonunda_mi:
                    # Engelin kenarına bypass waypoint ekle
                    offset = min_dist * 1.5
                    bypass_x = engel_x + offset
                    bypass_z = engel_z + offset
                    waypoints.append((bypass_x, bypass_z))
            
            # Hedef ekle
            waypoints.append((hedef[0], hedef[1]))
            
            return waypoints
            
        except Exception as e:
            print(f"⚠️ Engel dolaş hatası: {e}")
            return [(baslangic[0], baslangic[1]), (hedef[0], hedef[1])]

    def _nokta_hat_mesafesi(self, nokta, hat_baslangic, hat_sonu):
        """
        Bir nokta ile bir hat arasındaki minimum mesafeyi hesapla (2D).
        
        Returns:
            float: Mesafe (meter)
        """
        try:
            # Hat parametresi: 2D projeksiyon
            x0, y0 = nokta[0], nokta[1]
            x1, y1 = hat_baslangic[0], hat_baslangic[1]
            x2, y2 = hat_sonu[0], hat_sonu[1]
            
            # Nokta-hat mesafesi formülü
            dist_numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
            dist_denominator = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
            
            if dist_denominator == 0:
                return math.inf
            
            return dist_numerator / dist_denominator
            
        except Exception as e:
            print(f"⚠️ Mesafe hesaplama hatası: {e}")
            return math.inf

    def carpisma_enerjisi_hesapla(self, *args, **kwargs):
        """Hasar hesaplamalarını damage_system'a yönlendir."""
        return self.damage_system.carpisma_enerjisi_hesapla(*args, **kwargs)
    
    def rov_hasar_kontrol(self, *args, **kwargs):
        """Hasar kontrolünü damage_system'a yönlendir."""
        return self.damage_system.rov_hasar_kontrol_direct(*args, **kwargs)
    
    def rov_is_hit(self, *args, **kwargs):
        """Çarpışma kontrolünü damage_system'a yönlendir."""
        return self.damage_system.rov_is_hit(*args, **kwargs)

    def rov_verilerini_temizle(self, rov_id):
            """Silinen ROV'un tüm izlerini GNC hafızasından siler."""
            
            # Vektör (Ok) temizliği
            if hasattr(self.helper, 'apf_temizle'):
                self.helper.apf_temizle()
            
            # Kamera temizliği
            if self.camera_manager.kamera_var_mi(rov_id):
                self.camera_manager.kamera_kaldir(rov_id)
    # ============================================================
    # PATLAMA VE SİLME YÖNETİMİ
    # ============================================================
    def entity_patlat(self, hedef_entity, parca_sayisi=60):
            """
            Entity patlama efektini tetikler.
            animations.py'deki entity_patlat fonksiyonunu çağırır.
            """
            entity_patlat(hedef_entity, parca_sayisi, filo_ref=self)


        

    # ============================================================
    # HEDEF VE HAREKET YÖNETİMİ
    # ============================================================

    def git(self, rov_id: int, x, y: float = None, z: float = None, ai: bool = True, sessiz: bool = True):
        return self.helper.git(rov_id=rov_id, x=x, y=y, z=z, ai=ai, sessiz=sessiz)

    def git_path(self, rov_id, hedef, ai=True, isaret=False):
        return self.helper.git_path(rov_id, hedef, ai=ai, isaret=isaret)

    def move(self, rov_id: int, yon: str, guc: float = 1.0, sessiz: bool = True):
        return self.helper.move(rov_id=rov_id, yon=yon, guc=guc, sessiz=sessiz)

    def hedef(self, koordinat=None, rov_id=None, ciz=True):
        if koordinat is None:
            if rov_id is not None: return self._rov_hedefleri.get(rov_id)
            return self._rov_hedefleri
        
        if not self._is_main_thread():
            self._command_queue.put(('hedef', (koordinat[0], koordinat[1], koordinat[2] if len(koordinat)>2 else 0), {'rov_id': rov_id, 'ciz': ciz}))
            return koordinat

        return self._hedef_impl(*koordinat, rov_id=rov_id, ciz=ciz)

    def _hedef_impl(self, x, y, z, rov_id=None, ciz=True):
        if rov_id is None or not self.ortam_ref: return None
        
        # Güvenli Erişim
        try:
            rov = self.find_rov_by_id(rov_id)
            if not rov:
                return None
        except Exception as e:
            LogSystem.log_exception(e)
            return None

        self._rov_hedefleri[rov_id] = (x, y, z)
        self.git(rov_id, x, y, z, ai=True)
        
        # Görselleştirme (Sadece Lider için veya debug modunda)
        rov = self.find_rov_by_id(rov_id)
        if not rov:
            return None
        if rov.role == 1:
            self.hedef_pozisyon = (x, y, z)
            if ciz: self._hedef_gorsel_olustur(x, y, z)
            elif self.hedef_gorsel:
                destroy(self.hedef_gorsel)
                self.hedef_gorsel = None
                
        return (x, y, z)

    # ============================================================
    # VERİ ERİŞİMİ VE AYARLAR (GET/SET)
    # ============================================================

    def get(self, rov_id: int = None, veri_tipi: str = None, taraf: int = None, sessiz: bool = False):
        return self.helper.get(rov_id=rov_id, veri_tipi=veri_tipi, taraf=taraf, koordinator=Koordinator, sessiz=sessiz)

    def set(self, rov_id: int, ayar_adi: str, deger) -> bool:
        if not self._is_main_thread():
            self._command_queue.put(('set', (rov_id, ayar_adi, deger), {}))
            return True
        return self._set_impl(rov_id, ayar_adi, deger)

    def _set_impl(self, rov_id: int, ayar_adi: str, deger) -> bool:
        if not self.ortam_ref:
            return False
        rov = self.find_rov_by_id(rov_id)
        if not rov: return False
        
        try:
            rov.set(ayar_adi, deger)
            return True
        except Exception as e:
            self.ds = e
            return False

    # ============================================================
    # THREADING VE YARDIMCILAR
    # ============================================================

    def _is_main_thread(self):
        return threading.get_ident() == self._main_thread_id

    def _process_command_queue(self):
        try:
            while not self._command_queue.empty():
                cmd, args, kwargs = self._command_queue.get_nowait()
                if cmd == 'set':
                    self._set_impl(*args, **kwargs)
                elif cmd == 'hedef':
                    self._hedef_impl(*args, **kwargs)
                elif hasattr(self.helper, f"_{cmd}_impl"):
                    getattr(self.helper, f"_{cmd}_impl")(*args, **kwargs)
        except Exception as e:
            LogSystem.log_exception(e)

    def execute_queued_commands(self):
        self._process_command_queue()

    # ============================================================
    # WRAPPER METODLAR (Helper Yönlendirmeleri)
    # ============================================================
    # Bu metodlar kodun geri kalanıyla uyumluluk için tutulmuştur.
    # İş mantığı FiloHelper sınıfındadır.

    def hull(self, offset=40.0): return self.helper.hull(offset=offset)
    def ada_cevre(self, offset=0.0, sessiz=False): return self.helper.ada_cevre(offset=offset, sessiz=sessiz)
    def apf(self, rov_id): return self.helper.apf(rov_id)
    def apf_guncelle_tum(self): return self.helper.apf_guncelle_tum()
    def apf_temizle(self, rov_id=None): return self.helper.apf_temizle(rov_id)
    def formasyon(self, *args, **kwargs): return self.helper.formasyon(*args, **kwargs)
    def formasyon_sec(self, *args, **kwargs): 
        
        return self._executor.submit(self.helper._formasyon_sec_impl, *args, **kwargs)
    

    def _hedef_gorsel_olustur(self, x, y, z, id=None, debug=True): return self.helper.hedef_gorsel_olustur(x, y, z, id=id, debug=debug)
    def hedef_sil(self, id=None): return self.helper.hedef_sil(id=id)
    def debug_hedefleri_temizle(self): return self.helper.debug_hedefleri_temizle()
    def minimap(self, *args, **kwargs): return self.helper.minimap(*args, **kwargs)
    
    # ... Diğer wrapperlar (vektor, engel_bul vb.) ...
    def engel_bul(self, rov_id, menzil=None, debug=False): return self.helper.engel_bul(rov_id, menzil, debug)
    def vektor(self, *args, **kwargs): return self.helper.vektor(*args, **kwargs)
    def get_100_samples(self, *args, **kwargs): return self.helper.get_100_samples(*args, **kwargs)
    def gat_veri_uret(self): return self.helper.gat_veri_uret()
    def manuel_kontrol_all(self, aktif=True):
        for rov in self.rovs:
            if hasattr(rov, 'gnc'): rov.gnc.manuel_kontrol = aktif

    def find_leader_info(self,*args,**kwargs): return self.helper.find_leader_info(*args,**kwargs)

# ==========================================
# 2. TEMEL GNC SINIFI (SADELEŞTİRİLMİŞ)
# ==========================================
class TemelGNC:
    """Doğrudan ROV'a bağlı çalışan GNC birimi. Modem ve Rehber kaldırıldı."""
    
    def __init__(self, rov_entity, filo_ref=None):
        self.rov = rov_entity
        self.filo_ref = filo_ref
        
        self.hedef = None 
        self.hiz_limiti = 100.0 
        self.manuel_kontrol = False
        self.ai_aktif = True 
        self.gps_sinyal = 1  # GPS sinyali varsayilan aktif
        
        self.temel_gnc_helper = TemelGNCHelper(rov_entity, filo_ref, self)

        self.mod = 1
        self.batma_orani = 0

    @property
    def gps(self):
        """ROV'un guncel GPS koordinatini (sim koordinat sisteminde) doner: (x, y, z)"""
        if self.rov is None:
            return None
        return Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z)

    def hedef_atama(self, x, y, z):
        self.hedef = Vec3(x, y, z)
    
    def guncelle(self, gat_kodu=None):
        # GPS sinyal kontrolu: ROV'un en ust noktasi su yuzeyinden 5m+ asagidaysa sinyal=0
        if self.rov:
            
            derinlik = self.filo_ref.get(self.rov.id, 'gps')[2]  # Z koordinatı derinlik olarak varsayılmıştır
            if derinlik < -5.0:
                self.gps_sinyal = 0
            else:
                self.gps_sinyal = 1
        
        if self.temel_gnc_helper:
            return self.temel_gnc_helper.guncelle(gat_kodu=gat_kodu)
    
    # Yardımcı metodlar
    def _hedefe_varis_islemleri(self, fark):
        # Çoklu nokta geçişi
        if self.filo_ref and self._siradaki_noktaya_gec():
            return
        # Rota bitti
        self.hedef = None
        self.rov.velocity = Vec3(0, 0, 0)
        self.ai_aktif = False

    def _siradaki_noktaya_gec(self):
        try:
            my_id = self.rov.id
            nokta_listesi = self.filo_ref._git_nokta_listesi.get(my_id)
            mevcut_indeks = self.filo_ref._git_mevcut_nokta_indeksi.get(my_id, 0)
            
            if nokta_listesi and mevcut_indeks + 1 < len(nokta_listesi):
                yeni_indeks = mevcut_indeks + 1
                nxt = nokta_listesi[yeni_indeks]
                self.filo_ref._git_mevcut_nokta_indeksi[my_id] = yeni_indeks
                
                # Z koordinatını koru veya tamamla
                curr_z = self.hedef.z if self.hedef else Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z)[2]
                self.hedef = Vec3(nxt[0], nxt[1], curr_z)
                return True
            elif nokta_listesi:
                # Rota tamamlandı, listeyi temizle
                self.filo_ref._git_nokta_listesi.pop(my_id, None)
        except Exception as e:
            if self.filo_ref:
                self.filo_ref.ds = e
            pass
        return False


# Export sınıfları
__all__ = ['Filo', 'TemelGNC', 'Koordinator', 'SafeDict', 'DamageSystem']
