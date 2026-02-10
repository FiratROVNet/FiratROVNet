import builtins
import queue
import threading
import math
import random
import numpy as np
from ursina import *

# Yerel modül importları
from .config import cfg, GATLimitleri, SensorAyarlari, ModemAyarlari, HareketAyarlari, Formasyon, FizikSabitleri
from .iletisim import AkustikModem
from .hull import HullManager
from FiratROVNet.kutuphane.helper.gnc_helper import FiloHelper, TemelGNCHelper
import concurrent.futures
# ==========================================
# 0. YARDIMCI SINIFLAR
# ==========================================

class GelismisParcacik(Entity):
    def __init__(self, pos, tur="kivilcim", renk=color.orange):
        super().__init__(
            model='sphere' if tur != 'enkaz' else 'cube',
            position=pos,
            add_to_scene_entities=True,
            double_sided=True
        )
        self.tur = tur
        self.timer = 0

        # --- FİZİK VE GÖRSEL AYARLAR ---
        if tur == "alev":
            self.model = 'sphere'
            # Başlangıçta daha küçük başla ki büyüdüğü belli olsun
            self.scale = Vec3(0.5, 0.5, 0.5) * random.uniform(0.8, 1.5)
            self.color = color.rgba(255, 255, 100, 1) # Parlak Sarı (Opak)
            self.hiz = random.uniform(2, 6)
            self.surtunme = 0.5
            self.omur = random.uniform(1.2, 1.8) # Ömrü uzattık
            self.yercekimi = 3.0
            self.direction = Vec3(random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1)).normalized()
            self.unlit = True 
            self.billboard = True 

        elif tur == "enkaz":
            self.model = 'cube'
            # MOLOZLAR DEVASA VE ASİMETRİK
            # x, y, z scale değerleri farklı olsun ki yamuk parçalar oluşsun
            sx = random.uniform(0.5, 1.5)
            sy = random.uniform(0.5, 1.5)
            sz = random.uniform(0.5, 1.5)
            self.scale = Vec3(sx, sy, sz) 
            self.color = renk 
            self.hiz = random.uniform(8, 25) # Daha uzağa fırlasınlar
            self.surtunme = 0.8 # Suda daha az yavaşlasınlar
            self.omur = random.uniform(3.0, 5.0) # Ekranda daha uzun kalsın
            self.yercekimi = -15 # Daha ağır oldukları için hızlı düşsünler
            self.rotation_vel = Vec3(random.uniform(-700,700), random.uniform(-700,700), random.uniform(-700,700))
            self.direction = Vec3(random.uniform(-1,1), random.uniform(0,1), random.uniform(-1,1)).normalized()

        elif tur == "huzme": 
            self.model = 'cube'
            self.scale = Vec3(0.05, 0.05, 1.5) 
            self.color = color.rgba(255, 255, 240, 1) 
            self.hiz = random.uniform(30, 60)
            self.surtunme = 3.0
            self.omur = random.uniform(0.2, 0.4)
            self.yercekimi = 0
            self.direction = Vec3(random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1)).normalized()
            self.look_at(self.position + self.direction) 
            self.unlit = True

        elif tur == "duman": 
            self.model = 'sphere'
            self.scale = Vec3(2.0, 2.0, 2.0) * random.uniform(0.8, 1.5)
            self.color = color.rgba(40, 40, 40, 0.8) 
            self.hiz = random.uniform(1, 5)
            self.surtunme = 1.0
            self.omur = random.uniform(2.0, 4.0)
            self.yercekimi = 4 
            self.direction = Vec3(random.uniform(-1,1), random.uniform(0, 1), random.uniform(-1,1)).normalized()
            self.billboard = True 

        elif tur == "kivilcim":
            self.scale = Vec3(0.2, 0.2, 0.2) 
            self.color = color.rgba(255, 200, 50, 1) 
            self.hiz = random.uniform(15, 35)
            self.surtunme = 2.5
            self.omur = random.uniform(0.5, 1.0)
            self.yercekimi = -5 
            self.direction = Vec3(random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1)).normalized()
            self.unlit = True

    def update(self):
        dt = time.dt
        self.timer += dt
        ratio = self.timer / self.omur 

        # --- HAREKET ---
        self.position += self.direction * self.hiz * dt
        self.position.y += self.yercekimi * dt
        self.hiz = lerp(self.hiz, 0, dt * self.surtunme)

        # --- GÖRSEL DEĞİŞİM ---
        
        if self.tur == "alev":
            # BÜYÜME: Çok hızlı genişle
            buyume = dt * 12.0 
            self.scale += Vec3(buyume, buyume, buyume)
            
            # RENK GEÇİŞİ: Daha canlı ve belirgin
            if ratio < 0.3:
                # %0-30: Parlak Sarıdan Yoğun Turuncuya (Hala çok opak)
                self.color = lerp(
                    color.rgba(255, 255, 50, 1.0), 
                    color.rgba(255, 100, 0, 0.9), 
                    ratio * 3.3
                )
            elif ratio < 0.7:
                # %30-70: Turuncudan Kan Kırmızısına
                norm_ratio = (ratio - 0.3) * 2.5
                self.color = lerp(
                    color.rgba(255, 100, 0, 0.9), 
                    color.rgba(180, 20, 0, 0.6), 
                    norm_ratio
                )
            else:
                # %70-100: Kırmızıdan Siyah Dumana
                norm_ratio = (ratio - 0.7) * 3.3
                self.color = lerp(
                    color.rgba(180, 20, 0, 0.6), 
                    color.rgba(0, 0, 0, 0.0), 
                    norm_ratio
                )

        elif self.tur == "huzme":
            self.scale_z = lerp(self.scale_z, 0, dt * 8)
            self.alpha = lerp(1, 0, ratio)

        elif self.tur == "duman":
            self.scale += Vec3(dt*3, dt*3, dt*3)
            self.alpha = lerp(0.7, 0, ratio) 
        
        elif self.tur == "kivilcim":
            self.scale *= (1 - dt * 1.5)
            self.alpha = lerp(1, 0, ratio)

        elif self.tur == "enkaz":
            self.rotation += self.rotation_vel * dt 
            self.alpha = lerp(1, 0, ratio * ratio * ratio) # Son ana kadar görünür kalsın

        if self.timer > self.omur or self.scale_x < 0.01 or self.alpha <= 0.01:
            destroy(self)


class Koordinator:
    """Simülasyon (X:Sağ, Y:İleri, Z:Derinlik) <-> Ursina (X, Y:Yukarı, Z:İleri) dönüşümü."""
    @staticmethod
    def sim_to_ursina(sim_x, sim_y, sim_z):
        from FiratROVNet.kutuphane.helper.simulasyon_helper import sim_to_ursina as _stou
        return _stou(sim_x, sim_y, sim_z)

    @staticmethod
    def ursina_to_sim(u_x, u_y, u_z):
        from FiratROVNet.kutuphane.helper.simulasyon_helper import ursina_to_sim as _utot
        return _utot(u_x, u_y, u_z)

# ==========================================
# 1. FİLO (ROV FİLO YÖNETİCİSİ)
# ==========================================
class Filo:
    def __init__(self):
        # Temel Referanslar
        self.ortam_ref = None
        self.hull_manager = HullManager(self)
        self._command_queue = queue.Queue()
        self._main_thread_id = threading.get_ident()
        self.helper = FiloHelper(self)
        
        # Hedef ve Formasyon Durumu
        self.asil_hedef = None
        self.orijinal_lider_id = 0
        self.hedef_gorsel = None
        self.hedef_pozisyon = None
        
        # Formasyon Yönetimi
        self.aktif_formasyon = None
        self._formasyon_id_pool = list(range(len(Formasyon.TIPLER)))
        random.shuffle(self._formasyon_id_pool)
        self._formasyon_hedefleri = {}
        
        # Navigasyon Verileri
        self._git_nokta_listesi = {}      # {rov_id: [[x,y], ...]}
        self._git_mevcut_nokta_indeksi = {}
        self._git_isaret = {}
        self._git_hedef_yaw = {}
        self._rov_hedefleri = {}          # {rov_id: (x, y, z)}
        
        # Debug ve Kamera
        self._debug_noktalari = []
        self.aktif_kameralar = {}

        # Ayarlar
        self._git_hedef_mesafe_toleransi = 2.0
        self._maksimum_yaw_donme_hizi = 30.0
        self._git_maksimum_yaw_donme_hizi = 45.0
        self._formasyon_yaw_senkronizasyon_mesafesi = 5.0
        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

    # ============================================================
    # KURULUM VE SİSTEM YÖNETİMİ (SADELEŞTİRİLMİŞ)
    # ============================================================

    @property
    def rovs(self):
        """
        self.sistemler yerine doğrudan ortamdaki canlı ROV'ları döndürür.
        """
        if not self.ortam_ref or not hasattr(self.ortam_ref, 'rovs'):
            return []
        return [r for r in self.ortam_ref.rovs if r and not (hasattr(r, 'is_destroyed') and r.is_destroyed)]

    def otomatik_kurulum(self, rovs, lider_id=0, ortam_ref=None, baslangic_hedefleri=None, modem_ayarlari=None, sensor_ayarlari=None):
        """
        ROV'lara GNC sistemini entegre eder. self.sistemler listesi kullanılmaz.
        GNC nesnesi doğrudan rov.gnc içine gömülür.
        """
        if ortam_ref: self.ortam_ref = ortam_ref
        elif rovs: self.ortam_ref = rovs[0].environment_ref

        self.orijinal_lider_id = lider_id
        tum_modemler = {}
        
        # Varsayılan ayarları yükle
        modem_cfg_lider = modem_ayarlari.get('lider', ModemAyarlari.LIDER) if modem_ayarlari else ModemAyarlari.LIDER
        modem_cfg_takipci = modem_ayarlari.get('takipci', ModemAyarlari.TAKIPCI) if modem_ayarlari else ModemAyarlari.TAKIPCI

        for i, rov in enumerate(rovs):
            if not rov: continue
            
            # 1. Rol ve Ayarlar
            is_leader = (i == lider_id)
            rov.set("rol", 1 if is_leader else 0)
            
            # Sensör ayarlarını uygula (Varsa)
            if sensor_ayarlari:
                target_cfg = sensor_ayarlari.get('lider' if is_leader else 'takipci', {})
                for k, v in target_cfg.items(): rov.set(k, v)

            # 2. Modem Kurulumu
            cfg_to_use = modem_cfg_lider if is_leader else modem_cfg_takipci
            rov.modem = AkustikModem(rov_id=i, **cfg_to_use)
            tum_modemler[i] = rov.modem
            
            # 3. GNC Sistemini ROV'a Bağla
            rov.gnc = TemelGNC(rov, rov.modem, filo_ref=self)
            
            # 4. Başlangıç Hedefi
            if baslangic_hedefleri and i in baslangic_hedefleri:
                h = baslangic_hedefleri[i]
                x, y, z = h[0], h[1], h[2] if len(h) > 2 else 0
                rov.gnc.hedef_atama(x, y, z)
                if is_leader: self.asil_hedef = Vec3(x, y, z)
            elif is_leader and not baslangic_hedefleri:
                # Lider için varsayılan hedef
                rov.gnc.hedef_atama(40, 0, 60)
                self.asil_hedef = Vec3(40, 0, 60)

        # Rehber dağıtımı
        for rov in rovs:
            if hasattr(rov, 'gnc'):
                rov.gnc.rehber_guncelle(tum_modemler)
        
        return tum_modemler
    
    def _get_all_rovs_positions(self):
        """Tüm ROV'ların güncel konumlarını döner. {rov_id: (x, y, z)} formatında."""
        if not self.ortam_ref or not hasattr(self.ortam_ref, 'rovs'):
            return {}
        positions = {}
        for rov in self.ortam_ref.rovs:
            positions.append(self.filo.get(rov.id, 'gps'))
    def guncelle_hepsi(self, tahminler):
        """
        Tüm GNC sistemlerini günceller. 
        self.sistemler yerine doğrudan ortam.rovs üzerinden çalışır.
        """
        self._process_command_queue()
        
        if not self.ortam_ref: return

        # Canlı ROV'ların kopyasını al (Döngü sırasında silinirse çökmemesi için)
        mevcut_rovlar = [r for r in list(self.ortam_ref.rovs) if r and not (hasattr(r, 'is_destroyed') and r.is_destroyed)]
        
        for i, rov in enumerate(mevcut_rovlar):
            if not hasattr(rov, 'gnc') or rov.gnc is None:
                continue

            # GAT Tahmini (Liste sınır kontrolü ile)
            gat_kodu = tahminler[i] if tahminler is not None and i < len(tahminler) else None
            
            try:
                # GNC güncelle
                rov.gnc.guncelle(gat_kodu=gat_kodu)
            except Exception as e:
                # Patlama anındaki hataları sessizce geç
                if "!is_empty()" not in str(e):
                    print(f"⚠️ [FİLO] ROV-{rov.id} GNC Hatası: {e}")

        # Minimap Güncellemesi
        if self.ortam_ref.minimap:
            try:
                self.ortam_ref.minimap.gorsel_guncelle()
            except: pass

    # ============================================================
    # PATLAMA VE SİLME YÖNETİMİ
    # ============================================================
    def entity_patlat(self, hedef_entity, parca_sayisi=60):
            """
            Daha büyük, daha vahşi ve moloz dolu patlama.
            """
            if not hedef_entity or not hasattr(hedef_entity, 'world_position'): return

            pos = hedef_entity.world_position
            renk = getattr(hedef_entity, 'color', color.orange)
            
            # 1. MERKEZ PARLAMASI (Flash)
            flash = Entity(model='sphere', position=pos, scale=1.0, color=color.white, unlit=True, add_to_scene_entities=True)
            flash.animate_scale(25, duration=0.15, curve=curve.out_expo)
            flash.animate_color(color.rgba(255, 220, 100, 0), duration=0.3)
            destroy(flash, delay=0.35)

            # 2. ŞOK DALGASI
            shock = Entity(model='quad', texture='circle', position=pos, scale=1, rotation_x=90, 
                        color=color.rgba(255, 255, 255, 0.6), unlit=True, add_to_scene_entities=True)
            shock.animate_scale(30, duration=0.4, curve=curve.out_quad)
            shock.animate_color(color.clear, duration=0.4)
            destroy(shock, delay=0.45)

            # 3. ALEV TOPLARI (Daha yoğun)
            # parca_sayisi * 0.6 kadar alev çıkacak (Eskiden 0.4 idi)
            for _ in range(int(parca_sayisi * 0.6)):
                GelismisParcacik(pos=pos, tur="alev")

            # 4. ENKAZ (MOLOZ) - SAYI ARTTIRILDI
            # Sabit 25-30 adet büyük moloz fırlat
            for _ in range(25):
                GelismisParcacik(pos=pos, tur="enkaz", renk=renk)

            # 5. IŞIK HÜZMELERİ
            for _ in range(int(parca_sayisi * 0.3)):
                GelismisParcacik(pos=pos, tur="huzme")

            # 6. DUMAN
            for _ in range(int(parca_sayisi * 0.4)):
                GelismisParcacik(pos=pos, tur="duman")

            # 7. SİLME İŞLEMİ
            if hasattr(hedef_entity, 'cikar'):
                print(f"🔥 ROV-{getattr(hedef_entity, 'id', '?')} havaya uçtu!")
                hedef_entity.cikar()
            else:
                destroy(hedef_entity)
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
            if rov_id >= len(self.ortam_ref.rovs) or not self.ortam_ref.rovs[rov_id]: return None
        except: return None

        self._rov_hedefleri[rov_id] = (x, y, z)
        self.git(rov_id, x, y, z, ai=True)
        
        # Görselleştirme (Sadece Lider için veya debug modunda)
        rov = self.ortam_ref.rovs[rov_id]
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
        if not self.ortam_ref or rov_id >= len(self.ortam_ref.rovs): return False
        rov = self.ortam_ref.rovs[rov_id]
        if not rov: return False
        
        try:
            rov.set(ayar_adi, deger)
            return True
        except: return False

    # ============================================================
    # THREADING VE YARDIMCILAR
    # ============================================================

    def _is_main_thread(self):
        return threading.get_ident() == self._main_thread_id

    def _process_command_queue(self):
        try:
            while not self._command_queue.empty():
                cmd, args, kwargs = self._command_queue.get_nowait()
                if cmd == 'set': self._set_impl(*args, **kwargs)
                elif cmd == 'hedef': self._hedef_impl(*args, **kwargs)
                elif hasattr(self.helper, f"_{cmd}_impl"):
                    getattr(self.helper, f"_{cmd}_impl")(*args, **kwargs)
        except: pass

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

# ==========================================
# 2. TEMEL GNC SINIFI
# ==========================================
class TemelGNC:
    """Doğrudan ROV'a bağlı çalışan GNC birimi."""
    def __init__(self, rov_entity, modem, filo_ref=None):
        self.rov = rov_entity
        self.modem = modem
        self.filo_ref = filo_ref
        self.hedef = None 
        self.hiz_limiti = 100.0 
        self.manuel_kontrol = False
        self.ai_aktif = True 
        
        self.helper = TemelGNCHelper(rov_entity, filo_ref, self)

    def hedef_atama(self, x, y, z):
        self.hedef = Vec3(x, y, z)

    def rehber_guncelle(self, rehber):
        if self.modem: self.modem.rehber_guncelle(rehber)
    
    def guncelle(self, gat_kodu=None):
        if self.helper:
            return self.helper.guncelle(gat_kodu=gat_kodu)
    
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
        except: pass
        return False