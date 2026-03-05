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
from ..config import cfg, GATLimitleri, SensorAyarlari, HareketAyarlari, FizikSabitleri, Hidrodinamik, BasitKalmanFiltresi, HavuzAyarlari
# Motor sınıfı importu
from .motor import Motor
from ..kutuphane.helper.gnc_helper.mixins.formation import Formasyon
from ..hull import HullManager
from FiratROVNet.kutuphane.helper.gnc_helper import FiloHelper, TemelGNCHelper
import concurrent.futures
from FiratROVNet.lider_sec import liderlik_secimini_baslat
# Lazy import: FiratAnalizci circular import problemini önlemek için _basla_gat_modeli içinde import edilir

# Modüler yapı - GNC subpackage
from .koordinator import Koordinator, SafeDict
from .damage_system import DamageSystem
from .logs import LogSystem
from .hull_information import HullInformationManager
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
        # Motorlar listesi (her filo için erişilebilir)
        self.motorlar = {}  # Ör: self.motorlar.append(Motor(...))
        # Temel Referanslar
        self.ortam_ref = ortam_ref
        self.hull_manager = HullManager(self)
        self._command_queue = queue.Queue()
        self._main_thread_id = threading.get_ident()
        self.helper = FiloHelper(self)
        self.Motor=Motor
        self.motorlar_bv={}
        
        # Sorumlu Sistemler (ModülerYapı)
        self.damage_system = DamageSystem(filo_ref=self)
        self.camera_manager = CameraManager(filo_ref=self)
        self.leader_manager = LeaderManager(filo_ref=self)
        self.log_system = LogSystem()
        self.hull_info_manager = HullInformationManager(filo_ref=self)
        
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
        

        # 1. FİZİK DÜNYASI OLUŞTURULDU
        self.world = BulletWorld()
        #self.world.setGravity(Vec3(0, -9.81, 0)) # Su altı için -2.0 gibi düşük de yapılabilir
        
        if self.ortam_ref:
            self.ortam_ref.filo = self
            self._baslatma_tamamla()

    def _baslatma_tamamla(self):
            """ROV'lar için fiziksel gövdeleri ve motorları kurar."""
            from panda3d.bullet import BulletRigidBodyNode, BulletBoxShape
            from panda3d.core import Vec3

            for rov in self.ortam_ref.rovs:
                rov.gnc = TemelGNC(rov, self)

                if self.motorlar.get(rov.id) is None:
                    self.motorlar[rov.id] = []
                
                # A. Fiziksel Düğüm (RigidBody) — kütle ve sönümleme: config.Hidrodinamik
                node = BulletRigidBodyNode(f"ROV_{rov.id}")
                node.setMass(Hidrodinamik.KUTLE)
                node.setLinearDamping(Hidrodinamik.LINEAR_DAMPING)
                #node.setAngularDamping(Hidrodinamik.ANGULAR_DAMPING) 
                
                # Çarpışma Şekli (ROV boyutlarına göre)
                shape = BulletBoxShape(Vec3(0.5, 0.4, 0.7)) 
                node.addShape(shape)
                
                # Panda3D'ye ekle
                rov_np = render.attachNewNode(node)
                rov_np.setPos(rov.position) # Ursina pozisyonundan başlat
                self.world.attachRigidBody(node)
                #node.setGravity(Vec3(0, 0, 0)) # Bu ROV için yerçekimini sıfırla
                
                # B. Referansları Kaydet
                rov.physics_node = node
                rov.physics_np = rov_np
                

                # C. BlueROV2 benzeri 6 itki motoru (4 yatay, 2 dikey)
                #    ROV modelinde ileri yön -Z (Ursina/loader convention).
                #      - İleri:  -Z
                #      - Sağ:    +X
                #      - Yukarı: +Y
                #
                #    4 yatay motor: ROV önü (-Z) = 0°, 45° dışa (sol: -X, sağ: +X) + ileri (-Z).
                #      - Sol taraf: (-cos45, 0, -cos45)
                #      - Sağ taraf: (+cos45, 0, -cos45)
                #    Konumlar: ön z>0, arka z<0 (modelde ön -Z yönünde olduğu için ön motorlar z=+200).
                #
                #    2 dikey motor: m4, m5
                #    Motor ID: m0=ön-sol, m1=ön-sağ, m2=arka-sol, m3=arka-sağ, m4=dikey-sol, m5=dikey-sağ
                try:
                    self.BlueROV2_motor_konfigurasyonu(rov)
                    

                except Exception as e:
                    logging.warning(f"[Filo] ROV-{getattr(rov, 'id', '?')} için motor oluşturulamadı: {e}")

            self.minimap(scale=1.0)
            self.motor_sema_kaydet()
            self.tum_motor_bv_kutuphanelerini_guncelle()
            

    # ============================================================
    # KURULUM VE SİSTEM YÖNETİMİ (SADELEŞTİRİLMİŞ)
    # ============================================================

    def BlueROV2_motor_konfigurasyonu(self,rov):
                    # m0: ön-sol (yatay)
                    rov.m0 = Motor(rov)
                    rov.m0.ekle(koordinat=Vec3(-200.0, 0.0, 200.0), yon_vec=(90, 0.0, 45))
                    self.motorlar[rov.id].append(rov.m0)

                    # m1: ön-sağ (yatay)
                    rov.m1 = Motor(rov)
                    rov.m1.ekle(koordinat=Vec3(200.0, 0.0, 200.0), yon_vec=(90, 0.0, -45))
                    self.motorlar[rov.id].append(rov.m1)

                    # m2: arka-sol (yatay)
                    rov.m2 = Motor(rov)
                    rov.m2.ekle(koordinat=Vec3(-200.0, 0.0, -200.0), yon_vec=(90, 0.0, 135))
                    self.motorlar[rov.id].append(rov.m2)

                    # m3: arka-sağ (yatay)
                    rov.m3 = Motor(rov)
                    rov.m3.ekle(koordinat=Vec3(200.0, 0.0, -200.0), yon_vec=(90, 0.0, -135))
                    self.motorlar[rov.id].append(rov.m3)

                    # m4: dikey-sol (heave)
                    rov.m4 = Motor(rov)
                    rov.m4.ekle(koordinat=Vec3(-100, 0.0, 0.0), yon_vec=(0.0, 0, 0.0))
                    self.motorlar[rov.id].append(rov.m4)

                    # m5: dikey-sağ (heave)
                    rov.m5 = Motor(rov)
                    rov.m5.ekle(koordinat=Vec3(100, 0.0, 0.0), yon_vec=(0.0, 0, 0.0))
                    self.motorlar[rov.id].append(rov.m5)


    def _euler_deg_to_direction(self,rot_deg: Vec3,v=Vec3(0,1,0), Transpose=False):
        """
        REFERANS: schema_export.py içerisindeki mantıkla birebir aynıdır.
        Rotasyonlar tam olarak (X -> Z -> Y) sırasıyla matris çarpımıyla uygulanır.
        (0,0,0) -> (0,1,0) (dikey yukarı).
        """
        # Dereceleri radyana çevir
        rx, ry, rz = map(math.radians, [rot_deg.x, rot_deg.y, rot_deg.z])
        
        # Başlangıç: Motor dik duruyor (Azure durumu)
        v = np.array([v.x, v.y, v.z])

        # 1. X ekseninde yatır (Pitch) - rx: 90 ileri (+Z), -90 geri (-Z) yapar.
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(rx), -math.sin(rx)],
            [0, math.sin(rx), math.cos(rx)]
        ])
        
        # 2. Z ekseninde döndür (Roll/Açı) - Bu matris çizimdeki 'Ry' isimlendirmesine karşılık gelir
        # rz açısını kullanarak Y ekseni etrafındaki dönüşü simüle eder
        Ry_custom = np.array([
            [math.cos(rz), 0, math.sin(rz)],
            [0, 1, 0],
            [-math.sin(rz), 0, math.cos(rz)]
        ])
        
        # 3. Y ekseninde döndür (Yaw) - Bu matris çizimdeki 'Rz' isimlendirmesine karşılık gelir
        # ry açısını kullanarak Z ekseni etrafındaki dönüşü simüle eder
        Rz_custom = np.array([
            [math.cos(ry), -math.sin(ry), 0],
            [math.sin(ry), math.cos(ry), 0],
            [0, 0, 1]
        ])

        # Dönüşüm Zinciri (Schema dosyasındaki ile birebir aynı sıra):
        # res = Ry @ (Rz @ (Rx @ v))
        if not Transpose:   
            res = Ry_custom @ (Rz_custom @ (Rx @ v))
        else:
            res = Rx.T @ (Rz_custom.T @ (Ry_custom.T @ v))
        
        return Vec3(res[0], res[1], res[2])






    def tum_motor_bv_kutuphanelerini_guncelle(self):
            self.motorlar_bv = {} 
            for rov_id, motor_listesi in self.motorlar.items():
                rov = self.find_rov_by_id(rov_id)
                if not rov: continue
                
                # ROV'un güncel ölçek değerlerini al
                scale_v = Vec3(rov.scale_x, rov.scale_y, rov.scale_z)
                
                rov_icin_bv_listesi = []
                for motor in motor_listesi:
                    # 1. İtki Yönü (Birim Vektör - Ölçekten bağımsızdır)
                    rot = motor.motor_entity.rotation 
                    birim_vektor = self._euler_deg_to_direction(Vec3(rot.x, rot.y, rot.z))
                    motor.r_bv = birim_vektor
                    
                    # 2. GERÇEK MOMENT KOLU (Scaling Uygulanmış)
                    # Motorun modeldeki yerel pozisyonunu ROV ölçeğiyle çarpıyoruz
                    l_pos = motor.motor_entity.position # Örn: (-200, 0, 200)
                    r_real = Vec3(
                        l_pos.x * scale_v.x, 
                        l_pos.y * scale_v.y, 
                        l_pos.z * scale_v.z
                    )
                    #print(r_real,l_pos)
                    
                    # 3. GERÇEK TORK VEKTÖRÜ
                    # Artık tork, gerçek dünya metre birimleri üzerinden hesaplanıyor
                    motor.tork_bv = r_real.cross(birim_vektor).normalized()
                    
                    rov_icin_bv_listesi.append(birim_vektor)

                self.motorlar_bv[rov_id] = rov_icin_bv_listesi
                rov.motorlar = motor_listesi

    def tum_motorlarin_guclerini_hesapla(self,rov_id=0,hedef_vektor:Vec3=Vec3(0.0,0.0,0.0),guc:float=0.0):
        Powers=[v.dot(hedef_vektor)*guc for v in self.motorlar_bv[rov_id]]
        return Powers

    def hedef_tork_global(self, rov, hedef_vektor:Vec3):
        rov_forward = self._euler_deg_to_direction(rov.rotation, Vec3(0, 0, 1))
        tork_global_error = rov_forward.cross(hedef_vektor).normalized()
        return tork_global_error

    def hedef_tork_local(self, rov, hedef_vektor:Vec3):
        tork_global = self.hedef_tork_global(rov, hedef_vektor)
        tork_local = self._euler_deg_to_direction(rov.rotation,v=tork_global, Transpose=True)
        return tork_local

    def tork_gucleri_hesapla(self, rov=None, hedef_vektor:Vec3=Vec3(0.0,0.0,0.0),guc_orani:float=0.0):
        tork_local = self.hedef_tork_local(rov, hedef_vektor).normalized()
        tork_gucleri = []
        for motor in rov.motorlar:
            tork = motor.tork_bv.dot(tork_local)*guc_orani
            itki = motor.r_bv.dot(hedef_vektor)*guc_orani
            tork_gucleri.append(itki)
            if tork > 1:
                print(tork)
        return tork_gucleri





    def motor_sema_kaydet(self, rov=None, klasor=None, base_name="rov_motor_sema"):
            """
            ROV motor konfigürasyonunu ve saha üzerindeki konumunu/yönünü PDF olarak kaydeder.
            """
            # Klasör Hazırlığı
            schema_root = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "SCHEMA")
            if klasor is None:
                klasor = schema_root
                
            # ROV Seçimi
            if rov is None and getattr(self, "ortam_ref", None) and getattr(self.ortam_ref, "rovs", None):
                rovs = [r for r in self.ortam_ref.rovs if r and not (hasattr(r, "is_destroyed") and r.is_destroyed)]
                rov = rovs[0] if rovs else None
                
            if rov is None:
                print("⚠️ [Filo] motor_sema_kaydet: Aktif ROV bulunamadı.")
                return None

            rov_id = getattr(rov, "id", 0)
            rov_klasor = os.path.join(klasor, f"ROV{rov_id}")
            
            # 1. Motor Verilerini Topla (Yerel Bilgiler)
            entries = []
            for i in range(6):
                m = getattr(rov, f"m{i}", None)
                if m and hasattr(m, "l_pos"):
                    # Motorun yerel rotasyonu
                    rot = (0,0,0)
                    if m.motor_entity:
                        r = m.motor_entity.rotation
                        rot = (r.x, r.y, r.z)
                    
                    entries.append({
                        "name": f"m{i}",
                        "position": (m.l_pos.x, m.l_pos.y, m.l_pos.z),
                        "rotation": rot,
                    })

            # 2. Dünya Koordinatları ve Yönelimi (Global Bilgiler)
            # Ursina'da: x, y, z konum; rotation_x, rotation_y, rotation_z açılar
            world_pos = (rov.x, rov.y, rov.z)
            world_rot = (rov.rotation_x, rov.rotation_y, rov.rotation_z)
            pool_dimensions = (HavuzAyarlari.HAVUZ_TAM_GENISLIK, HavuzAyarlari.HAVUZ_TAM_GENISLIK)

            try:
                # Importları gerçekleştir
                from .schema_export import draw_rov_motor_schema, save_rov_schema_info
                
                # Bilgi JSON dosyasını kaydet
                save_rov_schema_info(rov_id=rov_id, motor_entries=entries, save_dir=rov_klasor)
                
                # Teknik Şemayı ve Saha Haritasını Çiz
                paths = draw_rov_motor_schema(
                    motor_entries=entries, 
                    save_dir=rov_klasor, 
                    world_pos=world_pos, 
                    world_rot=world_rot, 
                    pool_size=pool_dimensions, 
                    base_name=base_name
                )
                
                print(f"✅ [GNC] ROV-{rov_id} için 3 panelli teknik rapor oluşturuldu.")
                print(f"📂 Kayıt Yeri: {paths['pdf']}")
                return paths
                
            except Exception as e:
                import logging
                logging.error(f"[Filo] motor_sema_kaydet sırasında kritik hata: {e}")
                return None

    def motorlari_calistir(self,rov_id=0,gucler:list[float]=[]):
        for i in range(len(gucler)):
            self.motorlar[rov_id][i].calistir(gucler[i])


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
                print(target_pos)
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
        """Tüm ROV gruplarını döner. ortam_ref None ise boş SafeDict döner."""
        if not self.ortam_ref or not hasattr(self.ortam_ref, 'g_rovs'):
            return SafeDict({})
        return SafeDict(self.ortam_ref.g_rovs)

    def find_rov_by_id(self, rov_id):
        """ID'si verilen ROV'u tüm gruplar içerisinde arayıp bulur."""
        # Önce ortam_ref kontrolü
        if not self.ortam_ref:
            return None
        
        # g_rovs'dan arama yap
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
        """
        self._process_command_queue()

        # 1. FİZİK ADIMINI AT (En Önemli Satır)
        dt = time.dt
        # 10 alt-adım atarak FPS düşse bile gerçek zamanı yakala
        self.world.doPhysics(dt, 10, 1.0/60.0) 

        if not self.ortam_ref:
            return

        self.guncelle_navigasyon_kuyrugu()
        self.guncelle_gorseller_ve_renkler(tahminler)

        yeni_lider_id, skor = liderlik_secimini_baslat(self, self.asil_hedef)
        self.leader_manager.guncelle_liderler(yeni_lider_id)

        # 4. ROV BAŞINA İŞLEMLER
        for rov in self.rovs:
            
            # ==========================================
            # EKSİK OLAN KISIM: FİZİK -> GÖRSEL EŞLEŞTİRME
            # ==========================================
            if hasattr(rov, 'physics_np'):
                rov.position = rov.physics_np.getPos()
                rov.quaternion = rov.physics_np.getQuat()

            # --- GAT Tahmini ve İndeks Bulma ---
            try:
                rov_idx = self.ortam_ref.rovs.index(rov)
                gat_kodu = tahminler[rov_idx] if rov_idx < len(tahminler) else 0
            except (ValueError, IndexError):
                gat_kodu = 0

            # --- Hasar Kontrol ---
            joule_esigi = 10.0
            if self.damage_system.rov_hasar_kontrol_direct(rov, joule_esigi=joule_esigi):
                self.entity_patlat(rov, parca_sayisi=80)
                continue

            # --- Sensör Güncelleme ---
            try:
                if hasattr(rov, '_guncelle_sensorler'):
                    rov._guncelle_sensorler()
            except Exception as e:
                if "!is_empty()" not in str(e):
                    print(f"⚠️ [FİLO] ROV-{rov.id} Sensör Hatası: {e}")

            # --- GNC Sistem Güncelleme ---
            try:
                if hasattr(rov, 'gnc') and rov.gnc:
                    rov.gnc.guncelle(gat_kodu=gat_kodu)
            except Exception as e:
                if "!is_empty()" not in str(e):
                    print(f"⚠️[FİLO] ROV-{rov.id} GNC Hatası: {e}")
                LogSystem.log_exception(e)

            # ==========================================
            # MOTOR HAREKET KONTROLÜ (Düzeltildi)
            # ==========================================
            if held_keys['w']:
                self.ileri(rov_id=rov.id,guc=1.0)
                print(f"ROV-{rov.id} ileri hareketi")
            
            if held_keys['a']:
                # Sola dönmek için sağ motor ileri, sol motor geri
                rov.m_sag.calistir(1.0)
                rov.m_sol.calistir(-1.0)
                
            if held_keys['d']:
                # Sağa dönmek için sol motor ileri, sağ motor geri
                rov.m_sol.calistir(1.0)
                rov.m_sag.calistir(-1.0)

        # 5. SİSTEM GÜNCELLEMELERİ
        if self.ortam_ref:
            try:
                self.ortam_ref.guncelle_sonar_cizgileri()
            except Exception as e:
                LogSystem.log_exception(e)

        self.execute_queued_commands()

        if self.ortam_ref.minimap:
            try:
                self.ortam_ref.minimap._engel_bulutu_guncelle()
                self.ortam_ref.minimap.gorsel_guncelle()
            except Exception as e:
                LogSystem.log_exception(e)

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
    
    def get_engel_ve_ada(self, sessiz=True):
        """
        🔹 Engel bulutunu (engel_bulutu) + Ada cevrelerini birleştir
        
        Engel noktalarını dinamik_engelleri_basitlestir() ile Polygon'lara dönüştür,
        ada cevresi ile birleştirip tek liste olarak döndür.
        
        Returns:
            List: Birleştirilmiş engel (centroid noktaları) ve ada cevre noktaları
            
        Kullanım:
            tum_engeller = filo.get_engel_ve_ada()
            # A* algoritmasına geç:
            path = a_star_algorithm(baslangic, hedef, tum_engeller)
        """
        try:
            all_obstacles = []
            engel_count = 0
            ada_count = 0
            
            # 1️⃣ Mevcut ada cevrelerini al
            ada_list = self.ada_cevre(sessiz=True)
            if ada_list:
                all_obstacles.extend(ada_list)
                ada_count = len(ada_list)
            
            # 2️⃣ Engel bulutunu al ve dinamik_engelleri_basitlestir ile cluster oluştur
            if self.ortam_ref and hasattr(self.ortam_ref, 'engel_bulutu'):
                engel_points = self.ortam_ref.engel_bulutu
                
                if engel_points and len(engel_points) > 0:
                    # Dinamik engelleri basitleştir (Polygon listesi döndür)
                    simplified_obstacles = self.hull_manager.dinamik_engelleri_basitlestir(
                        engel_points,
                        kume_mesafesi=25.0,
                        buffer_radius=5.0,
                        min_kume_boyutu=3
                    )
                    
                    # Polygon'lardan merkez noktaları ve sınırları çıkar
                    if simplified_obstacles:
                        for poly in simplified_obstacles:
                            if hasattr(poly, 'centroid'):
                                # Centroid (merkez) noktası
                                centroid = poly.centroid
                                all_obstacles.append([float(centroid.x), float(centroid.y)])
                                engel_count += 1
                            
                            if hasattr(poly, 'exterior'):
                                # Polygon kenarlarındaki noktaları da ekle (precision için)
                                for coord in poly.exterior.coords:
                                    all_obstacles.append([float(coord[0]), float(coord[1])])
                                    engel_count += 1
            
            if not sessiz and all_obstacles:
                print(f"✅ Birleştirilmiş engeller: {len(all_obstacles)} nokta "
                      f"(Ada: {ada_count}, Dinamik Engel: {engel_count})")
            
            return all_obstacles if all_obstacles else None
            
        except Exception as e:
            if not sessiz:
                print(f"❌ get_engel_ve_ada hatası: {e}")
            return None
    
    def apf(self, rov_id): return self.helper.apf(rov_id)
    def apf_guncelle_tum(self): return self.helper.apf_guncelle_tum()
    def apf_temizle(self, rov_id=None): return self.helper.apf_temizle(rov_id)
    def formasyon(self, *args, **kwargs): return self.helper.formasyon(*args, **kwargs)
    def formasyon_sec(self, *args, **kwargs): 
        # 🔹 ASYNC WRAPPER: Future'ı track etmek için, sonuç cache'e yazılacak
        future = self._executor.submit(self.helper._formasyon_sec_impl, *args, **kwargs)
        self.helper.formasyon_future = future  # Future tracking
        return future
    

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
    
    # ============================================================
    # 🔹 HULL CONSOLE WRAPPERS (2 Main Functions)
    # ============================================================
    
    def get_hull_100_samples(self, hull_output=None, sample_count=100):
        """
        🎯 KONSOL FONKSİYONU: Hull'dan 100 örnek al (direkt + cache)
        
        Kullanım (önerilen):
            samples = filo.get_hull_100_samples()  # Hesapla + cache + döndür
            print(len(samples))  # 100
        
        Args:
            hull_output: Özel hull dict (None ise otomatik calc)
            sample_count: Örnek sayısı (default 100)
        
        Returns:
            [[x1,y1], [x2,y2], ...] (100 nokta) veya None
        """
        result = self.hull_info_manager.get_hull_100_samples(hull_output, sample_count)
        if result is not None:
            return result
        else:
            return None

    def get_hull_information(self, sample_count=50, g_id=0, kayit=False, sessiz=True, offset_threshold=20.0):
        """
        🎯 KONSOL FONKSİYONU: Kapsamlı hull + formasyon + grup bilgisi
        
        Kullanım (önerilen):
            info = filo.get_hull_information()                # Default 50 samples
            info = filo.get_hull_information(sample_count=100) # 100 samples
            info = filo.get_hull_information(kayit=True)      # Sonucu JSON'a kaydet (append mode)
        
        Çıktı:
            {
                'hull_center': [x, y],
                'hull_samples': [[x1,y1], [x2,y2], ...],          # 50 nokta
                'formasyon_id': 'LINE',
                'formasyon_aralik': 15.2,
                'lider_rov_id': 0,
                'lider_yaw': 90.0,
                'grup_id': 0,
                'grup_bilgisi': {
                    'rov_sayisi': 6,
                    'rov_idleri': [0, 1, 2, 3, 4, 5],
                    'rovlar': [
                        {'rov_id': 0, 'pozisyon': {...}, 'batarya': 0.98, 'gnc_mode': 1, ...},
                        ...
                    ]
                }
            }
        
        Args:
            sample_count: Hull üzerine kaç örnek nokta yerleştirilecek (default 50)
            g_id: Grup ID (default 0)
            kayit: True ise sonucu JSON dosyasına kaydet (append mode - dosya varsa altına ekle) (default False)
        
        Returns:
            Dict: Tüm bilgileri içeren JSON-serializable sonuç veya None
        """
        result = self.hull_info_manager.get_hull_information(sample_count=sample_count, g_id=g_id, sessiz=sessiz, offset_threshold=offset_threshold)
        if result:
            
            # 🔹 Eğer kayit=True ise, sonucu JSON dosyasına kaydet (append mode)
            if kayit:
                success = self.hull_info_manager.save_hull_information('hull_information.json', result, sessiz=sessiz)
                if not success:
                    print("⚠️ Hull information kaydedilemedi")
            
            return result
        else:
            if not sessiz:
                print("⚠️ get_hull_information: result None")
            return None
    


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
            if self.filo_ref.get(self.rov.id, 'gps')[2] < -5.0:
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
                
                # Hedef derinliği kullan (varsa), yoksa mevcut derinliği koru
                target_depth = None
                if hasattr(self.filo_ref, '_git_hedef_derinligi'):
                    target_depth = self.filo_ref._git_hedef_derinligi.get(my_id)
                
                if target_depth is not None:
                    curr_z = target_depth
                else:
                    curr_z = self.hedef.z if self.hedef else Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z)[2]
                
                self.hedef = Vec3(nxt[0], nxt[1], curr_z)
                return True
            elif nokta_listesi:
                # Rota tamamlandı, listeyi temizle
                self.filo_ref._git_nokta_listesi.pop(my_id, None)
                if hasattr(self.filo_ref, '_git_hedef_derinligi'):
                    self.filo_ref._git_hedef_derinligi.pop(my_id, None)
        except Exception as e:
            if self.filo_ref:
                self.filo_ref.ds = e
            pass
        return False
    



# Export sınıfları
__all__ = ['Filo', 'TemelGNC', 'Koordinator', 'SafeDict', 'DamageSystem', 'Motor']
