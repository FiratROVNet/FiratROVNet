"""
GNC Module
The main file for Mathematical calculations, geometric operations, and complex logic for Filo and TemelGNC classes.
"""

import builtins
import queue
import threading
import math
import random
import numpy as np  # type: ignore[import]
from ursina import *  # type: ignore[import]
from ursina import Vec3, color, time, window, camera # type: ignore[import]
from panda3d.bullet import BulletWorld  # type: ignore[import]

# Yerel modül importları
from ..config import cfg, GATLimitleri, SensorAyarlari, HareketAyarlari, BasitKalmanFiltresi, HavuzAyarlari  # type: ignore[import]
from ..kutuphane.helper.gnc_helper.mixins.formation import Formasyon  # type: ignore[import]
from ..hull import HullManager  # type: ignore[import]
from FiratROVNet.kutuphane.helper.gnc_helper import FiloHelper, TemelGNCHelper  # type: ignore[import]
from FiratROVNet.kutuphane.moduls import ModulYardimcisi, MotorDuzeni, PID, BARUI  # type: ignore[import]
# Lazy import: FiratAnalizci circular import problemini önlemek için _basla_gat_modeli içinde import edilir

# Modüler yapı - GNC subpackage
from .init import FiloInitMixin  # type: ignore[import]
from .koordinator import Koordinator, SafeDict  # type: ignore[import]
from .damage_system import DamageSystem  # type: ignore[import]
from .logs import LogSystem  # type: ignore[import]
from .hull_information import HullInformationManager  # type: ignore[import]
from ..animations import GelismisParcacik, entity_patlat  # type: ignore[import]
from ..camera_manager import CameraManager  # type: ignore[import]
from ..lider_sec import LeaderManager  # type: ignore[import]

import inspect
import os






from FiratROVNet.kutuphane.moduls.profiler import Profiler
# ==========================================
# 1. FİLO (ROV FİLO YÖNETİCİSİ)
# ==========================================
class Filo(FiloInitMixin):
    def __init__(self, ortam_ref=None):
        # Temel Referanslar
        self.ortam_ref = ortam_ref
        self.hull_manager = HullManager(self)
        self._command_queue = queue.Queue()
        self._main_thread_id = threading.get_ident()
        self.helper = FiloHelper(self)
        self.modul = ModulYardimcisi(self)
        self.motor_duzeni = MotorDuzeni(self)
        self.pid = PID()
        # PID barlarinin acilis degerleri (burayi degistirerek varsayilan baslangici ayarlayabilirsin)
        self.pid_default_params = {"Kp": 0.0, "Ki": 0.0, "Kd": 0.0}
        self.pid_params = dict(self.pid_default_params)
        self.pid_ui = BARUI()
        # Sorumlu Sistemler (ModülerYapı)
        self.damage_system = DamageSystem(filo_ref=self)
        self.camera_manager = CameraManager(filo_ref=self)
        self.leader_manager = LeaderManager(filo_ref=self)
        self.log_system = LogSystem()
        self.hull_info_manager = HullInformationManager(filo_ref=self)
        
        # Hedef ve Formasyon Durumu
        self.asil_hedef = None
        self.hedef_gorsel = None
        self.hedef_pozisyon: tuple | None = None
        
        # Formasyon Yönetimi
        self.aktif_formasyon = {}
        self.motorlar_bv={}
        self.motorlar={}

        self._formasyon_id_pool = list(range(len(Formasyon.TIPLER)))
        random.shuffle(self._formasyon_id_pool)
        self._formasyon_hedefleri = {}
        
        # 🔹 IGNORE_TUPLE CACHE (FPS Optimization)
        # Raycast ignore listesini frame başında bir kere hesapla
        self._ignore_tuple_cache: tuple = ()  # type: ignore[assignment]
        self._ignore_tuple_last_rov_count = 0
        
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
        self.yeni_pozisyonlar = None
        
        # === GAT Modeli ve Navigasyon Kuyruğu ===
        self._basla_gat_modeli()
        self.nav_queue: dict = {}               # {g_id: [{'pos': (x,y,z), 'id': n}, ...]}
        self.current_target_id: dict = {}       # {g_id: id}
        self.grup_hedefleri: dict = {}          # {g_id: (x, y, z)} aktif gorev hedefi
        self.target_counter = 0           # Her tıklamada artan benzersiz ID sayacı

        self.mevcut_rov_sayisi=0
        
        # Ortam referansını ayarla ve başlat
        self.world = BulletWorld()
        ortam = self.ortam_ref
        if ortam:
            # Ursina Entity.update() yerine tek merkezden (guncelle_hepsi) guncelleme
            # yapabilmek için ortamı "central update" moduna al.
            # (Örn: ROV.update içindeki sensör çağrısı bu modda devre dışı kalır.)
            try:
                setattr(ortam, 'central_update', True)
            except Exception:
                pass
            ortam.filo = self
            self._baslatma_tamamla()
    
    # 🔹 IGNORE TUPLE CACHE OPTIMIZASYON
    def _build_ignore_tuple(self):
        return super()._build_ignore_tuple()

    def _hazirla_global_ignore_listesi(self, rov_sayisi):
        return super()._hazirla_global_ignore_listesi(rov_sayisi)

    # ============================================================
    # KURULUM VE SİSTEM YÖNETİMİ (SADELEŞTİRİLMİŞ)
    # ============================================================
    
    def _baslatma_tamamla(self):
        sonuc = super()._baslatma_tamamla()
        self._init_pid_ui()
        self.toggle_pid_ui(True)
        return sonuc
            

    # ============================================================
    # KURULUM VE SİSTEM YÖNETİMİ (SADELEŞTİRİLMİŞ)
    # ============================================================

    def BlueROV2_motor_konfigurasyonu(self, rov):
        return self.motor_duzeni.BlueROV2_motor_konfigurasyonu(rov)



# ==================== KAMERA VE YOLO METOTLARI ====================

    def yolo_baslat(self, rov_id, model_path='yolov8n.pt', islem_hizi=3):
        """Seçili ROV kamerasında YOLO nesne tespitini başlatır."""
        return self.camera_manager.yolo_baslat(rov_id, model_path, islem_hizi)

    def yolo_durdur(self, rov_id):
        """Seçili ROV'un YOLO sistemini kapatır."""
        return self.camera_manager.yolo_durdur(rov_id)





    def _euler_deg_to_direction(self, rot_deg: Vec3, v=Vec3(0, 1, 0)):
        return self.modul._euler_deg_to_direction(rot_deg, v=v)

    def dunya_to_yerel_vektor(self, dunya_vektor: Vec3, rotasyon: Vec3) -> Vec3:
        return self.modul.dunya_to_yerel_vektor(dunya_vektor, rotasyon)

    def tum_motor_bv_kutuphanelerini_guncelle(self):
        return self.motor_duzeni.tum_motor_bv_kutuphanelerini_guncelle()
    def tum_motorlarin_guclerini_hesapla(self, rov_id=0, hedef_vektor_dunya: Vec3 = Vec3(0.0, 0.0, 0.0), guc: float = 0.0):
            return self.modul.tum_motorlarin_guclerini_hesapla(
                rov_id=rov_id,
                hedef_vektor_dunya=hedef_vektor_dunya,
                guc=guc,
            )


    def yaw_gucleri_hesapla(self, rov=None, hedef_vektor_dunya: Vec3 = Vec3(0.0, 0.0, 0.0), guc_orani: float = 0.0):
            return self.modul.yaw_gucleri_hesapla(
                rov=rov,
                hedef_vektor_dunya=hedef_vektor_dunya,
                guc_orani=guc_orani,
            )

        
    def roll_koru(self, rov=None, guc_orani: float = 1.0):
            return self.modul.roll_koru(rov=rov, guc_orani=guc_orani)

    def pitch_koru(self, rov=None, guc_orani: float = 1.0):
            return self.modul.pitch_koru(rov=rov, guc_orani=guc_orani)

    def yaw(self,rov,guc:float=0.1):
        return self.modul.yaw(rov, guc=guc)



    def roll(self,rov,guc:float=0.1):
        return self.modul.roll(rov, guc=guc)


    def pitch(self,rov,guc:float=0.1):
        return self.modul.pitch(rov, guc=guc)


    def _on_pid_bar_change(self, name: str, value: float):
        if name not in self.pid_params:
            return
        self.pid_params[name] = float(value)
        # Canli konsolda filo.pid.Kp/Ki/Kd degerlerinin anlik gorunmesi icin
        # sozlukten PID nesnesine hemen senkronla.
        setattr(self.pid, name, self.pid_params[name])

    def _init_pid_ui(self):
        self.pid_params['Kp'] = float(self.pid_default_params.get('Kp', 0.0))
        self.pid_params['Ki'] = float(self.pid_default_params.get('Ki', 0.0))
        self.pid_params['Kd'] = float(self.pid_default_params.get('Kd', 0.0))
        self.pid.Kp = self.pid_params['Kp']
        self.pid.Ki = self.pid_params['Ki']
        self.pid.Kd = self.pid_params['Kd']

        if not self.pid_ui.sliders:
            self.pid_ui.create_bar(
                name='Kp',
                min_value=0,
                max_value=0.1,
                default=self.pid_default_params['Kp'],
                position=(0.0, 0.07),
                callback=lambda v: self._on_pid_bar_change('Kp', v),
            )
            self.pid_ui.create_bar(
                name='Ki',
                min_value=0,
                max_value=0.1,
                default=self.pid_default_params['Ki'],
                position=(0.0, -0.015),
                callback=lambda v: self._on_pid_bar_change('Ki', v),
            )
            self.pid_ui.create_bar(
                name='Kd',
                min_value=0,
                max_value=0.1,
                default=self.pid_default_params['Kd'],
                position=(0.0, -0.10),
                callback=lambda v: self._on_pid_bar_change('Kd', v),
            )

            # Baslangicta bar konumlarini default degerlere zorla uygula.
            self.pid_ui.set_value('Kp', self.pid_default_params['Kp'])
            self.pid_ui.set_value('Ki', self.pid_default_params['Ki'])
            self.pid_ui.set_value('Kd', self.pid_default_params['Kd'])

    def set_pid_value(self, name: str, value: float):
        if name not in self.pid_params:
            return
        self.pid_params[name] = float(value)
        setattr(self.pid, name, self.pid_params[name])
        self.pid_ui.set_value(name, float(value))

    def toggle_pid_ui(self, force: bool | None = None):
        self._init_pid_ui()
        self.pid_ui.toggle_ui(force)




    # pid_hesapla fonksiyonu için düzeltilmiş versiyon
    def pid_hesapla(self, rov, yon):
        # PID parametrelerini güncelle
        self.pid.Kp = self.pid_params["Kp"]
        self.pid.Ki = self.pid_params["Ki"]
        self.pid.Kd = self.pid_params["Kd"]
        
        # dt'yi güvenli şekilde al
        dt = getattr(rov, "dt", 0.03)
        if dt <= 0 or dt > 1.0:  # Geçersiz dt değerlerini kontrol et
            dt = 0.03
        
        # Hedef ve durum değerlerini al
        if yon == "yaw":
            orientation = rov.sensor.imu.get("orientation", {})
            durum = orientation.get("yaw", 0)
            hedef = 0
            
        elif yon == "roll":
            orientation = rov.sensor.imu.get("orientation", {})
            durum = orientation.get("roll", 0)
            hedef = 0
            
        elif yon == "pitch":
            orientation = rov.sensor.imu.get("orientation", {})
            durum = orientation.get("pitch", 0)
            hedef = 0
        else:
            return 0  # Geçersiz yon
        
        # PID hesapla
        toplam = self.pid.compute(hedef=hedef, durum=durum, dt=dt, normalize=True)
        
        return toplam
                







    def motor_sema_kaydet(self, rov=None, klasor=None, base_name="rov_motor_sema"):
        """ROV motorlarının pozisyon ve yön vektörünü toplayıp şemaya gönderir. rov verilmezse ilk ROV kullanılır."""
        ortam = getattr(self, "ortam_ref", None)
        if rov is None and ortam is not None and getattr(ortam, "rovs", None):
            rovs = [r for r in ortam.rovs if r and not (hasattr(r, "is_destroyed") and r.is_destroyed)]  # type: ignore[union-attr]
            rov = rovs[0] if rovs else None
        if rov is None:
            return None

        from .schema_export import draw_rov_motor_schema, save_rov_schema_info  # type: ignore[import]

        schema_root = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "SCHEMA")
        if klasor is None:
            klasor = schema_root
        save_dir = os.path.join(klasor, f"ROV{rov.id}")  # type: ignore[union-attr]

        # Güncel motor pozisyonu ve yönü: motor_entity değiştirildiyse onu kullan, yoksa l_pos / yon_vec
        entries = []
        for i in range(6):
            m = getattr(rov, f"m{i}", None)
            if not m:
                continue
            # Pozisyon: görsel güncellendiyse motor_entity.position, yoksa l_pos
            if getattr(m, "motor_entity", None) is not None:
                pos = m.motor_entity.position
                pos_t = (getattr(pos, "x", pos[0]), getattr(pos, "y", pos[1]), getattr(pos, "z", pos[2]))
            elif hasattr(m, "l_pos") and m.l_pos is not None:
                pos_t = (m.l_pos.x, m.l_pos.y, m.l_pos.z)
            else:
                continue
            # Yön: görsel rotasyon güncellendiyse ondan hesapla, yoksa yon_vec
            if getattr(m, "motor_entity", None) is not None:
                rot = m.motor_entity.rotation
                rx = getattr(rot, "x", rot[0])
                ry = getattr(rot, "y", rot[1])
                rz = getattr(rot, "z", rot[2])
                dir_vec = self._euler_deg_to_direction(Vec3(rx, ry, rz), v=Vec3(0, 1, 0))
                dir_t = (dir_vec.x, dir_vec.y, dir_vec.z)
            elif getattr(m, "yon_vec", None) is not None:
                v = m.yon_vec
                dir_t = (getattr(v, "x", v[0]), getattr(v, "y", v[1]), getattr(v, "z", v[2]))
            else:
                continue
            entries.append({"name": f"m{i}", "position": pos_t, "direction": dir_t})
        if not entries:
            return None
        os.makedirs(save_dir, exist_ok=True)

        # 3. Bilgileri kaydet ve PDF/Şema çizdir
        save_rov_schema_info(rov_id=rov.id, motor_entries=entries, save_dir=save_dir)  # type: ignore[union-attr]
        
        return draw_rov_motor_schema(  # type: ignore[union-attr]
            motor_entries=entries,
            save_dir=save_dir,
            world_pos=(getattr(rov, 'x', 0), getattr(rov, 'y', 0), getattr(rov, 'z', 0)),
            world_rot=(getattr(rov, 'rotation_x', 0), getattr(rov, 'rotation_y', 0), getattr(rov, 'rotation_z', 0)),
            pool_size=(HavuzAyarlari.HAVUZ_TAM_GENISLIK, HavuzAyarlari.HAVUZ_TAM_GENISLIK),
            base_name=base_name,
        )


    def motorlari_calistir(self, rov_id=0, gucler: list[float] | None = None):
        return self.modul.motorlari_calistir(rov_id=rov_id, gucler=gucler)


    def _basla_gat_modeli(self):
        """GAT modelini yükle ve başlat. Başarısız olursa disable et."""
        self.gat = None  # type: ignore[assignment]
        try:
            # Lazy import: Circular import sorununu önle
            from GAT.gat_test import FiratAnalizci  # type: ignore[import]
            self.gat = FiratAnalizci(model_yolu="rov_modeli_multi.pth")  # type: ignore[assignment]
            print("✅ GAT modeli yüklendi.")
        except Exception as e:
            print(f"⚠️ GAT modeli yüklenemedi, AI devre dışı: {e}")
            self.gat = None  # type: ignore[assignment]

    def guncelle_navigasyon_kuyrugu(self):
        """Navigasyon kuyruğu ve varış yönetimi (grup bazlı)."""
        for g_id, grup_rovs in self.g_rovs.items():
            lider_bilgi = self.find_leader_info(g_id=g_id)
            lider_id = lider_bilgi[0] if lider_bilgi else None
            if lider_id is None:
                continue

            aktif_rota = self._git_nokta_listesi.get(lider_id)
            mevcut_hedef_id = self.current_target_id.get(g_id)

            # DURUM A: Hedefe Varildi mi?
            if mevcut_hedef_id is not None and not aktif_rota:
                print(f"✅ [NAV] Grup-{g_id} hedef {mevcut_hedef_id} noktasina varildi.")
                self.hedef_sil(mevcut_hedef_id)
                self.current_target_id[g_id] = None
                self.grup_hedefleri[g_id] = None

            # DURUM B: Yeni hedefe basla mi?
            grup_kuyruk = self.nav_queue.get(g_id, [])
            if not aktif_rota and grup_kuyruk:
                next_data = grup_kuyruk.pop(0)
                target_pos = next_data['pos']
                self.current_target_id[g_id] = next_data['id']
                self.grup_hedefleri[g_id] = tuple(target_pos) if isinstance(target_pos, (list, tuple)) else target_pos

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
            
            active_rovs = [r for r in self.ortam_ref.rovs if r and not (hasattr(r, 'is_destroyed') and r.is_destroyed)]  # type: ignore[union-attr]

            if ai_aktif and self.gat:  # type: ignore[union-attr]
                try:
                    tahminler_yeni, _, _ = self.gat.analiz_et(veri)  # type: ignore[union-attr]
                    
                    # GAT predictions'ı doğru indekslere ata
                    # tahminler_yeni active_rovs sırasında predictions döndürür
                    active_idx = 0
                    for all_idx, rov in enumerate(self.ortam_ref.rovs):
                        # Destroyed/None ROV'ları atla
                        if not rov or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
                            continue
                        
                        # Active index bounds check
                        if active_idx < len(tahminler_yeni) and all_idx < len(tahminler):  # type: ignore[arg-type]
                            tahminler[all_idx] = tahminler_yeni[active_idx]
                        active_idx += 1
                except Exception as e:
                    print(f"⚠️ GAT analiz hatası: {e}")
            
            # Tahmin boyutunu ROV sayısına göre ayarla
            if len(tahminler) < len(active_rovs):  # type: ignore[arg-type]
                tahminler.extend(np.zeros(len(active_rovs) - len(tahminler), dtype=int))
                
        except Exception as e:
            print(f"❌ GAT güncelleme hatası: {e}")

    def kuvvet_uygula(self, rov_entity, yerel_kuvvet, yerel_nokta):
            """
            ROV üzerindeki spesifik bir noktaya yerel bir kuvvet uygular.
            (Panda3D matrisleri YERİNE, Filo'nun kendi _euler_deg_to_direction metodu kullanılır)
            """
            from panda3d.core import Vec3 as P3Vec
            import math
            
            # 1. FİZİK DÜĞÜMÜNÜ AL
            physics_node = getattr(rov_entity, 'physics_node', None)
            if physics_node is None:
                if hasattr(rov_entity, 'physics_np'):
                    physics_node = rov_entity.physics_np.node()
                else:
                    return 

            physics_node.setActive(True)

            # 2. YEREL KUVVETİ -> DÜNYA KUVVETİNE ÇEVİR (Senin Euler Fonksiyonunla)
            v_yerel_kuvvet = Vec3(-yerel_kuvvet[0], -yerel_kuvvet[1], yerel_kuvvet[2])
            mag = v_yerel_kuvvet.length()
            
            if mag <= 1e-6:
                return
                
            # Sadece yönü döndür ve büyüklükle çarp (Ursina'nın olası Scale bozulmalarını önler)
            v_yerel_yon = v_yerel_kuvvet.normalized()
            ursina_dunya_yon = self._euler_deg_to_direction(rov_entity.rotation, v=v_yerel_yon)
            ursina_dunya_kuvvet = ursina_dunya_yon * mag

            # 3. YEREL UYGULAMA NOKTASINI (OFFSET) -> DÜNYA OFFSETİNE ÇEVİR
            # Sağ/Sol el tork uyuşmazlığını (Ters dönme sorunu) çözmek için yerel X eksenini eksi (-) alıyoruz
            v_yerel_nokta = Vec3(-yerel_nokta[0], yerel_nokta[1], yerel_nokta[2])
            ursina_dunya_offset = self._euler_deg_to_direction(rov_entity.rotation, v=v_yerel_nokta)

            # 4. URSINA DÜNYASI -> PANDA3D(BULLET) DÜNYASI ÇEVİRİMİ (KRİTİK!)
            bullet_force = P3Vec(ursina_dunya_kuvvet.x, ursina_dunya_kuvvet.y, ursina_dunya_kuvvet.z)
            bullet_offset = P3Vec(ursina_dunya_offset.x, ursina_dunya_offset.y, ursina_dunya_offset.z)

            # 5. FİZİK MOTORUNA UYGULA
            if (math.isfinite(bullet_force.x) and math.isfinite(bullet_force.y) and math.isfinite(bullet_force.z) and
                math.isfinite(bullet_offset.x) and math.isfinite(bullet_offset.y) and math.isfinite(bullet_offset.z)):
                
                physics_node.applyForce(bullet_force, bullet_offset)

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
        return [r for r in self.ortam_ref.rovs if r and not (hasattr(r, 'is_destroyed') and r.is_destroyed)]  # type: ignore[union-attr]

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
            if rov and not (hasattr(rov, 'is_destroyed') and rov.is_destroyed):  # type: ignore[union-attr]
                positions[rov.id] = self.get(rov.id, 'gps')  # type: ignore[union-attr]
        return positions
    
    def kamera_ayarla(self, *args, **kwargs):
        """Kamera yönetimini camera_manager'a yönlendir (kamera_ekle ile aynı API)."""
        kamera_id = self.camera_manager.aktif_kamera_listesi()
        for kamera in kamera_id:
            self.camera_manager.kamera_kaldir(kamera)
        return self.camera_manager.kamera_ekle(*args, **kwargs)
    
    def kamera_kaldir(self, rov_id):
        """Kamera kaldırma işlemini camera_manager'a yönlendir."""
        return self.camera_manager.kamera_kaldir(rov_id)


    def kamera_pozisyonu(self,x,z):
        size=window.size
        genislik=size[0]
        yukseklik=size[1]

        x1=(1/2)*(2*x/genislik - 0.2)
        x2=(1/2)*(2*x/genislik + 0.2)
        z1=(1/2)*(2*z/yukseklik - 0.2)
        z2=(1/2)*(2*z/yukseklik + 0.2)
        return x1,x2,z1,z2

    def ekran(self):
        return window.size

    def yuzey_bilgileri(self):
        """
        Okyanus yüzeyinin (ocean_surface) ana kameraya göre durumunu analiz eder.
        Returns:
            dict: mesafe (vektör: kamera -> yüzey), bagil_rotasyon veya None (ortam / ocean_surface yoksa).
        """
        ortam = self.ortam_ref
        if ortam is None:
            return None
        yuzey = getattr(ortam, "ocean_surface", None)
        if yuzey is None:
            return None
        kamera = getattr(ortam, "camera", None)
        if kamera is None:
            return None
        try:
            cam_pos = kamera.world_position
            yuzey_pos = yuzey.world_position
            # Kamera -> yüzey vektörü (x, y, z)
            mesafe = (
                float(yuzey_pos.x - cam_pos.x),
                float(yuzey_pos.y - cam_pos.y),
                float(yuzey_pos.z - cam_pos.z),
            )

            def _vec3_guvenli(v):
                if isinstance(v, Vec3):
                    return v
                if isinstance(v, (list, tuple)) and len(v) >= 3:
                    return Vec3(float(v[0]), float(v[1]), float(v[2]))
                if hasattr(v, "x") and hasattr(v, "y") and hasattr(v, "z"):
                    return Vec3(float(getattr(v, "x")), float(getattr(v, "y")), float(getattr(v, "z")))
                return Vec3(0, 0, 0)

            yuzey_rot = _vec3_guvenli(getattr(yuzey, "world_rotation", getattr(yuzey, "rotation", Vec3(0, 0, 0))))
            cam_rot = _vec3_guvenli(getattr(kamera, "world_rotation", getattr(kamera, "rotation", Vec3(0, 0, 0))))
            bagil_rotasyon = (
                yuzey_rot.x - cam_rot.x,
                yuzey_rot.y - cam_rot.y,
                yuzey_rot.z - cam_rot.z,
            )

            zoom=camera.position.z

            return {
                "mesafe": mesafe,
                "bagil_rotasyon": bagil_rotasyon,
                "zoom": float(zoom),
            }
        except Exception:
            return None

    # ============================================================
    # MERKEZI TICK PARCALARI (MODULER)
    # ============================================================
    def _tick_sistem_hazirligi(self):
        return super()._tick_sistem_hazirligi()

    def _tick_navigasyon_ve_gorseller(self, tahminler):
        return super()._tick_navigasyon_ve_gorseller(tahminler)

    def _tick_lider_yonetimi(self):
        return super()._tick_lider_yonetimi()

    def _tick_rovler(self, tahminler):
        return super()._tick_rovler(tahminler)

    def _tick_sistem_guncellemeleri(self, guncelle_gorseller: bool):
        return super()._tick_sistem_guncellemeleri(guncelle_gorseller)
            
    def guncelle_hepsi(self, tahminler, guncelle_gorseller=True):
        """
        Tüm GNC sistemlerini koordineli şekilde günceller.
        guncelle_gorseller=False iken sonar/minimap/engel bulut atlanır (FPS için throttle).
        Operasyon Sırası (Önem Sırasına Göre):
        1. Sistem Hazırlığı      → Command queue işle + 🔹 Ignore tuple cache güncelle
        2. Navigasyon Kuyruğu    → Hedef yönetimi
        3. Lider Yönetimi        → Yeni lider seç & değişim yap
        4. ROV Başına İşlemler   → Hasar, GNC, Motor komutları
        5. Sistem Güncellemeleri → Sonar, Minimap, engel bulut (guncelle_gorseller=True ise)
        """
        # 🔹 IGNORE TUPLE CACHE GÜNCELLE (Frame başında bir kere)
        self._build_ignore_tuple()
        self.pid_ui.update()
        
        Profiler.start("1_sistem_hazirligi")
        self._tick_sistem_hazirligi()
        Profiler.end("1_sistem_hazirligi")

        if not self.ortam_ref:
            return

        Profiler.start("2_navigasyon")
        self._tick_navigasyon_ve_gorseller(tahminler)
        Profiler.end("2_navigasyon")

        Profiler.start("3_lider_yonetimi")
        self._tick_lider_yonetimi()
        Profiler.end("3_lider_yonetimi")

        Profiler.start("4_rovlar")
        self._tick_rovler(tahminler)
        Profiler.end("4_rovlar")


        Profiler.start("5_sistem_guncellemeleri")
        self._tick_sistem_guncellemeleri(guncelle_gorseller)
        Profiler.end("5_sistem_guncellemeleri")

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


        
    def bat_gps(self, rov_id,z):
        gps=self.get(rov_id, 'gps')
        if gps is None:
            return None
        
        x,y=gps[0],gps[1]
        return self.helper.git(rov_id=rov_id, x=x, y=y, z=z)

    def bat(self, rov_id,guc):
        rov = self.find_rov_by_id(rov_id)
        if rov is None:
            return None

        m4=getattr(rov, 'm4', None)
        m5=getattr(rov, 'm5', None)
        m6=getattr(rov, 'm6', None)
        m7=getattr(rov, 'm7', None)

        if m4 is None or m5 is None or m6 is None or m7 is None:
            return None

        m4.calistir(guc)
        m5.calistir(guc)
        m6.calistir(guc)
        m7.calistir(guc)

        menzil = GATLimitleri.ENGEL
        l3 = rov.l3
        if l3 !=-1:
            print(l3)



    # ============================================================
    # HEDEF VE HAREKET YÖNETİMİ
    # ============================================================

    def git(self, rov_id: int, x, y: float | None = None, z: float | None = None, ai: bool = True, sessiz: bool = True):
        return self.helper.git(rov_id=rov_id, x=x, y=y, z=z, ai=ai, sessiz=sessiz)  # type: ignore[arg-type]

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

    def _normalize_hedef_konumu(self, hedef):
        if not isinstance(hedef, (list, tuple, np.ndarray)) or len(hedef) < 2:
            return None
        try:
            hx = float(hedef[0])
            hy = float(hedef[1])
            hz = float(hedef[2]) if len(hedef) >= 3 and hedef[2] is not None else 0.0
        except (TypeError, ValueError):
            return None
        if not (math.isfinite(hx) and math.isfinite(hy) and math.isfinite(hz)):
            return None
        return (hx, hy, hz)

    def aktif_grup_hedefi(self, g_id: int):
        """
        Lider seçiminde kullanılacak aktif grup hedefini çözer.
        Öncelik:
        1. Grup için tutulan aktif görev hedefi
        2. Kuyruktaki ilk bekleyen hedef
        3. Eski tekil `asil_hedef` fallback'i
        """
        hedef = self._normalize_hedef_konumu(self.grup_hedefleri.get(g_id))
        if hedef is not None:
            return hedef

        grup_kuyruk = self.nav_queue.get(g_id, [])
        if grup_kuyruk:
            ilk = grup_kuyruk[0]
            if isinstance(ilk, dict):
                hedef = self._normalize_hedef_konumu(ilk.get('pos'))
                if hedef is not None:
                    return hedef

        if isinstance(self.asil_hedef, dict):
            return self._normalize_hedef_konumu(self.asil_hedef.get(g_id))
        return self._normalize_hedef_konumu(self.asil_hedef)

    def aktif_liderlik_hedefleri(self):
        """Her grup için lider seçiminde kullanılacak hedefleri döndürür."""
        hedefler = {}
        for g_id in self.g_rovs.keys():
            hedefler[g_id] = self.aktif_grup_hedefi(g_id)
        return hedefler

    def _hedef_impl(self, x, y, z, rov_id=None, ciz=True):
        return self.helper._hedef_impl(x, y, z, rov_id=rov_id, ciz=ciz)

    # ============================================================
    # VERİ ERİŞİMİ VE AYARLAR (GET/SET)
    # ============================================================

    def get(self, rov_id: int | None = None, veri_tipi: str | None = None, taraf: int | None = None, sessiz: bool = False):
        return self.helper.get(rov_id=rov_id, veri_tipi=veri_tipi, taraf=taraf, koordinator=Koordinator, sessiz=sessiz)  # type: ignore[arg-type]

    def set(self, rov_id: int | None, ayar_adi: str | None, deger) -> bool:
        if rov_id is None or ayar_adi is None:
            return False
        rid: int = rov_id  # type: ignore[assignment]
        aname: str = ayar_adi  # type: ignore[assignment]
        if not self._is_main_thread():
            self._command_queue.put(('set', (rid, aname, deger), {}))
            return True
        return self._set_impl(rid, aname, deger)

    def _set_impl(self, rov_id: int, ayar_adi: str, deger) -> bool:
        if rov_id is None or ayar_adi is None or not self.ortam_ref:
            return False
        rov = self.find_rov_by_id(rov_id)
        if not rov: return False
        
        try:
            rov.set(ayar_adi, deger)
            return True
        except Exception as e:
            self._last_error = e  # type: ignore[assignment]
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
                    self._set_impl(*args, **kwargs)  # type: ignore[arg-type]
                elif cmd == 'hedef':
                    self._hedef_impl(*args, **kwargs)
                elif cmd == 'formasyon_sec_sync':
                    done_event = kwargs.pop('_done_event', None)
                    result_box = kwargs.pop('_result_box', None)
                    try:
                        result = self.helper._formasyon_sec_impl(*args, **kwargs)
                        if isinstance(result_box, dict):
                            result_box['result'] = result
                    except Exception as e:
                        if isinstance(result_box, dict):
                            result_box['error'] = e
                        LogSystem.log_exception(e)
                    finally:
                        if done_event is not None:
                            done_event.set()
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
                                engel_count += 1  # type: ignore[assignment]
                            
                            if hasattr(poly, 'exterior'):
                                # Polygon kenarlarındaki noktaları da ekle (precision için)
                                for coord in poly.exterior.coords:
                                    all_obstacles.append([float(coord[0]), float(coord[1])])
                                    engel_count += 1  # type: ignore[assignment]
            
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
        kwargs.setdefault('sessiz', False)
        # Worker/Future yolu kapali: formasyon_sec her zaman normal akista calisir.
        if self._is_main_thread():
            return self.helper._formasyon_sec_impl(*args, **kwargs)
        # Konsol thread'inden cagri geldiginde is main-thread queue'ya birakilir
        # ve senkron olarak sonuc beklenir.
        done_event = threading.Event()
        result_box = {}
        self._command_queue.put(('formasyon_sec_sync', args, {
            '_done_event': done_event,
            '_result_box': result_box,
            **kwargs,
        }))
        done_event.wait(timeout=10.0)
        if not done_event.is_set():
            print("⚠️ [FORMASYON] formasyon_sec zaman asimina ugradi (10s).")
            return None
        if 'error' in result_box:
            err = result_box['error']
            if isinstance(err, BaseException):
                raise err
            raise RuntimeError(str(err))
        return result_box.get('result')
    

    def _hedef_gorsel_olustur(self, x, y, z, id=None, debug=True): return self.helper.hedef_gorsel_olustur(x, y, z, id=id, debug=debug)
    def hedef_sil(self, id=None): return self.helper.hedef_sil(id=id)
    def debug_hedefleri_temizle(self): return self.helper.debug_hedefleri_temizle()
    def minimap(self, *args, **kwargs): return self.helper.minimap(*args, **kwargs)
    
    # ... Diğer wrapperlar (vektor, engel_bul vb.) ...
    def engel_bul(self, rov_id, menzil=None, debug=False): return self.helper.engel_bul(rov_id, float(menzil) if menzil is not None else 20.0, debug)
    def vektor(self, *args, **kwargs): return self.helper.vektor(*args, **kwargs)
    def get_100_samples(self, *args, **kwargs): return self.helper.get_100_samples(*args, **kwargs)
    def gat_veri_uret(self): return self.helper.gat_veri_uret()
    def manuel_kontrol_all(self, aktif=True):
        for rov in self.rovs:
            if hasattr(rov, 'gnc'): rov.gnc.manuel_kontrol = aktif  # type: ignore[union-attr]

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
        return self.hull_info_manager.get_hull_100_samples(hull_output, sample_count)

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
        return self.hull_info_manager.get_hull_information(
            sample_count=sample_count,
            g_id=g_id,
            kayit=kayit,
            sessiz=sessiz,
            offset_threshold=offset_threshold,
        )
    


# ==========================================
# 2. TEMEL GNC SINIFI (SADELEŞTİRİLMİŞ)
# ==========================================
class TemelGNC:
    """Doğrudan ROV'a bağlı çalışan GNC birimi. Modem ve Rehber kaldırıldı."""
    
    def __init__(self, rov_entity, filo_ref=None):
        self.rov = rov_entity
        self.filo_ref = filo_ref
        self.sensor = None
        
        self.hedef = None 
        self.manuel_kontrol = False
        self.gps_sinyal = 1  # GPS sinyali varsayilan aktif
        
        self.temel_gnc_helper = TemelGNCHelper(rov_entity, filo_ref, self)

        self.mod = 1
        self.batma_orani = 0
        self.r_bv = Vec3(0,0,0)
        self.bullet_yaw = 0.0
        self.bullet_pitch = 0.0
        self.bullet_roll = 0.0
        self._onceki_hiz: Vec3 = Vec3(0, 0, 0)
        self._onceki_bullet_yaw = 0.0
        self._onceki_bullet_pitch = 0.0
        self._onceki_bullet_roll = 0.0
        self._imu_hazir = False

    @property
    def gps(self):
        """ROV'un guncel GPS koordinatini (sim koordinat sisteminde) doner: (x, y, z)"""
        if self.rov is None:
            return None
        return Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z)

    def _sensor_ref_al(self):
        if self.rov is None:
            return None
        sensor = getattr(self.rov, "sensor", None)
        if sensor is not None:
            self.sensor = sensor
        return self.sensor

    def gps_sinyal_hesapla(self) -> int:
        gps = self.gps
        if gps is None or len(gps) < 3:
            return 0
        return 0 if float(gps[2]) < -5.0 else 1

    def sicaklik_hesapla(self):
        return None

    def _aci_farki_deg(self, yeni: float, eski: float) -> float:
        return (float(yeni) - float(eski) + 180.0) % 360.0 - 180.0

    def _vec3e_cevir(self, deger) -> Vec3:
        if isinstance(deger, Vec3):
            return Vec3(float(deger.x), float(deger.y), float(deger.z))
        if hasattr(deger, "x") and hasattr(deger, "y") and hasattr(deger, "z"):
            try:
                return Vec3(float(deger.x), float(deger.y), float(deger.z))
            except (TypeError, ValueError):
                return Vec3(0, 0, 0)
        return Vec3(0, 0, 0)

    def imu_verisi_hesapla(self) -> dict:
        raw_hiz = getattr(self.rov, "velocity", None) if self.rov is not None else None
        hiz = self._vec3e_cevir(raw_hiz)
        onceki_hiz = self._vec3e_cevir(self._onceki_hiz)

        dt = getattr(time, "dt", 0.0) or 0.0
        yaw_deg = float(getattr(self, "bullet_yaw", 0.0))
        pitch_deg = float(getattr(self, "bullet_pitch", 0.0))
        roll_deg = float(getattr(self, "bullet_roll", 0.0))

        if self._imu_hazir and dt > 1e-6:
            accel = {
                "x": (float(hiz.x) - float(onceki_hiz.x)) / dt,
                "y": (float(hiz.y) - float(onceki_hiz.y)) / dt,
                "z": (float(hiz.z) - float(onceki_hiz.z)) / dt,
            }
            gyro = {
                "x": self._aci_farki_deg(roll_deg, self._onceki_bullet_roll) / dt,
                "y": self._aci_farki_deg(pitch_deg, self._onceki_bullet_pitch) / dt,
                "z": self._aci_farki_deg(yaw_deg, self._onceki_bullet_yaw) / dt,
            }
        else:
            accel = {"x": 0.0, "y": 0.0, "z": 0.0}
            gyro = {"x": 0.0, "y": 0.0, "z": 0.0}

        yaw_rad = math.radians(yaw_deg)
        pitch_rad = math.radians(pitch_deg)
        mag = {
            "x": math.cos(yaw_rad) * math.cos(pitch_rad),
            "y": math.sin(yaw_rad) * math.cos(pitch_rad),
            "z": math.sin(pitch_rad),
        }
        orientation = {"yaw": yaw_deg, "pitch": pitch_deg, "roll": roll_deg}

        self._onceki_hiz = Vec3(float(hiz.x), float(hiz.y), float(hiz.z))
        self._onceki_bullet_yaw = yaw_deg
        self._onceki_bullet_pitch = pitch_deg
        self._onceki_bullet_roll = roll_deg
        self._imu_hazir = True

        return {
            "accel": accel,
            "gyro": gyro,
            "mag": mag,
            "orientation": orientation,
        }

    def bar_verisi_hesapla(self) -> dict:
        gps = self.gps
        derinlik_m = max(0.0, -float(gps[2])) if gps is not None and len(gps) >= 3 else 0.0
        basinc_bar = 1.0 + (derinlik_m / 10.0)
        return {
            "basinc_bar": basinc_bar,
            "derinlik": -derinlik_m,
            "derinlik_m": -derinlik_m,
        }

    def sensor_verilerini_guncelle(self):
        sensor = self._sensor_ref_al()
        gps_signal = self.gps_sinyal_hesapla()
        self.gps_sinyal = gps_signal
        if sensor is None:
            return
        sensor.guncelle(
            gps_signal=gps_signal,
            sicaklik=self.sicaklik_hesapla(),
            imu=self.imu_verisi_hesapla(),
            bar=self.bar_verisi_hesapla(),
        )

    def hedef_atama(self, x, y, z):
        self.hedef = Vec3(x, y, z)
    
    def guncelle(self, gat_kodu=None):
        filo = self.filo_ref
        # GPS sinyal kontrolu: ROV'un en ust noktasi su yuzeyinden 5m+ asagidaysa sinyal=0
        if self.rov and filo is not None:

            Profiler.start("9_r_bv_hesapla")
            # Bullet quaternion ile bakış yönünü hesapla (NumPy matris yerine → daha hızlı, gimbal lock yok)
            # Panda3D getQuat() → ROV'un dünya quaternion'ı
            # xform(local_vec) → vektörü dünya koordinatına dönüştür
            self.r_bv = filo._euler_deg_to_direction(rot_deg=self.rov.rotation, v=Vec3(0, 0, 1))
            self.sensor_verilerini_guncelle()
            Profiler.end("9_r_bv_hesapla")

        if self.temel_gnc_helper:
            Profiler.start("10_batma_orani_hesapla")
            self.batma_orani = self.batma_orani_hesapla()
            Profiler.end("10_batma_orani_hesapla")

            Profiler.start("11_temel_gnc_helper.guncelle")
            g=self.temel_gnc_helper.guncelle(gat_kodu=gat_kodu)
            Profiler.end("11_temel_gnc_helper.guncelle")
            return g
        
        
            
    def batma_orani_hesapla(self):
        """ROV govdesinin suyun icindeki yuzdesini doner (0.0 - 1.0)."""
        su_yuzeyi = 0.0
        rov_yukseklik = self.rov.scale.y * 500
        rov_y = self.rov.y
        en_ust_nokta = rov_y + (rov_yukseklik / 2)
        en_alt_nokta = rov_y - (rov_yukseklik / 2)

        if en_alt_nokta >= su_yuzeyi:
            return 0.0
        if en_ust_nokta <= su_yuzeyi:
            return 1.0

        suyun_altindaki_kisim = su_yuzeyi - en_alt_nokta
        oran = suyun_altindaki_kisim / rov_yukseklik
        return max(0.0, min(1.0, oran))

    # Yardımcı metodlar
    def _hedefe_varis_islemleri(self, fark):
        # Çoklu nokta geçişi
        if self.filo_ref is not None and self._siradaki_noktaya_gec():
            return
        # Rota bitti
        self.hedef = None
        self.rov.velocity = Vec3(0, 0, 0)

    def _siradaki_noktaya_gec(self):
        filo = self.filo_ref
        if filo is None:
            return False
        try:
            my_id = self.rov.id
            nokta_listesi = filo._git_nokta_listesi.get(my_id)
            mevcut_indeks = filo._git_mevcut_nokta_indeksi.get(my_id, 0)
            
            if nokta_listesi and mevcut_indeks + 1 < len(nokta_listesi):
                yeni_indeks = mevcut_indeks + 1
                nxt = nokta_listesi[yeni_indeks]
                filo._git_mevcut_nokta_indeksi[my_id] = yeni_indeks
                
                # Hedef derinliği kullan (varsa), yoksa mevcut derinliği koru
                target_depth = None
                if hasattr(filo, '_git_hedef_derinligi'):
                    target_depth = filo._git_hedef_derinligi.get(my_id)
                
                if target_depth is not None:
                    curr_z = target_depth
                else:
                    curr_z = self.hedef.z if self.hedef is not None else Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z)[2]  # type: ignore[union-attr]
                
                self.hedef = Vec3(nxt[0], nxt[1], curr_z)
                return True
            elif nokta_listesi:
                # Rota tamamlandı, listeyi temizle
                filo._git_nokta_listesi.pop(my_id, None)
                if hasattr(filo, '_git_hedef_derinligi'):
                    filo._git_hedef_derinligi.pop(my_id, None)
        except Exception as e:
            filo._last_error = e  # type: ignore[assignment]
        return False
    



class Sensor:
    """ROV'a bağlı sade sensör paketi."""

    def __init__(self, rov_entity, filo_ref=None, gnc_ref=None):
        self.rov = rov_entity
        self.filo_ref = filo_ref
        self.gnc_ref = gnc_ref
        self.sicaklik = None
        self.gps_signal = 1
        self.imu = {
            "accel": {"x": 0.0, "y": 0.0, "z": 0.0},
            "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
            "mag": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
        }
        self.bar = {"basinc_bar": 1.0, "derinlik": 0.0, "derinlik_m": 0.0}

    def guncelle(self, gps_signal=None, sicaklik=None, imu=None, bar=None):
        if gps_signal is not None:
            self.gps_signal = gps_signal
        self.sicaklik = sicaklik
        if imu is not None:
            self.imu = imu
        if bar is not None:
            self.bar = bar


# Export sınıfları
__all__ = ['Filo', 'TemelGNC', 'Sensor', 'Koordinator', 'SafeDict', 'DamageSystem']
