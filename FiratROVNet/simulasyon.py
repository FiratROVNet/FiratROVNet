import math
import random
import threading
import code
import numpy as np  # type: ignore[import-not-found]
from ursina import *  # type: ignore[import-not-found]
from ursina import (  # type: ignore[reportMissingImports]
    Entity, Vec3, destroy, raycast, Text, color, time,
    camera, Mesh, window, application, mouse, Ursina, EditorCamera,
    DirectionalLight, AmbientLight, Sky, lerp,
)

# Ursina Mesh sinifi icin 'lines' modunu (GeomLines) aktif et
from panda3d.core import GeomLines
if 'lines' not in Mesh._modes:
    Mesh._modes['lines'] = GeomLines

# Yerel modül importları
from FiratROVNet.config import (  # type: ignore[import-not-found]
    SensorAyarlari, GATLimitleri, HareketAyarlari, 
    FizikSabitleri, ROVModelleri
)
from FiratROVNet.utils import sim_to_ursina, ursina_to_sim  # type: ignore[import-not-found]
from FiratROVNet.kutuphane.helper.EntityLoader import EntityLoader  # type: ignore[import-not-found]
from FiratROVNet.kutuphane.helper.simulasyon_helper import OrtamHelper  # type: ignore[import-not-found]

# ============================================================
# 1. ROV SINIFI (Mantık ve Fizik)
# ============================================================
class ROV(Entity):
    """Kesikli çizgi segment sayısı (havuz boyutu). Create-once, sonra sadece gösterme/gizleme."""
    CIZGI_HAVUZ_SEGMENT = 25

    def __init__(self, rov_id,group_id, loader_ref=None, model_key='submarine', **kwargs):
        super().__init__()
        self.motorlar = []
        self.id = rov_id
        self.environment_ref = None
        
        # Fiziksel ve Durumsal Durum
        self.velocity = Vec3(0, 0, 0)
        self.battery, self.role, self.gat_kodu = 1.0, 0, 0
        self.rotation_y = 0.0
        
        # Sensör Verileri
        self.sensor_config = SensorAyarlari.VARSAYILAN.copy()
        self.sensor_config['engel_mesafesi'] = GATLimitleri.ENGEL  # GATLimitleri'ne göre sabitle (20.0m)
        self.son_sonar_mesafesi = -1.0
        self.son_lidar_mesafeleri: dict[int, float] = {0: -1.0, 1: -1.0, 2: -1.0, 3: -1.0}  # L0: İleri, L1: Sağ, L2: Sol, L3: Dip
        self.son_lidar_noktalari: dict[int, tuple[float, float, float, str] | None] = {0: None, 1: None, 2: None, 3: None}
        self.engel_mesafesi = 999.0
        
        # Görsel Referanslar
        self.engel_cizgi: Entity | None = None  # Sonar çizgisi (havuz)
        self.lidar_cizgileri: dict[int, Entity | None] = {0: None, 1: None, 2: None, 3: None}  # Lidar çizgileri (havuz)
        self.iletisim_rovlari = {}
        self.label = None
        self.safety_zone = None

        # Pozisyonlandırma
        pos = kwargs.get('position', Vec3(0, -10, 0))
        self.position = pos if isinstance(pos, Vec3) else sim_to_ursina(*pos)

        # Görseli Yükle
        if loader_ref:
            loader_ref.setup_rov(self, model_key)

        # Grup ID: her ROV dünyaya group_id=0 (üs) ile başlar;
        # GNC/UI tarafından gruba atandığında güncellenir
        self.group_id = group_id  # Grup ID bilgisi

        # Havuz: sonar ve lidar kesikli çizgileri tek sefer oluştur; çizimde sadece güncelle/göster/gizle
        self._cizgi_havuzlari_olustur()

    def _ui_durumu_kirlet(self):
        """UI durum yazimini hizlandirmak icin ortam seviyesinde dirty tetigi yollar."""
        env = getattr(self, 'environment_ref', None)
        if env is not None and hasattr(env, 'mark_ui_state_dirty'):
            try:
                env.mark_ui_state_dirty()
            except Exception:
                pass

    @property
    def group_id(self):
        """ROV grup kimligi (tek kaynak alan)."""
        return int(getattr(self, '_group_id', 0))

    @group_id.setter
    def group_id(self, value):
        """group_id degisince grup cache'i ve UI durumu aninda guncellenir."""
        self._group_id = int(value)
        env = getattr(self, 'environment_ref', None)
        if env is not None and hasattr(env, '_invalidate_g_rovs_cache'):
            env._invalidate_g_rovs_cache()
        self._ui_durumu_kirlet()

    @property
    def grup_id(self):
        """`group_id` ile geriye dönük uyumlu alias."""
        return self.group_id

    @grup_id.setter
    def grup_id(self, value):
        """Konsoldan `grup_id` yazıldığında gerçek `group_id` alanını günceller."""
        # Konsol akisi: filo.find_rov_by_id(4).grup_id = 0 -> bu setter calisir.
        self.group_id = int(value)

    def ekle(self, ortam_ref):
        if not ortam_ref: return False
        self.environment_ref = ortam_ref
        if not hasattr(ortam_ref, 'rovs'): ortam_ref.rovs = []
        
        # ID'yi mevcut maksimumdan bir ileri ata (yeniden numaralandirma yok)
        mevcut_ids = [getattr(r, 'id') for r in getattr(ortam_ref, 'rovs', []) if r is not None and hasattr(r, 'id')]
        self.id = (max(mevcut_ids) + 1) if mevcut_ids else 0
        if hasattr(ortam_ref, 'rovs') and isinstance(ortam_ref.rovs, list):
            ortam_ref.rovs.append(self)
        
        # g_rovs cache'ini geçersiz kıl
        if hasattr(ortam_ref, '_invalidate_g_rovs_cache'):
            ortam_ref._invalidate_g_rovs_cache()
        
        self._etiket_guncelle()
        return True

    def cikar(self):
            """ROV'u siler ve tüm sistemlerden izlerini temizler."""
            if not self.environment_ref: return
            
            ortam = self.environment_ref
            silinen_id = self.id

            # --- YENİ: Filo verilerini temizle ---
            filo_attr = getattr(ortam, 'filo', None)
            if filo_attr is not None:
                filo_attr.rov_verilerini_temizle(silinen_id)

            # --- YENİ: Sonar çizgilerini (İletişim okları) temizle ---
            sonar_dict = getattr(ortam, 'sonar_cizgiler', None)
            if isinstance(sonar_dict, dict):
                for pair in list(sonar_dict.keys()):
                    if silinen_id in pair:
                        data = sonar_dict[pair]
                        if isinstance(data, tuple):
                            destroy(data[0])
                        else:
                            destroy(data)
                        sonar_dict.pop(pair, None)

            # --- YENI: Grup listesinden temizle ---
            g_rovs = getattr(ortam, 'g_rovs', None)
            if isinstance(g_rovs, dict):
                grup = g_rovs.get(self.group_id)
                if grup:
                    g_rovs[self.group_id] = [r for r in grup if r and getattr(r, 'id', None) != silinen_id]

            # 1. Referansi None yap (id korunur)
            rovs = getattr(ortam, 'rovs', None)
            if isinstance(rovs, list):
                for idx, r in enumerate(rovs):
                    if r and getattr(r, 'id', None) == silinen_id:
                        rovs[idx] = None
                        break

            # g_rovs cache'ini geçersiz kıl
            if hasattr(ortam, '_invalidate_g_rovs_cache'):
                ortam._invalidate_g_rovs_cache()

            # 2. Fizik gövdesini BulletWorld'den temizle (KRİTİK: bu yapılmazsa doPhysics()
            #    her frame'de ölü NodePath'e setPos/setHpr çağırır → !is_empty() assertion)
            filo_ref = getattr(ortam, 'filo', None)
            _world = getattr(filo_ref, 'world', None)
            _physics_node = getattr(self, 'physics_node', None)
            _physics_np   = getattr(self, 'physics_np',   None)
            if _world is not None and _physics_node is not None:
                try:
                    _world.removeRigidBody(_physics_node)
                except Exception:
                    pass
            if _physics_np is not None:
                try:
                    _physics_np.removeNode()
                except Exception:
                    pass
            self.physics_node = None  # type: ignore[assignment]
            self.physics_np   = None  # type: ignore[assignment]

            # Filo'nun motor ve ID haritalarından temizle
            if filo_ref is not None:
                try:
                    filo_ref.motorlar.pop(silinen_id, None)
                except Exception:
                    pass
                try:
                    if hasattr(filo_ref, '_rov_id_map'):
                        filo_ref._rov_id_map.pop(silinen_id, None)
                except Exception:
                    pass

            # 3. Görselleri temizle (havuz konteynerleri)
            if hasattr(self, 'label') and self.label: destroy(self.label)
            if hasattr(self, 'engel_cizgi') and self.engel_cizgi: destroy(self.engel_cizgi)
            for lidar_id in (0, 1, 2, 3):
                cont = getattr(self, 'lidar_cizgileri', {}).get(lidar_id)
                if cont: destroy(cont)

            # 3. Listeyi yeniden numaralandirma yok
            print(f"✅ ROV-{silinen_id} ve tum gorsel izleri temizlendi.")
            destroy(self)
            

    def _etiket_guncelle(self):
        """ID/rol degistiginde etiketi gunceller. Create-once: label sadece yoksa olusturulur, sonra sadece .text guncellenir."""
        metin = f"{'LIDER' if self.role == 1 else 'ROV'}-{self.id}"
        if getattr(self, 'label', None):
            self.label.text = metin  # type: ignore
        else:
            self.label = Text(text=metin, parent=self, y=1.5, scale=15, origin=(0,0), color=color.white)

    # --- get metodunu bu 'Güvenli' haliyle DEĞİŞTİR ---
    def get(self, veri):
        # Obje silinmişse Panda3D koordinat hatası (AssertionError) vermemesi için kontrol
        if not self or (hasattr(self, 'is_destroyed') and getattr(self, 'is_destroyed')):
            return None
        try:
            d = {"gps": [self.x, self.y, self.z], 
                 "hiz": [self.velocity.x, self.velocity.y, self.velocity.z],
                 "batarya": self.battery, "yaw": self.rotation_y, 
                 "rol": self.role, "sonar": self.son_sonar_mesafesi,
                 "lidar": self.son_lidar_mesafeleri, "group_id": self.group_id,
                 "gorev": getattr(getattr(self, "gnc", None), "gorev", "idle"),
                 "gorev_hedef": getattr(getattr(self, "gnc", None), "gorev_hedef", None)}
            return np.array(d[veri]) if veri in d and veri not in ["lidar", "sonar", "group_id"] else d.get(veri)
        except:
            return None

    # 🔹 MERKEZI LIDAR PROPERTIES — Her çağrıldığında önbellek değeri döner
    @property
    def l0(self) -> float:
        """L0 (İleri lidar) mesafesi. Döner: float (>0 hit, -1 miss)"""
        return self.son_lidar_mesafeleri.get(0, -1.0)
    
    @property
    def l1(self) -> float:
        """L1 (Sağ lidar) mesafesi. Döner: float (>0 hit, -1 miss)"""
        return self.son_lidar_mesafeleri.get(1, -1.0)
    
    @property
    def l2(self) -> float:
        """L2 (Sol lidar) mesafesi. Döner: float (>0 hit, -1 miss)"""
        return self.son_lidar_mesafeleri.get(2, -1.0)
    
    @property
    def l3(self) -> float:
        """L3 (Dip lidar) mesafesi. Döner: float (>0 hit, -1 miss)"""
        return self.son_lidar_mesafeleri.get(3, -1.0)

    def _guncelle_sensorler(self):

            menzil = GATLimitleri.ENGEL
            origin = self.world_position + Vec3(0, 0.5, 0)
            origin_l3 = self.world_position + Vec3(0, -8, 0)

            # 🔹 MERKEZI IGNORE TUPLE: Filo frame başında guncelle_hepsi() içinde _build_ignore_tuple() 
            # çağrıyor, ortam_ref.ignore_tuple otomatik güncelleniyor. Burada sadece kullan.
            ignore_tuple = ()
            if self.environment_ref:
                ignore_tuple = getattr(self.environment_ref, 'ignore_tuple', ())

            def buluta_ekle(hit_point, lidar_idx):
                ortam = self.environment_ref
                if not ortam or not hasattr(ortam, 'engel_bulutu'):
                    return
                try:
                    hit_x = float(hit_point.x)
                    hit_z = float(hit_point.z)
                    hit_y = float(hit_point.y)
                except (AttributeError, TypeError, ValueError):
                    return
                if not math.isfinite(hit_x) or not math.isfinite(hit_y) or not math.isfinite(hit_z):
                    return
                surface_y = float(getattr(ortam, 'WATER_SURFACE_Y_BASE', 0.0))
                sea_floor_y = float(getattr(ortam, 'SEA_FLOOR_Y', -50.0))
                # Su yuzeyine yakin isabetleri (L0/L1/L2 sensorleri) filtrele.
                # Bu noktalari buluta yazmak minimapte yalanci engel olusturuyor.
                if lidar_idx in (0, 1, 2) and hit_y >= surface_y - 1.0:
                    return
                # Dip lidarinin taban vuruslarini 2D engel bulutuna yazmak,
                # top-down minimapte gercek engel degilken kare bloklar olusturur.
                if lidar_idx == 3 and hit_y <= sea_floor_y + 1.0:
                    return
                hit_data = (
                    hit_x,
                    hit_z,
                    hit_y,
                    f"L{lidar_idx}",
                )

                # Canli minimap icin her lidar/sensor noktasi tekil olarak tutulur.
                lidar_noktalari = getattr(self, 'son_lidar_noktalari', None)
                if isinstance(lidar_noktalari, dict):
                    lidar_noktalari[lidar_idx] = hit_data

                # Geriye donuk uyumluluk: diger sistemler hala engel_bulutu listesini okuyabilir.
                ortam.engel_bulutu.append(hit_data)
                try:
                    ortam_engel_bulutu = getattr(ortam, 'engel_bulutu', [])
                    if isinstance(ortam_engel_bulutu, list) and len(ortam_engel_bulutu) > 12000:
                        ortam.engel_bulutu = ortam_engel_bulutu[2000:]  # type: ignore
                except Exception:
                    pass

            def safe_raycast(origin, direction, dist, ignore_list):
                safe_start = origin + (direction * 1.5)
                ray_dist = max(1, int(float(dist) - 1.5))
                return raycast(safe_start, direction, distance=ray_dist, ignore=ignore_list, debug=False)

            # SONAR (L0)
            hit_sonar = safe_raycast(origin, self.forward, menzil, ignore_tuple)
            if hit_sonar.hit:
                self.engel_mesafesi = hit_sonar.distance + 1.5
                self.son_sonar_mesafesi = self.engel_mesafesi
                self._kesikli_cizgi_ciz(hit_sonar.world_point, self.engel_mesafesi)
            else:
                self.engel_mesafesi = 999.0
                self.son_sonar_mesafesi = -1.0
                if self.engel_cizgi:
                    self.engel_cizgi.enabled = False

            # LIDARLAR (L0, L1, L2, L3)
            directions = [
                (0, self.forward, color.cyan),
                (1, self.right, color.blue),
                (2, -self.right, color.green),
                (3, Vec3(0, -1, 0), color.magenta),
            ]

            for idx, dir_vec, clr in directions:
                if isinstance(getattr(self, 'son_lidar_noktalari', None), dict):
                    self.son_lidar_noktalari[idx] = None
                ray_origin = origin_l3 if idx == 3 else origin
                hit = safe_raycast(ray_origin, dir_vec, menzil, ignore_tuple)
                if hit.hit:
                    dist = hit.distance + 1.5
                    self.son_lidar_mesafeleri[idx] = dist
                    self._lidar_cizgi_ciz(idx, hit.world_point, dist, clr)
                    buluta_ekle(hit.world_point, idx)
                else:
                    self.son_lidar_mesafeleri[idx] = -1.0
                    self._lidar_cizgi_temizle(idx)
    def set(self, ayar, deger):
        """GNC sistemi tarafından çağrılır."""
        if ayar == "rol":
            self.role = int(deger)
            if getattr(self, 'label', None):
                self.label.text = f"{'LIDER' if self.role == 1 else 'ROV'}-{self.id}"  # type: ignore
                self.color = color.red if self.role == 1 else color.white
            self._ui_durumu_kirlet()
        elif ayar == "yaw":
            self.rotation_y = float(deger)
            self.rotation = Vec3(0, self.rotation_y, 0)
            self._ui_durumu_kirlet()
        elif ayar in self.sensor_config:
            self.sensor_config[ayar] = deger
            self._ui_durumu_kirlet()

    def move(self, komut, guc=1.0):
        if self.battery <= 0: return

    def _cizgi_havuzlari_olustur(self):
        """Sonar ve lidar kesikli çizgileri için tekil mesh nesnelerini oluşturur."""
        from ursina import Entity
        # Sonar (engel) çizgisi tekil Mesh
        self.engel_cizgi = Entity(add_to_scene_entities=True, enabled=False, unlit=True)
        # Lidar (4 yön) çizgisi tekil Mesh
        for lidar_id in (0, 1, 2, 3):
            cont = Entity(add_to_scene_entities=True, enabled=False, alpha=0.6, unlit=True)
            self.lidar_cizgileri[lidar_id] = cont

    def _kesikli_cizgi_ciz(self, hedef, mesafe):
        """Sonar engel çizgisi. Tekil mesh güncellenir."""
        cizgi = self.engel_cizgi
        if cizgi is None:
            return
        
        c = color.red if mesafe < 5 else (color.orange if mesafe < 10 else color.yellow)
        delta = hedef - self.world_position
        if delta.is_nan() or delta.length() <= 1e-6:
            cizgi.enabled = False
            return
            
        yon = delta.normalized()
        n_use = min(int(mesafe), self.CIZGI_HAVUZ_SEGMENT)
        
        verts = []
        colors = []
        
        for i in range(n_use):
            baslangic = self.world_position + yon * (i + 0.25)
            bitis = self.world_position + yon * (i + 0.75)
            verts.append((baslangic.x, baslangic.y, baslangic.z))
            verts.append((bitis.x, bitis.y, bitis.z))
            colors.extend([c, c])
            
        if verts:
            from ursina import Mesh
            cizgi.model = Mesh(vertices=verts, colors=colors, mode='lines', thickness=4)
            cizgi.enabled = True
        else:
            cizgi.enabled = False

    def _lidar_cizgi_ciz(self, lidar_id, hedef, mesafe, renk):
        """Lidar engel çizgisi. Tekil mesh güncellenir."""
        cont = self.lidar_cizgileri.get(lidar_id)
        if cont is None:
            return
            
        delta = hedef - self.world_position
        if delta.is_nan() or delta.length() <= 1e-6:
            cont.enabled = False
            return
            
        yon = delta.normalized()
        n_use = min(int(mesafe), self.CIZGI_HAVUZ_SEGMENT)
        
        verts = []
        colors = []
        
        for i in range(n_use):
            baslangic = self.world_position + yon * (i + 0.25)
            bitis = self.world_position + yon * (i + 0.75)
            verts.append((baslangic.x, baslangic.y, baslangic.z))
            verts.append((bitis.x, bitis.y, bitis.z))
            colors.extend([renk, renk])
            
        if verts:
            from ursina import Mesh
            cont.model = Mesh(vertices=verts, colors=colors, mode='lines', thickness=2)
            cont.enabled = True
        else:
            cont.enabled = False
    
    def _lidar_cizgi_temizle(self, lidar_id):
        """Belirli bir lidar çizgisini gizle."""
        cont = self.lidar_cizgileri.get(lidar_id)
        if cont is not None:
            cont.enabled = False

# ============================================================
# 2. MINIMAP SINIFI (UI)
# ============================================================
class Minimap(Entity):
    BASE_SCALE = 0.55

    def __init__(self, ortam_ref, **kwargs):
        # ... (init kısmındaki diğer kodlar aynı kalacak) ...
        # Ekranın sol alt köşesine ((-0.5, -0.5) + padding) yerleştir.
        # origin=(-0.5, -0.5) ile haritanın kendi sol alt köşesi referans alınır.
        padding = 0.05

        super().__init__(
            parent=camera.ui,  # type: ignore
            scale=(self.BASE_SCALE, self.BASE_SCALE),
            origin=(-0.5, -0.5),      # type: ignore
            position=(0.62, -0.21),  # type: ignore
            **kwargs
        )

        self.ada_cevre_entity = None # Ada çevre noktaları için referans
        self.ortam_ref = ortam_ref
        self.loader = ortam_ref.loader
        self.havuz_genisligi = getattr(ortam_ref, 'havuz_genisligi', 200)
        
        self.rov_ikonlari, self.statik_nesneler, self.engel_noktalari = {}, [], []
        self.vektor_cizgi_entities, self.git_hedef_isaret_entities = [], {}
        self.path_entity = None
        self.hull_entity = None # Hull (Gövde) referansı
        
        self._apf_cache_sig, self._engel_bulutu_cizilen_len = None, 0
        self.loader.setup_minimap_base(self)
        self._statik_yeniden_ciz()
        self.visible = False

        # (Minimap __init__ metodunun en alt kısmına ekle)
        self.hedef_ikonlari = {}       # ID bazlı kalıcı hedefler için sözlük
        self.gecici_hedef_ikonu = None # debug=True iken kullanılan geçici hedef
        # Minimap __init__ içinde:

        # Her engel noktasi ayri bir mesh-entity olarak cizilir (pool ile yeniden kullanilir).
        self.engel_nokta_havuzu = []
        
        self._engel_bulutu_cizilen_len = 0
        self.kayitli_noktalar = set()
        # APF vektor havuzu: create-once, her karede sadece mesh/renk güncellenir
        self._apf_vektor_pool = None
        self._apf_vektor_pool_size = 128

    # --- KRİTİK GÜNCELLEME: goster metodu ---
    def goster(self, durum=True, convex=False, a_star=False, scale=None, **kwargs):
        """GNC sisteminden gelen convex ve a_star parametrelerini karşılar."""
        if scale:
            eff = self.BASE_SCALE * float(scale)
            self.scale = (eff, eff)
        
        self.visible = durum
        # Tüm çocukları (grid, ikonlar vb) toplu kapat/aç
        for child in getattr(self, 'children', []):
            child.enabled = durum  # type: ignore
        
        # Hull ve Path görünürlüğünü özel olarak ayarla
        h_entity = getattr(self, 'hull_entity', None)
        p_entity = getattr(self, 'path_entity', None)
        if h_entity is not None: h_entity.enabled = (convex and durum)  # type: ignore
        if p_entity is not None: p_entity.enabled = (a_star and durum)  # type: ignore

    # --- KRİTİK GÜNCELLEME: update_hull metodu ---
    def update_hull(self, points):
        """Filo GNC tarafından gönderilen noktaları kullanarak Cyan güvenlik bölgesini çizer. Create-once, mesh güncelle."""
        h_entity = getattr(self, 'hull_entity', None)
        if not points or len(points) < 3:
            if h_entity is not None:
                h_entity.enabled = False  # type: ignore
            return
        verts = []
        for p in points:
            px, pz = p[0], p[1]
            mp = self.dunya_to_harita(px, pz)
            verts.append((mp.x, mp.y, -0.25))
        verts.append(verts[0])
        sig = tuple((round(v[0], 3), round(v[1], 3), round(v[2], 3)) for v in verts)
        if h_entity is not None and getattr(self, '_hull_mesh_sig', None) == sig:
            h_entity.enabled = self.visible  # type: ignore
            return
        self._hull_mesh_sig = sig
        if h_entity is None:
            self.hull_entity = Entity(
                parent=self,
                model=Mesh(vertices=verts, mode='line', thickness=2),  # type: ignore
                color=color.cyan,  # type: ignore
                alpha=0.6,
                enabled=self.visible
            )
        else:
            h_entity.model.vertices = verts  # type: ignore
            h_entity.model.generate()  # type: ignore
            h_entity.enabled = self.visible  # type: ignore

    # --- KRİTİK GÜNCELLEME: update_path metodu ---
    def update_path(self, path_points):
        """A* algoritmasından gelen yeşil rota çizgisini günceller. Create-once, mesh güncelle."""
        p_entity = getattr(self, 'path_entity', None)
        if not path_points or len(path_points) < 2:
            if p_entity is not None:
                p_entity.enabled = False  # type: ignore
            return
        verts = []
        for p in path_points:
            mp = self.dunya_to_harita(p[0], p[1])
            verts.append((mp.x, mp.y, -0.3))
        sig = tuple((round(v[0], 3), round(v[1], 3), round(v[2], 3)) for v in verts)
        if p_entity is not None and getattr(self, '_path_mesh_sig', None) == sig:
            p_entity.enabled = self.visible  # type: ignore
            return
        self._path_mesh_sig = sig
        if p_entity is None:
            self.path_entity = Entity(
                parent=self,
                model=Mesh(vertices=verts, mode='line', thickness=3),  # type: ignore
                color=color.lime,  # type: ignore
                alpha=0.9,
                enabled=self.visible
            )
        else:
            p_entity.model.vertices = verts  # type: ignore
            p_entity.model.generate()  # type: ignore
            p_entity.enabled = self.visible  # type: ignore

    def dunya_to_harita(self, x, z):
        f = 1.0 / (self.havuz_genisligi * 2)
        return Vec3(x * f, z * f, -0.4)

    def _statik_yeniden_ciz(self):
        for ent in list(getattr(self, 'statik_nesneler', [])):
            if ent is not None:
                destroy(ent)
        self.statik_nesneler = []
        self.loader.create_minimap_grid(self, self.havuz_genisligi)
        if hasattr(self.ortam_ref, 'island_positions'):
            for pos in [p for p in self.ortam_ref.island_positions if p]:
                self.statik_nesneler.append(self.loader.draw_static_circle(self, pos[0], pos[1], pos[2], self.havuz_genisligi))

    def gorsel_guncelle(self):
            if not self.visible or not self.ortam_ref: return
            
            if hasattr(self.ortam_ref, 'rovs'):
                mevcut_rovlar = [r for r in list(self.ortam_ref.rovs) if r and not (getattr(r, 'is_destroyed', False))]
                active_ids = {getattr(r, 'id', -1) for r in mevcut_rovlar}
                
                # --- ÖNCE SİLİNENLERİ KALDIR ---
                for rid in list(self.rov_ikonlari.keys()):
                    if rid not in active_ids:
                        destroy(self.rov_ikonlari[rid])
                        self.rov_ikonlari.pop(rid, None)

                # --- SONRA MEVCUTLARI GÜNCELLE ---
                for rov in mevcut_rovlar:
                    target = self.dunya_to_harita(rov.x, rov.z)  # type: ignore

                    if getattr(rov, 'id', -1) not in self.rov_ikonlari:
                        self.rov_ikonlari[getattr(rov, 'id', -1)] = self.loader.create_rov_icon(self, getattr(rov, 'id', -1), getattr(rov, 'color', color.white))
                    icon = self.rov_ikonlari.get(getattr(rov, 'id', -1))
                    if icon:
                        icon.position = target
                        icon.rotation_z = -float(rov.rotation_y)  # type: ignore
                        icon.color = getattr(rov, 'color', color.white)  # type: ignore

            self._vektor_ve_hedef_guncelle()

    def _vektor_ve_hedef_guncelle(self):
        """APF vektorleri: create-once tekil Mesh kullanir."""
        filo = getattr(self.ortam_ref, 'filo', None)
        helper = getattr(filo, 'helper', None) if filo else None
        apf_list = helper.get_apf_vektor_verts_list(self) if helper else []
        vektor_renkler = {
            'k': color.red, 'y': color.green, 'm': color.blue,
            's': color.yellow, 't': color.orange
        }
        z_line = -0.35
        
        verts = []
        colors = []
        
        if apf_list:
            for v_list, c_code in apf_list:
                if not v_list or len(v_list) < 2:
                    continue
                c = vektor_renkler.get(c_code, color.white)
                # Yuvarlama yaparak titremeyi engelle
                x0, y0 = round(float(v_list[0][0]), 3), round(float(v_list[0][1]), 3)
                x1, y1 = round(float(v_list[1][0]), 3), round(float(v_list[1][1]), 3)
                verts.append((x0, y0, z_line))
                verts.append((x1, y1, z_line))
                colors.extend([c, c])
        sig = (
            tuple((round(v[0], 3), round(v[1], 3), round(v[2], 3)) for v in verts),
            tuple(str(c) for c in colors),
        )
                
        if not hasattr(self, '_apf_vektor_mesh_entity') or self._apf_vektor_mesh_entity is None:
            from ursina import Mesh, Entity
            self._apf_vektor_mesh_entity = Entity(
                parent=self,
                model=Mesh(vertices=verts, colors=colors, mode='lines', thickness=2),
                unlit=True,
                alpha=0.95,
                z=z_line
            )
            self._apf_mesh_sig = sig
        else:
            if getattr(self, '_apf_mesh_sig', None) == sig:
                self._apf_vektor_mesh_entity.enabled = len(verts) > 0
                return
            self._apf_mesh_sig = sig
            self._apf_vektor_mesh_entity.model.vertices = verts
            self._apf_vektor_mesh_entity.model.colors = colors
            self._apf_vektor_mesh_entity.model.generate()
            self._apf_vektor_mesh_entity.enabled = len(verts) > 0

    def _apf_vektorlari_temizle(self):
        """APF vektorlerini gizler."""
        if hasattr(self, '_apf_vektor_mesh_entity') and self._apf_vektor_mesh_entity:
            self._apf_vektor_mesh_entity.enabled = False

    def _engel_bulutu_guncelle_yedek(self):
        # Engel gosterimi kapatildi: minimapte algilanan engeller cizilmez.
        for e in list(self.engel_noktalari):
            if e is not None:
                e.enabled = False  # type: ignore
        for e in list(getattr(self, 'engel_nokta_havuzu', [])):
            if e is not None:
                e.enabled = False  # type: ignore
        self._engel_bulutu_cizilen_len = 0

    def _engel_nokta_entity_al(self, idx: int):
        while len(self.engel_nokta_havuzu) <= idx:
            nokta_mesh = Mesh(vertices=[(0, 0, 0)], mode='point', thickness=3, static=False)
            nokta_entity = Entity(
                parent=self,
                model=nokta_mesh,
                color=color.white,
                z=-0.01,
                enabled=False,
            )
            self.engel_nokta_havuzu.append(nokta_entity)
        return self.engel_nokta_havuzu[idx]


    def _engel_bulutu_guncelle(self):
        # Engel gosterimi kapatildi: minimapte algilanan engeller cizilmez.
        for e in list(self.engel_noktalari):
            if e is not None:
                e.enabled = False  # type: ignore
        for e in list(getattr(self, 'engel_nokta_havuzu', [])):
            if e is not None:
                e.enabled = False  # type: ignore
        self.kayitli_noktalar = set()
        self._engel_bulutu_cizilen_len = 0

    def update_ada_cevre(self, points):
            """
            GNC sisteminden gelen sahil şeridi noktalarını minimap üzerinde çizer.
            Create-once: konteyner ve Mesh bir kez oluşturulur, her cagrida noktalar (vertices) güncellenir.
            """
            if not points:
                if hasattr(self, 'ada_cevre_entity') and self.ada_cevre_entity:
                    self.ada_cevre_entity.enabled = False  # type: ignore
                return
            
            ada_renk = color.hex('#CD853F')
            verts = []
            for p in points:
                mp = self.dunya_to_harita(p[0], p[1] if len(p) > 1 else 0)
                verts.append((mp.x, mp.y, -0.28))
            
            if getattr(self, 'ada_cevre_entity', None) is None:
                from ursina import Mesh, Entity
                mesh = Mesh(vertices=verts, mode='point', thickness=4)
                self.ada_cevre_entity = Entity(
                    parent=self,
                    model=mesh,
                    color=ada_renk,
                    alpha=0.85
                )
            else:
                self.ada_cevre_entity.model.vertices = verts
                self.ada_cevre_entity.model.generate()
                self.ada_cevre_entity.enabled = True

    def hedef_isaretle(self, x, z, id=None, debug=True):
        """"""
        from ursina import Entity, destroy, color, Text  # type: ignore[import-not-found]
        
        mp = self.dunya_to_harita(x, z)
        
        # --- DURUM 1: GEÇİCİ HEDEF (debug=True) ---
        if debug:
            if not hasattr(self, 'gecici_hedef_ikonu') or self.gecici_hedef_ikonu is None:
                self.gecici_hedef_ikonu = Entity(
                    parent=self,
                    model='circle',
                    color=color.red,
                    scale=0.035,
                    position=(mp.x, mp.y, -0.35),
                    enabled=self.visible
                )
            else:
                self.gecici_hedef_ikonu.position = (mp.x, mp.y, -0.35)  # type: ignore
                self.gecici_hedef_ikonu.enabled = self.visible  # type: ignore
            
        # --- DURUM 2: KALICI ID'Lİ HEDEF (debug=False) ---
        else:
            if id is None: return
            
            # Aynı ID varsa önce eskisini sil
            self.hedef_sil(id)
            
            yeni_ikon = Entity(
                parent=self,
                model='circle',
                color=color.cyan,
                scale=0.03,
                position=(mp.x, mp.y, -0.35),
                enabled=self.visible
            )
            
            # Noktanın yanına haritada ID numarasını yaz
            Text(
                parent=yeni_ikon,
                text=str(id),
                scale=35, # İkonun (0.03) child'ı olduğu için scale'i yüksek veriyoruz
                color=color.black,
                origin=(0, 0),
                y=1 # Çemberin hafif üstünde dursun
            )
            
            self.hedef_ikonlari[id] = yeni_ikon

    def hedef_sil(self, id):
        """Belirtilen ID'li hedefi minimap'ten siler."""
        from ursina import destroy  # type: ignore[import-not-found]
        if hasattr(self, 'hedef_ikonlari') and id in self.hedef_ikonlari:
            destroy(self.hedef_ikonlari[id])
            self.hedef_ikonlari.pop(id, None)

    def hedefleri_temizle(self):
        """Tüm kalıcı ve geçici hedefleri minimap'ten temizler."""
        from ursina import destroy  # type: ignore[import-not-found]
        # Kalıcı hedefleri sil
        if hasattr(self, 'hedef_ikonlari'):
            for hid in list(self.hedef_ikonlari.keys()):
                self.hedef_sil(hid)
        
        # Geçici hedefi sil
        if hasattr(self, 'gecici_hedef_ikonu') and self.gecici_hedef_ikonu:
            destroy(self.gecici_hedef_ikonu)
            self.gecici_hedef_ikonu = None

# ============================================================
# 3. ORTAM SINIFI (Simülasyon Dünyası)
# ============================================================
# ============================================================
# 3. ORTAM SINIFI (Dünya ve Simülasyon Yönetimi)
# ============================================================
class Ortam:
    _active_instance = None

    def __init__(self, verbose=False):
        # Tek proses/tek sahne garantisi: yan-etkili importlar ikinci Ortam yaratmaya
        # kalkarsa mevcut ornegi yeniden kullan.
        mevcut = Ortam._active_instance
        if mevcut is not None and getattr(mevcut, 'app', None) is not None:
            self.__dict__ = mevcut.__dict__
            return

        self.verbose = verbose
        
        # [FIX] Render debug mode - entity double-rendering sorunu tanısı
        self._entity_render_count = {}
        
        # [CRITICAL PRE-FIX] Ursina'nın Panda3D config'ini önceden set et
        # Stereo rendering disable, shadow mapping disable
        try:
            from panda3d.core import load_prc_file_data
            prc_data = """
            framebuffer-stereo #f
            framebuffer-srgb #f
            gl-force-depth-write 1
            gl-depth-test 1
            prefer-parasite-buffer #f
            """
            load_prc_file_data("", prc_data)
        except:
            pass
        
        # Pencere ayarlarını _setup_window içinde yapacağımız için burada temel başlatma yapıyoruz
        self.app = Ursina(
            vsync=False, 
            development_mode=False, 
            show_ursina_splash=False, 
            borderless=False,
            title="FıratROVNet Simülasyonu"
        )
        
        # [FIX] AGGRESSIVE: Panda3D'nin tüm secondary render pass'larını devre dışı bırak
        try:
            from panda3d.core import GraphicsOutput
            from direct.showbase.ShowBase import globalShowBase
            
            pd_base = globalShowBase
            if pd_base and hasattr(pd_base, 'win'):
                win = pd_base.win
                
                # [CRITICAL] render2d (2D rendering layer) deaktivate et
                # Ursina render + render2d = dual render!
                if hasattr(pd_base, 'render2d'):
                    pd_base.render2d.hide()
                    if hasattr(pd_base.render2d, 'set_active'):
                        pd_base.render2d.set_active(False)
                
                # [CRITICAL] Tüm extra buffers/regions kaldır
                if hasattr(win, 'get_num_display_regions'):
                    num_regions = win.get_num_display_regions()
                    regions_to_remove = []
                    
                    # Region 0 = main view, tüm kalanları sil
                    for i in range(1, num_regions):
                        region = win.get_display_region(i)
                        if region:
                            regions_to_remove.append(region)
                    
                    for region in regions_to_remove:
                        try:
                            win.remove_display_region(region)
                        except:
                            pass
                
                # [CRITICAL] Tüm auxiliary buffers'ı kapat (depth, shadow, etc.)
                if hasattr(pd_base, 'get_all_aux_buffers'):
                    try:
                        aux_buffers = pd_base.get_all_aux_buffers()
                        for buf in aux_buffers:
                            if buf and hasattr(buf, 'set_active'):
                                buf.set_active(False)
                    except:
                        pass
                
                # [CRITICAL] Internal cameras'ı kontrol et
                if hasattr(pd_base, 'camera'):
                    cam_node = pd_base.camera
                    # Camera parent'ı check et - hiçbir secondary camera olmamalı
                    if hasattr(cam_node, 'get_num_children'):
                        num_children = cam_node.get_num_children()
                        if num_children > 1:
                            # Secondary cameras'ı detach et
                            for i in range(1, num_children):
                                child = cam_node.get_child(i)
                                try:
                                    child.detach_node()
                                except:
                                    pass
        except Exception as e:
            pass


        # --- ESKİ AYARLAR: SABİT ADA VE ROV KONUMLARI ---
        self.FIXED_ISLAND_POSITIONS = [
            (-150.0, 150.0), (-50, 150), (50, 150.0), (150, 150),
            (-150.0, 100.0), (-50, 100), (50, 100), (150, 100),
            (-150.0, 50.0), (-50, 50), (50, 50.0), (150, 50),
            (-150.0, 0.0), (150, 0), (-100.0, 0.0), (100, 0),
            (-150.0, -50.0), (-50, -50), (50, -50.0), (150, -50),
            (-150.0, -100.0), (-50, -100), (50, -100), (150, -100),
            (-150.0, -150.0), (-50, -150), (50, -150.0), (150, -150),
        ]
        
        # Fiziksel Sabitler
        self.havuz_genisligi = 200
        self.su_hacmi_yuksekligi, self.su_hacmi_merkez_y = 50, -25
        self.WATER_SURFACE_Y_BASE, self.SEA_FLOOR_Y = 0.0, -50.0
        self.SONAR_MENZILI = GATLimitleri.ILETISIM_MENZILI

        self.loader = EntityLoader(self)
        self.helper = OrtamHelper(self)
        
        self.rovs, self.island_positions, self.island_entities = [], [], []
        self._g_rovs: dict = {}
        self._g_rovs_cache_len: int = -1   # cache invalidation: rovs listesi boyutu
        self._g_rovs_cache_sig: tuple = ()
        self.islands=self.island_entities
        self.engel_bulutu, self.konsol_verileri = [], {}
        self.sonar_cizgiler, self.filo = {}, None
        self.pool_human = None

        # --- KURULUM ---
        self._setup_window()
        self._setup_lighting()
        # [FIX] Minimap re-enabled with proper parent hierarchy  
        self.minimap = Minimap(ortam_ref=self)
        self.minimap.visible = True
        
        # [FIX] EditorCamera re-enabled 
        self.camera = EditorCamera(
            enabled=True, rotate_speed=15, pan_speed=(10, 10),
            zoom_speed=1, position=(0, 0, -50), rotation=(20, 0, 0)
        )
        # EditorCamera'nın 'r' (reset) kısayolunu devre dışı bırak — biz kullanıyoruz
        try:
            self.camera.shortcuts = {k: v for k, v in self.camera.shortcuts.items() if k != 'r'}
        except Exception:
            pass
        
        mouse.visible, mouse.locked = True, False

        # [FINAL FIX] Panda3D'nin kamera node'unda sadece BİR camera olduğundan emin ol
        try:
            from direct.showbase.ShowBase import globalShowBase
            pd_base = globalShowBase
            if pd_base and hasattr(pd_base, 'camera_node'):
                cam_node = pd_base.camera_node()
                # Parent'taki tüm child'ları kontrol et ve duplicate camera'ları sil
                parent = cam_node.get_parent()
                if parent and hasattr(parent, 'get_num_children'):
                    for i in range(parent.get_num_children()):
                        child = parent.get_child(i)
                        # Ana camera node dışındakini sil (gizli debug camera'ları)
                        if child != cam_node:
                            try:
                                child.detach_node()
                            except:
                                pass
        except:
            pass

        # [FIX] Window resize event handler - çift render sorunu düzeltmek için camera aspect ratio sıfırla
        self._setup_window_resize_handler()

        # Ilk basarili kurulumdan sonra aktif instance'i sabitle.
        Ortam._active_instance = self
        


    @property
    def g_rovs(self):
        # Cache: ROV listesi ve grup dağılımı değişmedikçe yeniden inşa etme
        current_len = len(self.rovs)
        current_sig = tuple(
            (int(getattr(rov, 'id', -1)), int(getattr(rov, 'group_id', 0)))
            for rov in self.rovs
            if rov and not getattr(rov, 'is_destroyed', False)
        )
        if (
            current_len == self._g_rovs_cache_len
            and current_sig == self._g_rovs_cache_sig
            and self._g_rovs
        ):
            return self._g_rovs
        self._g_rovs = {}
        for rov in self.rovs:
            if not rov or getattr(rov, 'is_destroyed', False):
                continue
            __group_id=getattr(rov, 'group_id', 0)
            if not self._g_rovs.get(__group_id, False):
                self._g_rovs[__group_id]=[]
            self._g_rovs[__group_id].append(rov)
        self._g_rovs_cache_len = current_len
        self._g_rovs_cache_sig = current_sig
        return self._g_rovs

    def _invalidate_g_rovs_cache(self):
        """ROV eklendiğinde veya çıkartıldığında cache'i geçersiz kıl."""
        self._g_rovs_cache_len = -1
        self._g_rovs_cache_sig = ()

    def _setup_window(self):
        """ESKİ AYARLAR: Pencere konfigürasyonu."""
        
        # [CRITICAL] Ursina'nın camera.ui setup'ını kontrol et
        # camera.ui Ursina'nın 2D overlay camera'sı (orthogonal projection)
        # Bu ile main 3D camera'nın conflict'ı "iki dünya" görünümü yaratabilir
        try:
            # camera.ui ortho-projection'ını explicitly set et
            from panda3d.core import OrthographicLens
            if hasattr(camera, 'ui') and hasattr(camera.ui, 'lens'):
                ui_lens = camera.ui.lens()
                if isinstance(ui_lens, OrthographicLens):
                    # Ortho camera'nın aspect ratio'yu pencereyle senkronize et
                    ui_lens.set_film_size(1280, 720)  # Default match
                # Make sure UI camera render order is AFTER main camera
                if hasattr(camera.ui, 'node'):
                    ui_node = camera.ui.node()
                    if hasattr(ui_node, 'set_active'):
                        ui_node.set_active(True)
        except:
            pass
        
        window.fullscreen = False
        window.exit_button.visible = False
        window.size = (1280, 720)
        window.center_on_screen()
        window.color = color.rgb(10, 30, 50)
        # application.run_in_background = True  # Ursina'da bu özellik yok veya farklı şekilde ayarlanıyor
        try: window.context_menu = False
        except: pass


        # FPS gostergesi: ekranin tam sag ust kosesi (origin 0.5,0.5 = metnin sag ustu)
        self.rov_label = Text(  # type: ignore
            text="FPS: --",
            parent=camera.ui,
            position=(0.69, 0.49),
            origin=(-0.5, 0.5),
            scale=1.0,
            color=color.lime,
            background=True,
        )
        self.rov_label.z = -10  # type: ignore
        if self.rov_label.background is not None:  # type: ignore
            self.rov_label.background.scale_x = 2  # type: ignore
            self.rov_label.background.scale_y = 2.2  # type: ignore
            self.rov_label.background.x = 0.1  # type: ignore
            self.rov_label.background.y = -0.07  # type: ignore

    def _setup_lighting(self):
        self.sun = DirectionalLight()  # type: ignore
        self.sun.look_at(Vec3(1, -1, -1))  # type: ignore
        self.ambient = AmbientLight(color=color.rgba(120, 120, 120, 1))  # type: ignore
        self.sky = Sky()  # type: ignore
        
        # [FIX] Shader ve post-processing devre dışı bırak (çift render kaynağı olabilir)
        try:
            from panda3d.core import ShaderAttrib
            from direct.showbase.ShowBase import globalShowBase
            
            pd_base = globalShowBase
            if pd_base:
                # Shadow mapping devre dışı bırak
                if hasattr(pd_base, 'render'):
                    pd_base.render.set_attrib(ShaderAttrib.make())
                
                # Fog render pass devre dışı bırak
                if hasattr(pd_base, 'set_fog'):
                    try:
                        pd_base.set_fog(None)
                    except:
                        pass
        except:
            pass

    def _setup_window_resize_handler(self):
        """
        [FIX] COMPREHENSIVE: Window resize sırasında Panda3D render state'ini reset et.
        Resize sırasında internal viewport/framebuffer corruption oluşabilir.
        """
        last_window_size = [window.size[0], window.size[1]]
        
        def on_render_frame_task(task):
            """Her frame'de window size değişim kontrol et."""
            try:
                current_size = window.size
                if (current_size[0] != last_window_size[0] or 
                    current_size[1] != last_window_size[1]):
                    
                    # [CRITICAL] Resize detected - render state reset
                    last_window_size[0] = current_size[0]
                    last_window_size[1] = current_size[1]
                    
                    # Camera lens aspect ratio güncelle
                    if hasattr(camera, 'lens') and camera.lens:
                        aspect = current_size[0] / max(current_size[1], 1)
                        camera.lens.set_aspect_ratio(aspect)
                    
                    # Panda3D internal render target'ı force-invalidate et
                    try:
                        from panda3d.core import GraphicsEngine
                        if hasattr(application, 'engine') and hasattr(application.engine, 'engine'):
                            gfx_engine = application.engine.engine
                            # Force render pipeline reset
                            if hasattr(gfx_engine, 'reset_framebuffer'):
                                gfx_engine.reset_framebuffer()
                    except:
                        pass
                    
                    # View matrix'i reset et
                    try:
                        if hasattr(camera, 'node') and hasattr(camera.node(), 'reset_projection_mat'):
                            camera.node().reset_projection_mat()
                    except:
                        pass
                    
            except Exception as e:
                pass
            
            return task.cont
        
        # Ursina's task manager ile frame-per-frame resize check
        try:
            application.task_mgr.add(on_render_frame_task, 'frame_resize_check')
        except Exception:
            pass
        
        # Also setup stereo rendering disable (in case it's enabled by default)
        try:
            if hasattr(application, 'win') and application.win:
                if hasattr(application.win, 'set_stereo'):
                    application.win.set_stereo(False)
        except:
            pass
        
    def konsola_ekle(self, isim, nesne): self.konsol_verileri[isim] = nesne

    def mark_ui_state_dirty(self):
        """Main tarafi override etmediyse no-op kalir."""
        return None
    
    def set_update_function(self, func):
        """Günceleme fonksiyonunu kaydet. Ursina'da doğrudan update attribute atanamaz."""
        self._custom_update_func = func  # type: ignore
        # Ursina update döngüsüne entegre etmek için (Ursina update hook mekanizması)
        # Bu kod main.py'de app.set_update_function() çağrısı yerine kullanılır
    
    def guncelle_ozel(self):
        """Custom update fonksiyonunu çalıştırır (Ursina tarafından çağrılır)."""
        if hasattr(self, '_custom_update_func') and getattr(self, '_custom_update_func', None):
            getattr(self, '_custom_update_func')()
    
    def simden_veriye(self): return self.helper.simden_veriye() if self.helper else []

    def _find_safe_rov_spawn_pos_yedek(self):
        """ESKİ AYARLAR: Adalardan uzak, güvenli spawn noktası bulur."""
        for _ in range(100):
            sx = float(random.uniform(-160, 160))
            sy = float(random.uniform(-160, 160))
            sz_depth = float(random.uniform(10, 25))
            is_safe = True
            for island in [p for p in self.island_positions if p]:
                # Adanın yarıçapı + 25m güvenlik payı
                if math.sqrt((sx-island[0])**2 + (sy-island[1])**2) < (island[2] + 25):
                    is_safe = False
                    break
            if is_safe: return (sx, sy, -sz_depth)  # type: ignore
        return (0, 0, -15) # Fallback
    

    def _find_safe_rov_spawn_pos(self, group_config: tuple, alan_genisligi=100, bosluk=10):
        """
        group_config: (3, 4, 1) gibi bir tuple alır.
        - 3 grup oluşturur.
        - Grup 0: 3 ROV, Grup 1: 4 ROV, Grup 2: 1 ROV yerleştirir.
        """
        import math, random
        
        all_groups_rovs: list = [] 
        
        # Tarama sınırları ve adım boyutu
        baslangic_x, baslangic_y = -180.0, -180.0
        bitis_x, bitis_y = 180.0, 180.0
        adim = float(alan_genisligi + bosluk)

        mevcut_x = float(baslangic_x)
        mevcut_y = float(baslangic_y)

        # Tuple içindeki her bir grup tanımı için dön
        for g_id, num_rovs in enumerate(group_config):
            bulundu = False
            
            # Uygun hücre bulana kadar taramaya devam et
            while mevcut_y <= bitis_y - alan_genisligi:
                while mevcut_x <= bitis_x - alan_genisligi:
                    
                    # Hücrenin merkezi
                    merkez_x = mevcut_x + (alan_genisligi / 2)  # type: ignore
                    merkez_y = mevcut_y + (alan_genisligi / 2)  # type: ignore
                    
                    # 1. ADA KONTROLÜ
                    hucre_kirli = False
                    for island in [p for p in self.island_positions if p]:
                        dist = math.sqrt((merkez_x - island[0])**2 + (merkez_y - island[1])**2)  # type: ignore
                        if dist < (island[2] + 10): # Ada yarıçapı + 15m emniyet
                            hucre_kirli = True
                            break
                    
                    if not hucre_kirli:
                        # 2. TEMİZ ALAN BULUNDU: Bu grubun ROV'larını yerleştir
                        bu_grubun_rovlari = []
                        
                        # Eğer grupta tek ROV varsa merkeze koy, çoksa çember yap
                        if num_rovs == 1:
                            rz = -random.uniform(0, 10)
                            bu_grubun_rovlari.append((merkez_x, merkez_y, rz))
                        else:
                            yaricap = 10.0 # ROV'lar arası yayılma mesafesi
                            for r_id in range(num_rovs):
                                angle = math.radians(r_id * (360 / num_rovs))
                                rx = float(merkez_x) + math.cos(angle) * yaricap
                                ry = float(merkez_y) + math.sin(angle) * yaricap
                                rz = -random.uniform(5, 10)
                                bu_grubun_rovlari.append((rx, ry, rz))
                        
                        all_groups_rovs.append(bu_grubun_rovlari)
                        
                        # Sonraki grup için imleci bir adım kaydır
                        mevcut_x += adim  # type: ignore
                        bulundu = True
                        break # İçteki X döngüsünden çık
                    
                    # Hücre kirliyse sağa kay
                    mevcut_x += adim  # type: ignore
                
                if bulundu: break # Y döngüsünden çık, sonraki gruba geç
                
                # Satır sonuna gelindiyse başa dön ve yukarı çık
                mevcut_x = baslangic_x
                mevcut_y += adim  # type: ignore

        return all_groups_rovs

    def _guvenli_rov_konumu(self, min_ada_mesafesi=35.0, sinir=160.0) -> tuple:
        """Ada konumlarından uzak, rastgele güvenli bir (x, z) noktası döndürür (Ursina koordinatı)."""
        import random
        ada_konumlari = [p for p in self.island_positions if p]
        for _ in range(200):
            rx = random.uniform(-sinir, sinir)
            rz = random.uniform(-sinir, sinir)
            cakisiyor = False
            for pos in ada_konumlari:
                # island_positions sim koordinatında (x, y) → Ursina x=x, z=y
                ada_x = float(pos[0]) if hasattr(pos, '__getitem__') else 0.0
                ada_z = float(pos[1]) if hasattr(pos, '__getitem__') else 0.0
                if ((rx - ada_x) ** 2 + (rz - ada_z) ** 2) ** 0.5 < min_ada_mesafesi:
                    cakisiyor = True
                    break
            if not cakisiyor:
                return rx, rz
        # 200 denemede bulunamazsa güvenli merkez
        return 0.0, -80.0

    def sim_olustur(self, n_rovs=(6,), n_islands=5, n_rocks=20, havuz_genisligi=200, rov_model='submarine'):
        self.havuz_genisligi = havuz_genisligi

        # Dunya katmanlarini yeniden kurmadan once eski entity'leri temizle.
        for attr_name in ("water_volume", "ocean_surface", "ocean_taban", "seabed", "cimen_katmani", "pool_human"):
            ent = getattr(self, attr_name, None)
            if ent is not None:
                try:
                    destroy(ent)
                except Exception:
                    pass
                setattr(self, attr_name, None)

        # Loader tarafinda biriken dinamik engeller/sinirlar da temizlensin.
        if hasattr(self.loader, "clear_rocks"):
            try:
                self.loader.clear_rocks()
            except Exception:
                pass
        if hasattr(self.loader, "clear_boundaries"):
            try:
                self.loader.clear_boundaries()
            except Exception:
                pass
        
        # Temizlik
        for obj in [r for r in self.rovs if r] + [i for i in self.island_entities if i]: 
            if obj: destroy(obj)
        self.rovs, self.island_entities, self.island_positions, self.engel_bulutu = [], [], [], []
        
        # Dünya İnşası - HER ŞEY AÇIK
        size = havuz_genisligi * 2
        self.loader.build_ocean(size=size)
        self.loader.build_seabed(size=size)
        self.loader.load_pool_human(havuz_genisligi=havuz_genisligi)
        self.loader.build_boundaries(havuz_genisligi)
        
        # Kayaları ekle (n_rocks parametresi ile)
        if n_rocks > 0:
            self.loader.spawn_rocks(count=n_rocks, havuz_genisligi=havuz_genisligi)
        
        # 1. Adaları Sabit Noktalardan Yerleştir
        count = min(n_islands, len(self.FIXED_ISLAND_POSITIONS))
        chosen_islands = random.sample(self.FIXED_ISLAND_POSITIONS, count)
        chosen_islands.insert(0,(0,0))
        for i, pos in enumerate(chosen_islands):
            self.Ada(i, x="ekle", y=pos)

        # 2. ROV'ları Güvenli Noktalara Yerleştir
        # n_rovs tuple'ından toplam ROV sayısını hesapla
        toplam_rov_sayisi = sum(n_rovs)
        print(f"🌊 Simülasyon Başlatılıyor: {toplam_rov_sayisi} ROV, {count} Ada")

        all_group = self._find_safe_rov_spawn_pos(n_rovs)

        # Global ROV ID counter
        global_rov_id: int = 0
        
        for group_id, rovlar in enumerate(all_group):
            for local_rov_id, rov_koordinat in enumerate(rovlar):
                # sim_pos: (x, z_depth, y_coordinate) -> ursina: (x, y, z)
                u_pos = Vec3(rov_koordinat[0], rov_koordinat[2], rov_koordinat[1])
                    
                new_rov = ROV(rov_id=global_rov_id, group_id=group_id, position=u_pos, loader_ref=self.loader, model_key=rov_model)
                new_rov.ekle(self)
                
                global_rov_id = int(global_rov_id + 1)  # type: ignore

        if getattr(self, "minimap", None):
            self.minimap._statik_yeniden_ciz()
        


    def yeni_rov_ekle(self, rov_model='submarine'):
        """UI'dan tek bir yeni ROV ekler. Ada olmayan güvenli bir konuma yerleştirir."""
        import random
        from ursina import Vec3

        # Yeni ID: mevcut maksimum + 1
        mevcut_ids = [getattr(r, 'id', 0) for r in self.rovs if r is not None]
        yeni_id = (max(mevcut_ids) + 1) if mevcut_ids else 0

        # Ada çakışması olmayan güvenli konum bul
        rx, rz = self._guvenli_rov_konumu()
        ry = -5.0  # Ursina Y = derinlik (su yüzeyinin altı)

        new_rov = ROV(rov_id=yeni_id, group_id=0, position=Vec3(rx, ry, rz),
                      loader_ref=self.loader, model_key=rov_model)
        new_rov.ekle(self)

        # Filo'ya sadece bu ROV'u ekle (tüm sistemi yeniden kurma)
        filo_ref = getattr(self, 'filo', None)
        if filo_ref is not None and hasattr(filo_ref, '_tek_rov_fizik_kur'):
            try:
                filo_ref._tek_rov_fizik_kur(new_rov)
                filo_ref.mevcut_rov_sayisi = len([r for r in self.rovs if r is not None])
                filo_ref.motor_sema_kaydet()
                filo_ref.tum_motor_bv_kutuphanelerini_guncelle()
            except Exception:
                import traceback
                traceback.print_exc()

        if getattr(self, "minimap", None):
            self.minimap._statik_yeniden_ciz()

        print(f"✅ Yeni ROV-{yeni_id} simülasyona eklendi @ ({rx:.1f}, {ry:.1f}, {rz:.1f})")
        return new_rov

    def ROV(self, rov_id, x=None, y=None, z=None):
        """Konsol: ROV rov_id konumunu (x, y, z) yapar. x,y,z verilmezse sadece mevcut ROV döner."""
        if not self.rovs or rov_id is None:
            return None
        rov = next((r for r in self.rovs if r and getattr(r, "id", None) == rov_id), None)
        if not rov:
            return None
        if x is not None:
            setattr(rov, 'x', float(x))
        if y is not None:
            setattr(rov, 'y', float(y))
        if z is not None:
            setattr(rov, 'z', float(z))
        return rov

    def Ada(self, ada_id, x=None, y=None):
        if x == "ekle":
            if y is None:
                return
            while len(self.island_positions) <= ada_id: self.island_positions.append(None)
            while len(self.island_entities) <= ada_id: self.island_entities.append(None)
            eski_ada = self.island_entities[ada_id]
            if eski_ada is not None:
                try:
                    destroy(eski_ada)
                except Exception:
                    pass
            ent, radius = self.loader.create_island(y[0], y[1])
            self.island_entities[ada_id], self.island_positions[ada_id] = ent, (y[0], y[1], radius)
            
    def guncelle_sonar_cizgileri(self):
            """
            ROV'lar arası sonar iletişimini KESİKLİ ÇİZGİ Mesh'leri ile gösterir.
            Create-once per pair: her çift için tekil Mesh oluşturulur, her karede güncellenir.
            """
            active_rovs = [r for r in self.rovs if r and not (hasattr(r, 'is_destroyed') and r.is_destroyed)]
            bu_frame_aktif_olanlar = set()
            
            BASE_KALINLIK = 2  # Kalınlık 0.5 oranında inceltildi
            segment_boyu = 1.2
            bosluk_boyu = 1.8
            adim_toplam = segment_boyu + bosluk_boyu

            for i, r1 in enumerate(active_rovs):
                for r2 in active_rovs[i+1:]:  # type: ignore
                    if r1.gat_kodu == 3 or r2.gat_kodu == 3:
                        continue

                    p1, p2 = r1.world_position, r2.world_position
                    delta = p2 - p1
                    if delta.is_nan():
                        continue
                    dist = delta.length()
                    
                    if 1e-6 < dist < self.SONAR_MENZILI:
                        pair = tuple(sorted((r1.id, r2.id)))
                        bu_frame_aktif_olanlar.add(pair)
                        
                        oran = dist / self.SONAR_MENZILI
                        carpan = float(lerp(1.25, 0.75, oran) or 1.0)
                        guncel_kalinlik = max(1, int(BASE_KALINLIK * carpan))
                        if dist < 25:
                            c = color.red
                        elif dist < 80:
                            c = color.orange
                        else:
                            c = color.white
                        
                        yon_vec = delta.normalized()
                        
                        verts = []
                        colors = []
                        curr = 0
                        while curr < dist:
                            kalin_uzunluk = min(segment_boyu, dist - curr)
                            if kalin_uzunluk <= 0.1:
                                break
                            baslangic = p1 + yon_vec * float(curr)
                            bitis = baslangic + yon_vec * float(kalin_uzunluk)
                            
                            verts.append((baslangic.x, baslangic.y, baslangic.z))
                            verts.append((bitis.x, bitis.y, bitis.z))
                            colors.extend([c, c])
                            curr += adim_toplam
                            
                        # Create-once: tekil Mesh entity
                        if pair not in self.sonar_cizgiler:
                            from ursina import Entity
                            mesh_entity = Entity(add_to_scene_entities=True, unlit=True, alpha=0.2)
                            self.sonar_cizgiler[pair] = mesh_entity
                        
                        mesh_entity = self.sonar_cizgiler[pair]
                        if verts:
                            sig = (
                                round(p1.x, 1), round(p1.y, 1), round(p1.z, 1),
                                round(p2.x, 1), round(p2.y, 1), round(p2.z, 1),
                                guncel_kalinlik, str(c),
                            )
                            if getattr(mesh_entity, "_sonar_sig", None) == sig:
                                mesh_entity.enabled = True
                                continue
                            mesh_entity._sonar_sig = sig
                            from ursina import Mesh
                            mesh_entity.model = Mesh(vertices=verts, colors=colors, mode='lines', thickness=guncel_kalinlik)
                            mesh_entity.enabled = True
                        else:
                            mesh_entity.enabled = False

            # Temizlik: sadece artık menzilde olmayan çiftleri sil (list kopyası üzerinden)
            for pair in list(self.sonar_cizgiler.keys()):
                if pair not in bu_frame_aktif_olanlar:
                    mesh_entity = self.sonar_cizgiler[pair]
                    if isinstance(mesh_entity, tuple): # Eski formata denk gelirse diye güvenlik
                        destroy(mesh_entity[0])
                    else:
                        destroy(mesh_entity)
                    self.sonar_cizgiler.pop(pair, None)

    def run(self, interaktif=False):
        if interaktif: threading.Thread(target=self._start_shell, daemon=True).start()
        self.app.run()

    def _start_shell(self):
        import time; time.sleep(1.5)
        print("\n🚀 FIRAT ROVNET CANLI KONSOL AKTİF")
        vars = {'rovs': self.rovs, 'app': self, 'filo': self.filo}
        vars.update(self.konsol_verileri)
        local_ns = dict(globals(), **vars)

        class _DirtyConsole(code.InteractiveConsole):
            def __init__(self, locals=None, ortam_ref=None):
                super().__init__(locals=locals)
                self._ortam_ref = ortam_ref

            def runcode(self, code):
                try:
                    super().runcode(code)
                finally:
                    try:
                        if self._ortam_ref is not None:
                            self._ortam_ref.mark_ui_state_dirty()
                    except Exception:
                        pass

        _DirtyConsole(locals=local_ns, ortam_ref=self).interact(banner="")
