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

        self.group_id = group_id # Grup ID bilgisi

        # Havuz: sonar ve lidar kesikli çizgileri tek sefer oluştur; çizimde sadece güncelle/göster/gizle
        self._cizgi_havuzlari_olustur()

    def ekle(self, ortam_ref):
        if not ortam_ref: return False
        self.environment_ref = ortam_ref
        if not hasattr(ortam_ref, 'rovs'): ortam_ref.rovs = []
        
        # ID'yi mevcut maksimumdan bir ileri ata (yeniden numaralandirma yok)
        mevcut_ids = [getattr(r, 'id') for r in getattr(ortam_ref, 'rovs', []) if r is not None and hasattr(r, 'id')]
        self.id = (max(mevcut_ids) + 1) if mevcut_ids else 0
        if hasattr(ortam_ref, 'rovs') and isinstance(ortam_ref.rovs, list):
            ortam_ref.rovs.append(self)
        
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

            # 2. Görselleri temizle (havuz konteynerleri)
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
                 "lidar": self.son_lidar_mesafeleri, "group_id": self.group_id}
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
        elif ayar == "yaw":
            self.rotation_y = float(deger)
            self.rotation = Vec3(0, self.rotation_y, 0)
        elif ayar in self.sensor_config:
            self.sensor_config[ayar] = deger

    def move(self, komut, guc=1.0):
        if self.battery <= 0: return

    def _cizgi_havuzlari_olustur(self):
        """Sonar ve lidar kesikli çizgileri için entity havuzunu tek sefer oluşturur. Çizimde sadece güncelle/göster/gizle."""
        n = self.CIZGI_HAVUZ_SEGMENT
        # Sonar (engel) çizgisi havuzu
        self.engel_cizgi = Entity(parent=self)
        self.engel_cizgi._segments = []
        for _ in range(n):
            seg = Entity(parent=self.engel_cizgi, model='cube', scale=(.1, .1, .5), color=color.red, unlit=True)
            self.engel_cizgi._segments.append(seg)
        self.engel_cizgi.enabled = False
        # Lidar (4 yön) çizgisi havuzları
        for lidar_id in (0, 1, 2, 3):
            cont = Entity(parent=self)
            cont._segments = []
            for _ in range(n):
                seg = Entity(parent=cont, model='cube', scale=(0.05, 0.05, 0.3), color=color.cyan, alpha=0.6, unlit=True)
                cont._segments.append(seg)
            cont.enabled = False
            self.lidar_cizgileri[lidar_id] = cont

    def _kesikli_cizgi_ciz(self, hedef, mesafe):
        """Sonar engel çizgisi. Havuz zaten __init__'te oluşturuldu; sadece konum/renk/göster-gizle güncellenir."""
        cizgi = self.engel_cizgi
        if cizgi is None:
            return
        seg_list = cizgi._segments
        max_seg = len(seg_list)
        c = color.red if mesafe < 5 else (color.orange if mesafe < 10 else color.yellow)
        delta = hedef - self.position
        if delta.is_nan() or delta.length() <= 1e-6:
            cizgi.enabled = False
            for seg in seg_list:
                seg.enabled = False
            return
        yon = delta.normalized()
        n_use = min(int(mesafe), max_seg)
        for i in range(n_use):
            seg = seg_list[i]
            seg.position = self.position + yon * (i + 0.5)
            seg.color = c
            if not (hedef - seg.position).is_nan() and (hedef - seg.position).length() > 1e-6:
                seg.look_at(hedef)
            seg.enabled = True
        for i in range(n_use, max_seg):
            seg_list[i].enabled = False
        cizgi.enabled = True

    def _lidar_cizgi_ciz(self, lidar_id, hedef, mesafe, renk):
        """Lidar engel çizgisi. Havuz zaten __init__'te oluşturuldu; sadece konum/renk/göster-gizle güncellenir."""
        cont = self.lidar_cizgileri.get(lidar_id)
        if cont is None:
            return
        seg_list = cont._segments
        max_seg = len(seg_list)
        delta = hedef - self.position
        if delta.is_nan() or delta.length() <= 1e-6:
            cont.enabled = False
            for seg in seg_list:
                seg.enabled = False
            return
        yon = delta.normalized()
        n_use = min(int(mesafe), max_seg)
        for i in range(n_use):
            seg = seg_list[i]
            seg.position = self.position + yon * (i + 0.5)
            seg.color = renk
            if not (hedef - seg.position).is_nan() and (hedef - seg.position).length() > 1e-6:
                seg.look_at(hedef)
            seg.enabled = True
        for i in range(n_use, max_seg):
            seg_list[i].enabled = False
        cont.enabled = True
    
    def _lidar_cizgi_temizle(self, lidar_id):
        """Belirli bir lidar çizgisini gizle (entity'leri yok etmeden)."""
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
        self.havuz_genisligi = getattr(ortam_ref, 'havuz_genisligi', HavuzAyarlari.HAVUZ_GENISLIK)
        
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
        """APF vektorleri: create-once havuz kullanir, her karede sadece mesh/renk güncellenir."""
        filo = getattr(self.ortam_ref, 'filo', None)
        helper = getattr(filo, 'helper', None) if filo else None
        apf_list = helper.get_apf_vektor_verts_list(self) if helper else []
        vektor_renkler = {
            'k': color.red, 'y': color.green, 'm': color.blue,
            's': color.yellow, 't': color.orange
        }
        z_line = -0.35
        if apf_list:
            sig_sum = 0.0
            for verts, _ in apf_list:
                if not verts:
                    continue
                x0, y0 = verts[0][0], verts[0][1]
                x1, y1 = verts[1][0], verts[1][1]
                sig_sum += round(float(x0), 4) + round(float(y0), 4) + round(float(x1), 4) + round(float(y1), 4)  # type: ignore
            sig = (len(apf_list), round(float(sig_sum), 4))  # type: ignore
        else:
            sig = (0, 0.0)
        if self._apf_cache_sig != sig:
            self._apf_cache_sig = sig  # type: ignore
            n_use = min(len(apf_list), self._apf_vektor_pool_size)
            if self._apf_vektor_pool is None:
                self._apf_vektor_pool = []  # type: ignore
                for _ in range(self._apf_vektor_pool_size):
                    mesh = Mesh(vertices=[(0, 0, z_line), (0, 0, z_line)], mode='line', thickness=2)
                    e = Entity(
                        parent=self,
                        model=mesh,
                        color=color.white,
                        alpha=0.95,
                        z=z_line
                    )
                    self._apf_vektor_pool.append(e)  # type: ignore
            pool = self._apf_vektor_pool
            # Titremeyi onlemek icin koordinatlari yuvarla (kucuk degisimler cizimi degistirmez)
            round_ = 3
            for i in range(n_use):
                verts, c_code = apf_list[i]  # type: ignore
                if not verts or len(verts) < 2:
                    pool[i].enabled = False  # type: ignore
                    continue
                c = vektor_renkler.get(c_code, color.white)
                verts_stable = [tuple(round(float(v), int(round_)) for v in pt) for pt in verts]  # type: ignore
                pool[i].model.vertices = verts_stable  # type: ignore
                pool[i].model.generate()  # type: ignore
                pool[i].color = c  # type: ignore
                pool[i].enabled = True  # type: ignore
            for i in range(n_use, len(pool if pool is not None else [])):  # type: ignore
                pool[i].enabled = False  # type: ignore

    def _apf_vektorlari_temizle(self):
        """APF vektorlerini gizler (havuz entity'leri yok edilmez, sadece cache sifirlanir)."""
        self._apf_cache_sig = None
        if self._apf_vektor_pool:
            for e in (self._apf_vektor_pool or []):
                e.enabled = False  # type: ignore

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
            Create-once: konteyner ve nokta entity havuzu bir kez oluşturulur, her cagrida sadece konumlar güncellenir.
            """
            if not points:
                if hasattr(self, 'ada_cevre_entity') and self.ada_cevre_entity:
                    self.ada_cevre_entity.enabled = False  # type: ignore
                return
            ada_renk = color.hex('#CD853F')
            max_nokta = 2000
            n_use = min(len(points), max_nokta)
            if not hasattr(self, 'ada_cevre_entity') or self.ada_cevre_entity is None:
                self.ada_cevre_entity = Entity(parent=self)
                self.ada_cevre_entity._ada_noktalari = []  # type: ignore
                for _ in range(max_nokta):
                    e = Entity(
                        parent=self.ada_cevre_entity,
                        model='circle',
                        scale=0.01,
                        position=(0, 0, -0.28),
                        color=ada_renk,
                        alpha=0.85
                    )
                    self.ada_cevre_entity._ada_noktalari.append(e)  # type: ignore
            pool = getattr(self.ada_cevre_entity, '_ada_noktalari', [])
            for i in range(n_use):
                p = points[i]
                mp = self.dunya_to_harita(p[0], p[1] if len(p) > 1 else 0)
                pool[i].position = (mp.x, mp.y, -0.28)  # type: ignore
                pool[i].enabled = True  # type: ignore
            for i in range(n_use, len(pool)):
                pool[i].enabled = False  # type: ignore
            self.ada_cevre_entity.enabled = True  # type: ignore

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
    def __init__(self, verbose=False):
        self.verbose = verbose
        # Pencere ayarlarını _setup_window içinde yapacağımız için burada temel başlatma yapıyoruz
        self.app = Ursina(
            vsync=False, 
            development_mode=False, 
            show_ursina_splash=False, 
            borderless=False,
            title="FıratROVNet Simülasyonu"
        )


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
        
        # Fiziksel Sabitler (havuz boyutu config'den)
        self.havuz_genisligi = HavuzAyarlari.HAVUZ_GENISLIK
        self.su_hacmi_yuksekligi, self.su_hacmi_merkez_y = 50, -25
        self.WATER_SURFACE_Y_BASE, self.SEA_FLOOR_Y = 0.0, -50.0
        self.SONAR_MENZILI = GATLimitleri.ILETISIM_MENZILI

        self.loader = EntityLoader(self)
        self.helper = OrtamHelper(self)
        
        self.rovs, self.island_positions, self.island_entities = [], [], []
        self._g_rovs={}
        self.islands=self.island_entities
        self.engel_bulutu, self.konsol_verileri = [], {}
        self.sonar_cizgiler, self.filo = {}, None
        self.pool_human = None

        # --- KURULUM ---
        self._setup_window()
        self._setup_lighting()
        self.minimap = Minimap(ortam_ref=self)
        
        self.camera = EditorCamera(
            enabled=True, rotate_speed=15, pan_speed=(10, 10),
            zoom_speed=1, position=(0, 0, -50), rotation=(20, 0, 0)
        )
        mouse.visible, mouse.locked = True, False
        


    @property
    def g_rovs(self):
        self._g_rovs={}
        for rov in self.rovs:
            if not rov or getattr(rov, 'is_destroyed', False):
                continue
            __group_id=getattr(rov, 'group_id', 0)
            if not self._g_rovs.get(__group_id,False):
                self._g_rovs[__group_id]=[]
            self._g_rovs[__group_id].append(rov)
        return self._g_rovs

    def _setup_window(self):
        """ESKİ AYARLAR: Pencere konfigürasyonu."""
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

    def konsola_ekle(self, isim, nesne): self.konsol_verileri[isim] = nesne
    
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
    

    def _find_safe_rov_spawn_pos(self, group_config: tuple, alan_genisligi=100, bosluk=10, havuz_genisligi=None):
        """
        group_config: (3, 4, 1) gibi bir tuple alır.
        ROV dağılım sınırları havuz boyutuna göre (havuz_genisligi) belirlenir.
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

    def sim_olustur(self, n_rovs=(6,), n_islands=5, n_rocks=10, rov_model='submarine', havuz_genisligi: float | None = None):
        if havuz_genisligi is None:
            havuz_genisligi = HavuzAyarlari.HAVUZ_GENISLIK
        self.havuz_genisligi = havuz_genisligi
        oran = havuz_genisligi / 200

        # Temizlik: ROV, adalar + eski su/deniz tabanı (havuz küçülünce deniz kumu da küçülsün)
        for obj in [r for r in self.rovs if r] + [i for i in self.island_entities if i]:
            if obj: destroy(obj)
        for attr in ('water_volume', 'ocean_surface', 'ocean_taban', 'seabed', 'cimen_katmani'):
            if hasattr(self, attr):
                e = getattr(self, attr, None)
                if e is not None: destroy(e)
        self.rovs, self.island_entities, self.island_positions, self.engel_bulutu = [], [], [], []

        # Dünya İnşası
        size = havuz_genisligi * 2
        self.loader.build_ocean(size=size)
        self.loader.build_seabed(size=size)
        self.loader.load_pool_human(havuz_genisligi=havuz_genisligi)
        self.loader.build_boundaries(havuz_genisligi)
        
        # Kayaları ekle (n_rocks parametresi ile)
        if n_rocks > 0:
            self.loader.spawn_rocks(count=n_rocks, havuz_genisligi=havuz_genisligi)
        
        # 1. Adaları Sabit Noktalardan Yerleştir (pozisyonlar havuz oranına göre ölçeklenir)
        count = min(n_islands, len(self.FIXED_ISLAND_POSITIONS))
        chosen_islands = random.sample(self.FIXED_ISLAND_POSITIONS, count)
        chosen_islands.insert(0, (0, 0))
        for i, pos in enumerate(chosen_islands):
            # Referans 200m için -150..150; havuz 100 ise -75..75 olacak şekilde ölçekle
            if pos == (0, 0):
                scaled_pos = (0, 0)
            else:
                scaled_pos = (pos[0] * oran, pos[1] * oran)
            self.Ada(i, x="ekle", y=scaled_pos)

        # 2. ROV'ları Güvenli Noktalara Yerleştir
        # n_rovs tuple'ından toplam ROV sayısını hesapla
        toplam_rov_sayisi = sum(n_rovs)
        print(f"🌊 Simülasyon Başlatılıyor: {toplam_rov_sayisi} ROV, {count} Ada")

        all_group = self._find_safe_rov_spawn_pos(n_rovs, havuz_genisligi=havuz_genisligi)

        # Global ROV ID counter
        global_rov_id: int = 0
        
        for group_id, rovlar in enumerate(all_group):
            for local_rov_id, rov_koordinat in enumerate(rovlar):
                # sim_pos: (x, z_depth, y_coordinate) -> ursina: (x, y, z)
                u_pos = Vec3(rov_koordinat[0], rov_koordinat[2], rov_koordinat[1])
                    
                new_rov = ROV(rov_id=global_rov_id, group_id=group_id, position=u_pos, loader_ref=self.loader, model_key=rov_model)
                new_rov.ekle(self)
                
                global_rov_id = int(global_rov_id + 1)  # type: ignore

                self.minimap._statik_yeniden_ciz()


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
            ent, radius = self.loader.create_island(y[0], y[1])
            self.island_entities[ada_id], self.island_positions[ada_id] = ent, (y[0], y[1], radius)
            
    def guncelle_sonar_cizgileri(self):
            """
            ROV'lar arası sonar iletişimini HACİMLİ KESİKLİ çizgilerle gösterir.
            Create-once per pair: konteyner ve segmentler bir kez oluşturulur, her karede sadece konum/ölçek güncellenir.
            """
            active_rovs = [r for r in self.rovs if r and not (hasattr(r, 'is_destroyed') and r.is_destroyed)]
            bu_frame_aktif_olanlar = set()
            
            BASE_KALINLIK = 0.12
            segment_boyu = 1.2
            bosluk_boyu = 1.8
            adim_toplam = segment_boyu + bosluk_boyu
            max_segments = max(1, int(self.SONAR_MENZILI / adim_toplam) + 1)

            for i, r1 in enumerate(active_rovs):
                for r2 in active_rovs[i+1:]:  # type: ignore
                    if r1.gat_kodu == 3 or r2.gat_kodu == 3:
                        continue

                    p1, p2 = r1.position, r2.position
                    delta = p2 - p1
                    if delta.is_nan():
                        continue
                    dist = delta.length()
                    
                    if 1e-6 < dist < self.SONAR_MENZILI:
                        pair = tuple(sorted((r1.id, r2.id)))
                        bu_frame_aktif_olanlar.add(pair)
                        
                        oran = dist / self.SONAR_MENZILI
                        carpan = float(lerp(1.25, 0.75, oran) or 1.0)
                        guncel_kalinlik = BASE_KALINLIK * carpan
                        if dist < 25:
                            c = color.red
                        elif dist < 80:
                            c = color.orange
                        else:
                            c = color.white
                        
                        yon_vec = delta.normalized()
                        # Create-once: konteyner ve segment listesi
                        if pair not in self.sonar_cizgiler:
                            cizgi_konteyner = Entity(add_to_scene_entities=True)
                            seg_list = []
                            for _ in range(max_segments):
                                parca = Entity(
                                    parent=cizgi_konteyner,
                                    model='cube',
                                    color=c,
                                    unlit=True
                                )
                                seg_list.append(parca)
                            self.sonar_cizgiler[pair] = (cizgi_konteyner, seg_list)
                        
                        konteyner, seg_list = self.sonar_cizgiler[pair]
                        curr = 0
                        idx = 0
                        while curr < dist and idx < len(seg_list if seg_list is not None else []):  # type: ignore
                            kalin_uzunluk = min(segment_boyu, dist - curr)  # type: ignore
                            if kalin_uzunluk <= 0.1:
                                break
                            parca_baslangic = p1 + yon_vec * float(curr)
                            parca_bitis = parca_baslangic + yon_vec * float(kalin_uzunluk)  # type: ignore
                            orta_nokta = (parca_baslangic + parca_bitis) / 2
                            parca = seg_list[idx]  # type: ignore
                            parca.position = orta_nokta
                            parca.scale = (guncel_kalinlik, guncel_kalinlik, kalin_uzunluk)
                            parca.color = c
                            if not (parca_bitis - orta_nokta).is_nan() and (parca_bitis - orta_nokta).length() > 1e-6:
                                parca.look_at(parca_bitis)
                            parca.enabled = True
                            idx += 1
                            curr += adim_toplam
                        for j in range(idx, len(seg_list if seg_list is not None else [])):  # type: ignore
                            seg_list[j].enabled = False  # type: ignore

            # Temizlik: sadece artık menzilde olmayan çiftleri sil (list kopyası üzerinden)
            for pair in list(self.sonar_cizgiler.keys()):
                if pair not in bu_frame_aktif_olanlar:
                    data = self.sonar_cizgiler[pair]
                    if isinstance(data, tuple):
                        destroy(data[0])
                    else:
                        destroy(data)
                    self.sonar_cizgiler.pop(pair, None)

    def run(self, interaktif=False):
        if interaktif: threading.Thread(target=self._start_shell, daemon=True).start()
        self.app.run()

    def _start_shell(self):
        import time; time.sleep(1.5)
        print("\n🚀 FIRAT ROVNET CANLI KONSOL AKTİF")
        vars = {'rovs': self.rovs, 'app': self, 'filo': self.filo}
        vars.update(self.konsol_verileri)
        code.interact(local=dict(globals(), **vars))