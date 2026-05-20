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
    FizikSabitleri, ROVModelleri, PerformansAyarlari
)
from FiratROVNet.utils import sim_to_ursina, ursina_to_sim  # type: ignore[import-not-found]
from FiratROVNet.kutuphane.helper.EntityLoader import EntityLoader  # type: ignore[import-not-found]
from FiratROVNet.kutuphane.helper.simulasyon_helper import OrtamHelper  # type: ignore[import-not-found]


def rov_aktif_mi(rov):
    if rov is None or getattr(rov, 'is_destroyed', False):
        return False
    try:
        is_empty = getattr(rov, 'is_empty', None)
        if callable(is_empty) and is_empty():
            return False
    except Exception:
        return False
    return True

# ============================================================
# 1. ROV SINIFI (Mantık ve Fizik)
# ============================================================
class ROV(Entity):
    """Kesikli çizgi segment sayısı (havuz boyutu). Create-once, sonra sadece gösterme/gizleme."""
    CIZGI_HAVUZ_SEGMENT = 25

    def __init__(self, rov_id=None, group_id=None, loader_ref=None, model_key='submarine', role=None, rol=None, **kwargs):
        super().__init__()
        self.motorlar = []
        self.id = rov_id
        self.environment_ref = None
        self._group_id = group_id
        
        # Fiziksel ve Durumsal Durum
        self.velocity = Vec3(0, 0, 0)
        ilk_rol = role if role is not None else rol
        self.battery, self.role, self.gat_kodu = 1.0, int(ilk_rol) if ilk_rol is not None else 0, 0
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

        # Havuz: performans modunda çizgi Entity'leri ilk ihtiyaç anında oluşturulur.
        self._cizgi_havuzlari_olustur()

    def ekle(self, ortam_ref):
        if not ortam_ref: return False
        self.environment_ref = ortam_ref
        if not hasattr(ortam_ref, 'rovs'): ortam_ref.rovs = []
        if any(r is self and rov_aktif_mi(r) for r in getattr(ortam_ref, 'rovs', [])):
            print(f"ℹ️ ROV-{self.id} zaten ortama ekli; tekrar ekleme atlandi.")
            return True
        
        # ID'yi mevcut maksimumdan bir ileri ata (yeniden numaralandirma yok)
        mevcut_ids = [getattr(r, 'id') for r in getattr(ortam_ref, 'rovs', []) if rov_aktif_mi(r) and hasattr(r, 'id')]
        self.id = (max(mevcut_ids) + 1) if mevcut_ids else 0
        # group_id None → üs (grupsuz); UI veya kullanıcı gruplama yapana kadar atanmaz
        if hasattr(ortam_ref, 'rovs') and isinstance(ortam_ref.rovs, list):
            ortam_ref.rovs.append(self)
        
        self._etiket_guncelle()
        filo_attr = getattr(ortam_ref, 'filo', None)
        if filo_attr is not None and hasattr(filo_attr, 'rov_sisteme_ekle'):
            if not filo_attr.rov_sisteme_ekle(self):
                print(f"⚠️ ROV-{self.id} ortama eklendi ancak Filo sistem kurulumu tamamlanamadi.")
                return False
        return True

    @property
    def group_id(self):
        return self._group_id

    def _usye_normalize(self):
        """group_id=None iken rol/mod ve filo hedeflerini üs (grupsuz) durumuna çeker."""
        if int(getattr(self, 'role', 0) or 0) != 0:
            self.role = 0
        gnc = getattr(self, 'gnc', None)
        if gnc is not None and int(getattr(gnc, 'mod', 0) or 0) != 0:
            gnc.mod = 0
        ortam = getattr(self, 'environment_ref', None)
        filo = getattr(ortam, 'filo', None) if ortam is not None else None
        if filo is not None:
            rid = getattr(self, 'id', None)
            if rid is not None:
                getattr(filo, '_rov_hedefleri', {}).pop(int(rid), None)
        if hasattr(self, '_etiket_guncelle'):
            self._etiket_guncelle()

    @group_id.setter
    def group_id(self, deger):
        eski = getattr(self, '_group_id', None)
        yeni = None if deger is None else int(deger)
        if eski == yeni:
            return
        self._group_id = yeni
        if yeni is None:
            self._usye_normalize()
        ortam = getattr(self, 'environment_ref', None)
        dirty = getattr(ortam, 'mark_ui_state_dirty', None) if ortam is not None else None
        if callable(dirty):
            dirty()

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
                    if r and (r is self or getattr(r, 'id', None) == silinen_id):
                        rovs[idx] = None

            # 2. Görselleri temizle (havuz konteynerleri)
            if hasattr(self, 'label') and self.label: destroy(self.label)
            if hasattr(self, 'safety_zone') and self.safety_zone: destroy(self.safety_zone)
            if hasattr(self, 'engel_cizgi') and self.engel_cizgi: destroy(self.engel_cizgi)
            for lidar_id in (0, 1, 2, 3):
                cont = getattr(self, 'lidar_cizgileri', {}).get(lidar_id)
                if cont: destroy(cont)
            physics_node = getattr(self, 'physics_node', None)
            physics_np = getattr(self, 'physics_np', None)
            filo_attr = getattr(ortam, 'filo', None)
            if filo_attr is not None and physics_node is not None:
                try:
                    filo_attr.world.removeRigidBody(physics_node)
                except Exception:
                    pass
            if physics_np is not None:
                try:
                    physics_np.removeNode()
                except Exception:
                    pass

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
        elif ayar == "yaw":
            self.rotation_y = float(deger)
            self.rotation = Vec3(0, self.rotation_y, 0)
        elif ayar in self.sensor_config:
            self.sensor_config[ayar] = deger

    def move(self, komut, guc=1.0):
        if self.battery <= 0: return

    def _cizgi_havuzlari_olustur(self):
        """Sonar ve lidar kesikli çizgileri için tekil mesh nesnelerini oluşturur."""
        if getattr(PerformansAyarlari, "ROV_SENSOR_CIZGILERI_LAZY", True):
            return
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
            cizgi = Entity(add_to_scene_entities=True, enabled=False, unlit=True)
            self.engel_cizgi = cizgi
        
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
            cont = Entity(add_to_scene_entities=True, enabled=False, alpha=0.6, unlit=True)
            self.lidar_cizgileri[lidar_id] = cont
            
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
        self._alan_secim_gecici_gorseller = []
        self._alan_gorev_gorseller = []
        self.alan_secim_noktalari = []
        self.alan_gorev_noktalari = []

    def _alan_cizim_entityleri(self) -> set:
        """Alan seçim/görev çizimleri — goster() bunları yanlışlıkla kapatmasın."""
        ents: set = set()
        for lst in (
            getattr(self, "_alan_secim_gecici_gorseller", None),
            getattr(self, "_alan_gorev_gorseller", None),
        ):
            if not lst:
                continue
            for ent in lst:
                if ent is None:
                    continue
                ents.add(ent)
                for sub in getattr(ent, "children", []) or []:
                    ents.add(sub)
        return ents

    def alan_gorev_gorsel_yenile(self):
        """goster() sonrası alan çizimlerinin görünür kalmasını sağlar."""
        if not self.visible:
            return
        for ent in self._alan_cizim_entityleri():
            try:
                ent.enabled = True  # type: ignore
            except Exception:
                pass

    # --- KRİTİK GÜNCELLEME: goster metodu ---
    def goster(self, durum=True, convex=False, a_star=False, scale=None, **kwargs):
        """GNC sisteminden gelen convex ve a_star parametrelerini karşılar."""
        if scale:
            eff = self.BASE_SCALE * float(scale)
            self.scale = (eff, eff)
        
        self.visible = durum
        alan_korunan = self._alan_cizim_entityleri()
        # Tüm çocukları (grid, ikonlar vb) toplu kapat/aç — alan çizgileri ayrı
        for child in getattr(self, 'children', []):
            if child in alan_korunan:
                child.enabled = True if durum else False  # Alan çizgileri minimap görünürse daima kalsın
            else:
                child.enabled = durum  # type: ignore
        
        # Hull ve Path görünürlüğünü özel olarak ayarla
        h_entity = getattr(self, 'hull_entity', None)
        p_entity = getattr(self, 'path_entity', None)
        if h_entity is not None: h_entity.enabled = (convex and durum)  # type: ignore
        if p_entity is not None: p_entity.enabled = (a_star and durum)  # type: ignore
        self.alan_gorev_gorsel_yenile()

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
        alan = getattr(self.ortam_ref, 'ileri_karakol_alani', None)
        if alan:
            x_min, x_max = alan.get("x", (150.0, 200.0))
            y_min, y_max = alan.get("y", (150.0, 200.0))
            f = 1.0 / (self.havuz_genisligi * 2)
            cx, cy = ((x_min + x_max) / 2.0) * f, ((y_min + y_max) / 2.0) * f
            sx, sy = (x_max - x_min) * f, (y_max - y_min) * f
            dolgu = Entity(
                parent=self, model='quad',
                position=(cx, cy, -0.235), scale=(sx, sy),
                color=color.rgb(22/255, 130/255, 145/255), unlit=True, transparent=True, alpha=0.36
            )
            cekirdek = Entity(
                parent=self, model='quad',
                position=(cx, cy, -0.252), scale=(sx * 0.42, sy * 0.42),
                color=color.rgb(238/255, 244/255, 236/255), unlit=True, transparent=True, alpha=0.58
            )
            verts = [
                (x_min * f, y_min * f, -0.245),
                (x_max * f, y_min * f, -0.245),
                (x_max * f, y_max * f, -0.245),
                (x_min * f, y_max * f, -0.245),
                (x_min * f, y_min * f, -0.245),
            ]
            kontur = Entity(
                parent=self,
                model=Mesh(vertices=verts, mode='line', thickness=3),
                color=color.rgb(54/255, 220/255, 210/255),
                alpha=0.9,
                unlit=True,
            )
            etiket = Text(
                text="KARAKOL",
                parent=self,
                position=(cx, cy + sy * 0.62, -0.255),
                scale=0.65,
                color=color.azure,
                origin=(0, 0),
            )
            self.statik_nesneler.extend([dolgu, cekirdek, kontur, etiket])

    def gorsel_guncelle(self):
            if not self.visible or not self.ortam_ref: return
            
            if hasattr(self.ortam_ref, 'rovs'):
                mevcut_rovlar = [r for r in list(self.ortam_ref.rovs) if rov_aktif_mi(r)]
                active_ids = {getattr(r, 'id', -1) for r in mevcut_rovlar}
                
                for rid in list(self.rov_ikonlari.keys()):
                    if rid not in active_ids:
                        destroy(self.rov_ikonlari[rid])
                        self.rov_ikonlari.pop(rid, None)

                for rov in mevcut_rovlar:
                    try:
                        target = self.dunya_to_harita(rov.x, rov.z)
                    except AssertionError:
                        continue

                    if getattr(rov, 'id', -1) not in self.rov_ikonlari:
                        self.rov_ikonlari[getattr(rov, 'id', -1)] = self.loader.create_rov_icon(self, getattr(rov, 'id', -1), getattr(rov, 'color', color.white))
                    icon = self.rov_ikonlari.get(getattr(rov, 'id', -1))
                    if icon:
                        icon.position = target
                        icon.rotation_z = -float(rov.rotation_y)
                        icon.color = getattr(rov, 'color', color.white)

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

    # ── UI alan seçimi (çokgen) ─────────────────────────────────────────────
    _ALAN_CIZGI_KALINLIK = 2.0   # seçim/görev çizgisi (önceki 4/5 değerinin ~0.5 katı)
    _ALAN_DOLGU_KALINLIK = 0.5
    _ALAN_Z_KENAR = -0.15        # grid (-0.1) üstünde görünsün
    _ALAN_Z_TARAMA = -0.16
    _ALAN_Z_NOKTA = -0.17
    _ALAN_TARAMA_CIZGI_KALINLIK = 1.0

    @staticmethod
    def _poligon_x_kesim_y_degerleri(noktalar: list, x: float) -> list[float]:
        """Dikey doğrunun çokgenle kesişim y değerleri (sim düzlemi)."""
        ys: list[float] = []
        n = len(noktalar)
        for i in range(n):
            x1, y1 = float(noktalar[i][0]), float(noktalar[i][1])
            x2, y2 = float(noktalar[(i + 1) % n][0]), float(noktalar[(i + 1) % n][1])
            if abs(x1 - x2) < 1e-9:
                if abs(x - x1) < 1e-6:
                    ys.extend([y1, y2])
                continue
            if x < min(x1, x2) - 1e-9 or x > max(x1, x2) + 1e-9:
                continue
            t = (x - x1) / (x2 - x1)
            if -1e-9 <= t <= 1.0 + 1e-9:
                ys.append(y1 + t * (y2 - y1))
        ys.sort()
        return ys

    def _poligon_tarama_seritleri(
        self, noktalar: list, serit_araligi: float = 15.0
    ) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        """Alan tarama ile aynı mantıkta dikey boustrophedon şeritleri (sim XY)."""
        if len(noktalar) < 3:
            return []
        xs = [float(p[0]) for p in noktalar]
        x_min, x_max = min(xs), max(xs)
        aralik = max(1.0, float(serit_araligi))
        segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
        x = x_min + aralik * 0.5
        ters = False
        while x <= x_max + 1e-6:
            ys = self._poligon_x_kesim_y_degerleri(noktalar, x)
            for j in range(0, len(ys) - 1, 2):
                if j + 1 >= len(ys):
                    break
                ya, yb = ys[j], ys[j + 1]
                if abs(yb - ya) < 0.5:
                    continue
                if ters:
                    segments.append(((x, yb), (x, ya)))
                else:
                    segments.append(((x, ya), (x, yb)))
            x += aralik
            ters = not ters
        return segments

    def _alan_tarama_deseni_ciz(self, noktalar: list, serit_araligi: float = 15.0) -> list:
        """Çokgen içine lawnmower tarama çizgileri."""
        from ursina import Entity, Mesh, color  # type: ignore[import-not-found]

        segmentler = self._poligon_tarama_seritleri(noktalar, serit_araligi)
        if not segmentler:
            return []
        verts: list[tuple[float, float, float]] = []
        for (x0, y0), (x1, y1) in segmentler:
            p0 = self.dunya_to_harita(x0, y0)
            p1 = self.dunya_to_harita(x1, y1)
            verts.append((p0.x, p0.y, self._ALAN_Z_TARAMA))
            verts.append((p1.x, p1.y, self._ALAN_Z_TARAMA))
        if len(verts) < 2:
            return []
        return [
            Entity(
                parent=self,
                model=Mesh(vertices=verts, mode="line", thickness=self._ALAN_TARAMA_CIZGI_KALINLIK),
                color=color.rgba(0, 220, 200, 140),
                enabled=self.visible,
                unlit=True,
                add_to_scene_entities=False,
            )
        ]

    def alan_secim_baslat(self):
        """Haritadan çokgen seçimi — yalnızca geçici köşe çizimini sıfırlar."""
        self.alan_secim_gecici_temizle()
        self.alan_secim_noktalari = []

    def alan_secim_gecici_temizle(self):
        """Sadece seçim sırasındaki turkuaz/kırmızı geçici çizgileri ve köşe noktalarını temizler."""
        from ursina import destroy  # type: ignore[import-not-found]
        if hasattr(self, '_gecici_secim_ent') and self._gecici_secim_ent:
            for ent in self._gecici_secim_ent:
                destroy(ent)
        self._gecici_secim_ent = []
        
        if hasattr(self, '_alan_secim_gecici_gorseller') and self._alan_secim_gecici_gorseller:
            for ent in self._alan_secim_gecici_gorseller:
                destroy(ent)
        self._alan_secim_gecici_gorseller = []




    def alan_gorev_temizle(self):
        """Tamamlanmış kalıcı görev alanını ve aktif görev çizimlerini temizler."""
        from ursina import destroy  # type: ignore[import-not-found]
        if hasattr(self, '_kalici_gorev_ent') and self._kalici_gorev_ent:
            for ent in self._kalici_gorev_ent:
                destroy(ent)
        self._kalici_gorev_ent = []
        
        for ent in list(getattr(self, "_alan_gorev_gorseller", []) or []):
            if ent is not None:
                destroy(ent)
        self._alan_gorev_gorseller = []
        self.alan_gorev_noktalari = []

    def alan_gecici_ciz(self, noktalar: list, mouse_pos=None):
        """Seçim sırasında fareyi takip eden anlık (geçici) alanı çizer."""
        self.alan_secim_gecici_temizle()
        if not noktalar:
            return
            
        from ursina import Entity, Mesh, color
        verts = []
        
        # Seçilen köşe noktaları
        for p in noktalar:
            mp = self.dunya_to_harita(p[0], p[1])
            verts.append((mp.x, mp.y, -0.15))
            
            nokta = Entity(
                parent=self, model='circle', scale=0.015,
                color=color.cyan, position=(mp.x, mp.y, -0.17),
                unlit=True, enabled=self.visible
            )
            self._gecici_secim_ent.append(nokta)

        # Fare imlecinin anlık pozisyonu
        if mouse_pos:
            mmp = self.dunya_to_harita(mouse_pos[0], mouse_pos[1])
            verts.append((mmp.x, mmp.y, -0.15))
            
            fare_nokta = Entity(
                parent=self, model='circle', scale=0.015,
                color=color.red, position=(mmp.x, mmp.y, -0.17),
                unlit=True, enabled=self.visible
            )
            self._gecici_secim_ent.append(fare_nokta)

        # Çizgileri birleştir
        if len(verts) > 1:
            cizgi = Entity(
                parent=self,
                model=Mesh(vertices=verts, mode='line', thickness=2),
                color=color.cyan, unlit=True, enabled=self.visible
            )
            self._gecici_secim_ent.append(cizgi)

    def _alan_poligon_ciz(
        self,
        noktalar: list,
        *,
        kapatildi: bool,
        kenar_kalinlik: float,
        dolgu_kalinlik: float,
    ) -> list:
        from ursina import Entity, Mesh, Text, color  # type: ignore[import-not-found]

        gorseller = []
        if not noktalar:
            return gorseller

        kenar_renk = color.rgba(255, 210, 0, 255) if kapatildi else color.rgba(0, 235, 255, 255)
        nokta_renk = color.rgba(120, 255, 200, 255) if kapatildi else color.rgba(0, 200, 255, 255)

        mp_noktalar = [self.dunya_to_harita(float(p[0]), float(p[1])) for p in noktalar]

        for i, mp in enumerate(mp_noktalar):
            ilk = i == 0
            olcek = (0.021 if ilk else 0.014)
            nokta = Entity(
                parent=self,
                model="circle",
                color=nokta_renk,
                scale=olcek,
                position=(mp.x, mp.y, self._ALAN_Z_NOKTA),
                enabled=self.visible,
                unlit=True,
                add_to_scene_entities=False,
            )
            gorseller.append(nokta)
            if ilk and kapatildi and len(mp_noktalar) >= 3:
                gorseller.append(
                    Text(
                        parent=nokta,
                        text="◎",
                        scale=22,
                        color=color.rgba(255, 230, 120, 255),
                        origin=(0, 0),
                    )
                )

        if len(mp_noktalar) >= 2:
            cizilecek = list(mp_noktalar)
            if kapatildi and len(cizilecek) >= 3:
                cizilecek = cizilecek + [cizilecek[0]]
            verts = [(v.x, v.y, self._ALAN_Z_KENAR) for v in cizilecek]
            gorseller.append(
                Entity(
                    parent=self,
                    model=Mesh(vertices=verts, mode="line", thickness=kenar_kalinlik),
                    color=kenar_renk,
                    enabled=self.visible,
                    unlit=True,
                    add_to_scene_entities=False,
                )
            )

        if kapatildi and len(mp_noktalar) >= 3:
            dolgu_verts = [(v.x, v.y, self._ALAN_Z_KENAR + 0.01) for v in mp_noktalar]
            dolgu_verts.append(dolgu_verts[0])
            gorseller.append(
                Entity(
                    parent=self,
                    model=Mesh(vertices=dolgu_verts, mode="line", thickness=dolgu_kalinlik),
                    color=color.rgba(0, 255, 180, 90),
                    enabled=self.visible,
                    unlit=True,
                    add_to_scene_entities=False,
                )
            )
        return gorseller

    def alan_secim_ciz(self, noktalar: list, kapatildi: bool = False):
        """Seçim sırasında geçici köşe/kenar çizimi (görev alanına dokunmaz)."""
        self.alan_secim_gecici_temizle()
        self.alan_secim_noktalari = [list(p) for p in (noktalar or [])]
        if not self.alan_secim_noktalari:
            return
        self._alan_secim_gecici_gorseller = self._alan_poligon_ciz(
            self.alan_secim_noktalari,
            kapatildi=kapatildi,
            kenar_kalinlik=self._ALAN_CIZGI_KALINLIK,
            dolgu_kalinlik=self._ALAN_DOLGU_KALINLIK,
        )

    def alan_gorev_goster(self, noktalar: list, serit_araligi: float = 15.0):
            """Seçim tamamlandıktan sonra görev bitene kadar KALICI alanı ve iç çapraz tarama çizgilerini gösterir."""
            from ursina import Entity, Mesh, color
            
            # Önce tüm geçici ve önceki kalıcı çizgileri temizleyelim ki üst üste binmesin
            self.alan_secim_gecici_temizle()
            self.alan_gorev_temizle()
            
            if not noktalar or len(noktalar) < 3:
                return
                
            # 1. HATA ÇÖZÜMÜ: Noktaları Temizleme
            # Görev başlatıldığında gelen noktalardaki tekrarları ve açık uçları temizliyoruz.
            temiz_noktalar = []
            for p in noktalar:
                mevcut = (float(p[0]), float(p[1]))
                # Ardışık tekrar eden noktaları atla
                if not temiz_noktalar or mevcut != temiz_noktalar[-1]:
                    temiz_noktalar.append(mevcut)
                    
            # Eğer son nokta, ilk noktanın aynısı ise onu çıkar (zaten zorla kapalı döngü çizeceğiz)
            if len(temiz_noktalar) > 2 and temiz_noktalar[-1] == temiz_noktalar[0]:
                temiz_noktalar.pop()
                
            n_points = len(temiz_noktalar)
            if n_points < 3:
                return
                
            self.alan_gorev_noktalari = [list(p) for p in temiz_noktalar]
            
            # --- 1. Kalıcı Sınır Çizgisi (Altın Rengi Çokgen) ---
            sinir_verts = []
            for i in range(n_points):
                p1 = temiz_noktalar[i]
                p2 = temiz_noktalar[(i + 1) % n_points] # (i+1) % N formülü son köşeyi İLK köşeye kesin bağlar
                
                mp1 = self.dunya_to_harita(p1[0], p1[1])
                mp2 = self.dunya_to_harita(p2[0], p2[1])
                
                sinir_verts.append((mp1.x, mp1.y, -0.15))
                sinir_verts.append((mp2.x, mp2.y, -0.15))
            
            kenarlik = Entity(
                parent=self,
                model=Mesh(vertices=sinir_verts, mode='lines', thickness=2.5),
                color=color.gold,
                unlit=True,
                enabled=self.visible
            )
            self._kalici_gorev_ent.append(kenarlik)
            
            # --- 2. İç Tarama Şeritleri (Sınır İhlalsiz Kusursuz Çapraz Tarama) ---
            tarama_verts = []
            
            # Çapraz çizgiler için x + z = c (45 derece) doğrusunu kullanacağız.
            c_degerleri = [p[0] + p[1] for p in temiz_noktalar]
            min_c = min(c_degerleri)
            max_c = max(c_degerleri)
            
            # Daha "sık" bir tarama deseni için görsel aralığı daraltıyoruz (%35'e indirdik)
            gorsel_aralik = max(1.5, serit_araligi * 0.35)
            
            c = min_c + gorsel_aralik
            while c < max_c:
                kesisimler = []
                
                # 2. HATA ÇÖZÜMÜ: Kusursuz Kesişim (Scanline Algoritması)
                for i in range(n_points):
                    p1 = temiz_noktalar[i]
                    p2 = temiz_noktalar[(i + 1) % n_points]
                    
                    x1, z1 = p1[0], p1[1]
                    x2, z2 = p2[0], p2[1]
                    
                    v1 = x1 + z1
                    v2 = x2 + z2
                    
                    if v1 == v2:
                        continue  # Çizgi kenara paralel ise yoksay
                    
                    # Sadece [min, max) aralığını kontrol et. 
                    # (Bu matematiksel kural taramanın tam köşeye denk gelmesi durumunda çizginin dışarı fırlamasını engeller)
                    v_min = min(v1, v2)
                    v_max = max(v1, v2)
                    
                    if v_min <= c < v_max:
                        # Çizginin kenarı tam olarak nerede kestiğini hesapla
                        t = (c - v1) / (v2 - v1)
                        kx = x1 + t * (x2 - x1)
                        kz = z1 + t * (z2 - z1)
                        kesisimler.append((kx, kz))
                
                # Kesişim noktalarını X eksenine göre sırala
                kesisimler.sort(key=lambda p: p[0])
                
                # Noktaları iç bölge olarak çiftler halinde bağla
                for i in range(0, len(kesisimler) - 1, 2):
                    p_bas = kesisimler[i]
                    p_bit = kesisimler[i+1]
                    
                    mp_bas = self.dunya_to_harita(p_bas[0], p_bas[1])
                    mp_bit = self.dunya_to_harita(p_bit[0], p_bit[1])
                    
                    # Z koordinatını -0.16 yaparak altın kenarlığın (-0.15) altında kalmasını sağlıyoruz
                    tarama_verts.append((mp_bas.x, mp_bas.y, -0.16))
                    tarama_verts.append((mp_bit.x, mp_bit.y, -0.16))
                    
                c += gorsel_aralik
                
            # Çizgileri ekrana altın sarısı (gold) renginde çizdir
            if tarama_verts:
                tarama_ent = Entity(
                    parent=self,
                    model=Mesh(vertices=tarama_verts, mode='lines', thickness=0.5),
                    color=color.gold,  # İÇ ÇİZGİLER ARTIK ALTIN SARISI
                    alpha=0.45,        # Sınır çizgisi kadar parlak olmaması için hafif saydam
                    unlit=True,
                    enabled=self.visible
                )
                self._kalici_gorev_ent.append(tarama_ent)
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
        
        # Fiziksel Sabitler
        self.havuz_genisligi = 200
        self.su_hacmi_yuksekligi, self.su_hacmi_merkez_y = 50, -25
        self.WATER_SURFACE_Y_BASE, self.SEA_FLOOR_Y = 0.0, -50.0
        self.SONAR_MENZILI = GATLimitleri.ILETISIM_MENZILI
        self.ileri_karakol_alani = {"x": (125.0, 200.0), "y": (125.0, 200.0), "depth": 0.0}
        self.ileri_karakol_gorselleri = []

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
            if not rov_aktif_mi(rov):
                continue
            __group_id=getattr(rov, 'group_id', None)
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
        window.color = color.rgb(10/255, 30/255, 50/255)
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
        # Gunesin maksimum parlakligini kisarak acik renkli yuzeylerde detay kaybini azalt.
        self.sun.color = color.rgba(0.75, 0.75, 0.75, 1.0)
        self.ambient = AmbientLight(color=color.rgba(120/255, 120/255, 120/255, 1.0))  # type: ignore
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

    def _rov_grup_konfig_normalize(self, n_rovs):
        if n_rovs is None:
            return ()
        if isinstance(n_rovs, int):
            return (n_rovs,) if n_rovs > 0 else ()
        try:
            return tuple(int(n) for n in n_rovs if int(n) > 0)
        except (TypeError, ValueError):
            return ()

    def sim_olustur(self, n_rovs=None, n_islands=5, n_rocks=20, havuz_genisligi=200, rov_model='submarine', seed=False):
        random_state = None
        if seed is not False and seed is not None:
            random_state = random.getstate()
            random.seed(seed)
            self.seed = seed
        else:
            self.seed = None

        try:
            self.havuz_genisligi = havuz_genisligi
            n_rovs = self._rov_grup_konfig_normalize(n_rovs)
            
            # Temizlik
            for obj in (
                [r for r in self.rovs if r]
                + [i for i in self.island_entities if i]
                + [e for e in getattr(self, 'ileri_karakol_gorselleri', []) if e]
                + [k for k in getattr(self.loader, 'rock_entities', []) if k]
            ):
                if obj: destroy(obj)
            self.rovs, self.island_entities, self.island_positions, self.engel_bulutu = [], [], [], []
            self.ileri_karakol_gorselleri = []
            self.loader.rock_entities = []
            
            # Dünya İnşası
            size = havuz_genisligi * 2
            self.loader.build_ocean(size=size)
            self.loader.build_seabed(size=size)
            self.loader.load_pool_human(havuz_genisligi=havuz_genisligi)
            self.loader.build_boundaries(havuz_genisligi)
            
            # 1. Adaları Sabit Noktalardan Yerleştir
            uygun_ada_pozisyonlari = [
                pos for pos in self.FIXED_ISLAND_POSITIONS
                if not self._ileri_karakol_icinde_mi(pos[0], pos[1], margin=20.0)
            ]
            count = min(n_islands, len(uygun_ada_pozisyonlari))
            chosen_islands = random.sample(uygun_ada_pozisyonlari, count)
            chosen_islands.insert(0,(0,0))
            for i, pos in enumerate(chosen_islands):
                self.Ada(i, x="ekle", y=pos)

            self.ileri_karakol_olustur()

            # Kayaları ekle (n_rocks parametresi ile)
            if n_rocks > 0:
                self.loader.spawn_rocks(count=n_rocks, havuz_genisligi=havuz_genisligi)

            # 2. ROV'ları Güvenli Noktalara Yerleştir
            # n_rovs tuple'ından toplam ROV sayısını hesapla
            toplam_rov_sayisi = sum(n_rovs)
            seed_metni = f", seed={seed}" if seed is not False and seed is not None else ""
            print(f"🌊 Simülasyon Başlatılıyor: {toplam_rov_sayisi} ROV, {count} Ada{seed_metni}")

            all_group = self._find_safe_rov_spawn_pos(n_rovs) if toplam_rov_sayisi > 0 else []

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
        finally:
            if random_state is not None:
                random.setstate(random_state)

    def _ileri_karakol_icinde_mi(self, x, y, margin=0.0):
        alan = getattr(self, 'ileri_karakol_alani', None) or {}
        x_min, x_max = alan.get("x", (150.0, 200.0))
        y_min, y_max = alan.get("y", (150.0, 200.0))
        return (x_min - margin) <= float(x) <= (x_max + margin) and (y_min - margin) <= float(y) <= (y_max + margin)

    def ileri_karakol_spawn_pozisyonu(self, rastgele=True):
        alan = getattr(self, 'ileri_karakol_alani', None) or {}
        x_min, x_max = alan.get("x", (150.0, 200.0))
        y_min, y_max = alan.get("y", (150.0, 200.0))
        depth = float(alan.get("depth", 0.0))
        if not rastgele:
            return ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0, depth)
        return (random.uniform(x_min + 5.0, x_max - 5.0), random.uniform(y_min + 5.0, y_max - 5.0), depth)

    def ileri_karakol_olustur(self):
        for ent in list(getattr(self, 'ileri_karakol_gorselleri', [])):
            if ent:
                destroy(ent)
        self.ileri_karakol_gorselleri = []

        alan = getattr(self, 'ileri_karakol_alani', None) or {}
        x_min, x_max = alan.get("x", (150.0, 200.0))
        y_min, y_max = alan.get("y", (150.0, 200.0))
        cx, cz = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0
        sx, sz = x_max - x_min, y_max - y_min
        surface_y = float(getattr(self, 'WATER_SURFACE_Y_BASE', 0.0)) + 2
        sea_floor_y = float(getattr(self, 'SEA_FLOOR_Y', -50.0))
        core_lift = 5.0
        elevated_y = surface_y + core_lift

        root = Entity(position=(0, 0, 0), name="ileri_karakol", add_to_scene_entities=True)
        self.ileri_karakol_gorselleri.append(root)

        govde_renk = color.rgb(18/255, 28/255, 34/255)
        guverte_renk = color.rgb(72/255, 86/255, 90/255)
        kenar_renk = color.rgb(44/255, 210/255, 198/255)
        cam_renk = color.rgb(105/255, 205/255, 232/255)
        panel_renk = color.rgb(20/255, 48/255, 78/255)
        isik_renk = color.rgb(255/255, 205/255, 82/255)

        verts = []
        tris = []
        colors = []

        def kutu_ekle(position, scale, renk):
            px, py, pz = position
            sx2, sy2, sz2 = scale[0] / 2.0, scale[1] / 2.0, scale[2] / 2.0
            base = len(verts)
            corners = [
                (px - sx2, py - sy2, pz - sz2), (px + sx2, py - sy2, pz - sz2),
                (px + sx2, py + sy2, pz - sz2), (px - sx2, py + sy2, pz - sz2),
                (px - sx2, py - sy2, pz + sz2), (px + sx2, py - sy2, pz + sz2),
                (px + sx2, py + sy2, pz + sz2), (px - sx2, py + sy2, pz + sz2),
            ]
            verts.extend(corners)
            colors.extend([renk] * 8)
            tris.extend([
                (base + 0, base + 1, base + 2), (base + 0, base + 2, base + 3),
                (base + 5, base + 4, base + 7), (base + 5, base + 7, base + 6),
                (base + 4, base + 0, base + 3), (base + 4, base + 3, base + 7),
                (base + 1, base + 5, base + 6), (base + 1, base + 6, base + 2),
                (base + 3, base + 2, base + 6), (base + 3, base + 6, base + 7),
                (base + 4, base + 5, base + 1), (base + 4, base + 1, base + 0),
            ])

        leg_top_y = elevated_y + 0.55
        leg_height = max(1.0, leg_top_y - sea_floor_y)
        leg_center_y = sea_floor_y + (leg_height / 2.0)
        bina_z = cz + sz * 0.08
        kule_x, kule_z = cx + sx * 0.26, cz + sz * 0.22

        kutu_ekle((cx, surface_y + 0.02, cz), (sx, 0.16, sz), color.rgb(36/255, 58/255, 64/255))
        kutu_ekle((cx, elevated_y + 0.33, cz), (sx * 0.72, 0.36, sz * 0.72), govde_renk)
        kutu_ekle((cx, elevated_y + 0.62, cz), (sx * 0.68, 0.18, sz * 0.68), guverte_renk)
        for pos, scale in (
            ((cx, elevated_y + 1.05, y_min + 1.0), (sx - 2.0, 0.32, 0.34)),
            ((cx, elevated_y + 1.05, y_max - 1.0), (sx - 2.0, 0.32, 0.34)),
            ((x_min + 1.0, elevated_y + 1.05, cz), (0.34, 0.32, sz - 2.0)),
            ((x_max - 1.0, elevated_y + 1.05, cz), (0.34, 0.32, sz - 2.0)),
        ):
            kutu_ekle(pos, scale, kenar_renk)
        for px in (x_min + 6.0, x_max - 6.0):
            for pz in (y_min + 6.0, y_max - 6.0):
                kutu_ekle((px, leg_center_y, pz), (1.35, leg_height, 1.35), color.rgb(42/255, 58/255, 60/255))
                kutu_ekle((px, elevated_y + 2.45, pz), (1.15, 1.15, 1.15), isik_renk)
        kutu_ekle((cx - sx * 0.05, elevated_y + 2.15, bina_z), (sx * 0.34, 3.0, sz * 0.22), color.rgb(160/255, 170/255, 175/255))
        kutu_ekle((cx - sx * 0.05, elevated_y + 2.58, bina_z - sz * 0.115), (sx * 0.31, 0.78, 0.6), cam_renk)
        kutu_ekle((cx - sx * 0.05, elevated_y + 2.58, bina_z + sz * 0.115), (sx * 0.31, 0.78, 0.6), cam_renk)
        kutu_ekle((cx - sx * 0.05, elevated_y + 3.86, bina_z), (sx * 0.39, 0.35, sz * 0.27), color.rgb(110/255, 120/255, 130/255))
        for px in (cx + sx * 0.15, cx + sx * 0.29):
            kutu_ekle((px, elevated_y + 1.65, cz - sz * 0.20), (sx * 0.16, 0.12, sz * 0.18), panel_renk)
            kutu_ekle((px, elevated_y + 1.73, cz - sz * 0.20), (sx * 0.14, 0.035, 0.18), color.rgb(82/255, 176/255, 220/255))
        kutu_ekle((cx, elevated_y + 0.82, y_min + 3.2), (sx * 0.36, 0.18, 5.8), color.rgb(92/255, 105/255, 104/255))
        kutu_ekle((cx, elevated_y + 0.95, y_min + 0.5), (sx * 0.26, 0.14, 1.1), kenar_renk)
        kutu_ekle((cx - sx * 0.12, elevated_y + 0.94, y_min + 3.4), (0.32, 0.72, 5.6), color.rgb(38/255, 170/255, 160/255))
        kutu_ekle((cx + sx * 0.12, elevated_y + 0.94, y_min + 3.4), (0.32, 0.72, 5.6), color.rgb(38/255, 170/255, 160/255))
        kutu_ekle((kule_x, elevated_y + 3.15, kule_z), (1.0, 5.2, 1.0), color.rgb(205/255, 214/255, 212/255))
        kutu_ekle((kule_x, elevated_y + 5.05, kule_z), (5.4, 0.18, 0.34), color.rgb(54/255, 220/255, 210/255))
        kutu_ekle((kule_x, elevated_y + 5.05, kule_z), (0.34, 0.18, 5.4), color.rgb(54/255, 220/255, 210/255))
        kutu_ekle((kule_x, elevated_y + 6.05, kule_z), (2.0, 1.0, 2.0), color.rgb(158/255, 226/255, 234/255))

        mesh = Mesh(vertices=verts, triangles=tris, colors=colors, static=True)
        karakol_mesh = Entity(parent=root, model=mesh, unlit=False, add_to_scene_entities=True)
        karakol_mesh.model.generate()
        self.ileri_karakol_gorselleri.append(karakol_mesh)

        collider = Entity(
            parent=root,
            model='cube',
            position=(cx, elevated_y + 1.2, cz),
            scale=(sx, 6.0, sz),
            color=color.clear,
            visible=False,
            collider='box',
            add_to_scene_entities=True,
        )
        self.ileri_karakol_gorselleri.append(collider)


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
            ROV'lar arası sonar iletişimini KESİKLİ ÇİZGİ Mesh'leri ile gösterir.
            Create-once per pair: her çift için tekil Mesh oluşturulur, her karede güncellenir.
            """
            active_rovs = [r for r in self.rovs if rov_aktif_mi(r)]
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
        code.interact(local=dict(globals(), **vars))
