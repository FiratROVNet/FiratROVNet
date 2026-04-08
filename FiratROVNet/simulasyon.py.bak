import math
import random
import threading
import code
import numpy as np
from ursina import *  # type: ignore[reportMissingImports]
from ursina import (  # type: ignore[reportMissingImports]
    Entity, Vec3, destroy, raycast, Text, color, time,
    camera, Mesh, window, application, mouse, Ursina, EditorCamera,
    DirectionalLight, AmbientLight, Sky, lerp,
)

# Yerel modül importları
from .config import (
    SensorAyarlari, GATLimitleri, HareketAyarlari, 
    FizikSabitleri, ROVModelleri
)
from .utils import sim_to_ursina, ursina_to_sim
from .kutuphane.helper.EntityLoader import EntityLoader
from .kutuphane.helper.simulasyon_helper import OrtamHelper

# ============================================================
# 1. ROV SINIFI (Mantık ve Fizik)
# ============================================================
class ROV(Entity):
    """Kesikli çizgi segment sayısı (havuz boyutu). Create-once, sonra sadece gösterme/gizleme."""
    CIZGI_HAVUZ_SEGMENT = 25

    def __init__(self, rov_id,group_id, loader_ref=None, model_key='submarine', **kwargs):
        super().__init__()
        self.id = rov_id
        self.environment_ref = None
        
        # Fiziksel ve Durumsal Durum
        self.velocity = Vec3(0, 0, 0)
        self.battery, self.role, self.gat_kodu = 1.0, 0, 0
        self.rotation_y = 0
        
        # Sensör Verileri
        self.sensor_config = SensorAyarlari.VARSAYILAN.copy()
        self.sensor_config['engel_mesafesi'] = GATLimitleri.ENGEL  # GATLimitleri'ne göre sabitle (20.0m)
        self.son_sonar_mesafesi = -1
        self.son_lidar_mesafeleri: dict[int, float] = {0: -1.0, 1: -1.0, 2: -1.0, 3: -1.0}  # L0: İleri, L1: Sağ, L2: Sol, L3: Dip
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
        mevcut_ids = [r.id for r in ortam_ref.rovs if r is not None and hasattr(r, 'id')]
        self.id = (max(mevcut_ids) + 1) if mevcut_ids else 0
        ortam_ref.rovs.append(self)
        
        self._etiket_guncelle()
        return True

    def cikar(self):
            """ROV'u siler ve tüm sistemlerden izlerini temizler."""
            if not self.environment_ref: return
            
            ortam = self.environment_ref
            silinen_id = self.id

            # --- YENİ: Filo verilerini temizle ---
            if hasattr(ortam, 'filo') and ortam.filo:
                ortam.filo.rov_verilerini_temizle(silinen_id)

            # --- YENİ: Sonar çizgilerini (İletişim okları) temizle ---
            if hasattr(ortam, 'sonar_cizgiler'):
                for pair in list(ortam.sonar_cizgiler.keys()):
                    if silinen_id in pair:
                        data = ortam.sonar_cizgiler[pair]
                        if isinstance(data, tuple):
                            destroy(data[0])
                        else:
                            destroy(data)
                        del ortam.sonar_cizgiler[pair]

            # --- YENI: Grup listesinden temizle ---
            if hasattr(ortam, 'g_rovs') and isinstance(ortam.g_rovs, dict):
                grup = ortam.g_rovs.get(self.group_id)
                if grup:
                    ortam.g_rovs[self.group_id] = [r for r in grup if r and r.id != silinen_id]

            # 1. Referansi None yap (id korunur)
            for idx, r in enumerate(ortam.rovs):
                if r and getattr(r, 'id', None) == silinen_id:
                    ortam.rovs[idx] = None
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
        if self.label:
            self.label.text = metin
        else:
            self.label = Text(text=metin, parent=self, y=1.5, scale=15, origin=(0,0), color=color.white)

    # --- get metodunu bu 'Güvenli' haliyle DEĞİŞTİR ---
    def get(self, veri):
        # Obje silinmişse Panda3D koordinat hatası (AssertionError) vermemesi için kontrol
        if not self or (hasattr(self, 'is_destroyed') and self.is_destroyed):
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


    def update(self):
        """
        Ursina Entity.update().
        Central-update modunda (ortam.central_update=True) per-entity update devre disidir;
        tum sensor/limit/batarya islemleri Filo.guncelle_hepsi icinden tek sefer yapilir.
        """
        if not self.environment_ref or getattr(self.environment_ref, 'central_update', False):
            return

        # Merkezi loop kullanılmıyorsa: sensör+batarya+limitleri burada sürdür.
        print("güncelle sensörler")
        self._guncelle_sensorler()
        if self.velocity.length() > 0.01:
            self.battery -= FizikSabitleri.BATARYA_SOMURME_KATSAYISI * time.dt * self.velocity.length()  # type: ignore[attr-defined]

        # Derinlik limitleri
        if self.y > 0:
            self.y = 0
        sea_floor_y = getattr(self.environment_ref, 'SEA_FLOOR_Y', -50.0)
        if self.y < sea_floor_y:
            self.y = sea_floor_y

    def _guncelle_sensorler(self):
            menzil = GATLimitleri.ENGEL
            base_origin = self.world_position + Vec3(0, 0.5, 0)

            # Filo frame-basinda hazirlanan global ignore tuple varsa onu kullan.
            ignore_tuple = ()
            if self.environment_ref:
                ignore_tuple = getattr(self.environment_ref, 'ignore_tuple', ())

            if not ignore_tuple and self.environment_ref and hasattr(self.environment_ref, 'rovs'):
                ignores = []
                for r in self.environment_ref.rovs:
                    if not r:
                        continue
                    ignores.append(r)
                    for child in getattr(r, 'children', []):
                        ignores.append(child)
                        ignores.extend(getattr(child, 'children', []))
                ignore_tuple = tuple(ignores)

            def buluta_ekle(hit_point, lidar_idx):
                ortam = self.environment_ref
                if not ortam or not hasattr(ortam, 'engel_bulutu'):
                    return
                ortam.engel_bulutu.append((
                    float(hit_point.x),
                    float(hit_point.z),
                    float(hit_point.y),
                    f"L{lidar_idx}",
                ))
                if len(ortam.engel_bulutu) > 12000:
                    del ortam.engel_bulutu[:2000]

            def safe_raycast(origin, direction, dist, ignore_list):
                safe_start = origin + (direction * 1.5)
                ray_dist = max(1, int(float(dist) - 1.5))
                return raycast(safe_start, direction, distance=ray_dist, ignore=ignore_list, debug=False)

            # SONAR (L0)
            hit_sonar = safe_raycast(base_origin, self.forward, menzil, ignore_tuple)
            if hit_sonar.hit:
                self.engel_mesafesi = hit_sonar.distance + 1.5
                self.son_sonar_mesafesi = self.engel_mesafesi
                self._kesikli_cizgi_ciz(hit_sonar.world_point, self.engel_mesafesi)
                buluta_ekle(hit_sonar.world_point, 0)
            else:
                self.engel_mesafesi = 999.0
                self.son_sonar_mesafesi = -1.0
                if self.engel_cizgi:
                    self.engel_cizgi.enabled = False

            # LIDARLAR (L0, L1, L2)
            directions = [
                (0, self.forward, color.cyan),
                (1, self.right, color.blue),
                (2, -self.right, color.green),
            ]

            for idx, dir_vec, clr in directions:
                hit = safe_raycast(base_origin, dir_vec, menzil, ignore_tuple)
                if hit.hit:
                    dist = hit.distance + 1.5
                    self.son_lidar_mesafeleri[idx] = dist
                    self._lidar_cizgi_ciz(idx, hit.world_point, dist, clr)
                    buluta_ekle(hit.world_point, idx)
                else:
                    self.son_lidar_mesafeleri[idx] = -1.0
                    self._lidar_cizgi_temizle(idx)

            # L3: Dip lidar
            origin_l3 = self.world_position + Vec3(0, -2, 0)
            hit_l3 = raycast(origin_l3, Vec3(0, -1, 0), distance=max(1, int(menzil)), ignore=list(ignore_tuple), debug=False)
            if hit_l3.hit:
                self.son_lidar_mesafeleri[3] = hit_l3.distance
                self._lidar_cizgi_ciz(3, hit_l3.world_point, hit_l3.distance, color.magenta)
                buluta_ekle(hit_l3.world_point, 3)
            else:
                self.son_lidar_mesafeleri[3] = -1.0
                self._lidar_cizgi_temizle(3)
    def set(self, ayar, deger):
        """GNC sistemi tarafından çağrılır."""
        if ayar == "rol":
            self.role = int(deger)
            if self.label:
                self.label.text = f"{'LIDER' if self.role == 1 else 'ROV'}-{self.id}"
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
        yon = (hedef - self.position).normalized()
        n_use = min(int(mesafe), max_seg)
        for i in range(n_use):
            seg = seg_list[i]
            seg.position = self.position + yon * (i + 0.5)
            seg.color = c
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
        yon = (hedef - self.position).normalized()
        n_use = min(int(mesafe), max_seg)
        for i in range(n_use):
            seg = seg_list[i]
            seg.position = self.position + yon * (i + 0.5)
            seg.color = renk
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
            parent=camera.ui,
            scale=(self.BASE_SCALE, self.BASE_SCALE),
            origin=(-0.5, -0.5),      # Haritanın kendi sol altı
            position=(0.62, -0.21),  # Sol alt köşeden içeri
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

        self.obstacle_cloud_entity = None # Tek bir entity kullanacağız

        # 1. Engel noktalarının koordinatlarını tutacak liste
        self.engel_vertex_listesi = []
        self.engel_color_listesi = []

        # 2. Tek bir Mesh oluşturuyoruz (mode='point' önemli)
        # thickness=2 yaparak o 'kırmızı blok' sorununu baştan çözüyoruz.
        self.engel_mesh = Mesh(vertices=[], colors=[], mode='point', thickness=int(0.016 * 1000) // 1000 or 1, static=False)

        # 3. Bu Mesh'i ekranda gösterecek TEK Entity
        self.engel_gorseli = Entity(
            parent=self, # Veya self.minimap_panel, nereye koyuyorsan
            model=self.engel_mesh,
            color=color.white,
            z=-0.01 # Haritanın hafif önünde
        )
        
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
        for child in self.children:
            child.enabled = durum
        
        # Hull ve Path görünürlüğünü özel olarak ayarla
        if self.hull_entity: self.hull_entity.enabled = (convex and durum)
        if self.path_entity: self.path_entity.enabled = (a_star and durum)

    # --- KRİTİK GÜNCELLEME: update_hull metodu ---
    def update_hull(self, points):
        """Filo GNC tarafından gönderilen noktaları kullanarak Cyan güvenlik bölgesini çizer. Create-once, mesh güncelle."""
        if not points or len(points) < 3:
            if self.hull_entity:
                self.hull_entity.enabled = False
            return
        verts = []
        for p in points:
            px, pz = p[0], p[1]
            mp = self.dunya_to_harita(px, pz)
            verts.append((mp.x, mp.y, -0.25))
        verts.append(verts[0])
        if self.hull_entity is None:
            self.hull_entity = Entity(
                parent=self,
                model=Mesh(vertices=verts, mode='line', thickness=2),
                color=color.cyan,
                alpha=0.6,
                enabled=self.visible
            )
        else:
            self.hull_entity.model.vertices = verts
            self.hull_entity.model.generate()
            self.hull_entity.enabled = self.visible

    # --- KRİTİK GÜNCELLEME: update_path metodu ---
    def update_path(self, path_points):
        """A* algoritmasından gelen yeşil rota çizgisini günceller. Create-once, mesh güncelle."""
        if not path_points or len(path_points) < 2:
            if self.path_entity:
                self.path_entity.enabled = False
            return
        verts = []
        for p in path_points:
            mp = self.dunya_to_harita(p[0], p[1])
            verts.append((mp.x, mp.y, -0.3))
        if self.path_entity is None:
            self.path_entity = Entity(
                parent=self,
                model=Mesh(vertices=verts, mode='line', thickness=3),
                color=color.lime,
                alpha=0.9,
                enabled=self.visible
            )
        else:
            self.path_entity.model.vertices = verts
            self.path_entity.model.generate()
            self.path_entity.enabled = self.visible

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
                mevcut_rovlar = [r for r in list(self.ortam_ref.rovs) if r and not (hasattr(r, 'is_destroyed') and r.is_destroyed)]
                active_ids = {r.id for r in mevcut_rovlar}
                
                # --- ÖNCE SİLİNENLERİ KALDIR ---
                for rid in list(self.rov_ikonlari.keys()):
                    if rid not in active_ids:
                        destroy(self.rov_ikonlari[rid])
                        del self.rov_ikonlari[rid]

                # --- SONRA MEVCUTLARI GÜNCELLE ---
                for rov in mevcut_rovlar:
                    target = self.dunya_to_harita(rov.x, rov.z)

                    if rov.id not in self.rov_ikonlari:
                        self.rov_ikonlari[rov.id] = self.loader.create_rov_icon(self, rov.id, rov.color)
                    icon = self.rov_ikonlari[rov.id]
                    icon.position = target
                    icon.rotation_z = -rov.rotation_y
                    icon.color = rov.color

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
                sig_sum += round(x0, 4) + round(y0, 4) + round(x1, 4) + round(y1, 4)
            sig = (len(apf_list), round(sig_sum, 4))
        else:
            sig = (0, 0.0)
        if self._apf_cache_sig != sig:
            self._apf_cache_sig = sig
            n_use = min(len(apf_list), self._apf_vektor_pool_size)
            if self._apf_vektor_pool is None:
                self._apf_vektor_pool = []
                for _ in range(self._apf_vektor_pool_size):
                    mesh = Mesh(vertices=[(0, 0, z_line), (0, 0, z_line)], mode='line', thickness=2)
                    e = Entity(
                        parent=self,
                        model=mesh,
                        color=color.white,
                        alpha=0.95,
                        z=z_line
                    )
                    self._apf_vektor_pool.append(e)
            pool = self._apf_vektor_pool
            # Titremeyi onlemek icin koordinatlari yuvarla (kucuk degisimler cizimi degistirmez)
            round_ = 3
            for i in range(n_use):
                verts, c_code = apf_list[i]
                if not verts or len(verts) < 2:
                    pool[i].enabled = False
                    continue
                c = vektor_renkler.get(c_code, color.white)
                verts_stable = [tuple(round(v, round_) for v in pt) for pt in verts]
                pool[i].model.vertices = verts_stable
                pool[i].model.generate()
                pool[i].color = c
                pool[i].enabled = True
            for i in range(n_use, len(pool)):
                pool[i].enabled = False

    def _apf_vektorlari_temizle(self):
        """APF vektorlerini gizler (havuz entity'leri yok edilmez, sadece cache sifirlanir)."""
        self._apf_cache_sig = None
        if self._apf_vektor_pool:
            for e in self._apf_vektor_pool:
                e.enabled = False

    def _engel_bulutu_guncelle_yedek(self):
        bulut = getattr(self.ortam_ref, 'engel_bulutu', [])
        if len(bulut) < self._engel_bulutu_cizilen_len:
            for e in list(self.engel_noktalari):
                destroy(e)
            self.engel_noktalari.clear()
            self._engel_bulutu_cizilen_len = 0
        for i in range(self._engel_bulutu_cizilen_len, len(bulut)):
            pos = self.dunya_to_harita(bulut[i][0], bulut[i][1])
            if abs(pos.x) < 0.5 and abs(pos.y) < 0.5:
                self.engel_noktalari.append(self.loader.create_obstacle_dot(self, pos))
                if len(self.engel_noktalari) > 150: destroy(self.engel_noktalari.pop(0))
        self._engel_bulutu_cizilen_len = len(bulut)


    def _engel_bulutu_guncelle(self):
        bulut = getattr(self.ortam_ref, 'engel_bulutu', [])
        
        # Reset durumunda hafızayı da temizle
        if len(bulut) < self._engel_bulutu_cizilen_len:
            self.engel_vertex_listesi.clear()
            self.kayitli_noktalar.clear() # Hafızayı sil
            self.engel_color_listesi.clear()
            self.engel_mesh.vertices = []
            self.engel_mesh.colors = []
            self.engel_mesh.generate()
            self._engel_bulutu_cizilen_len = 0

        yeni_veri_var = False
        
        # Sadece yeni gelen verilere bakıyoruz
        for i in range(self._engel_bulutu_cizilen_len, len(bulut)):
            # 1. Koordinatı harita düzlemine çevir
            pos = self.dunya_to_harita(bulut[i][0], bulut[i][1])
            
            # 2. Koordinatları YUVARLA (Çok Önemli!)
            # Virgülden sonra 3 hane hassasiyet yeterlidir. 
            # (Örn: 0.12345 ile 0.12346 aynı nokta sayılsın istiyoruz)
            x_key = round(pos.x, 3)
            y_key = round(pos.y, 3)
            point_key = (x_key, y_key)
            
            # 3. KONTROL: Bu noktayı daha önce çizdik mi?
            if point_key not in self.kayitli_noktalar:
                
                # Sınır kontrolü
                if abs(pos.x) < 0.8 and abs(pos.y) < 0.8:
                    # Listeye ekle
                    self.engel_vertex_listesi.append(Vec3(pos.x, pos.y, 0))

                    # Derinlige gore renk belirle (gri tonlar)
                    # -10m uzeri engeller sabit kahverengi
                    y_val = bulut[i][2] if bulut[i] is not None and len(bulut[i]) >= 3 else 0.0
                    if float(y_val) > -10.0:
                        c = color.rgb(0.45, 0.3, 0.2)
                    else:
                        surface_y = getattr(self.ortam_ref, 'WATER_SURFACE_Y_BASE', 0.0)
                        floor_y = getattr(self.ortam_ref, 'SEA_FLOOR_Y', -50.0)
                        depth_span = max(0.001, surface_y - floor_y)
                        t = (surface_y - float(y_val)) / depth_span
                        t = max(0.0, min(1.0, t))
                        dark_gray = color.rgb(0.2, 0.2, 0.2)
                        light_gray = color.rgb(0.85, 0.85, 0.85)
                        c = color.rgb(
                            dark_gray.r + (light_gray.r - dark_gray.r) * t,
                            dark_gray.g + (light_gray.g - dark_gray.g) * t,
                            dark_gray.b + (light_gray.b - dark_gray.b) * t,
                        )
                    self.engel_color_listesi.append(c)
                    
                    # Hafızaya kaydet (Set'e ekle)
                    self.kayitli_noktalar.add(point_key)
                    
                    yeni_veri_var = True

        # Mesh'i sadece yeni nokta eklendiyse güncelle
        if yeni_veri_var:
            self.engel_mesh.vertices = self.engel_vertex_listesi
            self.engel_mesh.colors = self.engel_color_listesi
            self.engel_mesh.generate()

        self._engel_bulutu_cizilen_len = len(bulut)

    def update_ada_cevre(self, points):
            """
            GNC sisteminden gelen sahil şeridi noktalarını minimap üzerinde çizer.
            Create-once: konteyner ve nokta entity havuzu bir kez oluşturulur, her cagrida sadece konumlar güncellenir.
            """
            if not points:
                if hasattr(self, 'ada_cevre_entity') and self.ada_cevre_entity:
                    self.ada_cevre_entity.enabled = False
                return
            ada_renk = color.hex('#CD853F')
            max_nokta = 2000
            n_use = min(len(points), max_nokta)
            if not hasattr(self, 'ada_cevre_entity') or self.ada_cevre_entity is None:
                self.ada_cevre_entity = Entity(parent=self)
                self.ada_cevre_entity._ada_noktalari = []
                for _ in range(max_nokta):
                    e = Entity(
                        parent=self.ada_cevre_entity,
                        model='circle',
                        scale=0.01,
                        position=(0, 0, -0.28),
                        color=ada_renk,
                        alpha=0.85
                    )
                    self.ada_cevre_entity._ada_noktalari.append(e)
            pool = self.ada_cevre_entity._ada_noktalari
            for i in range(n_use):
                p = points[i]
                mp = self.dunya_to_harita(p[0], p[1] if len(p) > 1 else 0)
                pool[i].position = (mp.x, mp.y, -0.28)
                pool[i].enabled = True
            for i in range(n_use, len(pool)):
                pool[i].enabled = False
            self.ada_cevre_entity.enabled = True

    def hedef_isaretle(self, x, z, id=None, debug=True):
        """3D ortamda oluşturulan hedefi Minimap üzerinde 2D olarak çizer. Gecici hedef: create-once, sadece konum guncelle."""
        from ursina import Entity, destroy, color, Text
        
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
                self.gecici_hedef_ikonu.position = (mp.x, mp.y, -0.35)
                self.gecici_hedef_ikonu.enabled = self.visible
            
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
        from ursina import destroy
        if hasattr(self, 'hedef_ikonlari') and id in self.hedef_ikonlari:
            destroy(self.hedef_ikonlari[id])
            del self.hedef_ikonlari[id]

    def hedefleri_temizle(self):
        """Tüm kalıcı ve geçici hedefleri minimap'ten temizler."""
        from ursina import destroy
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
        
        # Fiziksel Sabitler
        self.havuz_genisligi = 200
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
            if not rov or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
                continue
            __group_id=rov.group_id
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
        self.rov_label = Text(
            text="FPS: --",
            parent=camera.ui,
            position=(0.69, 0.49),
            origin=(-0.5, 0.5),
            scale=1.0,
            color=color.lime,
            background=True,
        )
        self.rov_label.z = -10
        if self.rov_label.background is not None:
            self.rov_label.background.scale_x = 2
            self.rov_label.background.scale_y = 2.2
            self.rov_label.background.x = 0.1
            self.rov_label.background.y = -0.07

    def _setup_lighting(self):
        self.sun = DirectionalLight()
        self.sun.look_at(Vec3(1, -1, -1))
        self.ambient = AmbientLight(color=color.rgba(120, 120, 120, 1))
        self.sky = Sky()

    def konsola_ekle(self, isim, nesne): self.konsol_verileri[isim] = nesne
    
    def set_update_function(self, func):
        """Günceleme fonksiyonunu kaydet. Ursina'da doğrudan update attribute atanamaz."""
        self._custom_update_func = func
        # Ursina update döngüsüne entegre etmek için (Ursina update hook mekanizması)
        # Bu kod main.py'de app.set_update_function() çağrısı yerine kullanılır
    
    def guncelle_ozel(self):
        """Custom update fonksiyonunu çalıştırır (Ursina tarafından çağrılır)."""
        if hasattr(self, '_custom_update_func') and self._custom_update_func:
            self._custom_update_func()
    
    def simden_veriye(self): return self.helper.simden_veriye() if self.helper else []

    def _find_safe_rov_spawn_pos_yedek(self):
        """ESKİ AYARLAR: Adalardan uzak, güvenli spawn noktası bulur."""
        for _ in range(100):
            sx = random.uniform(-160, 160)
            sy = random.uniform(-160, 160)
            sz_depth = random.uniform(10, 25)
            is_safe = True
            for island in [p for p in self.island_positions if p]:
                # Adanın yarıçapı + 25m güvenlik payı
                if math.sqrt((sx-island[0])**2 + (sy-island[1])**2) < (island[2] + 25):
                    is_safe = False
                    break
            if is_safe: return (sx, sy, -sz_depth)
        return (0, 0, -15) # Fallback
    

    def _find_safe_rov_spawn_pos(self, group_config: tuple, alan_genisligi=100, bosluk=10):
        """
        group_config: (3, 4, 1) gibi bir tuple alır.
        - 3 grup oluşturur.
        - Grup 0: 3 ROV, Grup 1: 4 ROV, Grup 2: 1 ROV yerleştirir.
        """
        import math, random
        
        all_groups_rovs = [] 
        
        # Tarama sınırları ve adım boyutu
        baslangic_x, baslangic_y = -180, -180
        bitis_x, bitis_y = 180, 180
        adim = alan_genisligi + bosluk

        mevcut_x = baslangic_x
        mevcut_y = baslangic_y

        # Tuple içindeki her bir grup tanımı için dön
        for g_id, num_rovs in enumerate(group_config):
            bulundu = False
            
            # Uygun hücre bulana kadar taramaya devam et
            while mevcut_y <= bitis_y - alan_genisligi:
                while mevcut_x <= bitis_x - alan_genisligi:
                    
                    # Hücrenin merkezi
                    merkez_x = mevcut_x + (alan_genisligi / 2)
                    merkez_y = mevcut_y + (alan_genisligi / 2)
                    
                    # 1. ADA KONTROLÜ
                    hucre_kirli = False
                    for island in [p for p in self.island_positions if p]:
                        dist = math.sqrt((merkez_x - island[0])**2 + (merkez_y - island[1])**2)
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
                                rx = merkez_x + math.cos(angle) * yaricap
                                ry = merkez_y + math.sin(angle) * yaricap
                                rz = -random.uniform(5, 10)
                                bu_grubun_rovlari.append((rx, ry, rz))
                        
                        all_groups_rovs.append(bu_grubun_rovlari)
                        
                        # Sonraki grup için imleci bir adım kaydır
                        mevcut_x += adim
                        bulundu = True
                        break # İçteki X döngüsünden çık
                    
                    # Hücre kirliyse sağa kay
                    mevcut_x += adim
                
                if bulundu: break # Y döngüsünden çık, sonraki gruba geç
                
                # Satır sonuna gelindiyse başa dön ve yukarı çık
                mevcut_x = baslangic_x
                mevcut_y += adim

        return all_groups_rovs

    def sim_olustur(self, n_rovs=(6,), n_islands=5, n_rocks=20, havuz_genisligi=200, rov_model='submarine'):
        self.havuz_genisligi = havuz_genisligi
        
        # Temizlik
        for obj in [r for r in self.rovs if r] + [i for i in self.island_entities if i]: 
            if obj: destroy(obj)
        self.rovs, self.island_entities, self.island_positions, self.engel_bulutu = [], [], [], []
        
        # Dünya İnşası
        size = havuz_genisligi * 2
        self.loader.build_ocean(size=size)
        self.loader.build_seabed(size=size)
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
        global_rov_id = 0
        
        for group_id, rovlar in enumerate(all_group):
            for local_rov_id, rov_koordinat in enumerate(rovlar):
                # sim_pos: (x, z_depth, y_coordinate) -> ursina: (x, y, z)
                u_pos = Vec3(rov_koordinat[0], rov_koordinat[2], rov_koordinat[1])
                    
                new_rov = ROV(rov_id=global_rov_id, group_id=group_id, position=u_pos, loader_ref=self.loader, model_key=rov_model)
                new_rov.ekle(self)
                
                global_rov_id += 1

                self.minimap._statik_yeniden_ciz()


    def ROV(self, rov_id, x=None, y=None, z=None):
        """Konsol: ROV rov_id konumunu (x, y, z) yapar. x,y,z verilmezse sadece mevcut ROV döner."""
        if not self.rovs or rov_id is None:
            return None
        rov = next((r for r in self.rovs if r and getattr(r, "id", None) == rov_id), None)
        if not rov:
            return None
        if x is not None:
            rov.x = float(x)
        if y is not None:
            rov.y = float(y)
        if z is not None:
            rov.z = float(z)
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
                for r2 in active_rovs[i+1:]:
                    if r1.gat_kodu == 3 or r2.gat_kodu == 3:
                        continue

                    p1, p2 = r1.position, r2.position
                    dist = (p2 - p1).length()
                    
                    if dist < self.SONAR_MENZILI:
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
                        
                        yon_vec = (p2 - p1).normalized()
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
                        while curr < dist and idx < len(seg_list):
                            kalin_uzunluk = min(segment_boyu, dist - curr)
                            if kalin_uzunluk <= 0.1:
                                break
                            parca_baslangic = p1 + yon_vec * curr
                            parca_bitis = parca_baslangic + yon_vec * kalin_uzunluk
                            orta_nokta = (parca_baslangic + parca_bitis) / 2
                            parca = seg_list[idx]
                            parca.position = orta_nokta
                            parca.scale = (guncel_kalinlik, guncel_kalinlik, kalin_uzunluk)
                            parca.color = c
                            parca.look_at(parca_bitis)
                            parca.enabled = True
                            idx += 1
                            curr += adim_toplam
                        for j in range(idx, len(seg_list)):
                            seg_list[j].enabled = False

            # Temizlik: sadece artık menzilde olmayan çiftleri sil (list kopyası üzerinden)
            for pair in list(self.sonar_cizgiler.keys()):
                if pair not in bu_frame_aktif_olanlar:
                    data = self.sonar_cizgiler[pair]
                    if isinstance(data, tuple):
                        destroy(data[0])
                    else:
                        destroy(data)
                    del self.sonar_cizgiler[pair]

    def run(self, interaktif=False):
        if interaktif: threading.Thread(target=self._start_shell, daemon=True).start()
        self.app.run()

    def _start_shell(self):
        import time; time.sleep(1.5)
        print("\n🚀 FIRAT ROVNET CANLI KONSOL AKTİF")
        vars = {'rovs': self.rovs, 'app': self, 'filo': self.filo}
        vars.update(self.konsol_verileri)
        code.interact(local=dict(globals(), **vars))