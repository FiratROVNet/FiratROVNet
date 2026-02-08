import os
import sys
import math
import random
import threading
import code
import numpy as np
from ursina import *

# Yerel modül importları
from .config import (
    SensorAyarlari, GATLimitleri, HareketAyarlari, 
    FizikSabitleri, SimulasyonSabitleri, ROVModelleri
)
from .utils import sim_to_ursina, ursina_to_sim
from .kutuphane.helper.EntityLoader import EntityLoader
from .kutuphane.helper.simulasyon_helper import OrtamHelper

# ============================================================
# 1. ROV SINIFI (Mantık ve Fizik)
# ============================================================
class ROV(Entity):
    def __init__(self, rov_id, loader_ref=None, model_key='submarine', **kwargs):
        super().__init__()
        self.id = rov_id
        self.environment_ref = None
        
        # Fiziksel ve Durumsal Durum
        self.velocity = Vec3(0, 0, 0)
        self.battery, self.role, self.gat_kodu = 1.0, 0, 0
        self.rotation_y = 0
        
        # Sensör Verileri
        self.sensor_config = SensorAyarlari.VARSAYILAN.copy()
        self.son_sonar_mesafesi = -1
        self.son_lidar_mesafeleri = {0: -1, 1: -1, 2: -1}
        self.engel_mesafesi = 999.0
        
        # Görsel Referanslar
        self.engel_cizgi = None
        self.iletisim_rovlari = {}
        self.label = None
        self.safety_zone = None

        # Pozisyonlandırma
        pos = kwargs.get('position', Vec3(0, -10, 0))
        self.position = pos if isinstance(pos, Vec3) else sim_to_ursina(*pos)

        # Görseli Yükle
        if loader_ref:
            loader_ref.setup_rov(self, model_key)

    def ekle(self, ortam_ref):
        if not ortam_ref: return False
        self.environment_ref = ortam_ref
        if not hasattr(ortam_ref, 'rovs'): ortam_ref.rovs = []
        while len(ortam_ref.rovs) <= self.id: ortam_ref.rovs.append(None)
        if ortam_ref.rovs[self.id] is not None: return False
        ortam_ref.rovs[self.id] = self
        return True

    def update(self):
        """Ursina Ana Döngüsü: Hareket, Sürtünme ve Sensörler."""
        # Su Direnci (Damping) ve Hareket
        
        if self.environment_ref:
            self._guncelle_sensorler()
            if self.velocity.length() > 0.01:
                self.battery -= FizikSabitleri.BATARYA_SOMURME_KATSAYISI * time.dt

    def _guncelle_sensorler(self):
            """Raycast taraması yaparken diğer tüm ROV'ları görmezden gelir."""
            menzil = self.sensor_config.get("engel_mesafesi", 10.0)
            origin = self.world_position + Vec3(0, 0.5, 0)
            
            # --- IGNORE LİSTESİ OLUŞTUR ---
            # Kendini ve çevredeki tüm ROV parçalarını listeye ekle
            ignores = [self]
            if self.environment_ref:
                for r in self.environment_ref.rovs:
                    if r:
                        ignores.append(r)
                        # ROV'un yan bileşenlerini de (etiket, koruma küresi) ekleyelim
                        if hasattr(r, 'safety_zone'): ignores.append(r.safety_zone)
                        if hasattr(r, 'label'): ignores.append(r.label)
            
            # Raycast: Sadece gerçek engellere çarpar
            hit = raycast(origin, self.forward, distance=menzil, ignore=tuple(ignores))
            
            if hit.hit:
                self.engel_mesafesi = hit.distance
                self.son_sonar_mesafesi = hit.distance
                self._kesikli_cizgi_ciz(hit.world_point, hit.distance)
                if hasattr(self.environment_ref, 'engel_bulutu'):
                    self.environment_ref.engel_bulutu.append((hit.world_point.x, hit.world_point.z))
            else:
                self.engel_mesafesi = 999.0
                if self.engel_cizgi: destroy(self.engel_cizgi); self.engel_cizgi = None

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
        t = guc * FizikSabitleri.HIZLANMA_CARPANI * time.dt
        rad = math.radians(self.rotation_y)
        
        if komut == "ileri":   self.velocity += Vec3(math.sin(rad)*t, 0, math.cos(rad)*t)
        elif komut == "geri":  self.velocity -= Vec3(math.sin(rad)*t, 0, math.cos(rad)*t)
        elif komut == "sag":   self.velocity += Vec3(math.cos(rad)*t, 0, -math.sin(rad)*t)
        elif komut == "sol":   self.velocity -= Vec3(math.cos(rad)*t, 0, -math.sin(rad)*t)
        elif komut == "cik":   self.velocity.y += t
        elif komut == "bat":   self.velocity.y -= t if self.role != 1 else 0
        elif komut == "dur":   self.velocity = Vec3(0,0,0)

    def get(self, veri):
        d = {"gps": [self.x, self.y, self.z], "hiz": [self.velocity.x, self.velocity.y, self.velocity.z],
             "batarya": self.battery, "yaw": self.rotation_y, "rol": self.role, "sonar": self.son_sonar_mesafesi}
        return np.array(d[veri]) if veri in d else None

    def _kesikli_cizgi_ciz(self, hedef, mesafe):
        if self.engel_cizgi: destroy(self.engel_cizgi)
        c = color.red if mesafe < 5 else (color.orange if mesafe < 10 else color.yellow)
        self.engel_cizgi = Entity()
        yon = (hedef - self.position).normalized()
        for i in range(int(mesafe)):
            Entity(parent=self.engel_cizgi, model='cube', scale=(.1,.1,.5), 
                   position=self.position + yon*(i + 0.5), color=c, unlit=True).look_at(hedef)

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
        """Filo GNC tarafından gönderilen noktaları kullanarak Cyan güvenlik bölgesini çizer."""
        if self.hull_entity: 
            destroy(self.hull_entity)
            self.hull_entity = None
            
        if not points or len(points) < 3: 
            return

        # Noktaları harita koordinatına çevir
        verts = []
        for p in points:
            px, pz = p[0], p[1]
            mp = self.dunya_to_harita(px, pz)
            verts.append((mp.x, mp.y, -0.25)) # Katman: Adaların üstü, ROV'un altı
            
        verts.append(verts[0]) # Çizgiyi kapat

        self.hull_entity = Entity(
            parent=self,
            model=Mesh(vertices=verts, mode='line', thickness=2),
            color=color.cyan,
            alpha=0.6,
            enabled=self.visible # Minimap kapalıysa gizli kalsın
        )

    # --- KRİTİK GÜNCELLEME: update_path metodu ---
    def update_path(self, path_points):
        """A* algoritmasından gelen yeşil rota çizgisini günceller."""
        if self.path_entity: 
            destroy(self.path_entity)
            self.path_entity = None
            
        if not path_points or len(path_points) < 2: 
            return

        verts = []
        for p in path_points:
            mp = self.dunya_to_harita(p[0], p[1])
            verts.append((mp.x, mp.y, -0.3))

        self.path_entity = Entity(
            parent=self,
            model=Mesh(vertices=verts, mode='line', thickness=3),
            color=color.lime,
            alpha=0.9,
            enabled=self.visible
        )

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
        
        # ROV İkonları
        if hasattr(self.ortam_ref, 'rovs'):
            active_ids = {r.id for r in self.ortam_ref.rovs if r}
            for rov in [r for r in self.ortam_ref.rovs if r]:
                target = self.dunya_to_harita(rov.x, rov.z)
                if rov.id not in self.rov_ikonlari:
                    self.rov_ikonlari[rov.id] = self.loader.create_rov_icon(self, rov.id, rov.color)
                icon = self.rov_ikonlari[rov.id]
                icon.color = rov.color
                if (target - icon.position).length_squared() > 0.04: icon.position = target
                else: icon.position = lerp(icon.position, target, min(1.0, time.dt * 18))
                icon.rotation_z = -rov.rotation_y
            
            for rid in list(self.rov_ikonlari.keys()):
                if rid not in active_ids: destroy(self.rov_ikonlari[rid]); del self.rov_ikonlari[rid]

        self._vektor_ve_hedef_guncelle()
        self._engel_bulutu_guncelle()

    def _vektor_ve_hedef_guncelle(self):
        filo = getattr(self.ortam_ref, 'filo', None)
        helper = getattr(filo, 'helper', None) if filo else None
        apf_list = helper.get_apf_vektor_verts_list(self) if helper else []
        sig = len(apf_list) + (apf_list[0][0][0][0] if apf_list else 0)
        if self._apf_cache_sig != sig:
            self._apf_cache_sig = sig
            for e in self.vektor_cizgi_entities: destroy(e)
            self.vektor_cizgi_entities.clear()
            for v, c in apf_list:
                try: self.vektor_cizgi_entities.append(self.loader.create_vector_mesh(self, v, c))
                except: pass

    def _engel_bulutu_guncelle(self):
        bulut = getattr(self.ortam_ref, 'engel_bulutu', [])
        if len(bulut) < self._engel_bulutu_cizilen_len:
            for e in self.engel_noktalari: destroy(e)
            self.engel_noktalari.clear(); self._engel_bulutu_cizilen_len = 0
        for i in range(self._engel_bulutu_cizilen_len, len(bulut)):
            pos = self.dunya_to_harita(bulut[i][0], bulut[i][1])
            if abs(pos.x) < 0.5 and abs(pos.y) < 0.5:
                self.engel_noktalari.append(self.loader.create_obstacle_dot(self, pos))
                if len(self.engel_noktalari) > 150: destroy(self.engel_noktalari.pop(0))
        self._engel_bulutu_cizilen_len = len(bulut)

    def update_ada_cevre(self, points):
            """
            GNC sisteminden gelen sahil şeridi noktalarını minimap üzerinde 
            küçük noktalar halinde (nokta bulutu gibi) çizer.
            """
            # Önceki noktaları temizle
            if hasattr(self, 'ada_cevre_entity') and self.ada_cevre_entity:
                destroy(self.ada_cevre_entity)
                self.ada_cevre_entity = None
                
            if not points: return
            
            # Tüm noktaları tek bir konteyner (parent) altında topla
            self.ada_cevre_entity = Entity(parent=self)
            ada_renk = color.hex('#CD853F') # Peru/Toprak rengi (Ada ile uyumlu)
            
            for p in points:
                # Dünya koordinatını harita koordinatına çevir
                # p[0]=x, p[1]=z (bazı durumlarda y_coord olarak da gelebilir)
                mp = self.dunya_to_harita(p[0], p[1] if len(p) > 1 else 0)
                
                # Küçük bir nokta oluştur
                Entity(
                    parent=self.ada_cevre_entity,
                    model='circle',
                    scale=0.01, # İkonlardan çok daha küçük (0.022 vs 0.008)
                    position=(mp.x, mp.y, -0.28), # Adaların biraz üstünde
                    color=ada_renk,
                    alpha=0.85
                )

    def hedef_isaretle(self, x, z, id=None, debug=True):
        """3D ortamda oluşturulan hedefi Minimap üzerinde 2D olarak çizer."""
        from ursina import Entity, destroy, color, Text
        
        # Dünya (3D) koordinatını Minimap (2D) koordinatına çevir
        mp = self.dunya_to_harita(x, z)
        
        # --- DURUM 1: GEÇİCİ HEDEF (debug=True) ---
        if debug:
            if hasattr(self, 'gecici_hedef_ikonu') and self.gecici_hedef_ikonu:
                destroy(self.gecici_hedef_ikonu)
                
            self.gecici_hedef_ikonu = Entity(
                parent=self,
                model='circle',
                color=color.red,
                scale=0.035, # Harita üzerindeki boyutu
                position=(mp.x, mp.y, -0.35), # Z ekseninde ikonların vs. üstünde durması için
                enabled=self.visible # Minimap kapalıysa gizli kalsın
            )
            
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
        self.islands=self.island_entities
        self.engel_bulutu, self.konsol_verileri = [], {}
        self.sonar_cizgiler, self.filo = {}, None

        # --- KURULUM ---
        self._setup_window()
        self._setup_lighting()
        self.minimap = Minimap(ortam_ref=self)
        
        self.camera = EditorCamera(
            enabled=True, rotate_speed=15, pan_speed=(10, 10), 
            zoom_speed=1, position=(0, 20, -50), rotation=(20, 0, 0)
        )
        mouse.visible, mouse.locked = True, False

    def _setup_window(self):
        """ESKİ AYARLAR: Pencere konfigürasyonu."""
        window.fullscreen = False
        window.exit_button.visible = False
        window.fps_counter.enabled = True
        window.size = (1280, 720)
        window.center_on_screen()
        window.color = color.rgb(10, 30, 50)
        application.run_in_background = True
        try: window.context_menu = False
        except: pass

    def _setup_lighting(self):
        self.sun = DirectionalLight()
        self.sun.look_at(Vec3(1, -1, -1))
        self.ambient = AmbientLight(color=color.rgba(120, 120, 120, 1))
        self.sky = Sky()

    def konsola_ekle(self, isim, nesne): self.konsol_verileri[isim] = nesne
    def set_update_function(self, func): self.app.update = func
    def simden_veriye(self): return self.helper.simden_veriye() if self.helper else []

    def _find_safe_rov_spawn_pos(self):
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

    def sim_olustur(self, n_rovs=6, n_islands=5, havuz_genisligi=200, rov_model='submarine'):
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
        
        # 1. Adaları Sabit Noktalardan Yerleştir
        count = min(n_islands, len(self.FIXED_ISLAND_POSITIONS))
        chosen_islands = random.sample(self.FIXED_ISLAND_POSITIONS, count)
        for i, pos in enumerate(chosen_islands):
            self.Ada(i, x="ekle", y=pos)

        # 2. ROV'ları Güvenli Noktalara Yerleştir
        print(f"🌊 Simülasyon Başlatılıyor: {n_rovs} ROV, {count} Ada")
        for i in range(n_rovs):
            sim_pos = self._find_safe_rov_spawn_pos()
            # sim_pos: (x, z_depth, y_coordinate) -> ursina: (x, y, z)
            u_pos = Vec3(sim_pos[0], sim_pos[2], sim_pos[1])
            
            new_rov = ROV(rov_id=i, position=u_pos, loader_ref=self.loader, model_key=rov_model)
            new_rov.ekle(self)

        self.minimap._statik_yeniden_ciz()

    def Ada(self, ada_id, x=None, y=None):
        if x == "ekle":
            while len(self.island_positions) <= ada_id: self.island_positions.append(None)
            while len(self.island_entities) <= ada_id: self.island_entities.append(None)
            ent, radius = self.loader.create_island(y[0], y[1])
            self.island_entities[ada_id], self.island_positions[ada_id] = ent, (y[0], y[1], radius)

    def guncelle_sonar_cizgileri(self):
            """ROV'lar arası sonar iletişimini dinamik kesikli çizgilerle gösterir."""
            active_rovs = [r for r in self.rovs if r]
            
            # Kesikli çizgi ayarları
            segment_boyu = 1.5  # Her bir çizgi parçasının uzunluğu (metre)
            bosluk_boyu = 1.0   # Çizgiler arasındaki boşluk
            adim = segment_boyu + bosluk_boyu

            for i, r1 in enumerate(active_rovs):
                for r2 in active_rovs[i+1:]:
                    p1, p2 = r1.position, r2.position
                    fark = p2 - p1
                    dist = fark.length()
                    pair = tuple(sorted((r1.id, r2.id)))
                    
                    # İletişim şartları: Menzil içi ve kopuk değilse
                    if dist < self.SONAR_MENZILI and r1.gat_kodu != 3 and r2.gat_kodu != 3:
                        # Renk belirleme (Mesafe bazlı)
                        c = color.red if dist < 10 else (color.orange if dist < 60 else color.cyan)
                        
                        # --- KESİKLİ ÇİZGİ VERTEX HESAPLAMA ---
                        verts = []
                        yon = fark.normalized()
                        curr = 0
                        
                        while curr < dist:
                            # Çizgi parçasının başı
                            v_start = p1 + yon * curr
                            # Çizgi parçasının sonu (mesafeyi aşmamalı)
                            v_end = p1 + yon * min(curr + segment_boyu, dist)
                            
                            verts.append(v_start)
                            verts.append(v_end)
                            
                            curr += adim
                        
                        # Entity yönetimi
                        if pair not in self.sonar_cizgiler:
                            self.sonar_cizgiler[pair] = Entity(
                                model=Mesh(vertices=verts, mode='line', static=False),
                                color=c,
                                alpha=0.4
                            )
                        else:
                            # Mevcut çizgiyi güncelle (Hızlı mesh güncelleme)
                            line_ent = self.sonar_cizgiler[pair]
                            line_ent.model.vertices = verts
                            line_ent.model.generate() # Mesh'i yeniden oluştur
                            line_ent.color = c
                    
                    # Menzil dışına çıktıysa veya koptuysa çizgiyi sil
                    elif pair in self.sonar_cizgiler:
                        destroy(self.sonar_cizgiler[pair])
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