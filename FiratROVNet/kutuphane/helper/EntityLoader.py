import os
import math
from ursina import *
from FiratROVNet.utils import sim_to_ursina
from FiratROVNet.config import ROVModelleri, GATLimitleri, HavuzAyarlari

class EntityLoader:
    def __init__(self, ortam_ref):
        self.ortam = ortam_ref
        self.rock_entities = []  # Kaya entity'lerini havuzda tutmak için

    def _havuz_scale_oran(self):
        """Havuz boyutuna göre scale çarpanı: mevcut_havuz_boyutu / 200 (referans).
        Havuz yarıya inince scale'ler de yarıya iner; iki katına çıkınca iki katına çıkar."""
        return HavuzAyarlari.HAVUZ_GENISLIK/200



    # =========================================================================
    # MINIMAP GÖRSEL YÜKLEYİCİLERİ (ESKİ KODUN BİREBİR TASARIMI)
    # =========================================================================
    def create_obstacle_dot(self, minimap_entity, map_pos):
            """Minimap üzerinde kırmızı engel noktası oluşturur."""
            return Entity(
                parent=minimap_entity,
                model='circle',
                scale=0.015, # Eski kodunuzdaki boyut
                color=color.red,
                position=map_pos, # (x, y, -0.35) olarak gelecek
                alpha=0.8
            )
    def create_vector_mesh(self, minimap_entity, vertices, color_code):
        """APF Vektörlerini çizer (Renkli çizgiler)."""
        # Renk Haritası (Eski koddan)
        vektor_renkler = {
            'k': color.red,      # İtme (Engel)
            'y': color.green,    # Çekme (Hedef)
            'm': color.blue,     # Bileşke
            's': color.yellow,   # Akıntı
            't': color.orange    # Teğet
        }
        c = vektor_renkler.get(color_code, color.white)

        return Entity(
            parent=minimap_entity,
            model=Mesh(vertices=vertices, mode='line', thickness=2),
            color=c,
            alpha=0.95,
            z=-0.35 # İkonların altında, haritanın üstünde
        )

    def create_target_marker(self, minimap_entity):
        """ROV'un gittiği hedef noktayı işaretleyen cyan daire."""
        return Entity(
            parent=minimap_entity,
            model='circle',
            scale=0.02, # İkondan biraz küçük
            color=color.hex('#00CED1'), # Dark Turquoise
            alpha=0.9,
            z=-0.32
        )

    def setup_minimap_base(self, minimap_entity):
        """Minimap arka planı ve çerçevesi (Referans koddan alındı)."""
        # 1. Arka Plan: color.rgba(255, 255, 255, 0.01) -> Çok şeffaf beyaz
        minimap_entity.bg = Entity(
            parent=minimap_entity,
            model='quad',
            color=color.rgba(255, 255, 255, 0.01), 
            scale=(1.0, 1.0),
            z=0.0
        )
        
        # 2. Çerçeve: color.white, alpha=0.5, z=0.01
        minimap_entity.border = Entity(
            parent=minimap_entity,
            model='quad',
            color=color.white,
            scale=(1.02, 1.02),
            z=0.01,
            alpha=0.4
        )

    def create_minimap_grid(self, minimap_entity, havuz_genisligi, grid_sayisi=None):
        """Grid ve havuz sınırları; havuz boyutu HavuzAyarlari ile uyumlu (grid adımı config'den)."""
        if hasattr(minimap_entity, 'grid_items'):
            for item in minimap_entity.grid_items: destroy(item)
        minimap_entity.grid_items = []

        # Grid ve havuz sınırı ayarları config'den (HavuzAyarlari)
        grid_unit = HavuzAyarlari.MINIMAP_GRID_UNIT
        grid_z = -0.1
        label_z = grid_z - 0.05
        grid_color = color.rgba(255, 255, 255, 0.5)
        line_thick = 0.004
        label_scale = 1.25

        # Havuz sınırları: yarı genişlik = havuz_genisligi (ortamdan gelir, config ile uyumlu)
        half = havuz_genisligi
        factor = 1.0 / (half * 2) if half > 0 else 1.0 / HavuzAyarlari.HAVUZ_TAM_GENISLIK

        # Grid adımı: grid_sayisi verilirse (2*havuz)/N, yoksa HavuzAyarlari.MINIMAP_GRID_UNIT
        if grid_sayisi:
            step = (havuz_genisligi * 2) / grid_sayisi
        else:
            step = grid_unit
        
        # --- Ana Eksenler ---
        minimap_entity.grid_items.append(Entity(parent=minimap_entity, model='quad', scale=(1, 0.005), color=color.rgba(255,255,255,100), z=grid_z))
        minimap_entity.grid_items.append(Entity(parent=minimap_entity, model='quad', scale=(0.005, 1), color=color.rgba(255,255,255,100), z=grid_z))

        # --- Dikey Çizgiler ve X Etiketleri ---
        world_x = -half
        while world_x <= half:
            local_x = world_x * factor
            if world_x != 0: # Eksen üzerine çizme
                minimap_entity.grid_items.append(Entity(
                    parent=minimap_entity, model='quad',
                    position=(local_x, 0, grid_z),
                    scale=(line_thick, 1),
                    color=grid_color
                ))
            # Etiket (Alt kısım)
            if world_x != -half:
                lbl = Text(
                    text=str(int(world_x)),
                    parent=minimap_entity,
                    position=(local_x, -0.50, label_z),
                    scale=label_scale,
                    color=color.rgba(0, 0, 0, 1),
                    origin=(0.5, 0.5),
                    z=label_z
                )
                minimap_entity.grid_items.append(lbl)
            world_x += step

        # --- Yatay Çizgiler ve Y Etiketleri ---
        world_z = -half
        while world_z <= half:
            local_y = world_z * factor
            if world_z != 0:
                minimap_entity.grid_items.append(Entity(
                    parent=minimap_entity, model='quad',
                    position=(0, local_y, grid_z),
                    scale=(1, line_thick),
                    color=grid_color
                ))
            # Etiket (Sol kısım)
            lbl = Text(
                text=str(int(world_z)),
                parent=minimap_entity,
                position=(-0.52, local_y, label_z),
                scale=label_scale,
                color=color.rgba(0, 0, 0, 1),
                origin=(0.5, 0.5),
                z=label_z
            )
            minimap_entity.grid_items.append(lbl)
            world_z += step

        # --- Bilgi Metni ---
        step_m = step
        olcek_metre_birim = 2 * half
        info_z = label_z - 0.02
        t = Text(
            parent=minimap_entity,
            text=f"1 grid={step_m:.0f}m | 1 birim={olcek_metre_birim:.0f}m",
            position=(0, -0.54, info_z),
            scale=0.7,
            color=color.rgba(0, 0, 0, 1),
            origin=(0.5, 0.5),
            z=info_z
        )
        minimap_entity.grid_items.append(t)

        # --- Menzil Halkaları (havuz sınırına göre: 33%, 66%, 100% - ROV dağılım gösterimi) ---
        for r in [0.33, 0.66, 1.0]:
            minimap_entity.grid_items.append(Entity(
                parent=minimap_entity,
                model=Circle(resolution=60, radius=0.5 * r, mode='line', thickness=2),
                color=color.rgba(255,255,0,50),
                z=grid_z
            ))

    def draw_island_polygon(self, minimap_entity, vertices):
        """Ada çevresini çizgi olarak çizer (Filo GNC verisi için)."""
        return Entity(
            parent=minimap_entity,
            model=Mesh(vertices=vertices, mode='line', thickness=2),
            color=color.black, # Siyah kontür (Eski koddan: color.black, alpha=0.5)
            alpha=0.5,
            z=-0.22
        )

    def draw_static_circle(self, minimap_entity, x, z, r, havuz_genisligi, color_val=None):
        """Basit dairesel ada/engel çizimi. Havuz sınırı için havuz_genisligi (config referanslı) kullanılır."""
        if color_val is None: color_val = color.hex('#8B5A3C') # Toprak rengi
        # Havuz sınırları: ortamdan gelen havuz_genisligi; yoksa config
        h = havuz_genisligi if havuz_genisligi and havuz_genisligi > 0 else HavuzAyarlari.HAVUZ_GENISLIK
        map_scale = (r * 2) / (h * 2)
        factor = 1.0 / (h * 2)
        
        return Entity(
            parent=minimap_entity,
            model='circle',
            color=color_val,
            position=(x * factor, z * factor, -0.21),
            scale=(map_scale, map_scale),
            alpha=0.8
        )

    def create_rov_icon(self, minimap_entity, rov_id, rov_color):
        """ROV İkonu (Eski koddan birebir kopyalandı)."""
        # Gövde
        govde = Entity(
            parent=minimap_entity, 
            model='circle', 
            scale=0.022,  # ESKİ KOD DEĞERİ
            color=rov_color,
            position=(0,0,-0.4)
        )
        # Yön Oku
        Entity(
            parent=govde, 
            model='quad', 
            scale=(0.1, 0.4), # ESKİ KOD DEĞERİ
            y=0.2, 
            color=color.white
        )
        # ID Yazısı
        Text(
            parent=govde, 
            text=str(rov_id), 
            position=(0, 1.5, 0), 
            scale=40, # ESKİ KOD DEĞERİ
            color=color.rgb(255,0,0), 
            origin=(0.5, 0.5)
        )
        return govde



    # --- 1. HAVUZ VE OKYANUS ---
    def build_ocean(self, size=600):
        # Su Hacmi
        self.ortam.water_volume = Entity(
            model='cube', scale=(size, self.ortam.su_hacmi_yuksekligi, size),
            color=color.cyan, alpha=0.2, y=self.ortam.su_hacmi_merkez_y,
            unlit=False, transparent=True, collider=None
        )

        # Su Yüzeyi
        tex = "Models-3D/water/my_models/water4.jpg"
        self.ortam.ocean_surface = Entity(
            model="plane", scale=(size, 1, size), 
            position=(0, self.ortam.WATER_SURFACE_Y_BASE, 0),
            texture=tex if os.path.exists(tex) else None, 
            double_sided=True, color=color.rgb(0.5, 0.65, 0.9), 
            alpha=0.5, transparent=True
        )
        
        # Basit dalga animasyonu
        def ocean_update():
            dt = time.dt if time.dt > 0 else 0.016
            if not hasattr(self.ortam.ocean_surface, 'sim_time'):
                self.ortam.ocean_surface.sim_time = 0
            self.ortam.ocean_surface.sim_time += dt
            self.ortam.ocean_surface.y = self.ortam.WATER_SURFACE_Y_BASE + math.sin(self.ortam.ocean_surface.sim_time * 0.8) * 0.5
            self.ortam.ocean_surface.texture_offset = (self.ortam.ocean_surface.sim_time * 0.02, 0)
        self.ortam.ocean_surface.update = ocean_update

    # --- 2. DENİZ TABANI KATMANLARI ---
    def build_seabed(self, size=600):
        fbx_mod = "Models-3D/water/my_models/ocean_taban/sand_envi_034.fbx"
        fbx_tex = "Models-3D/water/my_models/ocean_taban/sand_envi_034-0.jpg"
        # Referans (200m) scale: 1.8, 0.6, 1.55 → mevcut_havuz/200 oranıyla çarpılır
        oran = self._havuz_scale_oran()
        scalex = 1.8 * oran
        scaley = 0.6 * oran
        scalez = 1.55 * oran
        
        self.ortam.ocean_taban = Entity(
            model=fbx_mod, scale=(scalex, scaley, scalez), 
            position=(0, self.ortam.SEA_FLOOR_Y - 4, 16), 
            texture=fbx_tex if os.path.exists(fbx_tex) else None,
            double_sided=True,collider='mesh', unlit=True
        )
        
        sk = self.ortam.su_hacmi_yuksekligi * 0.25
        self.ortam.seabed = Entity(
            model='cube', scale=(size, sk, size), 
            y=self.ortam.SEA_FLOOR_Y - (sk/2), color=color.rgb(139, 90, 43), 
            texture='brick', unlit=True,collider='box'
        )
        ck = self.ortam.su_hacmi_yuksekligi * 0.5
        self.ortam.cimen_katmani = Entity(
            model='cube', scale=(size, ck, size), 
            y=(self.ortam.SEA_FLOOR_Y - sk) - (ck/2), color=color.rgb(34, 139, 34), 
            texture='grass', unlit=True,collider="box"
        )

        # --- 3. ADALAR ---
    def create_island(self, sx, sz):
            """Çift modelli adayı (görsel + fizik) tek bir Entity ile yükler. Scale havuz boyutuna orantılı."""
            oran = self._havuz_scale_oran()
            ux, _, uz = sim_to_ursina(sx, sz, 0)
            model_path = "Models-3D/lowpoly-island/source/island1_design4_c4d.glb"
            # Referans scale (200m havuz): (0.23, 0.5, 0.24), offset (4, 6, 6)
            base_scale = (0.23, 0.5, 0.24)
            base_offset = (4, 6.0, 6)
            island = Entity(
                model=model_path,
                position=(ux + base_offset[0] * oran, base_offset[1] * oran, uz + base_offset[2] * oran),
                scale=(base_scale[0] * oran, base_scale[1] * oran, base_scale[2] * oran),
                double_sided=True
            )

            # 2. Modelin içindeki "Collider" parçasını bul
            # island.model, Panda3D düğümüne (node) erişim sağlar.
            collider_node = island.model.find("**/Island_Collider")

            # 3. Ayarları yap
            if not collider_node.isEmpty():
                # Collider parçasını GİZLE
                collider_node.hide()
                
                # Sadece o gizli parçayı FİZİKSEL DUVAR yap
                island.collider = MeshCollider(island, mesh=collider_node)
            else:
                # Eğer bulunamazsa, tüm adayı collider yap (Güvenlik önlemi)
                island.collider = 'mesh'

            # Minimap yarıçapı da havuz oranına göre (referans 38m)
            return island, 38 * oran

    # --- 4. MERKEZİ KAYALIK ---
    def load_rocky_reef(self, show_on_minimap=True):
        path = "Models-3D/floating_island_with_roots_and_rocks/scene2.glb"
        path = path.replace('\\', '/') if os.name != 'nt' else path.replace('/', '\\')
        oran = self._havuz_scale_oran()
        if os.path.exists(path):
            try:
                # Referans (200m): scale (3.5, 3.4, 3.5), position y=5
                self.ortam.central_reef = Entity(
                    model=path,
                    scale=(3.5 * oran, 3.4 * oran, 3.5 * oran),
                    position=(0, 5 * oran, 0),
                    collider='box', unlit=True, double_sided=True
                )
                if show_on_minimap:
                    if not hasattr(self.ortam, 'static_obstacles'):
                        self.ortam.static_obstacles = []
                    self.ortam.static_obstacles.append({
                        'pos': (0, 0), 'radius': 35.0 * oran, 'color': color.gray, 'name': 'Merkez Kayalık'
                    })
                    if hasattr(self.ortam, 'minimap') and self.ortam.minimap:
                        self.ortam.minimap._statik_yeniden_ciz()
                print(f"✅ Merkez Kayalığı yüklendi.")
            except Exception as e:
                print(f"⚠️ Merkez kayalığı hatası: {e}")

    # --- 5. SINIRLAR ---
    def build_boundaries(self, s):
        oran = self._havuz_scale_oran()
        # Referans (200m) yükseklikler: 150, 100
        p = {'model': 'cube', 'color': color.clear, 'visible': False, 'collider': 'box'}
        Entity(x=s+2, scale=(1, 150 * oran, s*2.5), position=(s+2, 0, 0), **p)
        Entity(x=-s-2, scale=(1, 100 * oran, s*2.5), position=(-s-2, 0, 0), **p)
        Entity(z=s+2, scale=(s*2.5, 150 * oran, 1), position=(0, 0, s+2), **p)
        Entity(z=-s-2, scale=(s*2.5, 150 * oran, 1), position=(0, 0, -s-2), **p)

    # --- 6. ROV MODEL YÜKLEME (YENİ) ---
    def setup_rov(self, rov_entity, model_key):
        """ROV modelini bulur, yükler ve sensör görselleştirmelerini ekler."""
        if isinstance(model_key, str): model_key = model_key.lower().strip()
        if model_key not in ROVModelleri.MODELLER: model_key = ROVModelleri.VARSAYILAN
        
        model_info = ROVModelleri.MODELLER[model_key]
        rel_path = model_info['path']
        
        # Dosya yolu bulma mantığı
        script_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) # Kütüphane yapısına göre adjust edilebilir
        cwd = os.getcwd()
        path_parts = rel_path.replace('\\', '/').split('/')
        
        potential_paths = [
            rel_path,
            os.path.join(script_dir, *path_parts),
            os.path.join(cwd, *path_parts),
            os.path.abspath(rel_path)
        ]
        
        final_path = 'cube' # Fallback
        rov_entity.color = color.orange
        rov_entity.unlit = True
        
        for path in potential_paths:
            if os.path.exists(path):
                final_path = path
                rov_entity.color = color.white
                rov_entity.unlit = False
                break
        
        # Entity Ayarları
        rov_entity.model = final_path
        rov_entity.scale = model_info['scale']
        rov_entity.collider = 'box'
        
        # Görsel Yardımcılar (Label, Sensör Alanı)
        rov_entity.label = Text(
            text=f"ROV-{rov_entity.id}", parent=rov_entity, y=3.0, scale=20,
            billboard=False, color=color.white, origin=(0, 0),
            rotation=(0, -90, 0)
        )
        
        # Güvenlik Alanı (Safety Zone)
        radius = rov_entity.sensor_config.get("engel_mesafesi", GATLimitleri.ENGEL) / 2.0
        rov_entity.safety_zone = Entity(
            parent=rov_entity, model='cube', scale=radius * 2,
            collider=None, color=color.rgba(255, 0, 0, 50),
            visible=True, unlit=True
        )


# --- 4. MERKEZİ KAYALIK (GÜNCELLENDİ) ---
    def load_rocky_reef(self, show_on_minimap=True):
        """
        secene4.glb dosyasını yükler. 
        Bu dosya içinde hem görsel hem de collider mesh varsa otomatik algılar.
        """
        # Dosya yolu (Senin verdiğin konuma göre)
        # Not: Ursina 'Models-3D' klasörünü otomatik tanıyabilir ama tam yol garantidir.
        model_path = "Models-3D/floating_island_with_roots_and_rocks/scene2.glb"
        
        # İşletim sistemine göre yol düzeltmesi (\ veya /)
        if os.name == 'nt': 
            model_path = model_path.replace('/', '\\')
        
        # Dosya var mı kontrolü
        if not os.path.exists(model_path):
            print(f"⚠️ [UYARI] Merkez kayalık dosyası bulunamadı: {model_path}")
            return

    
        # 1. Modeli Yükle (scale havuz boyutuna orantılı: referans 200m için (2, 3, 2))
        oran = self._havuz_scale_oran()
        self.ortam.central_reef = Entity(
                model=model_path,
                scale=(2 * oran, 3 * oran, 2 * oran),
                position=(0, 5 * oran, 0),
                double_sided=True,
                collider="cube"
            )


    def rock(self, scale, position):
        model_path = "Models-3D/rock/stone.glb"

        if os.path.exists(model_path):
            rock_entity = Entity(model=model_path, scale=scale, position=position, collider='mesh', unlit=True)
            self.rock_entities.append(rock_entity)  # Havuzda tut
            return rock_entity
        else:
            print(f"⚠️ Kaya modeli bulunamadı: {model_path}")
            return None

    def spawn_rocks(self, count=20, havuz_genisligi=None):
        """
        Havuz içinde rastgele konumlarda kayalar oluşturur.
        
        Args:
            count: Oluşturulacak kaya sayısı
            havuz_genisligi: Havuz yarı genişliği (None ise config'den alınır)
        """
        import random
        if havuz_genisligi is None:
            havuz_genisligi = HavuzAyarlari.HAVUZ_GENISLIK

        # Havuz sınırları: ±havuz_genisligi (x ve z için)
        min_coord = -(havuz_genisligi - 20)
        max_coord = (havuz_genisligi - 20)
        oran = havuz_genisligi / HavuzAyarlari.HAVUZ_GENISLIK  # mevcut_havuz/200

        for _ in range(count):
            # Scale: referans 200m için 20-50; havuz boyutuna orantılı
            scale = random.uniform(20, 50) * oran
            
            # Position: x ve z havuz içinde, y=-30 sabit (derinlik)
            x = random.uniform(min_coord, max_coord)
            z = random.uniform(min_coord, max_coord)
            y = -40-4*(40/scale)  # Sabit derinlik
            
            position = Vec3(x, y, z)
            self.rock(scale=scale, position=position)
        
        print(f"✅ {count} adet kaya oluşturuldu (derinlik: -30m)")
