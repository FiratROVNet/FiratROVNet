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
    FizikSabitleri, ROVModelleri
)
from .utils import sim_to_ursina, ursina_to_sim
from .kutuphane.helper.EntityLoader import EntityLoader
from .kutuphane.helper.simulasyon_helper import OrtamHelper

# ============================================================
# 1. ROV SINIFI (Mantık ve Fizik)
# ============================================================
class ROV(Entity):
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

        self.group_id = group_id # Grup ID bilgisi

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
                        destroy(ortam.sonar_cizgiler[pair])
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

            # 2. Görselleri temizle
            if hasattr(self, 'label') and self.label: destroy(self.label)
            if hasattr(self, 'engel_cizgi') and self.engel_cizgi: destroy(self.engel_cizgi)

            # 3. Listeyi yeniden numaralandirma yok
            print(f"✅ ROV-{silinen_id} ve tum gorsel izleri temizlendi.")
            destroy(self)
            

    def _etiket_guncelle(self):
        """ID değiştiğinde üzerindeki yazıyı günceller."""
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
                 "rol": self.role, "sonar": self.son_sonar_mesafesi}
            return np.array(d[veri]) if veri in d else None
        except:
            return None


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
        # Minimap __init__ içinde:

        self.obstacle_cloud_entity = None # Tek bir entity kullanacağız

        # 1. Engel noktalarının koordinatlarını tutacak liste
        self.engel_vertex_listesi = [] 

        # 2. Tek bir Mesh oluşturuyoruz (mode='point' önemli)
        # thickness=2 yaparak o 'kırmızı blok' sorununu baştan çözüyoruz.
        self.engel_mesh = Mesh(vertices=[], mode='point', thickness=0.016, static=False)

        # 3. Bu Mesh'i ekranda gösterecek TEK Entity
        self.engel_gorseli = Entity(
            parent=self, # Veya self.minimap_panel, nereye koyuyorsan
            model=self.engel_mesh,
            color=color.brown, # Engeller kırmızı olsun
            z=-0.01 # Haritanın hafif önünde
        )
        
        self._engel_bulutu_cizilen_len = 0
        self.kayitli_noktalar = set() 


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
                    
                    # Eğer ikon yoksa oluştur (ID kayması durumunda yeni ID'ye ikon atanır)
                    if rov.id not in self.rov_ikonlari:
                        self.rov_ikonlari[rov.id] = self.loader.create_rov_icon(self, rov.id, rov.color)
                    
                    icon = self.rov_ikonlari[rov.id]
                    icon.position = target
                    icon.rotation_z = -rov.rotation_y
                    icon.color = rov.color

            self._vektor_ve_hedef_guncelle()

    def _vektor_ve_hedef_guncelle(self):
        filo = getattr(self.ortam_ref, 'filo', None)
        helper = getattr(filo, 'helper', None) if filo else None
        apf_list = helper.get_apf_vektor_verts_list(self) if helper else []
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
            for e in self.vektor_cizgi_entities: destroy(e)
            self.vektor_cizgi_entities.clear()
            for v, c in apf_list:
                try: self.vektor_cizgi_entities.append(self.loader.create_vector_mesh(self, v, c))
                except: pass

    def _engel_bulutu_guncelle_yedek(self):
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


    def _engel_bulutu_guncelle(self):
        bulut = getattr(self.ortam_ref, 'engel_bulutu', [])
        
        # Reset durumunda hafızayı da temizle
        if len(bulut) < self._engel_bulutu_cizilen_len:
            self.engel_vertex_listesi.clear()
            self.kayitli_noktalar.clear() # Hafızayı sil
            self.engel_mesh.vertices = []
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
                    
                    # Hafızaya kaydet (Set'e ekle)
                    self.kayitli_noktalar.add(point_key)
                    
                    yeni_veri_var = True

        # Mesh'i sadece yeni nokta eklendiyse güncelle
        if yeni_veri_var:
            self.engel_mesh.vertices = self.engel_vertex_listesi
            self.engel_mesh.generate()

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
            zoom_speed=1, position=(0, 20, -50), rotation=(20, 0, 0)
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

    def sim_olustur(self, n_rovs=(6,), n_islands=5, havuz_genisligi=200, rov_model='submarine'):
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
        
        # Kayaları ekle
        self.loader.spawn_rocks(count=20, havuz_genisligi=havuz_genisligi)
        
        # 1. Adaları Sabit Noktalardan Yerleştir
        count = min(n_islands, len(self.FIXED_ISLAND_POSITIONS))
        chosen_islands = random.sample(self.FIXED_ISLAND_POSITIONS, count)
        chosen_islands.insert(0,(0,0))
        for i, pos in enumerate(chosen_islands):
            self.Ada(i, x="ekle", y=pos)

        # 2. ROV'ları Güvenli Noktalara Yerleştir
        print(f"🌊 Simülasyon Başlatılıyor: {n_rovs} ROV, {count} Ada")

        all_group = self._find_safe_rov_spawn_pos(n_rovs)

        for group_id,rovlar in enumerate(all_group):
            for rov_id,rov_koordinat in enumerate(rovlar):


                # sim_pos: (x, z_depth, y_coordinate) -> ursina: (x, y, z)
                u_pos = Vec3(rov_koordinat[0], rov_koordinat[2], rov_koordinat[1])
                    
                new_rov = ROV(rov_id=i,group_id=group_id, position=u_pos, loader_ref=self.loader, model_key=rov_model)
                new_rov.ekle(self)

                self.minimap._statik_yeniden_ciz()


    def Ada(self, ada_id, x=None, y=None):
        if x == "ekle":
            while len(self.island_positions) <= ada_id: self.island_positions.append(None)
            while len(self.island_entities) <= ada_id: self.island_entities.append(None)
            ent, radius = self.loader.create_island(y[0], y[1])
            self.island_entities[ada_id], self.island_positions[ada_id] = ent, (y[0], y[1], radius)
            
    def guncelle_sonar_cizgileri(self):
            """
            ROV'lar arası sonar iletişimini HACİMLİ KESİKLİ çizgilerle gösterir.
            Dinamik Kalınlık: Yakınken kalın (1.25x), uzakken ince (0.75x).
            """
            active_rovs = [r for r in self.rovs if r and not (hasattr(r, 'is_destroyed') and r.is_destroyed)]
            bu_frame_aktif_olanlar = set()
            
            # --- AYARLAR ---
            BASE_KALINLIK = 0.12 # Mevcut temel kalınlığın
            segment_boyu = 1.2
            bosluk_boyu = 1.8
            adim_toplam = segment_boyu + bosluk_boyu

            for i, r1 in enumerate(active_rovs):
                for r2 in active_rovs[i+1:]:
                    if r1.gat_kodu == 3 or r2.gat_kodu == 3:
                        continue

                    p1, p2 = r1.position, r2.position
                    dist = (p2 - p1).length()
                    
                    if dist < self.SONAR_MENZILI:
                        pair = tuple(sorted((r1.id, r2.id)))
                        bu_frame_aktif_olanlar.add(pair)
                        
                        # --- DİNAMİK KALINLIK HESABI ---
                        # lerp(başlangıç, bitiş, oran) 
                        # Mesafe 0 iken -> 1.25 | Mesafe MAX iken -> 0.75
                        oran = dist / self.SONAR_MENZILI
                        carpan = lerp(1.25, 0.75, oran)
                        guncel_kalinlik = BASE_KALINLIK * carpan
                        
                        # --- RENK MANTIĞI ---
                        if dist < 25:
                            c = color.red
                        elif dist < 80:
                            c = color.orange
                        else:
                            c = color.white
                        
                        # Önceki Entity'yi temizle
                        if pair in self.sonar_cizgiler:
                            destroy(self.sonar_cizgiler[pair])
                        
                        cizgi_konteyner = Entity(add_to_scene_entities=True)
                        self.sonar_cizgiler[pair] = cizgi_konteyner
                        
                        yon_vec = (p2 - p1).normalized()
                        curr = 0
                        while curr < dist:
                            kalin_uzunluk = min(segment_boyu, dist - curr)
                            if kalin_uzunluk <= 0.1: break
                            
                            parca_baslangic = p1 + yon_vec * curr
                            parca_bitis = parca_baslangic + yon_vec * kalin_uzunluk
                            orta_nokta = (parca_baslangic + parca_bitis) / 2
                            
                            parca = Entity(
                                parent=cizgi_konteyner,
                                model='cube',
                                position=orta_nokta,
                                # Dinamik kalınlık burada uygulanıyor:
                                scale=(guncel_kalinlik, guncel_kalinlik, kalin_uzunluk),
                                color=c,
                                unlit=True
                            )
                            parca.look_at(parca_bitis)
                            curr += adim_toplam

            # Temizlik döngüsü
            tum_cizilmis_ciftler = list(self.sonar_cizgiler.keys())
            for pair in tum_cizilmis_ciftler:
                if pair not in bu_frame_aktif_olanlar:
                    if self.sonar_cizgiler[pair]:
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