from ursina import *
import numpy as np
import random
import threading
import code
import sys
    
from .config import cfg # <-- BU SATIRI EKLE

  

# --- FİZİK SABİTLERİ ---
SURTUNME_KATSAYISI = 0.95
HIZLANMA_CARPANI = 0.5
KALDIRMA_KUVVETI = 2.0



class ROV(Entity):
    def __init__(self, rov_id, **kwargs):
        super().__init__()
        self.model = 'cube'
        self.color = color.orange # Turuncu her zaman görünür
        self.scale = (1.5, 0.8, 2.5)
        self.collider = 'box'
        self.unlit = True 
        
        if 'position' in kwargs: self.position = kwargs['position']
        else: self.position = (0, -5, 0)

        self.label = Text(text=f"ROV-{rov_id}", parent=self, y=1.5, scale=5, billboard=True, color=color.white)
        
        self.id = rov_id
        self.velocity = Vec3(0, 0, 0)
        self.battery = 100.0
        self.role = 0 
        
        self.sensor_config = {
            "engel_mesafesi": 20.0,
            "iletisim_menzili": 35.0,
            "min_pil_uyarisi": 10.0
        }
        self.environment_ref = None 

    def update(self):
        # Fizik
        self.position += self.velocity * time.dt
        self.velocity *= SURTUNME_KATSAYISI
        
        if self.role == 1: # Lider
            if self.y < 0:
                self.velocity.y += KALDIRMA_KUVVETI * time.dt
                if self.y > -0.5: self.velocity.y *= 0.5
            if self.y < -2: self.y = -2
            if self.y > 0.5: 
                self.y = 0.5
                self.velocity.y = 0
        else: # Takipçi
            if self.y > 0: 
                self.y = 0
                self.velocity.y = 0
            if self.y < -100: 
                self.y = -100
                self.velocity.y = 0

        if self.velocity.length() > 0.01: 
            self.battery -= 0.01 * time.dt

    def move(self, komut, guc=1.0):
        thrust = guc * HIZLANMA_CARPANI * time.dt
        if self.battery <= 0: return

        if komut == "ileri":  self.velocity.z += thrust
        elif komut == "geri": self.velocity.z -= thrust
        elif komut == "sag":  self.velocity.x += thrust
        elif komut == "sol":  self.velocity.x -= thrust
        elif komut == "cik":  self.velocity.y += thrust 
        elif komut == "bat":  
            if self.role == 1: pass
            else: self.velocity.y -= thrust 
        elif komut == "dur":
            self.velocity = Vec3(0,0,0)

    def set(self, ayar_adi, deger):
        if ayar_adi == "rol":
            self.role = int(deger)
            if self.role == 1:
                self.color = color.red
                self.label.text = f"LIDER-{self.id}"
                print(f"✅ ROV-{self.id} artık LİDER.")
            else:
                self.color = color.orange
                self.label.text = f"ROV-{self.id}"
                print(f"✅ ROV-{self.id} artık TAKİPÇİ.")
        elif ayar_adi in self.sensor_config: 
            self.sensor_config[ayar_adi] = deger

    def get(self, veri_tipi):
        if veri_tipi == "gps": return np.array([self.x, self.y, self.z])
        elif veri_tipi == "hiz": return np.array([self.velocity.x, self.velocity.y, self.velocity.z])
        elif veri_tipi == "batarya": return self.battery
        elif veri_tipi == "rol": return self.role
        elif veri_tipi == "sonar":
            min_dist = 999.0
            if self.environment_ref:
                for engel in self.environment_ref.engeller:
                    avg_scale = (engel.scale_x + engel.scale_z) / 2
                    d = distance(self, engel) - (avg_scale / 2)
                    if d < min_dist: min_dist = d
            menzil = self.sensor_config["engel_mesafesi"]
            return min_dist if min_dist < menzil else -1
        return None

class Ortam:
    def __init__(self):
        # --- Ursina Ayarları ---
        self.app = Ursina(
            vsync=False,
            development_mode=False,
            show_ursina_splash=False,
            borderless=False,
            title="FıratROVNet Simülasyonu"
        )
        
        window.fullscreen = False
        window.exit_button.visible = False
        window.fps_counter.enabled = True
        window.size = (1024, 768)
        window.center_on_screen()
        application.run_in_background = True
        window.color = color.rgb(10, 30, 50)  # Arka plan
        
        # Sağ tıklama menüsünü kapat (mouse.right event'lerini yakalamak için)
        try:
            window.context_menu = False
        except:
            pass
        EditorCamera()
        self.editor_camera = EditorCamera()
        self.editor_camera.enabled = False  # Başlangıçta kapalı

        # --- Sahne Nesneleri ---
        self.surface = Entity(
            model='plane',
            scale=(500,1,500),
            color=color.cyan,
            alpha=0.3,
            y=0,
            unlit=True,
            double_sided=True,
            transparent=True
        )

        self.water_volume = Entity(
            model='cube',
            scale=(500,100,500),
            color=color.cyan,
            alpha=0.2,
            y=-50,
            unlit=True,
            double_sided=True,
            transparent=True
        )

        self.seabed = Entity(
            model='plane',
            scale=(500,1,500),
            color=color.rgb(20,20,30),
            y=-100,
            unlit=True,
            texture='grass'
        )

        # ROV ve engel listeleri
        self.rovs = []
        self.engeller = []

        # Konsol verileri
        self.konsol_verileri = {}

    # --- Simülasyon Nesnelerini Oluştur ---
    def sim_olustur(self, n_rovs=3, n_engels=15):
        # Engeller
        for _ in range(n_engels):
            x = random.uniform(-40, 40)
            z = random.uniform(0, 80)
            y = random.uniform(-90, -10)

            s_x = random.uniform(4,12)
            s_y = random.uniform(4,12)
            s_z = random.uniform(4,12)

            gri = random.randint(80,180)
            kaya_rengi = color.rgb(gri, gri, gri)

            engel = Entity(
                model='icosphere',
                color=kaya_rengi,
                texture='noise',
                scale=(s_x,s_y,s_z),
                position=(x,y,z),
                rotation=(random.randint(0,360), random.randint(0,360), random.randint(0,360)),
                collider='mesh',
                unlit=True
            )
            self.engeller.append(engel)

        # ROV'lar
        for i in range(n_rovs):
            x = random.uniform(-10,10)
            z = random.uniform(-10,10)
            new_rov = ROV(rov_id=i, position=(x,-2,z))  # ROV sınıfın kendi tanımlı olmalı
            new_rov.environment_ref = self
            self.rovs.append(new_rov)

        print(f"🌊 Simülasyon Hazır: {n_rovs} ROV, {n_engels} Gri Kaya.")

    # --- İnteraktif Shell ---
    def _start_shell(self):
        import time
        time.sleep(1)
        print("\n" + "="*60)
        print("🚀 FIRAT ROVNET CANLI KONSOL")
        print("Çıkmak için Ctrl+D veya 'exit()' yazın.")
        print("="*60 + "\n")

        local_vars = {
            'rovs': self.rovs,
            'engeller': self.engeller,
            'app': self,
            'ursina': sys.modules['ursina'],
            'cfg': cfg
        }
        if hasattr(self, 'konsol_verileri'):
            local_vars.update(self.konsol_verileri)

        try:
            code.interact(local=dict(globals(), **local_vars))
        except SystemExit:
            pass
        except Exception as e:
            print(f"Konsol Hatası: {e}")
        finally:
            print("Konsol kapatılıyor...")
            import os
            os.system('stty sane')
            os._exit(0)

    # --- Update Fonksiyonunu Set Et ---
    def set_update_function(self, func):
        self.app.update = func

    # --- Konsola Veri Ekle ---
    def konsola_ekle(self, isim, nesne):
        self.konsol_verileri[isim] = nesne

    # --- Main Run Fonksiyonu ---
    def run(self, interaktif=False):
        if interaktif:
            t = threading.Thread(target=self._start_shell)
            t.daemon = True
            t.start()
        self.app.run()
