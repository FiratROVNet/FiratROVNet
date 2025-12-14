from ursina import *
import numpy as np
import random
import threading
import code
import sys
import torch
    
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
        # Havuz sınır kontrolü
        if self.environment_ref:
            havuz_genisligi = getattr(self.environment_ref, 'havuz_genisligi', 200)
            havuz_yari_genislik = havuz_genisligi / 2
            
            # X ve Z sınırları
            if abs(self.x) > havuz_yari_genislik:
                self.x = np.sign(self.x) * havuz_yari_genislik
                self.velocity.x = 0  # Sınırda durdur
            
            if abs(self.z) > havuz_yari_genislik:
                self.z = np.sign(self.z) * havuz_yari_genislik
                self.velocity.z = 0  # Sınırda durdur
        
        # Fizik
        self.position += self.velocity * time.dt
        self.velocity *= SURTUNME_KATSAYISI
        
        # Çarpışma kontrolü
        if self.environment_ref:
            self._carpisma_kontrolu()
        
        if self.role == 1: # Lider - Su yüzeyinde kalmalı, batırılamaz
            # Lider her zaman su yüzeyine çıkar
            if self.y < 0:
                self.velocity.y += KALDIRMA_KUVVETI * 2.0 * time.dt  # Daha güçlü kaldırma
                if self.y > -0.3: self.velocity.y *= 0.3
            # Su yüzeyi limitleri
            if self.y < -1.0: 
                self.y = -1.0  # Su yüzeyine zorla çıkar
                self.velocity.y = max(0, self.velocity.y)  # Aşağı hızı sıfırla
            if self.y > 0.5: 
                self.y = 0.5
                self.velocity.y = 0
            # Lider asla batırılamaz - aşağı hızı engelle
            if self.velocity.y < 0:
                self.velocity.y = 0
        else: # Takipçi - Serbest hareket
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
            # Lider batırılamaz
            if self.role == 1: 
                pass  # Lider için bat komutu işe yaramaz
            else: 
                self.velocity.y -= thrust 
        elif komut == "dur":
            self.velocity = Vec3(0,0,0)

    def set(self, ayar_adi, deger):
        if ayar_adi == "rol":
            eski_rol = self.role
            self.role = int(deger)
            if self.role == 1:
                self.color = color.red
                self.label.text = f"LIDER-{self.id}"
                # Lider olduğunda su yüzeyine çıkar
                if self.y < 0:
                    self.y = 0
                    self.velocity.y = 0
                print(f"✅ ROV-{self.id} artık LİDER (Su yüzeyinde).")
            else:
                self.color = color.orange
                self.label.text = f"ROV-{self.id}"
                # Takipçi olduğunda artık batırılabilir
                print(f"✅ ROV-{self.id} artık TAKİPÇİ (Batırılabilir).")
        elif ayar_adi == "renk":
            # Renk ayarlama
            if isinstance(deger, (tuple, list)) and len(deger) >= 3:
                self.color = color.rgb(int(deger[0]), int(deger[1]), int(deger[2]))
            elif isinstance(deger, str):
                # Renk ismi ile
                renk_dict = {
                    'kirmizi': color.red, 'mavi': color.blue, 'yesil': color.green,
                    'sari': color.yellow, 'turuncu': color.orange, 'mor': color.magenta,
                    'beyaz': color.white, 'siyah': color.black
                }
                self.color = renk_dict.get(deger.lower(), color.white)
        elif ayar_adi in self.sensor_config: 
            self.sensor_config[ayar_adi] = deger

    def get(self, veri_tipi):
        if veri_tipi == "gps": return np.array([self.x, self.y, self.z])
        elif veri_tipi == "hiz": return np.array([self.velocity.x, self.velocity.y, self.velocity.z])
        elif veri_tipi == "batarya": return self.battery
        elif veri_tipi == "rol": return self.role
        elif veri_tipi == "renk": return self.color
        elif veri_tipi == "sensör" or veri_tipi == "sensor":
            return self.sensor_config.copy()
        elif veri_tipi == "engel_mesafesi": return self.sensor_config.get("engel_mesafesi")
        elif veri_tipi == "iletisim_menzili": return self.sensor_config.get("iletisim_menzili")
        elif veri_tipi == "min_pil_uyarisi": return self.sensor_config.get("min_pil_uyarisi")
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

    def _carpisma_kontrolu(self):
        """
        Çarpışma kontrolü ve momentum korunumu ile gerçekçi çarpışma.
        """
        if not self.environment_ref:
            return
        
        # ROV kütlesi (basitleştirilmiş)
        rov_kutlesi = 1.0
        
        # Diğer ROV'larla çarpışma
        for diger_rov in self.environment_ref.rovs:
            if diger_rov.id == self.id:
                continue
            
            mesafe = distance(self.position, diger_rov.position)
            min_mesafe = 2.0  # ROV boyutlarına göre minimum mesafe
            
            if mesafe < min_mesafe:
                # Çarpışma tespit edildi
                # Normalize edilmiş çarpışma yönü
                carpisma_yonu = (self.position - diger_rov.position).normalized()
                
                # Göreceli hız
                goreceli_hiz = self.velocity - diger_rov.velocity
                goreceli_hiz_buyuklugu = goreceli_hiz.length()
                
                if goreceli_hiz_buyuklugu > 0.1:
                    # Momentum korunumu (elastik çarpışma)
                    # Basitleştirilmiş: Her iki ROV da aynı kütlede
                    diger_rov_kutlesi = 1.0
                    
                    # Çarpışma sonrası hızlar (momentum korunumu)
                    # v1' = v1 - 2*m2/(m1+m2) * (v1-v2) · n * n
                    # v2' = v2 - 2*m1/(m1+m2) * (v2-v1) · n * n
                    
                    nokta_carpim = goreceli_hiz.dot(carpisma_yonu)
                    
                    if nokta_carpim < 0:  # Birbirine yaklaşıyorlar
                        # Yeni hızlar
                        self.velocity = self.velocity - (2 * diger_rov_kutlesi / (rov_kutlesi + diger_rov_kutlesi)) * nokta_carpim * carpisma_yonu
                        diger_rov.velocity = diger_rov.velocity - (2 * rov_kutlesi / (rov_kutlesi + diger_rov_kutlesi)) * (-nokta_carpim) * (-carpisma_yonu)
                        
                        # Çarpışma sonrası pozisyonları ayır
                        ayirma_mesafesi = (min_mesafe - mesafe) / 2
                        self.position += carpisma_yonu * ayirma_mesafesi
                        diger_rov.position -= carpisma_yonu * ayirma_mesafesi
        
        # Kayalarla çarpışma
        for engel in self.environment_ref.engeller:
            mesafe = distance(self.position, engel.position)
            # Engel boyutuna göre minimum mesafe
            engel_yari_cap = max(engel.scale_x, engel.scale_y, engel.scale_z) / 2
            min_mesafe = engel_yari_cap + 1.0
            
            if mesafe < min_mesafe:
                # Kaya ile çarpışma
                carpisma_yonu = (self.position - engel.position).normalized()
                
                # Hızı yansıt (kaya sabit, ROV geri seker)
                hiz_buyuklugu = self.velocity.length()
                if hiz_buyuklugu > 0.1:
                    # Yansıma (momentum korunumu - kaya çok ağır, ROV geri seker)
                    nokta_carpim = self.velocity.dot(carpisma_yonu)
                    if nokta_carpim < 0:  # Kayaya doğru gidiyor
                        self.velocity = self.velocity - 2 * nokta_carpim * carpisma_yonu
                        
                        # Pozisyonu ayır
                        ayirma_mesafesi = (min_mesafe - mesafe)
                        self.position += carpisma_yonu * ayirma_mesafesi

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

        # Su hacmi parametreleri
        su_hacmi_yuksekligi = 100.0
        su_hacmi_merkez_y = -50.0
        
        self.water_volume = Entity(
            model='cube',
            scale=(500, su_hacmi_yuksekligi, 500),
            color=color.cyan,
            alpha=0.2,
            y=su_hacmi_merkez_y,
            unlit=True,
            double_sided=True,
            transparent=True
        )

        # Deniz tabanı kalınlığı: Su hacmi yüksekliğinin 0.1'i
        seabed_kalinligi = su_hacmi_yuksekligi * 0.1
        # Deniz tabanı alt yüzeyi: Su hacminin altı
        seabed_alt_yuzey = su_hacmi_merkez_y - (su_hacmi_yuksekligi / 2)
        # Deniz tabanı merkez y: Alt yüzeyin üstünde kalınlığın yarısı kadar
        seabed_merkez_y = seabed_alt_yuzey - (seabed_kalinligi / 2)
        
        # Deniz tabanı - Kalın, opak, kum/toprak görünümlü
        self.seabed = Entity(
            model='cube',
            scale=(500, seabed_kalinligi, 500),
            color=color.rgb(139, 90, 43),  # Kahverengi/kum rengi
            y=seabed_merkez_y,
            unlit=True,
            texture='brick',  # Kum/toprak görünümü için
            double_sided=False
        )
        
        # Çimen katmanı kalınlığı: Su hacmi yüksekliğinin 0.25'i
        cimen_kalinligi = su_hacmi_yuksekligi * 0.25
        # Çimen katmanı alt yüzeyi: Deniz tabanının altı
        cimen_alt_yuzey = seabed_merkez_y - (seabed_kalinligi / 2)
        # Çimen katmanı merkez y
        cimen_merkez_y = cimen_alt_yuzey - (cimen_kalinligi / 2)
        
        # Çimen katmanı - Deniz tabanının altında
        self.cimen_katmani = Entity(
            model='cube',
            scale=(500, cimen_kalinligi, 500),
            color=color.rgb(34, 139, 34),  # Çimen yeşili
            y=cimen_merkez_y,
            unlit=True,
            texture='grass',  # Çimen texture'ı
            double_sided=False
        )

        # ROV ve engel listeleri
        self.rovs = []
        self.engeller = []
        
        # Hedef nokta (varsayılan)
        self.hedef_nokta = Vec3(40, 0, 60)
        
        # Havuz genişliği (varsayılan)
        self.havuz_genisligi = 200.0

        # Konsol verileri
        self.konsol_verileri = {}

    # --- Simülasyon Nesnelerini Oluştur ---
    def sim_olustur(self, n_rovs=3, n_engels=15, hedef_nokta=None, havuz_genisligi=200):
        """
        Simülasyon nesnelerini oluşturur.
        
        Args:
            n_rovs: ROV sayısı
            n_engels: Engel sayısı
            hedef_nokta: Hedef nokta (Vec3) - engeller bu noktadan uzak olur
            havuz_genisligi: Havuz genişliği (varsayılan: 200)
        """
        # Havuz genişliğini kaydet
        self.havuz_genisligi = havuz_genisligi
        
        # Hedef noktayı ayarla
        if hedef_nokta is None:
            self.hedef_nokta = Vec3(40, 0, 60)
        else:
            self.hedef_nokta = hedef_nokta
        
        # Havuz sınırları
        havuz_yari_genislik = havuz_genisligi / 2
        hedef_guvenlik_mesafesi = 30.0  # Hedeften minimum mesafe
        
        # Engeller - Tüm havuza yayılmış, hedeften uzak
        engel_deneme_sayisi = 0
        max_deneme = n_engels * 10  # Maksimum deneme sayısı
        
        while len(self.engeller) < n_engels and engel_deneme_sayisi < max_deneme:
            engel_deneme_sayisi += 1
            
            # Tüm havuza geniş şekilde yayılmış random pozisyon (daha geniş alan)
            x = random.uniform(-havuz_yari_genislik * 0.9, havuz_yari_genislik * 0.9)
            z = random.uniform(-havuz_yari_genislik * 0.9, havuz_yari_genislik * 0.9)
            y = random.uniform(-90, -10)
            
            # Hedef noktadan uzaklık kontrolü
            engel_pos = Vec3(x, y, z)
            if distance(engel_pos, self.hedef_nokta) < hedef_guvenlik_mesafesi:
                continue  # Hedefe çok yakın, tekrar dene
            
            # Boyutlar
            s_x = random.uniform(4, 12)
            s_y = random.uniform(4, 12)
            s_z = random.uniform(4, 12)
            
            # Geniş gri aralığında kaya renkleri (daha geniş spektrum)
            # 40-200 arası gri tonları
            gri_tonu = random.randint(40, 200)
            # Biraz varyasyon ekle (benek efekti için)
            r_varyasyon = random.randint(-15, 15)
            g_varyasyon = random.randint(-15, 15)
            b_varyasyon = random.randint(-15, 15)
            
            kaya_rengi = color.rgb(
                max(30, min(220, gri_tonu + r_varyasyon)),
                max(30, min(220, gri_tonu + g_varyasyon)),
                max(30, min(220, gri_tonu + b_varyasyon))
            )
            
            engel = Entity(
                model='icosphere',
                color=kaya_rengi,
                texture='noise',  # Benek efekti için noise texture
                scale=(s_x, s_y, s_z),
                position=(x, y, z),
                rotation=(random.randint(0, 360), random.randint(0, 360), random.randint(0, 360)),
                collider='mesh',
                unlit=True
            )
            self.engeller.append(engel)
        
        # ROV'lar - Engellerden ve hedeften uzak oluştur
        rov_guvenlik_mesafesi = 15.0  # Engellerden minimum mesafe
        rov_deneme_sayisi = 0
        max_rov_deneme = n_rovs * 20
        
        for i in range(n_rovs):
            gecerli_pozisyon = False
            deneme = 0
            
            while not gecerli_pozisyon and deneme < max_rov_deneme:
                deneme += 1
                x = random.uniform(-havuz_yari_genislik * 0.3, havuz_yari_genislik * 0.3)
                z = random.uniform(-havuz_yari_genislik * 0.3, havuz_yari_genislik * 0.3)
                y = -2  # Su yüzeyine yakın
                
                rov_pos = Vec3(x, y, z)
                gecerli_pozisyon = True
                
                # Hedeften uzak mı?
                if distance(rov_pos, self.hedef_nokta) < hedef_guvenlik_mesafesi:
                    gecerli_pozisyon = False
                    continue
                
                # Engellerden uzak mı?
                for engel in self.engeller:
                    if distance(rov_pos, engel.position) < rov_guvenlik_mesafesi:
                        gecerli_pozisyon = False
                        break
                
                if gecerli_pozisyon:
                    new_rov = ROV(rov_id=i, position=(x, y, z))
                    new_rov.environment_ref = self
                    self.rovs.append(new_rov)
                    break
            
            # Eğer geçerli pozisyon bulunamazsa varsayılan pozisyon
            if not gecerli_pozisyon:
                x = -20 + (i * 10)
                z = -20 + (i * 10)
                new_rov = ROV(rov_id=i, position=(x, -2, z))
                new_rov.environment_ref = self
                self.rovs.append(new_rov)
        
        print(f"🌊 Simülasyon Hazır: {n_rovs} ROV, {len(self.engeller)} Kaya, Hedef: {self.hedef_nokta}")

    # --- GAT Veri Dönüşüm Fonksiyonu ---
    def simden_veriye(self, limitler=None):
        """
        Fiziksel dünyayı Matematiksel matrise çevirir (GAT Girdisi)
        
        Args:
            limitler (dict, optional): Mesafe limitleri. Varsayılan değerler:
                - 'LEADER': 60.0 (Liderden uzaklık limiti)
                - 'DISCONNECT': 35.0 (Bağlantı kopma limiti)
                - 'OBSTACLE': 20.0 (Engel tespit limiti)
                - 'COLLISION': 8.0 (Çarpışma tespit limiti)
        
        Returns:
            MiniData: x (özellik matrisi) ve edge_index (graf bağlantıları) içeren nesne
        """
        # Varsayılan limitler
        if limitler is None:
            limitler = {
                'LEADER': 60.0,
                'DISCONNECT': 35.0,
                'OBSTACLE': 20.0,
                'COLLISION': 8.0
            }
        
        rovs = self.rovs
        engeller = self.engeller
        n = len(rovs)
        x = torch.zeros((n, 7), dtype=torch.float)
        positions = [r.position for r in rovs]
        sources, targets = [], []

        for i in range(n):
            code = 0
            # Liderden uzaklık kontrolü
            if i != 0 and distance(positions[i], positions[0]) > limitler['LEADER']: 
                code = 5
            
            # Bağlantı kopma kontrolü
            dists = [distance(positions[i], positions[j]) for j in range(n) if i != j]
            if dists and min(dists) > limitler['DISCONNECT']: 
                code = 3
            
            # Engel tespiti
            min_engel = 999
            for engel in engeller:
                d = distance(positions[i], engel.position) - 6 
                if d < min_engel: 
                    min_engel = d
            if min_engel < limitler['OBSTACLE']: 
                code = 1
            
            # Çarpışma kontrolü
            for j in range(n):
                if i != j and distance(positions[i], positions[j]) < limitler['COLLISION']:
                    code = 2
                    break
            
            # Özellik vektörü oluştur
            x[i][0] = code / 5.0
            x[i][1] = rovs[i].battery / 100.0
            x[i][2] = 0.9  # SNR (sabit)
            x[i][3] = abs(rovs[i].y) / 100.0
            x[i][4] = rovs[i].velocity.x
            x[i][5] = rovs[i].velocity.z
            x[i][6] = rovs[i].role

            # Graf bağlantıları (iletişim menzili içindeki ROV'lar)
            for j in range(n):
                if i != j and distance(positions[i], positions[j]) < limitler['DISCONNECT']:
                    sources.append(i)
                    targets.append(j)

        edge_index = torch.tensor([sources, targets], dtype=torch.long)
        
        # MiniData sınıfı (PyG Data yapısını taklit eder)
        class MiniData:
            def __init__(self, x, edge_index): 
                self.x, self.edge_index = x, edge_index
        
        return MiniData(x, edge_index)

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
