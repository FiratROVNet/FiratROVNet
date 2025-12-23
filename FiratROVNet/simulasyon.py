from ursina import *
import numpy as np
import random
import threading
import code
import sys
import torch
from math import sin, cos, atan2, degrees, pi
import os
import matplotlib
# Windows'ta thread-safe matplotlib için backend ayarı (modül yüklenmeden önce)
import sys
if sys.platform == 'win32':
    try:
        # TkAgg backend'i Windows'ta daha güvenilir ve thread-safe
        matplotlib.use('TkAgg', force=False)
    except Exception:
        pass  # Backend zaten ayarlanmışsa devam et

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import queue
    
from .config import cfg, SensorAyarlari, GATLimitleri, HareketAyarlari

# ============================================================
# KOORDİNAT SİSTEMİ TANIMI
# ============================================================
# Bu simülasyonda kullanılan koordinat sistemi:
# - X ekseni: 2D düzlemde yatay (horizontal - birinci boyut)
# - Y ekseni: 2D düzlemde dikey (horizontal - ikinci boyut)
# - Z ekseni: Derinlik (depth) - pozitif değerler yüzeye yakın, negatif değerler derinlik
#
# Ursina engine'in kendi koordinat sistemi:
# - Ursina X: horizontal (sağ-sol)
# - Ursina Y: vertical (yukarı-aşağı)
# - Ursina Z: depth (ileri-geri)
#
# Dönüşüm: Simülasyon (x_2d, y_2d, z_depth) -> Ursina (x_2d, z_depth, y_2d)
# ============================================================

def sim_to_ursina(x_2d, y_2d, z_depth):
    """
    Simülasyon koordinat sisteminden Ursina koordinat sistemine dönüşüm.
    
    Args:
        x_2d: 2D düzlemde yatay (horizontal - birinci boyut)
        y_2d: 2D düzlemde dikey (horizontal - ikinci boyut)
        z_depth: Derinlik (depth)
    
    Returns:
        (ursina_x, ursina_y, ursina_z): Ursina koordinatları
    """
    return (x_2d, z_depth, y_2d)

def ursina_to_sim(ursina_x, ursina_y, ursina_z):
    """
    Ursina koordinat sisteminden simülasyon koordinat sistemine dönüşüm.
    
    Args:
        ursina_x: Ursina X (horizontal)
        ursina_y: Ursina Y (vertical)
        ursina_z: Ursina Z (depth)
    
    Returns:
        (x_2d, y_2d, z_depth): Simülasyon koordinatları
    """
    return (ursina_x, ursina_z, ursina_y)

# --- FİZİK SABİTLERİ ---
SURTUNME_KATSAYISI = 0.95
HIZLANMA_CARPANI = 30  # Artırıldı: 0.5 -> 5.0 (daha hızlı hareket için)
KALDIRMA_KUVVETI = 2.0
BATARYA_SOMURME_KATSAYISI = 0.001  # Batarya sömürme katsayısı (gerçekçi değer: maksimum güçte ~66 saniye dayanır)



class ROV(Entity):
    def __init__(self, rov_id, **kwargs):
        super().__init__()
        
        # FBX model kontrolü
        rov_model_path = "./Models-3D/water/my_models/submarine/submarine1.fbx"
        
        if os.path.exists(rov_model_path):
            # FBX model kullan - Model çok büyük olduğu için yaklaşık 1000 kat küçültülüyor
            self.model = rov_model_path
            self.scale = (0.01, 0.01, 0.01)  # FBX model için çok küçük scale (1000 kat küçültme)
            self.collider = 'mesh'  # FBX model için mesh collider
            self.unlit = False  # FBX model için lighting açık
            self.color = color.white  # FBX model için beyaz (GAT kodları için override edilebilir)
            self.gat_kodu = 0  # GAT kodu için değişken (başlangıç: 0 = OK)
        else:
            # Fallback: Mevcut cube model
            self.model = 'cube'
            self.color = color.orange  # Turuncu her zaman görünür
            self.scale = (1.5, 0.8, 2.5)
            self.collider = 'box'
            self.unlit = True
            self.gat_kodu = 0  # GAT kodu için değişken 
        
        # Pozisyon: (x_2d, y_2d, z_depth) formatında
        # Ursina'ya dönüştürülerek atanır: (x_2d, z_depth, y_2d)
        if 'position' in kwargs:
            pos = kwargs['position']
            # Eğer 3 elemanlı tuple ise, simülasyon koordinat sisteminden Ursina'ya dönüştür
            if isinstance(pos, (tuple, list)) and len(pos) == 3:
                x_2d, y_2d, z_depth = pos
                self.position = sim_to_ursina(x_2d, y_2d, z_depth)
            else:
                self.position = pos
        else:
            # Varsayılan pozisyon: (x_2d=-100, y_2d=0, z_depth=-10)
            self.position = sim_to_ursina(-100, 0, -10)

        self.label = Text(text=f"ROV-{rov_id}", parent=self, y=3.0, scale=20, billboard=True, color=color.white, origin=(0, 0))
        
        self.id = rov_id
        self.velocity = Vec3(0, 0, 0)
        self.battery = 1.0  # Batarya 0-1 arası (1.0 = %100 dolu)
        self.role = 0
        self.calistirilan_guc = 0.0  # ROV'un çalıştırdığı güç (0.0-1.0 arası) 
        
        # Sensör ayarları config.py'den alınır (GAT limitleri ile tutarlı)
        from .config import SensorAyarlari
        self.sensor_config = SensorAyarlari.VARSAYILAN.copy()
        self.environment_ref = None
        
        # Manuel hareket kontrolü (sürekli hareket için)
        self.manuel_hareket = {
            'yon': None,  # 'ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur'
            'guc': 0.0    # 0.0 - 1.0 arası güç
        }
        
        # Engel tespit bilgisi (kesikli çizgi için)
        self.tespit_edilen_engel = None  # En yakın engel referansı
        self.engel_mesafesi = 999.0  # En yakın engel mesafesi
        self.engel_cizgi = None  # Kesikli çizgi entity'si
        
        # Sonar iletişim bilgisi (ROV'lar arası kesikli çizgi için)
        self.iletisim_rovlari = {}  # {rov_id: {'mesafe': float, 'cizgi': Entity, 'yuzey_iletisimi': bool}}
        
        # İletişim durumu (liderle iletişim var mı?)
        self.lider_ile_iletisim = False  # Liderle iletişim durumu
        self.yuzeyde = False  # Yüzeyde mi? (z_depth >= 0, yani derinlik pozitif) 

    def update(self):
        # Manuel hareket kontrolü (sürekli hareket için)
        if self.manuel_hareket['yon'] is not None and self.manuel_hareket['guc'] > 0:
            if self.manuel_hareket['yon'] == 'dur':
                self.velocity *= 0.8  # Yavaşça dur (momentum korunumu)
                if self.velocity.length() < 0.1:
                    self.velocity = Vec3(0, 0, 0)
                    self.manuel_hareket['yon'] = None
                    self.manuel_hareket['guc'] = 0.0
            else:
                # Sürekli hareket: move metodunu çağır
                yon = self.manuel_hareket['yon']
                guc = self.manuel_hareket['guc']
                self.move(yon, guc)
        
        # Engel tespiti (her zaman çalışır, manuel kontrol olsun olmasın)
        if self.environment_ref:
            self._engel_tespiti()
        
        # Sonar iletişim tespiti (ROV'lar arası kesikli çizgi)
        if self.environment_ref:
            self._sonar_iletisim()
        
        # Yüzey durumu güncelle
        # Not: Ursina'da y ekseni vertical, ama simülasyonda z_depth derinlik
        # Ursina position=(x_2d, z_depth, y_2d) formatında
        # Yüzey kontrolü: Ursina'da y >= 0 (z_depth >= 0)
        # Ursina'dan simülasyon koordinat sistemine dönüşüm
        x_2d, y_2d, z_depth = ursina_to_sim(self.position.x, self.position.y, self.position.z)
        self.yuzeyde = z_depth >= 0  # Derinlik pozitif ise yüzeyde
        
        # Liderle iletişim kontrolü (takipçi ROV'lar için)
        if self.role == 0 and self.environment_ref:  # Takipçi ise
            self._lider_iletisim_kontrolu()
        
        # Fizik
        self.position += self.velocity * time.dt
        self.velocity *= SURTUNME_KATSAYISI
        
        # Simülasyon sınır kontrolü (ROV'ların dışarı çıkmasını önle)
        # Sınırlar: +-havuz_genisligi (yani +-200 birim)
        if self.environment_ref:
            havuz_genisligi = getattr(self.environment_ref, 'havuz_genisligi', 200)
            havuz_sinir = havuz_genisligi  # +-havuz_genisligi
            
            # X ve Z sınırları
            if abs(self.x) > havuz_sinir:
                self.x = np.sign(self.x) * havuz_sinir
                self.velocity.x = 0  # Sınırda durdur
            
            if abs(self.z) > havuz_sinir:
                self.z = np.sign(self.z) * havuz_sinir
                self.velocity.z = 0  # Sınırda durdur
        
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
            self.battery -= BATARYA_SOMURME_KATSAYISI * time.dt
        
        # Yakınlaşma önleme (10 metre mesafede uzaklaşma)
        if self.environment_ref:
            self._yaklasma_onleme()
        
        # Çarpışma kontrolü
        if self.environment_ref:
            self._carpisma_kontrolu()

    def move(self, komut, guc=1.0):
        # Batarya bitmişse hareket ettirme
        if self.battery <= 0:
            return
        thrust = guc * HIZLANMA_CARPANI * time.dt

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
        if veri_tipi == "gps": 
            return np.array([self.x, self.y, self.z])
        elif veri_tipi == "hiz": 
            return np.array([self.velocity.x, self.velocity.y, self.velocity.z])
        elif veri_tipi == "batarya": 
            return self.battery
        elif veri_tipi == "rol": 
            return self.role
        elif veri_tipi == "renk": 
            return self.color
        elif veri_tipi == "sensör" or veri_tipi == "sensor":
            return self.sensor_config.copy()
        elif veri_tipi == "engel_mesafesi": 
            return self.sensor_config.get("engel_mesafesi")
        elif veri_tipi == "iletisim_menzili": 
            return self.sensor_config.get("iletisim_menzili")
        elif veri_tipi == "min_pil_uyarisi": 
            return self.sensor_config.get("min_pil_uyarisi")
        elif veri_tipi == "kacinma_mesafesi":
            return self.sensor_config.get("kacinma_mesafesi")
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
    
    def _engel_tespiti(self):
        """
        GÜNCELLENMİŞ: Hem algılama hem de çizim noktasını düzeltir.
        Çizgiyi merkeze değil, engelin yüzeyine çizer.
        Havuz sınırları da engel olarak algılanır.
        """
        if not self.environment_ref or not hasattr(self.environment_ref, 'engeller'):
            return
        
        min_mesafe = 999.0
        en_yakin_engel = None
        en_yakin_nokta = None  # Çizgi çekilecek nokta
        
        engel_mesafesi_limit = self.sensor_config.get("engel_mesafesi", SensorAyarlari.VARSAYILAN["engel_mesafesi"])
        
        # Havuz sınırlarını kontrol et (sanal engeller)
        # Sınırlar: +-havuz_genisligi (yani +-200 birim)
        if hasattr(self.environment_ref, 'havuz_genisligi'):
            havuz_genisligi = self.environment_ref.havuz_genisligi
            havuz_sinir = havuz_genisligi  # +-havuz_genisligi
            
            # X ve Z sınırlarına mesafe kontrolü
            # X sınırları (sağ ve sol duvarlar)
            x_mesafe_sag = havuz_sinir - self.position.x  # Sağ duvara mesafe
            x_mesafe_sol = self.position.x - (-havuz_sinir)  # Sol duvara mesafe
            
            # Z sınırları (ön ve arka duvarlar)
            z_mesafe_on = havuz_sinir - self.position.z  # Ön duvara mesafe
            z_mesafe_arka = self.position.z - (-havuz_sinir)  # Arka duvara mesafe
            
            # En yakın havuz sınırını bul
            en_yakin_sinir_mesafe = min(x_mesafe_sag, x_mesafe_sol, z_mesafe_on, z_mesafe_arka)
            
            if en_yakin_sinir_mesafe < engel_mesafesi_limit and en_yakin_sinir_mesafe < min_mesafe:
                min_mesafe = en_yakin_sinir_mesafe
                # Havuz sınırı için özel bir işaret objesi oluştur (Entity yerine)
                en_yakin_engel = type('HavuzSiniri', (), {'position': None})()  # Minimal obje
                
                # Hangi sınıra yakın olduğunu belirle ve çizgi noktasını hesapla
                if en_yakin_sinir_mesafe == x_mesafe_sag:
                    # Sağ duvar
                    en_yakin_nokta = Vec3(havuz_sinir, self.position.y, self.position.z)
                elif en_yakin_sinir_mesafe == x_mesafe_sol:
                    # Sol duvar
                    en_yakin_nokta = Vec3(-havuz_sinir, self.position.y, self.position.z)
                elif en_yakin_sinir_mesafe == z_mesafe_on:
                    # Ön duvar
                    en_yakin_nokta = Vec3(self.position.x, self.position.y, havuz_sinir)
                else:
                    # Arka duvar
                    en_yakin_nokta = Vec3(self.position.x, self.position.y, -havuz_sinir)
        
        for engel in self.environment_ref.engeller:
            if not engel or not hasattr(engel, 'position') or engel.position is None:
                continue
            
            # 1. Scale alma (Güvenli yöntem)
            if hasattr(engel, 'scale'):
                if hasattr(engel.scale, 'x'):
                    s_x, s_y, s_z = engel.scale.x, engel.scale.y, engel.scale.z
                else:
                    s_x = engel.scale[0] if len(engel.scale) > 0 else 1.0
                    s_y = engel.scale[1] if len(engel.scale) > 1 else 1.0
                    s_z = engel.scale[2] if len(engel.scale) > 2 else 1.0
            else:
                s_x, s_y, s_z = 1.0, 1.0, 1.0
            
            # Ada hitbox'ları için özel işleme (daha hassas algılama)
            # Eğer engel bir cylinder modeli ise (ada sınır çizgisi), daha hassas hesaplama yap
            is_island_boundary = (hasattr(engel, 'model') and 
                                 engel.model == 'cylinder' and
                                 hasattr(engel, 'visible') and 
                                 engel.visible == True)
            
            if is_island_boundary:
                # Ada sınır çizgisi için silindirik algılama
                yatay_yaricap = max(s_x, s_z) / 2
                dikey_yaricap = s_y / 2
                
                # Vektör hesaplamaları
                fark_vektoru = self.position - engel.position
                
                # Yatay uzaklık (X-Z düzlemi)
                yatay_uzaklik = (fark_vektoru.x**2 + fark_vektoru.z**2)**0.5
                
                # Dikey uzaklık (Y ekseni)
                dy = abs(fark_vektoru.y)
                
                # Silindirik algılama: Y ekseni içindeyse ve yatay mesafe yarıçap içindeyse
                dikey_tolerans = 5.0  # Daha hassas tolerans
                
                if dy <= (dikey_yaricap + dikey_tolerans):
                    duvara_mesafe = yatay_uzaklik - yatay_yaricap
                    
                    # İçindeyse mesafe 0
                    if duvara_mesafe < 0:
                        duvara_mesafe = 0
                    
                    if duvara_mesafe < min_mesafe:
                        min_mesafe = duvara_mesafe
                        en_yakin_engel = engel
                        
                        # Yüzey noktasını hesapla
                        if yatay_uzaklik > 0.001:
                            yon_x = fark_vektoru.x / yatay_uzaklik
                            yon_z = fark_vektoru.z / yatay_uzaklik
                        else:
                            yon_x, yon_z = 1, 0
                        
                        hedef_x = engel.position.x + (yon_x * yatay_yaricap)
                        hedef_z = engel.position.z + (yon_z * yatay_yaricap)
                        en_yakin_nokta = Vec3(hedef_x, self.position.y, hedef_z)
            else:
                # Normal engel algılama (kayalar vb.)
                yatay_yaricap = max(s_x, s_z) / 2
                dikey_yaricap = s_y / 2
                
                # 2. Vektör Hesaplamaları
                # Engelin merkezinden ROV'a doğru olan vektör
                fark_vektoru = self.position - engel.position
                
                # Yatay uzaklık (Y eksenini yok sayarak)
                yatay_uzaklik = (fark_vektoru.x**2 + fark_vektoru.z**2)**0.5
                
                # Dikey uzaklık
                dy = abs(fark_vektoru.y)
                
                # 3. Kapsama Alanı Kontrolü
                dikey_tolerans = HareketAyarlari.DIKEY_TOLERANS_ENGEL  # Config'den alınan dikey tolerans 
                
                if dy <= (dikey_yaricap + dikey_tolerans):
                    duvara_mesafe = yatay_uzaklik - yatay_yaricap
                    
                    # İçindeyse mesafe 0
                    if duvara_mesafe < 0:
                        duvara_mesafe = 0
                    
                    if duvara_mesafe < min_mesafe:
                        min_mesafe = duvara_mesafe
                        en_yakin_engel = engel
                        
                        # --- KRİTİK DÜZELTME: YÜZEY NOKTASINI HESAPLA ---
                        # Merkeze değil, yüzeye çizgi çekeceğiz.
                        # Engelin merkezinden ROV'a doğru, yarıçap kadar git.
                        if yatay_uzaklik > 0.001:  # Sıfıra bölme hatasını önle
                            yon_x = fark_vektoru.x / yatay_uzaklik
                            yon_z = fark_vektoru.z / yatay_uzaklik
                        else:
                            yon_x, yon_z = 1, 0
                            
                        # Yüzeydeki nokta (X ve Z'de sınırda, Y'de ROV ile aynı hizada olsun ki çizgi düz dursun)
                        hedef_x = engel.position.x + (yon_x * yatay_yaricap)
                        hedef_z = engel.position.z + (yon_z * yatay_yaricap)
                        
                        en_yakin_nokta = Vec3(hedef_x, self.position.y, hedef_z)

        # Tespit Sonucu
        if en_yakin_engel and min_mesafe < engel_mesafesi_limit:
            self.tespit_edilen_engel = en_yakin_engel
            self.engel_mesafesi = min_mesafe
            
            # Çizgi fonksiyonuna artık hesapladığımız NOKTAYI gönderiyoruz
            # Havuz sınırı için de çizgi çiz
            if en_yakin_nokta:
                self._kesikli_cizgi_ciz(en_yakin_nokta, min_mesafe)
        else:
            self.tespit_edilen_engel = None
            self.engel_mesafesi = 999.0
            if hasattr(self, 'engel_cizgi') and self.engel_cizgi:
                destroy(self.engel_cizgi)
                self.engel_cizgi = None
    
    def _kesikli_cizgi_ciz(self, hedef_nokta, mesafe):
        """
        ROV'dan belirli bir hedef noktaya (engel yüzeyine) kesikli çizgi çizer.
        Argüman: hedef_nokta (Vec3) - Engelin yüzeyindeki nokta
        """
        # Eski çizgiyi temizle
        if self.engel_cizgi:
            if hasattr(self.engel_cizgi, 'children'):
                for child in self.engel_cizgi.children:
                    destroy(child)
            destroy(self.engel_cizgi)
        
        # Renk belirle
        if mesafe < 5.0:
            cizgi_rengi = color.red
        elif mesafe < 10.0:
            cizgi_rengi = color.orange
        else:
            cizgi_rengi = color.yellow
        
        if hedef_nokta is None:
            return

        baslangic = self.position
        bitis = hedef_nokta  # Artık doğrudan hesaplanan yüzey noktası
        
        yon = (bitis - baslangic)
        toplam_mesafe = yon.length()
        
        if toplam_mesafe == 0:
            return
            
        yon = yon.normalized()
        
        # Parça ayarları
        parca_uzunlugu = 2.0
        bosluk_uzunlugu = 1.0
        
        self.engel_cizgi = Entity()
        
        mevcut_pozisyon = 0.0
        
        while mevcut_pozisyon < toplam_mesafe:
            parca_baslangic = baslangic + yon * mevcut_pozisyon
            
            kalin_uzunluk = min(parca_uzunlugu, toplam_mesafe - mevcut_pozisyon)
            if kalin_uzunluk <= 0: 
                break
            
            parca_bitis = parca_baslangic + yon * kalin_uzunluk
            orta_nokta = (parca_baslangic + parca_bitis) / 2
            
            Entity(
                model='cube',
                position=orta_nokta,
                scale=(0.15, 0.15, kalin_uzunluk),
                color=cizgi_rengi,
                parent=self.engel_cizgi,
                unlit=True
            ).look_at(parca_bitis, up=Vec3(0,1,0))
            
            mevcut_pozisyon += parca_uzunlugu + bosluk_uzunlugu
    
    def _sonar_iletisim(self):
        """
        Yakın ROV'ları tespit eder ve aralarında kesikli çizgi çizer (sonar iletişimi).
        Manuel kontrol olsun olmasın her zaman çalışır.
        
        YENİ: Yüzey iletişimi desteği - yüzeydeki ROV'lar arası iletişim sınırsızdır.
        """
        if not self.environment_ref:
            return
        
        # İletişim menzili (su altı için)
        iletisim_menzili = self.sensor_config.get("iletisim_menzili", 35.0)
        
        # Yüzey kontrolü (y >= 0 ise yüzeyde sayılır)
        self_yuzeyde = self.y >= 0
        
        # Mevcut iletişimdeki ROV'ları kontrol et
        aktif_iletisim_rovlari = {}
        
        # Tüm ROV'ları kontrol et (sadece kendinden büyük ID'li ROV'lara çizgi çiz, çift çizgiyi önlemek için)
        for diger_rov in self.environment_ref.rovs:
            if diger_rov.id == self.id:
                continue
            
            # Sadece kendinden büyük ID'li ROV'lara çizgi çiz (her çift için tek çizgi)
            if diger_rov.id <= self.id:
                continue
            
            mesafe = distance(self.position, diger_rov.position)
            diger_rov_yuzeyde = diger_rov.y >= 0
            
            # YÜZEY İLETİŞİMİ: Her iki ROV da yüzeydeyse iletişim sınırsız
            if self_yuzeyde and diger_rov_yuzeyde:
                # Yüzeydeki ROV'lar arası iletişim sınırsız (radyo dalgaları)
                aktif_iletisim_rovlari[diger_rov.id] = {
                    'rov': diger_rov,
                    'mesafe': mesafe,
                    'yuzey_iletisimi': True  # Yüzey iletişimi işareti
                }
            # SU ALTI İLETİŞİMİ: Normal menzil kontrolü
            elif mesafe < iletisim_menzili:
                aktif_iletisim_rovlari[diger_rov.id] = {
                    'rov': diger_rov,
                    'mesafe': mesafe,
                    'yuzey_iletisimi': False
                }
        
        # Eski iletişim çizgilerini temizle (artık iletişimde olmayanlar)
        silinecek_rovlar = []
        for rov_id, iletisim_bilgisi in self.iletisim_rovlari.items():
            if rov_id not in aktif_iletisim_rovlari:
                # İletişim koptu, çizgiyi kaldır
                if iletisim_bilgisi.get('cizgi'):
                    destroy(iletisim_bilgisi['cizgi'])
                silinecek_rovlar.append(rov_id)
        
        for rov_id in silinecek_rovlar:
            del self.iletisim_rovlari[rov_id]
        
        # Yeni iletişim çizgileri çiz veya güncelle
        for rov_id, iletisim_bilgisi in aktif_iletisim_rovlari.items():
            diger_rov = iletisim_bilgisi['rov']
            mesafe = iletisim_bilgisi['mesafe']
            yuzey_iletisimi = iletisim_bilgisi.get('yuzey_iletisimi', False)
            
            # Eğer zaten iletişim varsa güncelle, yoksa yeni çiz
            if rov_id in self.iletisim_rovlari:
                # Mevcut çizgiyi güncelle
                if self.iletisim_rovlari[rov_id].get('cizgi'):
                    destroy(self.iletisim_rovlari[rov_id]['cizgi'])
            
            # Yeni çizgi çiz (yüzey iletişimi için özel stil)
            cizgi = self._rov_arasi_cizgi_ciz(diger_rov, mesafe, yuzey_iletisimi=yuzey_iletisimi)
            
            # İletişim bilgisini güncelle
            self.iletisim_rovlari[rov_id] = {
                'rov': diger_rov,
                'mesafe': mesafe,
                'cizgi': cizgi,
                'yuzey_iletisimi': yuzey_iletisimi
            }
    
    def _rov_arasi_cizgi_ciz(self, diger_rov, mesafe, yuzey_iletisimi=False):
        """
        İki ROV arasında kesikli çizgi çizer (sonar iletişimi veya yüzey iletişimi).
        
        Args:
            diger_rov: İletişim kurulan diğer ROV
            mesafe: İki ROV arasındaki mesafe
            yuzey_iletisimi: True ise yüzey iletişimi (radyo dalgaları), False ise su altı (sonar)
        
        Returns:
            Entity: Çizgi entity'si
        """
        # YÜZEY İLETİŞİMİ: Yeşil renk (radyo dalgaları)
        if yuzey_iletisimi:
            cizgi_rengi = color.green
        else:
            # SU ALTI İLETİŞİMİ: Mesafeye göre renk (yakın = mavi, uzak = cyan)
            iletisim_menzili = self.sensor_config.get("iletisim_menzili", 35.0)
            mesafe_orani = mesafe / iletisim_menzili
            
            if mesafe_orani < 0.3:  # Çok yakın
                cizgi_rengi = color.blue
            elif mesafe_orani < 0.6:  # Orta mesafe
                cizgi_rengi = color.cyan
            else:  # Uzak ama hala menzil içinde
                cizgi_rengi = color.rgb(100, 200, 255)  # Açık mavi
        
        # Kesikli çizgi için noktalar oluştur
        baslangic = self.position
        bitis = diger_rov.position
        yon = (bitis - baslangic)
        if yon.length() == 0:
            return None
        yon = yon.normalized()
        toplam_mesafe = distance(baslangic, bitis)
        
        # Kesikli çizgi parçaları (her 1.5 birimde bir parça, daha ince)
        parca_uzunlugu = 1.5
        bosluk_uzunlugu = 0.8
        
        # Ana çizgi entity'si (parçaları tutmak için)
        cizgi_entity = Entity()
        
        # Çizgi parçalarını oluştur
        mevcut_pozisyon = 0.0
        
        while mevcut_pozisyon < toplam_mesafe:
            # Parça başlangıcı
            parca_baslangic = baslangic + yon * mevcut_pozisyon
            
            # Parça bitişi
            parca_bitis_uzunlugu = min(parca_uzunlugu, toplam_mesafe - mevcut_pozisyon)
            if parca_bitis_uzunlugu <= 0:
                break
            
            parca_bitis = parca_baslangic + yon * parca_bitis_uzunlugu
            
            # Parça entity'si oluştur (daha ince, iletişim çizgisi için)
            parca = Entity(
                model='cube',
                position=(parca_baslangic + parca_bitis) / 2,
                scale=(0.1, 0.1, parca_bitis_uzunlugu),
                color=cizgi_rengi,
                parent=cizgi_entity,
                unlit=True
            )
            
            # Yönlendirme
            parca.look_at(parca_bitis, up=Vec3(0, 1, 0))
            
            # Sonraki parça için pozisyon güncelle
            mevcut_pozisyon += parca_uzunlugu + bosluk_uzunlugu
        
        return cizgi_entity
    
    def _lider_iletisim_kontrolu(self):
        """
        Takipçi ROV'un liderle iletişim durumunu kontrol eder.
        İletişim koptuysa, ROV otomatik olarak lider olur (GNC sistemi tarafından işlenecek).
        ÖNEMLİ: ROV'lar birbirine çok yakın olduğunda (10m içinde) iletişim kopmasını görmezden gel.
        """
        if not self.environment_ref or self.role == 1:  # Lider ise kontrol etme
            return
        
        # Lider ROV'u bul
        lider_rov = None
        for rov in self.environment_ref.rovs:
            if rov.role == 1:
                lider_rov = rov
                break
        
        if lider_rov is None:
            # Lider yok, iletişim yok
            self.lider_ile_iletisim = False
            return
        
        mesafe = distance(self.position, lider_rov.position)
        self_yuzeyde = self.y >= 0
        lider_yuzeyde = lider_rov.y >= 0
        
        # YÜZEY İLETİŞİMİ: Her iki ROV da yüzeydeyse iletişim var
        if self_yuzeyde and lider_yuzeyde:
            self.lider_ile_iletisim = True
        # SU ALTI İLETİŞİMİ: Normal menzil kontrolü
        else:
            iletisim_menzili = self.sensor_config.get("iletisim_menzili", 35.0)
            
            # ÖNEMLİ: ROV'lar birbirine çok yakın olduğunda iletişim kopmasını görmezden gel
            # Bu, çarpışma önleme mekanizmasının neden olduğu geçici iletişim kopmalarını önler (Config'den)
            yakin_mesafe_esigi = HareketAyarlari.YAKIN_MESAFE_ESIGI
            if mesafe < yakin_mesafe_esigi:
                # Çok yakınsa, iletişim var say (geçici kopmaları önle)
                self.lider_ile_iletisim = True
            else:
                self.lider_ile_iletisim = mesafe < iletisim_menzili
    
    def _yaklasma_onleme(self):
        """
        Sensör mesafesine göre ROV'lar ve engellerden uzaklaşma.
        Çarpışmayı önlemek için proaktif kaçınma davranışı.
        """
        if not self.environment_ref:
            return
        
        # Kaçınma mesafesini sensör ayarlarından al
        kacinma_mesafesi = self.sensor_config.get("kacinma_mesafesi", None)
        if kacinma_mesafesi is None:
            # Eğer kacinma_mesafesi yoksa, engel_mesafesi'nin bir kısmını kullan (Config'den katsayı)
            engel_mesafesi = self.sensor_config.get("engel_mesafesi", SensorAyarlari.VARSAYILAN["engel_mesafesi"])
            kacinma_mesafesi = engel_mesafesi * HareketAyarlari.KACINMA_MESAFESI_FALLBACK_KATSAYISI
        
        uzaklasma_vektoru = Vec3(0, 0, 0)
        
        # Diğer ROV'lardan uzaklaşma
        # ÖNEMLİ: Lider takipçilerden uzaklaşmaz - hedefe gitmek için sürüden ayrılabilir
        if self.role != 1:  # Sadece takipçiler diğer ROV'lardan uzaklaşır
            for diger_rov in self.environment_ref.rovs:
                if diger_rov.id == self.id:
                    continue
                
                mesafe = distance(self.position, diger_rov.position)
                
                # ÖNEMLİ: ROV'lar birbirine çok yakın olduğunda (2m içinde) kaçınma mekanizmasını devre dışı bırak
                # Bu, ROV'ların birbirini sürekli itmesini önler
                minimum_mesafe = 2.0  # 2 metre - çok yakınsa kaçınma yok
                if mesafe < minimum_mesafe:
                    continue  # Çok yakınsa kaçınma yapma
                
                # Kaçınma mesafesi veya daha küçük mesafede uzaklaş
                if mesafe <= kacinma_mesafesi and mesafe > 0:
                    # Uzaklaşma yönü (bu ROV'dan diğer ROV'a)
                    uzaklasma_yonu = (self.position - diger_rov.position).normalized()
                    # Mesafe ne kadar küçükse, o kadar güçlü uzaklaş
                    # Ancak gücü daha da yumuşat (çok agresif olmasın)
                    uzaklasma_gucu = (kacinma_mesafesi - mesafe) / kacinma_mesafesi
                    uzaklasma_gucu *= 0.3  # Gücü %30'a indir (daha yumuşak)
                    uzaklasma_vektoru += uzaklasma_yonu * uzaklasma_gucu
        
        # Engellerden uzaklaşma
        for engel in self.environment_ref.engeller:
            mesafe = distance(self.position, engel.position)
            
            # Ada sınır çizgisi kontrolü (daha hassas algılama)
            is_island_boundary = (hasattr(engel, 'model') and 
                                 engel.model == 'sphere' and
                                 hasattr(engel, 'visible') and 
                                 engel.visible == True and
                                 hasattr(engel, 'scale') and
                                 # Y ekseni uzun, X-Z eksenleri eşit (silindir benzeri) kontrolü
                                 abs(engel.scale.y - max(engel.scale.x, engel.scale.z)) > 5.0)
            
            if is_island_boundary:
                # Ada sınır çizgisi için silindirik mesafe hesaplama
                engel_yari_cap = max(engel.scale_x, engel.scale_z) / 2
                engel_yukseklik = engel.scale_y / 2
                
                # Yatay mesafe (X-Z düzlemi)
                fark_vektoru = self.position - engel.position
                yatay_mesafe = (fark_vektoru.x**2 + fark_vektoru.z**2)**0.5
                dikey_fark = abs(fark_vektoru.y)
                
                # Silindirik algılama: Y ekseni içindeyse ve yatay mesafe yarıçap içindeyse
                if dikey_fark <= (engel_yukseklik + 5.0):  # Dikey tolerans
                    gercek_mesafe = yatay_mesafe - engel_yari_cap
                else:
                    continue  # Dikey olarak çok uzaksa atla
            else:
                # Normal engel algılama (kayalar vb.)
                engel_yari_cap = max(engel.scale_x, engel.scale_y, engel.scale_z) / 2
                gercek_mesafe = mesafe - engel_yari_cap
            
            # ÖNEMLİ: Engel çok yakınsa (engel yarıçapı + 1m içinde) kaçınma mekanizmasını devre dışı bırak
            # Bu, ROV'ların engellere çok yaklaşmasını önler ama sürekli itmeyi engeller
            minimum_engel_mesafe = engel_yari_cap + 1.0  # Engel yarıçapı + 1 metre
            if gercek_mesafe < minimum_engel_mesafe:
                continue  # Çok yakınsa kaçınma yapma (sadece çarpışma kontrolü yeterli)
            
            # Kaçınma mesafesi veya daha küçük mesafede uzaklaş
            if gercek_mesafe <= kacinma_mesafesi and gercek_mesafe > 0:
                # Uzaklaşma yönü (bu ROV'dan engele)
                if is_island_boundary:
                    # Ada sınırı için yatay uzaklaşma (sadece X-Z düzleminde)
                    if yatay_mesafe > 0.001:
                        uzaklasma_yonu = Vec3(fark_vektoru.x / yatay_mesafe, 0, fark_vektoru.z / yatay_mesafe)
                    else:
                        uzaklasma_yonu = Vec3(1, 0, 0)  # Varsayılan yön
                else:
                    uzaklasma_yonu = (self.position - engel.position).normalized()
                
                # Mesafe ne kadar küçükse, o kadar güçlü uzaklaş
                # Config'den alınan uzaklaşma gücü katsayısı
                uzaklasma_gucu = (kacinma_mesafesi - gercek_mesafe) / kacinma_mesafesi
                uzaklasma_gucu *= HareketAyarlari.UZAKLASMA_GUC_KATSAYISI
                uzaklasma_vektoru += uzaklasma_yonu * uzaklasma_gucu
        
        # Havuz sınırlarından uzaklaşma (sanal engeller)
        # Sınırlar: +-havuz_genisligi (yani +-200 birim)
        if hasattr(self.environment_ref, 'havuz_genisligi'):
            havuz_genisligi = self.environment_ref.havuz_genisligi
            havuz_sinir = havuz_genisligi  # +-havuz_genisligi
            
            # X sınırları (sağ ve sol duvarlar)
            x_mesafe_sag = havuz_sinir - self.position.x
            x_mesafe_sol = self.position.x - (-havuz_sinir)
            
            # Z sınırları (ön ve arka duvarlar)
            z_mesafe_on = havuz_sinir - self.position.z
            z_mesafe_arka = self.position.z - (-havuz_sinir)
            
            # En yakın sınıra mesafe
            en_yakin_sinir_mesafe = min(x_mesafe_sag, x_mesafe_sol, z_mesafe_on, z_mesafe_arka)
            
            # Kaçınma mesafesi içindeyse uzaklaş
            if en_yakin_sinir_mesafe <= kacinma_mesafesi and en_yakin_sinir_mesafe > 0:
                # Hangi sınıra yakın olduğunu belirle ve uzaklaşma yönünü hesapla
                if en_yakin_sinir_mesafe == x_mesafe_sag:
                    # Sağ duvar - sola doğru uzaklaş
                    uzaklasma_yonu = Vec3(-1, 0, 0)
                elif en_yakin_sinir_mesafe == x_mesafe_sol:
                    # Sol duvar - sağa doğru uzaklaş
                    uzaklasma_yonu = Vec3(1, 0, 0)
                elif en_yakin_sinir_mesafe == z_mesafe_on:
                    # Ön duvar - geriye doğru uzaklaş
                    uzaklasma_yonu = Vec3(0, 0, -1)
                else:
                    # Arka duvar - ileriye doğru uzaklaş
                    uzaklasma_yonu = Vec3(0, 0, 1)
                
                # Uzaklaşma gücü
                uzaklasma_gucu = (kacinma_mesafesi - en_yakin_sinir_mesafe) / kacinma_mesafesi
                uzaklasma_gucu *= 0.3  # Gücü %30'a indir (daha yumuşak)
                uzaklasma_vektoru += uzaklasma_yonu * uzaklasma_gucu
        
        # Uzaklaşma vektörünü uygula
        if uzaklasma_vektoru.length() > 0:
            # Normalize et ve güç uygula
            uzaklasma_vektoru = uzaklasma_vektoru.normalized()
            uzaklasma_gucu = min(uzaklasma_vektoru.length(), 1.0)  # Maksimum %100 güç
            
            # Daha yumuşak uzaklaşma için gücü azalt (Config'den yumuşaklık çarpanı)
            uzaklasma_gucu *= HareketAyarlari.YUMUSAKLIK_CARPANI
            
            # Hız vektörüne ekle (momentum korunumu için)
            uzaklasma_hizi = uzaklasma_vektoru * uzaklasma_gucu * HIZLANMA_CARPANI * time.dt
            self.velocity += uzaklasma_hizi
            
            # Hız limiti (aşırı hızlanmayı önle)
            max_hiz = 50.0
            if self.velocity.length() > max_hiz:
                self.velocity = self.velocity.normalized() * max_hiz
    
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
                        # Ursina'da Vec3 * float çalışır, float * Vec3 çalışmaz
                        carpan1 = (2 * diger_rov_kutlesi / (rov_kutlesi + diger_rov_kutlesi)) * nokta_carpim
                        self.velocity = self.velocity - carpisma_yonu * carpan1
                        
                        carpan2 = (2 * rov_kutlesi / (rov_kutlesi + diger_rov_kutlesi)) * (-nokta_carpim)
                        diger_rov.velocity = diger_rov.velocity - (-carpisma_yonu) * carpan2
                        
                        # Çarpışma sonrası pozisyonları ayır
                        ayirma_mesafesi = (min_mesafe - mesafe) / 2
                        self.position += carpisma_yonu * ayirma_mesafesi
                        diger_rov.position -= carpisma_yonu * ayirma_mesafesi
        
        # Kayalarla ve ada sınırlarıyla çarpışma
        for engel in self.environment_ref.engeller:
            # Ada sınır çizgisi kontrolü (daha hassas algılama)
            is_island_boundary = (hasattr(engel, 'model') and 
                                 engel.model == 'sphere' and
                                 hasattr(engel, 'visible') and 
                                 engel.visible == True and
                                 hasattr(engel, 'scale') and
                                 # Y ekseni uzun, X-Z eksenleri eşit (silindir benzeri) kontrolü
                                 abs(engel.scale.y - max(engel.scale.x, engel.scale.z)) > 5.0)
            
            if is_island_boundary:
                # Ada sınır çizgisi için silindirik çarpışma kontrolü
                engel_yari_cap = max(engel.scale_x, engel.scale_z) / 2
                engel_yukseklik = engel.scale_y / 2
                
                # Yatay mesafe (X-Z düzlemi)
                fark_vektoru = self.position - engel.position
                yatay_mesafe = (fark_vektoru.x**2 + fark_vektoru.z**2)**0.5
                dikey_fark = abs(fark_vektoru.y)
                
                # Silindirik çarpışma: Y ekseni içindeyse ve yatay mesafe yarıçap içindeyse
                if dikey_fark <= (engel_yukseklik + HareketAyarlari.DIKEY_TOLERANS_ENGEL):  # Config'den dikey tolerans
                    min_mesafe = engel_yari_cap + 1.0
                    
                    if yatay_mesafe < min_mesafe:
                        # Ada sınırı ile çarpışma
                        if yatay_mesafe > 0.001:
                            carpisma_yonu = Vec3(fark_vektoru.x / yatay_mesafe, 0, fark_vektoru.z / yatay_mesafe)
                        else:
                            carpisma_yonu = Vec3(1, 0, 0)  # Varsayılan yön
                        
                        # Hızı yansıt (sadece yatay düzlemde)
                        hiz_buyuklugu = (self.velocity.x**2 + self.velocity.z**2)**0.5
                        if hiz_buyuklugu > 0.1:
                            # Yatay hız vektörü
                            yatay_hiz = Vec3(self.velocity.x, 0, self.velocity.z)
                            nokta_carpim = yatay_hiz.dot(carpisma_yonu)
                            if nokta_carpim < 0:  # Ada sınırına doğru gidiyor
                                # Yatay hızı yansıt
                                self.velocity.x = self.velocity.x - carpisma_yonu.x * (2 * nokta_carpim)
                                self.velocity.z = self.velocity.z - carpisma_yonu.z * (2 * nokta_carpim)
                                
                                # Pozisyonu ayır (yatay düzlemde)
                                ayirma_mesafesi = (min_mesafe - yatay_mesafe)
                                self.position += carpisma_yonu * ayirma_mesafesi
            else:
                # Normal engel çarpışma kontrolü (kayalar vb.)
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
                            # Ursina'da Vec3 * float çalışır, float * Vec3 çalışmaz
                            self.velocity = self.velocity - carpisma_yonu * (2 * nokta_carpim)
                            
                            # Pozisyonu ayır
                            ayirma_mesafesi = (min_mesafe - mesafe)
                            self.position += carpisma_yonu * ayirma_mesafesi

# ============================================================
# HARİTA SİSTEMİ (Matplotlib - Ayrı Pencere)
# ============================================================
class Harita:
    """
    Google Maps benzeri harita sistemi (Matplotlib ile ayrı pencerede).
    ROV'ları ok şeklinde, adaları ve engelleri gösterir.
    """
    def __init__(self, ortam_ref, pencere_boyutu=(800, 800)):
        """
        Args:
            ortam_ref: Ortam sınıfı referansı
            pencere_boyutu: Harita penceresi boyutu (genişlik, yükseklik)
        """
        self.hedef_pozisyon = None  # Hedef pozisyonu (x, y) formatında
        self.ortam_ref = ortam_ref
        self.pencere_boyutu = pencere_boyutu
        self.manuel_engeller = []  # Elle eklenen engeller [(x_2d, y_2d), ...]
        
        # Durum Değişkenleri
        self.gorunur = False
        self.fig = None
        self.ax = None
        
        # Thread Güvenliği İçin İstek Bayrakları
        self._ac_istegi = False
        self._kapat_istegi = False
        
        # Havuz genişliği
        self.havuz_genisligi = getattr(ortam_ref, 'havuz_genisligi', 200)
        
        print("✅ Harita sistemi hazır. Kullanım: harita.goster(True)")
    
    def _setup_figure(self):
        """Bu fonksiyon mutlaka ANA THREAD içinde çağrılmalıdır."""
        try:
            import sys
            import time
            
            plt.ion()
            # Yeni pencere oluştur
            self.fig, self.ax = plt.subplots(figsize=(self.pencere_boyutu[0]/100, self.pencere_boyutu[1]/100))
            self.fig.canvas.manager.set_window_title('ROV Haritasi')
            
            # Pencere kapatıldığında algıla
            self.fig.canvas.mpl_connect('close_event', self._on_close)
            
            # İlk çizimi yap
            self._ciz()
            
            # Windows'ta thread-safe show (GIL sorunlarını önlemek için)
            if sys.platform == 'win32':
                try:
                    # Windows'ta plt.show() ve plt.pause() GIL sorunlarına yol açabilir
                    # Bu yüzden draw_idle ve time.sleep kullan
                    self.fig.canvas.draw_idle()
                    plt.show(block=False)
                    # plt.pause yerine time.sleep kullan (daha güvenli)
                    time.sleep(0.05)
                except Exception as e:
                    print(f"⚠️ Harita penceresi açılırken uyarı: {e}")
            else:
                plt.show(block=False)
                plt.pause(0.1)
        except Exception as e:
            print(f"❌ Harita penceresi başlatılamadı: {e}")
            import traceback
            traceback.print_exc()

    def _on_close(self, event):
        """Pencere çarpıdan kapatıldığında."""
        self.gorunur = False
        self.fig = None
        self.ax = None
    
    def _ciz_gps_pin(self, x, y, renk, yon=None):
        """Uyarı vermeyen GPS pin çizimi."""
        if not self.ax:
            return
        pin_boyut = 8.0
        angle = atan2(yon[1], yon[0]) if yon and (yon[0] != 0 or yon[1] != 0) else pi/2
        
        ucu_x, ucu_y = x + cos(angle)*pin_boyut, y + sin(angle)*pin_boyut
        t1x, t1y = x + cos(angle+pi/2)*pin_boyut*0.6, y + sin(angle+pi/2)*pin_boyut*0.6
        t2x, t2y = x + cos(angle-pi/2)*pin_boyut*0.6, y + sin(angle-pi/2)*pin_boyut*0.6

        # 'color' yerine 'facecolor' kullanarak UserWarning önlendi
        from matplotlib import patches
        # Renk tuple'ını matplotlib renk formatına çevir
        if isinstance(renk, tuple) and len(renk) == 3:
            renk_matplotlib = renk
        else:
            renk_matplotlib = (1.0, 0.5, 0.0)  # Varsayılan turuncu
        
        self.ax.add_patch(patches.Polygon([(ucu_x, ucu_y), (t1x, t1y), (t2x, t2y)],
                          facecolor=renk_matplotlib, edgecolor='black', linewidth=1, zorder=10))
        self.ax.plot(x, y, 'o', color='white', markersize=3, zorder=11, 
                    markeredgecolor='black', markeredgewidth=1)

    def _ciz_ada_sekli(self, x, y, boyut):
        """Ada şeklinde çizim (ters koni/oval şekil)."""
        from matplotlib.patches import Ellipse
        ust_yaricap = boyut * 1.2
        alt_yaricap = boyut * 0.6
        ada_sekli = Ellipse((x, y), width=ust_yaricap*2, height=alt_yaricap*2, 
                           angle=0, facecolor='#8B5A3C', edgecolor='black', 
                           linewidth=2, zorder=4, alpha=0.8)
        self.ax.add_patch(ada_sekli)
        
        # Ada üzerinde küçük detaylar (ağaç/tepe gibi) - sabit pozisyonlar
        detay_positions = [
            (0.3, 0.4), (-0.4, 0.2), (0.2, -0.3), (-0.3, -0.2), (0.0, 0.5)
        ]
        for dx, dy in detay_positions:
            detay_x = x + dx * ust_yaricap * 0.6
            detay_y = y + dy * alt_yaricap * 0.6
            self.ax.plot(detay_x, detay_y, 'o', color='#654321', markersize=3, zorder=5)
    
    def _ciz(self):
        """Eksenleri temizle ve her şeyi yeniden çiz."""
        if self.ax is None:
            return
        
        self.ax.clear()
        self.ax.set_xlim(-self.havuz_genisligi, self.havuz_genisligi)
        self.ax.set_ylim(-self.havuz_genisligi, self.havuz_genisligi)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.2)
        self.ax.set_xlabel('X (2D Duzlem)', fontsize=10)
        self.ax.set_ylabel('Y (2D Duzlem)', fontsize=10)
        self.ax.set_title("FIRAT ROVNET - Gercek Zamanli Takip", fontsize=12, fontweight='bold')

        # ROV'ları Çiz
        if hasattr(self.ortam_ref, 'rovs') and self.ortam_ref.rovs:
            for rov in self.ortam_ref.rovs:
                # Ursina koordinat sisteminden simülasyon koordinat sistemine dönüşüm
                x_2d, y_2d, z_depth = ursina_to_sim(rov.position.x, rov.position.y, rov.position.z)
                
                # ROV rengini matplotlib renk formatına çevir
                if hasattr(rov, 'color'):
                    if hasattr(rov.color, 'r'):
                        renk = (rov.color.r, rov.color.g, rov.color.b)
                    else:
                        renk = (1.0, 0.5, 0.0)  # Varsayılan turuncu
                else:
                    renk = (1.0, 0.5, 0.0)
                
                # Velocity bilgisi (yön için)
                yon = None
                if hasattr(rov, 'velocity') and rov.velocity.length() > 0.1:
                    yon = (rov.velocity.x, rov.velocity.z)
                
                self._ciz_gps_pin(x_2d, y_2d, renk, yon)

        # Hedef pozisyonunu çiz (büyük X işareti)
        if self.hedef_pozisyon:
            x_hedef, y_hedef = self.hedef_pozisyon
            # Büyük X işareti çiz
            x_boyutu = 8.0
            # İlk çapraz çizgi (sol üst -> sağ alt)
            self.ax.plot([x_hedef - x_boyutu, x_hedef + x_boyutu], 
                        [y_hedef - x_boyutu, y_hedef + x_boyutu], 
                        'r-', linewidth=3, zorder=10, label='Hedef' if not hasattr(self, '_hedef_label_cizildi') else '')
            # İkinci çapraz çizgi (sağ üst -> sol alt)
            self.ax.plot([x_hedef + x_boyutu, x_hedef - x_boyutu], 
                        [y_hedef - x_boyutu, y_hedef + x_boyutu], 
                        'r-', linewidth=3, zorder=10)
            # Merkez nokta
            self.ax.plot(x_hedef, y_hedef, 'ro', markersize=8, zorder=11)
            # Çember (hedef alanı)
            from matplotlib.patches import Circle
            circle = Circle((x_hedef, y_hedef), radius=5, fill=False, 
                          edgecolor='red', linestyle='--', linewidth=2, zorder=9)
            self.ax.add_patch(circle)
            self._hedef_label_cizildi = True
        
        # Adaları Çiz
        if hasattr(self.ortam_ref, 'island_positions') and self.ortam_ref.island_positions:
            from matplotlib import patches
            for is_pos in self.ortam_ref.island_positions:
                if len(is_pos) == 3:
                    rad = is_pos[2]
                else:
                    rad = self.havuz_genisligi * 0.08  # Varsayılan boyut
                ada = patches.Ellipse((is_pos[0], is_pos[1]), width=rad*2.4, height=rad*1.2, 
                                     facecolor='#8B5A3C', edgecolor='black', alpha=0.7, zorder=4)
                self.ax.add_patch(ada)

        # Manuel Engeller
        if self.manuel_engeller:
            ex = [p[0] for p in self.manuel_engeller]
            ey = [p[1] for p in self.manuel_engeller]
            self.ax.scatter(ex, ey, c='red', marker='X', s=80, label="Engel", zorder=10,
                          edgecolors='darkred', linewidths=2)
        
        # Legend (sadece engeller için)
        if self.manuel_engeller:
            self.ax.legend(loc='upper right', fontsize=9)

        self.fig.canvas.draw_idle()
    
    def goster(self, durum):
        """
        Konsoldan (Shell Thread) çağrılır. 
        Sadece istek bırakır, işlemi update() (Main Thread) yapar.
        """
        # String gelme ihtimaline karşı kontrol ("True" -> True)
        if isinstance(durum, str):
            durum = durum.lower() == "true"
            
        if durum:
            self._ac_istegi = True
            self._kapat_istegi = False
        else:
            self._kapat_istegi = True
            self._ac_istegi = False

    def update(self):
        """Ursina tarafından her karede (Main Thread) çağrılır."""
        
        # 1. Kapatma İsteğini İşle
        if self._kapat_istegi:
            self._kapat_istegi = False
            if self.fig is not None:
                plt.close(self.fig)
                self.fig = None
                self.ax = None
                self.gorunur = False
                print("✅ Harita kapatıldı.")

        # 2. Açma İsteğini İşle
        if self._ac_istegi:
            self._ac_istegi = False
            if self.fig is None:
                self._setup_figure()
                self.gorunur = True
                print("✅ Harita açıldı.")

        # 3. Rutin Çizim Güncellemesi
        if self.gorunur and self.fig is not None:
            if not hasattr(self, '_up_cnt'):
                self._up_cnt = 0
            self._up_cnt += 1
            
            if self._up_cnt >= 30:  # 30 karede bir (Performans)
                self._up_cnt = 0
                try:
                    # Havuz genişliğini güncelle (sim_olustur'da değişebilir)
                    self.havuz_genisligi = getattr(self.ortam_ref, 'havuz_genisligi', 200)
                    self._ciz()
                    
                    # Windows'ta thread-safe flush_events
                    import sys
                    if sys.platform == 'win32':
                        # Windows'ta canvas.draw() kullan (flush_events yerine)
                        try:
                            self.fig.canvas.draw()
                            self.fig.canvas.flush_events()
                        except Exception:
                            # Pencere kapatılmış olabilir
                            pass
                    else:
                        self.fig.canvas.flush_events()
                except Exception:
                    # Pencere harici bir sebeple kapandıysa
                    self.fig = None
                    self.ax = None
                    self.gorunur = False
    
    def ekle(self, x_2d, y_2d, tip='engel'):
        """
        Haritaya elle engel/nesne ekler.
        
        Args:
            x_2d, y_2d: 2D düzlem koordinatları
            tip: Nesne tipi ('engel', 'hedef', vb.)
        
        Returns:
            bool: Başarılı ise True
        """
        if tip == 'engel':
            # Engel listesine ekle
            self.manuel_engeller.append((x_2d, y_2d))
            
            # Ortam'a engel entity'si ekle
            if hasattr(self.ortam_ref, 'engeller'):
                # Engel entity'si oluştur (görünmez hitbox)
                engel = Entity(
                    model='icosphere',
                    position=sim_to_ursina(x_2d, y_2d, self.ortam_ref.SEA_FLOOR_Y),
                    scale=(20, 20, 20),
                    visible=False,
                    collider='sphere',
                    color=color.red,
                    unlit=True
                )
                self.ortam_ref.engeller.append(engel)
            
            # Haritayı güncelle (sadece görünürse ve pencere varsa)
            if self.gorunur and self.fig is not None:
                self._ciz()
            print(f"✅ Engel eklendi: ({x_2d:.1f}, {y_2d:.1f})")
            return True
        
        return False
    
    def temizle(self):
        """Haritayı temizler (elle eklenen engelleri siler)"""
        self.manuel_engeller = []
        if self.gorunur and self.fig is not None:
            self._ciz()
        print("Harita temizlendi (elle eklenen engeller silindi)")
    
    def kapat(self):
        """Harita penceresini tamamen kapatır"""
        self.goster(False)


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
        window.size = (1280, 720)  # Daha geniş pencere boyutu (16:9 aspect ratio)
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
# --- IŞIKLANDIRMA (Adanın ve ROV'ların net görünmesi için şart) ---
        # Güneş ışığı (Gölgeler için)
        self.sun = DirectionalLight()
        self.sun.look_at(Vec3(1, -1, -1))
        self.sun.color = color.white
        
        # Ortam ışığı (Karanlıkta kalan yerleri aydınlatmak için)
        self.ambient = AmbientLight()
        self.ambient.color = color.rgba(100, 100, 100, 1) # Hafif gri ortam ışığı
        
        # Gökyüzü (Arka planın mavi olması için)
        self.sky = Sky()
        # --- Sahne Nesneleri ---
        # Su hacmi parametreleri
        su_hacmi_yuksekligi = 100.0
        su_hacmi_merkez_y = -50.0
        # Su yüzeyi
        self.WATER_SURFACE_Y_BASE = su_hacmi_merkez_y + (su_hacmi_yuksekligi / 2)  # Su yüzeyi base pozisyonu
        
        # 1. GÖRÜNTÜ AYARI: texture_scale değerini (10, 10) gibi makul bir değere düşürdük.
        self.ocean_surface = Entity(
            model="plane",
            scale=(500, 1, 500),
            position=(0, self.WATER_SURFACE_Y_BASE, 0),
            texture="./Models-3D/water/my_models/water4.jpg",
            texture_scale=(1, 1),  # 50 yerine 10 yaptık, artık küçük kareler görünmeyecek
            normals=Texture('./Models-3D/water/my_models/map/water4_normal.png'),
            double_sided=True,
            color=color.rgb(0.3, 0.5, 0.9),
            alpha=0.25,  # Biraz daha görünür yaptık
            render_queue=0  # Önce su yüzeyini render et (z-order)
        )


        
        self.SEA_FLOOR_Y = su_hacmi_merkez_y - (su_hacmi_yuksekligi / 2)  # Deniz tabanı pozisyonu
        
        # Animasyon değişkenlerini self.ocean_surface içine gömüyoruz ki kaybolmasınlar
        self.ocean_surface.sim_time = 0.0
        self.ocean_surface.u_offset = 0.0
        self.ocean_surface.v_offset = 0.0
        self.ocean_surface.WAVE_SPEED_U = 0.02
        self.ocean_surface.WAVE_SPEED_V = 0.005
        self.ocean_surface.WAVE_AMP = 1.5
        self.ocean_surface.WAVE_FREQ = 0.8
        self.ocean_surface.Y_BASE = self.WATER_SURFACE_Y_BASE
        
        # 2. HAREKET AYARI: Update fonksiyonunu doğrudan nesneye tanımlıyoruz.
        # Bu fonksiyon Ursina tarafından otomatik olarak her karede çağrılır.
        def update_ocean():
            # Zamanı ilerlet
            dt = time.dt if hasattr(time, 'dt') and time.dt > 0 else 0.016
            self.ocean_surface.sim_time += dt
            
            # Dalga Yüksekliği (Fiziksel)
            self.ocean_surface.y = self.ocean_surface.Y_BASE + \
                                   sin(self.ocean_surface.sim_time * self.ocean_surface.WAVE_FREQ) * \
                                   self.ocean_surface.WAVE_AMP
            
            # Doku Kaydırma (Görsel Akıntı)
            self.ocean_surface.u_offset += dt * self.ocean_surface.WAVE_SPEED_U
            self.ocean_surface.v_offset += dt * self.ocean_surface.WAVE_SPEED_V
            
            self.ocean_surface.texture_offset = (
                self.ocean_surface.u_offset % 1.0, 
                self.ocean_surface.v_offset % 1.0
            )
        
        # Fonksiyonu entity'nin update slotuna bağlıyoruz
        self.ocean_surface.update = update_ocean
        ocean_taban_model_path = "./Models-3D/water/my_models/ocean_taban/sand_envi_034.fbx"
        ocean_taban_texture_path = "./Models-3D/water/my_models/ocean_taban/sand_envi_034-0.jpg"
        if os.path.exists(ocean_taban_model_path):
            self.ocean_taban = Entity(
                model=ocean_taban_model_path,
                scale=(2.2 * (500 / 500), 1, 1.8 * (500 / 500)),
                position=(0, self.SEA_FLOOR_Y-8, 0),
                texture=ocean_taban_texture_path,
                double_sided=True,
                collider='mesh',
                unlit=False,
                alpha=1.0,
                transparent=True,
                render_queue=0
            )
        else:
            self.ocean_taban = None


        
        # Ada modeli (su yüzeyinin üstünde, deniz tabanına değen)
        # 1-5 arasında random ada oluştur
        island_model_path = "./Models-3D/lowpoly-island/source/island1_design2_c4d.obj"
        island_texture_path = "./Models-3D/lowpoly-island/textures/textureSurface_Color_2.jpg"
        
        # Referans ölçekler (orijinal ada)
        ref_visual_scale = (0.3, 0.8, 0.3)
        # Hitbox boyutları artırıldı (ROV'ların daha erken algılaması için)
        ref_hitbox_scales = [
            (75, 20, 75),   # Katman 1 (en geniş) - 55'ten 75'e çıkarıldı
            (55, 25, 55),   # Katman 2 - 40'tan 55'e çıkarıldı
            (45, 25, 45),   # Katman 3 - 30'dan 45'e çıkarıldı
            (20, 25, 20)    # Katman 4 - 10'dan 20'ye çıkarıldı
        ]
        ref_hitbox_positions = [
            (0, -5, 0),     # Katman 1
            (0, -25, 0),    # Katman 2
            (0, -45, 0),    # Katman 3
            (0, -65, 0)     # Katman 4
        ]
        
        # Ada pozisyonlarını sakla (ROV yerleştirme için)
        self.island_positions = []
        self.island_hitboxes = []
        
        # Havuz genişliği (varsayılan 200, sim_olustur'da güncellenebilir)
        self.havuz_genisligi = 200
        
        if os.path.exists(island_model_path):
            # ============================================================
            # ADA OLUŞTURMA AYARLARI
            # ============================================================
            n_islands = random.randint(1, 7)  # 1-7 arası random ada sayısı
            
            # Engel listesini hazırla (eğer yoksa oluştur)
            if not hasattr(self, 'engeller'):
                self.engeller = []
            
            # Ada Y pozisyonu (su yüzeyinin üstünde sabit)
            max_wave_height = self.WATER_SURFACE_Y_BASE + 1.5
            island_y_position = max_wave_height + 5
            
            # Havuz sınırları: +-havuz_genisligi (yani +-200 birim)
            # X ve Z eksenleri random, Y ekseni su yüzeyinin üstünde sabit
            havuz_sinir = self.havuz_genisligi  # +-havuz_genisligi
            min_x = -havuz_sinir
            max_x = havuz_sinir
            min_z = -havuz_sinir
            max_z = havuz_sinir
            
            # Mevcut ada pozisyonları (çakışma kontrolü için)
            placed_island_positions = []
            
            # ============================================================
            # HER ADA İÇİN OLUŞTURMA DÖNGÜSÜ
            # ============================================================
            for island_idx in range(n_islands):
                # --- 1. ÖLÇEK HESAPLAMA ---
                scale_multiplier = random.uniform(0.5, 1.5)
                
                # Ada yarıçapı hesaplama (en geniş hitbox katmanına göre)
                # En geniş katman: scale=(55, 15, 55), yarıçap = max(55, 55) / 2 = 27.5
                max_hitbox_radius = max(ref_hitbox_scales[0][0], ref_hitbox_scales[0][2]) / 2
                island_radius = max_hitbox_radius * scale_multiplier
                
                # Minimum mesafe (2.5 kat güvenlik payı ile)
                min_distance_between_islands = island_radius * 3
                
                # --- 2. GÜVENLİ POZİSYON BULMA (X ve Z random, Y sabit) ---
                island_x, island_z = self._find_safe_island_position(
                    placed_island_positions=placed_island_positions,
                    min_x=min_x,
                    max_x=max_x,
                    min_z=min_z,
                    max_z=max_z,
                    min_distance=min_distance_between_islands,
                    max_attempts=100
                )
                
                # --- 3. GÖRSEL ADA OLUŞTURMA ---
                visual_scale = (
                    ref_visual_scale[0] * scale_multiplier,
                    ref_visual_scale[1] * scale_multiplier,
                    ref_visual_scale[2] * scale_multiplier
                )
                
                island = Entity(
                    model=island_model_path,
                    position=(island_x, island_y_position, island_z),  # X ve Z random
                    scale=visual_scale,
                    texture=island_texture_path if os.path.exists(island_texture_path) else None,
                    collider='mesh',
                    unlit=False,
                    double_sided=True, 
                    color=color.white,
                    alpha=1.0,
                    transparent=True,
                    render_queue=0
                )
                
                # İlk adayı self.island olarak sakla (geriye uyumluluk için)
                if island_idx == 0:
                    self.island = island
                
                # --- 4. ÇOK KATMANLI HİTBOX SİSTEMİ OLUŞTURMA ---
                hitbox_katmanlari = self._create_island_hitboxes(
                    island_x=island_x,
                    island_z=island_z,
                    scale_multiplier=scale_multiplier,
                    ref_hitbox_scales=ref_hitbox_scales,
                    ref_hitbox_positions=ref_hitbox_positions,
                    island_idx=island_idx
                )
                
                # Hitbox'ları engel listesine ekle
                for parca in hitbox_katmanlari:
                    self.engeller.append(parca)
                
                # Ada pozisyonunu, yarıçapını ve hitbox'larını sakla
                # Koordinat sistemi: (x_2d, y_2d, radius) - z_depth her zaman aynı (su yüzeyinin üstünde)
                # radius: Ada yarıçapı (harita çizimi için)
                self.island_positions.append((island_x, island_z, island_radius))  # (x_2d, y_2d, radius)
                self.island_hitboxes.extend(hitbox_katmanlari)
                
                # Yerleştirilen ada pozisyonunu kaydet (sonraki adalar için çakışma kontrolü)
                placed_island_positions.append((island_x, island_z))
        else:
            # Fallback: Ada yoksa None
            self.island = None
            self.island_positions = []
        
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
        seabed_kalinligi = su_hacmi_yuksekligi * 0.25
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
        cimen_kalinligi = su_hacmi_yuksekligi * 0.5
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
        self.filo = None  # Filo referansı (main.py'den set edilecek)
        # engeller listesi ada hitbox'ları eklendikten sonra oluşturuldu (ada varsa)
        # Eğer ada yoksa veya engeller listesi oluşturulmadıysa, şimdi oluştur
        if not hasattr(self, 'engeller'):
            self.engeller = []

        # Konsol verileri
        self.konsol_verileri = {}
        
        # Harita sistemi (Matplotlib - ayrı pencere)
        try:
            self.harita = Harita(ortam_ref=self, pencere_boyutu=(800, 800))
            print("✅ Harita sistemi başarıyla oluşturuldu (Matplotlib penceresi)")
        except Exception as e:
            print(f"❌ Harita oluşturulurken hata: {e}")
            import traceback
            traceback.print_exc()
            self.harita = None
    
    # ============================================================
    # YARDIMCI FONKSİYONLAR: ADA OLUŞTURMA
    # ============================================================
    
    def _find_safe_island_position(self, placed_island_positions, min_x, max_x, min_z, max_z, min_distance, max_attempts=100):
        """
        Adaların birbirine çakışmaması için güvenli (X, Z) pozisyonu bulur.
        Y ekseni su yüzeyinin üstünde sabit (island_y_position).
        
        Args:
            placed_island_positions: Mevcut ada pozisyonları listesi [(x, z), ...]
            min_x, max_x: Havuz X sınırları
            min_z, max_z: Havuz Z sınırları
            min_distance: Minimum mesafe (ada yarıçapı * güvenlik payı)
            max_attempts: Maksimum deneme sayısı
            
        Returns:
            (island_x, island_z): Güvenli ada pozisyonu (X ve Z random)
        """
        # İlk ada ise, merkezden uzak bir yere yerleştir
        if not placed_island_positions:
            return (
                random.choice([min_x + 20, max_x - 20]),
                random.choice([min_z + 20, max_z - 20])
            )
        
        # Güvenli pozisyon bul (maksimum deneme sayısı kadar)
        for attempt in range(max_attempts):
            # Random X ve Z pozisyonları (havuz sınırları içinde)
            candidate_x = random.uniform(min_x, max_x)
            candidate_z = random.uniform(min_z, max_z)
            
            # Mevcut adalardan yeterince uzak mı kontrol et (2D mesafe: X-Z düzlemi)
            too_close = False
            for existing_x, existing_z in placed_island_positions:
                # 2D yatay mesafe hesabı (X-Z düzlemi)
                dx = candidate_x - existing_x
                dz = candidate_z - existing_z
                distance = (dx**2 + dz**2)**0.5  # 2D Öklid mesafesi
                
                if distance < min_distance:
                    too_close = True
                    break
            
            if not too_close:
                return (candidate_x, candidate_z)
        
        # Eğer güvenli pozisyon bulunamadıysa, mevcut adalardan en uzak noktayı seç
        if placed_island_positions:
            # Mevcut adaların X ve Z ortalaması
            avg_x = sum(x for x, z in placed_island_positions) / len(placed_island_positions)
            avg_z = sum(z for x, z in placed_island_positions) / len(placed_island_positions)
            
            # Ortalamadan uzak bir nokta bul
            if avg_x > 0:
                fallback_x = max(min_x + 20, avg_x - min_distance)
            else:
                fallback_x = min(max_x - 20, avg_x + min_distance)
            
            if avg_z > 0:
                fallback_z = max(min_z + 20, avg_z - min_distance)
            else:
                fallback_z = min(max_z - 20, avg_z + min_distance)
            
            return (fallback_x, fallback_z)
        
        # Son çare: Merkezden uzak bir yere yerleştir
        return (
            random.choice([min_x + 20, max_x - 20]),
            random.choice([min_z + 20, max_z - 20])
        )
    
    def _create_island_hitboxes(self, island_x, island_z, scale_multiplier, ref_hitbox_scales, ref_hitbox_positions, island_idx):
        """
        Ada için çok katmanlı hitbox sistemi oluşturur.
        Görsel sınır çizgisi ve hassas algılama için optimize edilmiştir.
        
        Args:
            island_x, island_z: Ada pozisyonu
            scale_multiplier: Ölçek çarpanı
            ref_hitbox_scales: Referans hitbox ölçekleri listesi
            ref_hitbox_positions: Referans hitbox pozisyonları listesi
            island_idx: Ada indeksi (renk farklılaştırması için)
            
        Returns:
            hitbox_katmanlari: Hitbox entity'leri listesi
        """
        hitbox_katmanlari = []
        
        # En geniş katmanın yarıçapını hesapla (görsel sınır çizgisi için)
        max_radius = max(ref_hitbox_scales[0][0], ref_hitbox_scales[0][2]) / 2 * scale_multiplier
        max_height = ref_hitbox_scales[0][1] * scale_multiplier
        
        # Ada çevresine görsel sınır çizgisi ekle (yarı saydam sphere - cylinder yerine)
        # Bu, ROV'ların ada sınırlarını görmesini sağlar
        # Ursina'da cylinder modeli yok, bu yüzden sphere kullanıyoruz
        sinir_cizgisi = Entity(
            model='sphere',
            position=(island_x, -max_height/2, island_z),
            scale=(max_radius * 2, max_height, max_radius * 2),  # Y ekseni uzun, X-Z eksenleri eşit (silindir benzeri)
            color=color.rgba(255, 200, 0, 0.3),  # Turuncu-sarı, yarı saydam
            visible=True,  # Görünür (sınır çizgisi)
            double_sided=True,
            unlit=True,
            transparent=True,  # Şeffaflık için
            texture=None
        )
        
        hitbox_katmanlari.append(sinir_cizgisi)
        
        # Renkleri farklılaştır (her ada için farklı ton)
        color_offset = island_idx * 50
        colors = [
            color.rgba(255 - color_offset, 0, 0, 0.3),      # Kırmızı
            color.rgba(0, 255 - color_offset, 0, 0.3),      # Yeşil
            color.rgba(0, 0, 255 - color_offset, 0.3),       # Mavi
            color.rgba(255 - color_offset, 255 - color_offset, 0, 0.3)  # Sarı
        ]
        
        # Her katman için hitbox oluştur (daha hassas algılama için)
        for layer_idx, (ref_scale, ref_pos) in enumerate(zip(ref_hitbox_scales, ref_hitbox_positions)):
            # Ölçeği çarpanla çarp
            scaled_size = (
                ref_scale[0] * scale_multiplier,
                ref_scale[1] * scale_multiplier,
                ref_scale[2] * scale_multiplier
            )
            
            # Pozisyonu ada pozisyonuna göre ayarla (Y aynı kalacak)
            hitbox_pos = (island_x, ref_pos[1], island_z)
            
            # Hitbox'lar görünmez ama algılama için aktif
            hitbox_katmanlari.append(Entity(
                model='icosphere',
                position=hitbox_pos,
                scale=scaled_size,
                visible=False,  # Görünmez (sadece algılama için)
                collider='sphere',
                color=colors[layer_idx],
                unlit=True
            ))
        
        return hitbox_katmanlari

    # --- Simülasyon Nesnelerini Oluştur ---
    def sim_olustur(self, n_rovs=3, n_engels=15, havuz_genisligi=200):
        # Havuz genişliğini güncelle (ada oluşturma için)
        self.havuz_genisligi = havuz_genisligi
        
        # Ada hitbox'larını ve pozisyonlarını koru (eğer varsa)
        ada_hitbox_backup = []
        ada_positions_backup = []
        if hasattr(self, 'island_hitboxes') and self.island_hitboxes:
            ada_hitbox_backup = self.island_hitboxes.copy()
        if hasattr(self, 'island_positions') and self.island_positions:
            ada_positions_backup = self.island_positions.copy()
        
        # Engeller (Kayalar) - Listeyi sıfırla ama ada hitbox'larını koruyacağız
        self.engeller = []
        
        # Ada hitbox'larını geri ekle (eğer varsa)
        if ada_hitbox_backup:
            for hitbox in ada_hitbox_backup:
                self.engeller.append(hitbox)
        
        # Ada pozisyonlarını geri yükle (eğer varsa)
        if ada_positions_backup:
            self.island_positions = ada_positions_backup
        
        # Engeller (Kayalar)
        # Kayalar su altında oluşmalı ve tabanları deniz tabanına değmeli
        for _ in range(n_engels):
            x = random.uniform(-200, 200)
            z = random.uniform(-200, 200)
            
            # Kaya boyutları
            s_x = random.uniform(15, 40)
            s_y = random.uniform(15, 40)
            s_z = random.uniform(15, 60)  # Z ekseni de pozitif olmalı (küre için)

            # Kaya pozisyonu: Tabanı deniz tabanına değmeli, üstü su yüzeyinin altında olmalı
            # Kaya merkez pozisyonu = deniz tabanı + (kaya yüksekliği / 2) ile su yüzeyi - (kaya yüksekliği / 2) arasında
            kaya_alt_sinir = self.SEA_FLOOR_Y  # Kayanın alt kısmı deniz tabanında
            kaya_ust_sinir = self.WATER_SURFACE_Y_BASE - (s_y / 2) - 2  # Kayanın üst kısmı su yüzeyinin 2 birim altında
            
            # Eğer kaya çok büyükse ve su yüzeyine sığmıyorsa, deniz tabanına yerleştir
            if kaya_ust_sinir < kaya_alt_sinir:
                y = self.SEA_FLOOR_Y + (s_y / 2)  # Tabanı deniz tabanında
            else:
                y = random.uniform(kaya_alt_sinir, kaya_ust_sinir)

            gri = random.randint(80, 100)
            kaya_rengi = color.rgb(gri, gri, gri)

            engel = Entity(
                model='icosphere',
                color=kaya_rengi,
                texture='noise',
                scale=(s_x, s_y, s_z),
                position=(x, self.SEA_FLOOR_Y, z),
                rotation=(random.randint(0, 360), random.randint(0, 360), random.randint(0, 360)),
                collider='mesh',
                unlit=True
            )
            self.engeller.append(engel)

        # ============================================================
        # ROV YERLEŞTİRME (Adaların dışına - Ada radyuslarına göre)
        # ============================================================
        # Havuz sınırları: +-havuz_genisligi (yani +-200 birim)
        havuz_sinir = self.havuz_genisligi  # +-havuz_genisligi
        min_x = -havuz_sinir
        max_x = havuz_sinir
        min_z = -havuz_sinir
        max_z = havuz_sinir
        
        # Güvenlik payı (ada radyusuna ek olarak bırakılacak minimum mesafe)
        GUVENLIK_PAYI = 50.0  # birim
        
        # Ada pozisyonları ve radyusları kontrolü (eğer varsa)
        ada_bilgileri = []
        if hasattr(self, 'island_positions') and self.island_positions:
            for island_data in self.island_positions:
                if len(island_data) == 3:
                    island_x_2d, island_y_2d, island_radius = island_data
                    ada_bilgileri.append({
                        'x': island_x_2d,
                        'y': island_y_2d,
                        'radius': island_radius,
                        'min_safe_distance': island_radius + GUVENLIK_PAYI
                    })
                elif len(island_data) == 2:
                    # Geriye uyumluluk: Radyus yoksa varsayılan değer kullan
                    island_x_2d, island_y_2d = island_data
                    varsayilan_radius = 50.0  # Güvenli varsayılan değer
                    ada_bilgileri.append({
                        'x': island_x_2d,
                        'y': island_y_2d,
                        'radius': varsayilan_radius,
                        'min_safe_distance': varsayilan_radius + GUVENLIK_PAYI
                    })
        
        for i in range(n_rovs):
            max_attempts = 100  # Daha fazla deneme hakkı
            placed = False
            
            # Güvenli pozisyon bul (maksimum deneme sayısı kadar)
            for attempt in range(max_attempts):
                # Random pozisyon (havuz sınırları içinde)
                # Koordinat sistemi: (x_2d, y_2d, z_depth)
                x_2d = random.uniform(min_x, max_x)
                y_2d = random.uniform(min_z, max_z)  # Not: min_z/max_z aslında y_2d sınırları
                z_depth = random.uniform(-20, -5)  # Su altında (derinlik negatif)
                
                # Ada kontrolü: ROV'un adaların içinde olup olmadığını kontrol et
                too_close_to_island = False
                
                if ada_bilgileri:
                    for ada_info in ada_bilgileri:
                        # 2D yatay mesafe hesabı (X_2D - Y_2D düzlemi)
                        # Z_DEPTH (derinlik) farklı olduğu için sadece yatay mesafe kontrol edilir
                        dx_2d = x_2d - ada_info['x']
                        dy_2d = y_2d - ada_info['y']
                        horizontal_distance = (dx_2d**2 + dy_2d**2)**0.5
                        
                        # Ada radyusuna göre dinamik güvenli mesafe kontrolü
                        if horizontal_distance < ada_info['min_safe_distance']:
                            too_close_to_island = True
                            break
                
                # Güvenli pozisyon bulundu
                if not too_close_to_island:
                    # Ursina'ya dönüştür: (x_2d, z_depth, y_2d)
                    x, y, z = sim_to_ursina(x_2d, y_2d, z_depth)
                    new_rov = ROV(rov_id=i, position=(x, y, z))
                    new_rov.environment_ref = self
                    if hasattr(self, 'filo'):
                        new_rov.filo_ref = self.filo
                    self.rovs.append(new_rov)
                    placed = True
                    break
            
            # Eğer yerleştirilemediyse, ada olmayan bölgelere zorla yerleştir
            if not placed:
                # Ada olmayan bölgeleri bul (ada merkezlerinden uzak noktalar)
                if ada_bilgileri:
                    # Ada merkezlerinden uzak bir pozisyon bul
                    best_x, best_y = None, None
                    best_min_distance = 0
                    
                    for fallback_attempt in range(50):
                        test_x = random.uniform(min_x, max_x)
                        test_y = random.uniform(min_z, max_z)
                        
                        # En yakın ada mesafesini bul
                        min_dist_to_any_island = float('inf')
                        for ada_info in ada_bilgileri:
                            dx = test_x - ada_info['x']
                            dy = test_y - ada_info['y']
                            dist = (dx**2 + dy**2)**0.5
                            min_dist_to_any_island = min(min_dist_to_any_island, dist)
                        
                        # En uzak mesafeyi seç
                        if min_dist_to_any_island > best_min_distance:
                            best_min_distance = min_dist_to_any_island
                            best_x, best_y = test_x, test_y
                    
                    if best_x is not None and best_y is not None:
                        x_2d, y_2d = best_x, best_y
                    else:
                        # Son çare: Merkezden uzak bir nokta
                        x_2d = random.uniform(min_x, max_x)
                        y_2d = random.uniform(min_z, max_z)
                else:
                    # Ada yoksa normal rastgele yerleştir
                    x_2d = random.uniform(min_x, max_x)
                    y_2d = random.uniform(min_z, max_z)
                
                z_depth = random.uniform(-20, -5)
                x, y, z = sim_to_ursina(x_2d, y_2d, z_depth)
                new_rov = ROV(rov_id=i, position=(x, y, z))
                new_rov.environment_ref = self
                if hasattr(self, 'filo'):
                    new_rov.filo_ref = self.filo
                self.rovs.append(new_rov)
                print(f"⚠️ ROV-{i} zorla yerleştirildi (ada kontrolü başarısız)")

        print(f"🌊 Simülasyon Hazır: {n_rovs} ROV, {n_engels} Gri Kaya.")
    
    # --- Ada ve ROV Konum Yönetimi (Senaryo Modülü İçin) ---
    def Ada(self, ada_id, x=None, y=None):
        """
        Ada pozisyonunu değiştirir veya konumunu döndürür.
        
        Args:
            ada_id: Ada ID'si
            x: Yeni X koordinatı (None ise mevcut konumu döndürür)
            y: Yeni Y koordinatı (Z ekseni, None ise mevcut konumu döndürür)
        
        Returns:
            tuple: (x, y) koordinatları veya None
        
        Örnek:
            # Ada konumunu değiştir
            app.Ada(0, 50, 60)
            
            # Ada konumunu al
            konum = app.Ada(0)  # (x, y) tuple döner
        """
        # Ada pozisyonları kontrolü
        if not hasattr(self, 'island_positions') or not self.island_positions:
            # Ada yoksa oluştur
            if not hasattr(self, 'island_positions'):
                self.island_positions = []
            # Ada ID'si için yeterli kapasite yoksa genişlet
            while len(self.island_positions) <= ada_id:
                self.island_positions.append((0, 0, 50.0))  # Varsayılan pozisyon ve radius
        
        # Konum değiştirme
        if x is not None and y is not None:
            # Ada pozisyonunu güncelle
            radius = self.island_positions[ada_id][2] if len(self.island_positions[ada_id]) > 2 else 50.0
            old_pos = self.island_positions[ada_id]
            self.island_positions[ada_id] = (x, y, radius)
            
            # Ada hitbox'larını güncelle (eğer varsa)
            if hasattr(self, 'island_hitboxes') and self.island_hitboxes:
                # Ada hitbox'larını bul ve güncelle
                # Her ada için birden fazla hitbox olabilir (katmanlı sistem)
                # Ada ID'sine göre hitbox'ları bulmak için pozisyon karşılaştırması yapılır
                for hitbox in self.island_hitboxes:
                    if hasattr(hitbox, 'position'):
                        # Eski pozisyona yakın hitbox'ları bul
                        old_x, old_y = old_pos[0], old_pos[1]
                        hitbox_x = hitbox.position.x
                        hitbox_z = hitbox.position.z
                        mesafe = ((hitbox_x - old_x)**2 + (hitbox_z - old_y)**2)**0.5
                        
                        # Eğer hitbox bu ada'ya aitse (yakın mesafede)
                        if mesafe < radius * 2:
                            # Hitbox pozisyonunu güncelle
                            hitbox.position = (x, hitbox.position.y, y)
            
            # Engeller listesindeki ada hitbox'larını da güncelle
            if hasattr(self, 'engeller') and self.engeller:
                old_x, old_y = old_pos[0], old_pos[1]
                for engel in self.engeller:
                    if hasattr(engel, 'position') and hasattr(engel, 'model'):
                        # Ada sınır çizgisi kontrolü (sphere modeli ve görünür)
                        is_island_boundary = (engel.model == 'sphere' and 
                                             hasattr(engel, 'visible') and 
                                             engel.visible == True)
                        if is_island_boundary:
                            engel_x = engel.position.x
                            engel_z = engel.position.z
                            mesafe = ((engel_x - old_x)**2 + (engel_z - old_y)**2)**0.5
                            if mesafe < radius * 2:
                                engel.position = (x, engel.position.y, y)
            
            print(f"✅ Ada-{ada_id} pozisyonu güncellendi: ({x}, {y})")
            return (x, y)
        else:
            # Mevcut konumu döndür
            if ada_id < len(self.island_positions):
                ada_pos = self.island_positions[ada_id]
                return (ada_pos[0], ada_pos[1])
            else:
                return None
    
    def ROV(self, rov_id, x=None, y=None, z=None):
        """
        ROV pozisyonunu değiştirir veya konumunu döndürür.
        
        Args:
            rov_id: ROV ID'si
            x: Yeni X koordinatı (None ise mevcut konumu döndürür)
            y: Yeni Y koordinatı (derinlik, None ise mevcut konumu döndürür)
            z: Yeni Z koordinatı (None ise mevcut konumu döndürür)
        
        Returns:
            tuple: (x, y, z) koordinatları veya None
        
        Örnek:
            # ROV konumunu değiştir
            app.ROV(0, 10, -5, 20)
            
            # ROV konumunu al
            konum = app.ROV(0)  # (x, y, z) tuple döner
        """
        if rov_id >= len(self.rovs):
            print(f"⚠️ ROV ID {rov_id} bulunamadı.")
            return None
        
        rov = self.rovs[rov_id]
        
        # Konum değiştirme
        if x is not None and y is not None and z is not None:
            # Ursina koordinat sistemine dönüştür: (x_2d, z_depth, y_2d)
            ursina_x, ursina_y, ursina_z = sim_to_ursina(x, z, y)
            
            # ROV pozisyonunu güncelle
            if hasattr(rov, 'position'):
                rov.position = Vec3(ursina_x, ursina_y, ursina_z)
            if hasattr(rov, 'x'):
                rov.x = ursina_x
                rov.y = ursina_y
                rov.z = ursina_z
            
            print(f"✅ ROV-{rov_id} pozisyonu güncellendi: ({x}, {y}, {z})")
            return (x, y, z)
        else:
            # Mevcut konumu döndür (simülasyon koordinat sistemine dönüştür)
            if hasattr(rov, 'position') and hasattr(rov.position, 'x'):
                ursina_x, ursina_y, ursina_z = rov.position.x, rov.position.y, rov.position.z
                x_2d, y_2d, z_depth = ursina_to_sim(ursina_x, ursina_y, ursina_z)
                return (x_2d, z_depth, y_2d)
            elif hasattr(rov, 'x'):
                ursina_x, ursina_y, ursina_z = rov.x, rov.y, rov.z
                x_2d, y_2d, z_depth = ursina_to_sim(ursina_x, ursina_y, ursina_z)
                return (x_2d, z_depth, y_2d)
            else:
                return None
    
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

    # --- Veri Toplama Fonksiyonu (GAT Girdisi) ---
    def simden_veriye(self):
        """
        Fiziksel dünyayı Matematiksel matrise çevirir (GAT Girdisi)
        
        Returns:
            MiniData: GAT modeli için hazırlanmış veri yapısı (x, edge_index)
        """
        rovs = self.rovs
        engeller = self.engeller
        n = len(rovs)
        x = torch.zeros((n, 7), dtype=torch.float)
        positions = [r.position for r in rovs]
        sources, targets = [], []

        L = {'LEADER': 60.0, 'DISCONNECT': 35.0, 'OBSTACLE': 20.0, 'COLLISION': 8.0}

        for i in range(n):
            code = 0
            if i != 0 and distance(positions[i], positions[0]) > L['LEADER']: 
                code = 5
            dists = [distance(positions[i], positions[j]) for j in range(n) if i != j]
            if dists and min(dists) > L['DISCONNECT']: 
                code = 3
            
            min_engel = 999
            for engel in engeller:
                d = distance(positions[i], engel.position) - 6 
                if d < min_engel: 
                    min_engel = d
            if min_engel < L['OBSTACLE']: 
                code = 1
            
            for j in range(n):
                if i != j and distance(positions[i], positions[j]) < L['COLLISION']:
                    code = 2
                    break
            
            x[i][0] = code / 5.0
            x[i][1] = rovs[i].battery  # Batarya artık 0-1 arası, bölmeye gerek yok
            x[i][2] = 0.9
            x[i][3] = abs(rovs[i].y) / 100.0
            x[i][4] = rovs[i].velocity.x
            x[i][5] = rovs[i].velocity.z
            x[i][6] = rovs[i].role

            for j in range(n):
                if i != j and distance(positions[i], positions[j]) < L['DISCONNECT']:
                    sources.append(i)
                    targets.append(j)

        edge_index = torch.tensor([sources, targets], dtype=torch.long)
        class MiniData:
            def __init__(self, x, edge_index): 
                self.x, self.edge_index = x, edge_index
        return MiniData(x, edge_index)

    # --- Main Run Fonksiyonu ---
    def run(self, interaktif=False):
        if interaktif:
            t = threading.Thread(target=self._start_shell)
            t.daemon = True
            t.start()
        self.app.run()
