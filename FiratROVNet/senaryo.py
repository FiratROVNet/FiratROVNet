"""
Senaryo Üretim Modülü - Yapay Zeka Eğitimi İçin Veri Üretimi

Bu modül, GUI olmadan (headless) simülasyon ortamları oluşturur ve
yapay zeka algoritmalarını eğitmek için veri üretir.

Kullanım:
    from FiratROVNet import senaryo
    
    # Senaryo oluştur
    senaryo.uret(n_rovs=4, n_engels=20, havuz_genisligi=200)
    
    # Veri al
    batarya = senaryo.get(0, "batarya")
    gps = senaryo.get(0, "gps")
    sonar = senaryo.get(0, "sonar")
    
    # veya Filo üzerinden
    batarya = senaryo.filo.get(0, "batarya")
"""

import os
import sys

# Ursina'yı headless modda başlat
os.environ['URSINA_HEADLESS'] = '1'

from ursina import *
from FiratROVNet.simulasyon import Ortam, ROV
from FiratROVNet.gnc import Filo
import numpy as np
import random

# Global senaryo instance
_senaryo_instance = None


class Senaryo:
    """
    Senaryo üretim sınıfı - Headless simülasyon ortamı oluşturur.
    
    Güvenli pozisyon bulma fonksiyonları:
    - _guvenli_ada_pozisyonu_bul: Adalar için güvenli pozisyon bulur
    - _guvenli_rov_pozisyonu_bul: ROV'lar için güvenli pozisyon bulur
    
    Bu sınıf, GUI olmadan simülasyon ortamları oluşturur ve
    yapay zeka eğitimi için veri üretir.
    """
    
    def __init__(self, verbose=False):
        self.app = None
        self.filo = None
        self.ortam = None
        self.aktif = False
        self.verbose = verbose  # Log mesajlarını kontrol eder
        
        # Önbellek mekanizması (hızlı senaryo üretimi için)
        self._cache_n_rovs = None
        self._cache_n_engels = None
        self._cache_havuz_genisligi = None
    
    def _guvenli_ada_pozisyonu_bul(self, mevcut_adalar, havuz_genisligi, ada_radius, min_mesafe_ada, max_deneme=100):
        """
        Güvenli ada pozisyonu bulur (adalar üst üste gelmemeli).
        
        Args:
            mevcut_adalar: Mevcut ada pozisyonları [(x, z, radius), ...]
            havuz_genisligi: Havuz genişliği
            ada_radius: Ada yarıçapı
            min_mesafe_ada: Adalardan minimum mesafe
            max_deneme: Maksimum deneme sayısı
        
        Returns:
            (x, z): Güvenli ada pozisyonu veya None
        """
        havuz_yari_genislik = havuz_genisligi * 0.45
        
        for _ in range(max_deneme):
            x = random.uniform(-havuz_yari_genislik, havuz_yari_genislik)
            z = random.uniform(-havuz_yari_genislik, havuz_yari_genislik)
            pos = np.array([x, z])
            
            # Mevcut adalardan uzak mı?
            guvenli = True
            for ada_x, ada_z, mevcut_radius in mevcut_adalar:
                mesafe = np.linalg.norm(pos - np.array([ada_x, ada_z]))
                min_mesafe = ada_radius + mevcut_radius + min_mesafe_ada
                if mesafe < min_mesafe:
                    guvenli = False
                    break
            
            if guvenli:
                return (x, z)
        
        return None
    
    def _guvenli_rov_pozisyonu_bul(self, mevcut_rov_positions, mevcut_adalar, min_mesafe_rov, min_mesafe_ada, havuz_genisligi, max_deneme=100):
        """
        Güvenli ROV pozisyonu bulur (ROV'lar birbirine çarpmamalı, adalara binmemeli).
        
        Args:
            mevcut_rov_positions: Mevcut ROV pozisyonları [[x, y, z], ...]
            mevcut_adalar: Ada pozisyonları ve yarıçapları [(x, z, radius), ...]
            min_mesafe_rov: ROV'lar arası minimum mesafe
            min_mesafe_ada: Adalardan minimum mesafe
            havuz_genisligi: Havuz genişliği
            max_deneme: Maksimum deneme sayısı
        
        Returns:
            (x, z): Güvenli ROV pozisyonu veya None
        """
        havuz_yari_genislik = havuz_genisligi * 0.45
        
        for _ in range(max_deneme):
            x = random.uniform(-havuz_yari_genislik, havuz_yari_genislik)
            z = random.uniform(-havuz_yari_genislik, havuz_yari_genislik)
            pos = np.array([x, z])
            
            # Mevcut ROV pozisyonlarından uzak mı?
            guvenli = True
            for rov_pos in mevcut_rov_positions:
                if len(rov_pos) >= 2:
                    mesafe = np.linalg.norm(pos - np.array(rov_pos[:2]))
                    if mesafe < min_mesafe_rov:
                        guvenli = False
                        break
            
            if not guvenli:
                continue
            
            # Adalardan uzak mı?
            for ada_x, ada_z, ada_radius in mevcut_adalar:
                mesafe = np.linalg.norm(pos - np.array([ada_x, ada_z]))
                if mesafe < (ada_radius + min_mesafe_ada):
                    guvenli = False
                    break
            
            if guvenli:
                return (x, z)
        
        return None
    
    def _nesneleri_yeniden_dagit(self):
        """
        Entity'leri yok etmeden SADECE koordinatlarını değiştirir. (Çok Hızlı)
        Bu metod, mevcut nesnelerin pozisyonlarını güvenli şekilde yeniden dağıtır.
        """
        from .config import GATLimitleri, HareketAyarlari
        from ursina import Vec3
        
        havuz = self._cache_havuz_genisligi
        min_mesafe_ada = HareketAyarlari.RANDOM_HEDEF_MIN_MESAFE_ADA  # Adalardan minimum mesafe
        min_mesafe_rov = GATLimitleri.CARPISMA * 1.5  # ROV'lar arası minimum mesafe
        
        yeni_ada_pos = []  # [(x, z, r)]
        
        # --- 1. ADALARI (Engelleri) DAĞIT ---
        # Adaları güvenli pozisyonlara taşı
        if hasattr(self.ortam, 'island_positions') and self.ortam.island_positions:
            for i, ada_data in enumerate(self.ortam.island_positions):
                if len(ada_data) >= 3:
                    _, _, radius = ada_data
                else:
                    radius = 30.0  # Varsayılan yarıçap
                
                pos = self._guvenli_ada_pozisyonu_bul(yeni_ada_pos, havuz, radius, min_mesafe_ada)
                if pos:
                    yeni_x, yeni_z = pos
                    # Ada metodunu kullanarak taşı (hitbox'ları da otomatik güncellenir)
                    # Ada() metodu: Ada(ada_id, x, y) formatında - y parametresi aslında z koordinatı
                    if hasattr(self.ortam, 'Ada') and callable(getattr(self.ortam, 'Ada', None)):
                        self.ortam.Ada(i, yeni_x, yeni_z)  # y parametresi aslında z koordinatı
                    else:
                        # Fallback: Manuel güncelleme
                        self.ortam.island_positions[i] = [yeni_x, yeni_z, radius]
                        if hasattr(self.ortam, 'island_entities') and i < len(self.ortam.island_entities):
                            ada_entity = self.ortam.island_entities[i]
                            if hasattr(ada_entity, 'position'):
                                ada_entity.position = Vec3(yeni_x, 0, yeni_z)
                            elif hasattr(ada_entity, 'x'):
                                ada_entity.x = yeni_x
                                ada_entity.z = yeni_z
                        if hasattr(self.ortam, 'island_hitboxes'):
                            ada_hitbox_start = i * 5
                            for hitbox_idx in range(ada_hitbox_start, min(ada_hitbox_start + 5, len(self.ortam.island_hitboxes))):
                                hitbox = self.ortam.island_hitboxes[hitbox_idx]
                                if hasattr(hitbox, 'position'):
                                    hitbox.position = Vec3(yeni_x, 0, yeni_z)
                                elif hasattr(hitbox, 'x'):
                                    hitbox.x = yeni_x
                                    hitbox.z = yeni_z
                    
                    yeni_ada_pos.append((yeni_x, yeni_z, radius))
                else:
                    # Güvenli pozisyon bulunamazsa eski pozisyonu kullan
                    if len(ada_data) >= 2:
                        yeni_ada_pos.append((ada_data[0], ada_data[1], radius))
        
        # --- 2. DİĞER ENGELLERİ (Kayaları) DAĞIT ---
        # island_hitboxes dışındaki engeller kayalardır
        ada_hitboxlar = getattr(self.ortam, 'island_hitboxes', [])
        for engel in self.ortam.engeller:
            if engel in ada_hitboxlar:
                continue  # Adaları zaten taşıdık
            
            # Kayaları rastgele boşluklara at (güvenli mesafede)
            max_deneme = 50
            for _ in range(max_deneme):
                engel_x = random.uniform(-havuz * 0.45, havuz * 0.45)
                engel_z = random.uniform(-havuz * 0.45, havuz * 0.45)
                
                # Adalardan uzak mı kontrol et
                guvenli = True
                for ada_x, ada_z, ada_radius in yeni_ada_pos:
                    mesafe = np.linalg.norm(np.array([engel_x, engel_z]) - np.array([ada_x, ada_z]))
                    if mesafe < (ada_radius + min_mesafe_ada):
                        guvenli = False
                        break
                
                if guvenli:
                    break
            
            # Engel pozisyonunu güncelle
            if hasattr(engel, 'position'):
                if hasattr(engel.position, 'x'):
                    engel.position.x = engel_x
                    engel.position.z = engel_z
                    engel.position.y = getattr(self.ortam, 'SEA_FLOOR_Y', -100)
            elif hasattr(engel, 'x'):
                engel.x = engel_x
                engel.z = engel_z
                engel.y = getattr(self.ortam, 'SEA_FLOOR_Y', -100)
        
        # --- 3. ROV'LARI DAĞIT ---
        mevcut_rov_pos = []
        for rov in self.ortam.rovs:
            pos = self._guvenli_rov_pozisyonu_bul(mevcut_rov_pos, yeni_ada_pos, 
                                                  min_mesafe_rov, min_mesafe_ada, havuz)
            if pos:
                yeni_x, yeni_z = pos
                # ROV pozisyonunu güncelle
                if hasattr(rov, 'position'):
                    rov.position = Vec3(yeni_x, -5, yeni_z)
                elif hasattr(rov, 'x'):
                    rov.x = yeni_x
                    rov.y = -5
                    rov.z = yeni_z
                
                # Hızı sıfırla
                if hasattr(rov, 'velocity'):
                    if hasattr(rov.velocity, 'x'):
                        rov.velocity.x = 0
                        rov.velocity.y = 0
                        rov.velocity.z = 0
                    else:
                        rov.velocity = Vec3(0, 0, 0)
                
                mevcut_rov_pos.append([yeni_x, -5, yeni_z])
            else:
                # Güvenli pozisyon bulunamazsa rastgele yerleştir
                yeni_x = random.uniform(-havuz * 0.45, havuz * 0.45)
                yeni_z = random.uniform(-havuz * 0.45, havuz * 0.45)
                
                if hasattr(rov, 'position'):
                    rov.position = Vec3(yeni_x, -5, yeni_z)
                elif hasattr(rov, 'x'):
                    rov.x = yeni_x
                    rov.y = -5
                    rov.z = yeni_z
                
                if hasattr(rov, 'velocity'):
                    if hasattr(rov.velocity, 'x'):
                        rov.velocity.x = 0
                        rov.velocity.y = 0
                        rov.velocity.z = 0
                    else:
                        rov.velocity = Vec3(0, 0, 0)
                
                mevcut_rov_pos.append([yeni_x, -5, yeni_z])
        
        # Lideri random seç
        if len(self.ortam.rovs) > 0:
            yeni_lider_id = random.randint(0, len(self.ortam.rovs) - 1)
            for i, rov in enumerate(self.ortam.rovs):
                if hasattr(rov, 'set'):
                    rov.set('rol', 1 if i == yeni_lider_id else 0)
                elif hasattr(rov, 'role'):
                    rov.role = 1 if i == yeni_lider_id else 0
        
        if self.verbose:
            print(f"🔄 Senaryo Yeniden Düzenlendi (ID'ler ve Nesneler Korundu)")
        return self
        
    def uret(self, n_rovs=None, n_engels=None, havuz_genisligi=None, 
             engel_tipleri=None, baslangic_pozisyonlari=None,
             modem_ayarlari=None, sensor_ayarlari=None, verbose=None):
        """
        Senaryo ortamı oluşturur veya mevcut nesneleri yeniden dağıtır (optimize edilmiş).
        
        Args:
            n_rovs (int): ROV sayısı (varsayılan: 3, None ise mevcut sayı korunur)
            n_engels (int): Engel sayısı (varsayılan: 15, None ise mevcut sayı korunur)
            havuz_genisligi (float): Havuz genişliği (varsayılan: 200)
            engel_tipleri (list, optional): Engel tipleri listesi (sadece ilk kurulumda kullanılır)
            baslangic_pozisyonlari (dict, optional): ROV başlangıç pozisyonları (sadece ilk kurulumda)
            modem_ayarlari (dict, optional): Modem ayarları
            sensor_ayarlari (dict, optional): Sensör ayarları
        
        Returns:
            Senaryo: Kendi instance'ını döndürür (method chaining için)
        
        Örnek:
            # İlk kurulum (yavaş - ortam oluşturulur)
            senaryo.uret(n_rovs=4, n_engels=10)
            
            # Hızlı pozisyon güncelleme (çok hızlı - sadece pozisyonlar değişir)
            senaryo.uret()  # Aynı sayılar, farklı koordinatlar
        """
        # Verbose parametresini güncelle
        if verbose is not None:
            self.verbose = verbose
        
        # 1. Kontrol: Eğer ortam zaten varsa ve parametreler değişmediyse SADECE YER DEĞİŞTİR
        if self.aktif and self.ortam is not None:
            # Parametre kontrolü
            n_rovs_changed = (n_rovs is not None and n_rovs != self._cache_n_rovs)
            n_engels_changed = (n_engels is not None and n_engels != self._cache_n_engels)
            havuz_changed = (havuz_genisligi is not None and havuz_genisligi != self._cache_havuz_genisligi)
            
            if not n_rovs_changed and not n_engels_changed and not havuz_changed:
                # Parametreler değişmedi, sadece pozisyonları güncelle (ÇOK HIZLI!)
                return self._nesneleri_yeniden_dagit()
        
        # 2. Önbellek Güncelleme
        if n_rovs is not None:
            self._cache_n_rovs = n_rovs
        elif self._cache_n_rovs is None:
            self._cache_n_rovs = 3  # Varsayılan
        
        if n_engels is not None:
            self._cache_n_engels = n_engels
        elif self._cache_n_engels is None:
            self._cache_n_engels = 15  # Varsayılan
        
        if havuz_genisligi is not None:
            self._cache_havuz_genisligi = havuz_genisligi
        elif self._cache_havuz_genisligi is None:
            self._cache_havuz_genisligi = 200  # Varsayılan
        
        n_rovs = self._cache_n_rovs
        n_engels = self._cache_n_engels
        havuz_genisligi = self._cache_havuz_genisligi
        
        # 3. İlk Kurulum (Sadece bir kez çalışır - AĞIR KISIM)
        if self.ortam is None:
            # Ursina'yı headless modda başlat
            if self.app is None:
                # Headless mod için özel ayarlar
                os.environ['URSINA_HEADLESS'] = '1'
                
                try:
                    self.app = Ursina(
                        vsync=False,
                        development_mode=False,
                        show_ursina_splash=False,
                        borderless=True,
                        title="FıratROVNet Senaryo Üretimi (Headless)"
                    )
                    
                    # Window özelliklerini güvenli şekilde ayarla
                    try:
                        if hasattr(window, 'fullscreen'):
                            window.fullscreen = False
                    except:
                        pass
                    
                    try:
                        if hasattr(window, 'show'):
                            window.show = False
                    except:
                        pass
                    
                    try:
                        if hasattr(window, 'fps_counter'):
                            window.fps_counter.enabled = False
                    except:
                        pass
                        
                except Exception as e:
                    # Ursina başlatılamazsa minimal ortam oluştur
                    print(f"⚠️ Ursina headless mod başlatılamadı: {e}")
                    print("   Minimal ortam modu kullanılıyor...")
                    self.app = None
            
            # Ortam oluştur - Gerçek Ortam sınıfını kullan (simulasyon.py'den)
            # Bu sayede Ada ve ROV fonksiyonları kullanılabilir
            try:
                from FiratROVNet.simulasyon import Ortam as OrtamSinifi
                self.ortam = OrtamSinifi(verbose=self.verbose)
                # Ortam'a verbose flag'ini aktar
                if hasattr(self.ortam, 'verbose'):
                    self.ortam.verbose = self.verbose
                
                # Headless mod için görsel özellikleri kapat
                if hasattr(self.ortam, 'app'):
                    try:
                        if hasattr(self.ortam.app, 'window'):
                            self.ortam.app.window.show = False
                    except:
                        pass
                
                # Harita sistemini kapat (headless mod için)
                if hasattr(self.ortam, 'harita') and self.ortam.harita:
                    try:
                        self.ortam.harita.goster(False)
                    except:
                        pass
            except Exception as e:
                # Ortam sınıfı yüklenemezse minimal ortam objesi oluştur
                print(f"⚠️ Ortam sınıfı yüklenemedi, minimal mod kullanılıyor: {e}")
                self.ortam = type('Ortam', (), {
                    'rovs': [],
                    'engeller': [],
                    'havuz_genisligi': havuz_genisligi,
                    'filo': None,
                    'island_positions': [],
                    'island_hitboxes': [],
                    'island_entities': []
                })()
        
        # 4. Nesne Sayıları Değiştiyse sim_olustur çağır
        # Not: sim_olustur nesneleri (Entity) yaratır veya günceller
        sim_olustur_basarili = False
        if hasattr(self.ortam, 'sim_olustur'):
            try:
                self.ortam.sim_olustur(
                    n_rovs=n_rovs,
                    n_engels=n_engels,
                    havuz_genisligi=havuz_genisligi
                )
                sim_olustur_basarili = True
            except Exception as e:
                print(f"⚠️ sim_olustur hatası: {e}")
                sim_olustur_basarili = False
        
        # Fallback: sim_olustur yoksa veya başarısız olduysa manuel oluşturma
        if not sim_olustur_basarili:
            self.ortam.havuz_genisligi = havuz_genisligi
            if not hasattr(self.ortam, 'engeller') or not self.ortam.engeller:
                self.ortam.engeller = []
            if not hasattr(self.ortam, 'rovs') or not self.ortam.rovs:
                self.ortam.rovs = []
            
            # Eksik engelleri oluştur
            while len(self.ortam.engeller) < n_engels:
                i = len(self.ortam.engeller)
                engel_tipi = 'kaya'
                if engel_tipleri and i < len(engel_tipleri):
                    engel_tipi = engel_tipleri[i]
                
                x = random.uniform(-havuz_genisligi/2, havuz_genisligi/2)
                z = random.uniform(-havuz_genisligi/2, havuz_genisligi/2)
                y = random.uniform(-90, 0)
                
                if engel_tipi == 'kaya':
                    s_x = random.uniform(15, 40)
                    s_y = random.uniform(15, 40)
                    s_z = random.uniform(15, 40)
                    engel_rengi = color.rgb(random.randint(80, 100), random.randint(80, 100), random.randint(80, 100))
                elif engel_tipi == 'agac':
                    s_x = random.uniform(5, 10)
                    s_y = random.uniform(20, 40)
                    s_z = random.uniform(5, 10)
                    engel_rengi = color.rgb(34, 139, 34)
                else:
                    s_x = random.uniform(10, 30)
                    s_y = random.uniform(10, 30)
                    s_z = random.uniform(10, 30)
                    engel_rengi = color.gray
                
                try:
                    if self.app is not None:
                        engel = Entity(
                            model='icosphere',
                            color=engel_rengi,
                            scale=(s_x, s_y, s_z),
                            position=(x, y, z),
                            rotation=(random.randint(0, 360), random.randint(0, 360), random.randint(0, 360)),
                            collider='mesh',
                            unlit=True
                        )
                        try:
                            engel.visible = False
                        except:
                            pass
                    else:
                        raise Exception("Ursina app yok")
                except Exception:
                    engel = type('Engel', (), {
                        'position': type('Vec3', (), {'x': x, 'y': y, 'z': z})(),
                        'scale_x': s_x,
                        'scale_y': s_y,
                        'scale_z': s_z,
                        'scale': (s_x, s_y, s_z)
                    })()
                
                self.ortam.engeller.append(engel)
            
            # Eksik ROV'ları oluştur
            while len(self.ortam.rovs) < n_rovs:
                i = len(self.ortam.rovs)
                if baslangic_pozisyonlari and i in baslangic_pozisyonlari:
                    pozisyon = baslangic_pozisyonlari[i]
                else:
                    x = random.uniform(-10, 10)
                    z = random.uniform(-10, 10)
                    pozisyon = (x, -2, z)
                
                try:
                    if self.app is not None:
                        rov = ROV(rov_id=i, position=pozisyon)
                        rov.environment_ref = self.ortam
                        try:
                            rov.visible = False
                        except:
                            pass
                    else:
                        raise Exception("Ursina app yok")
                except Exception:
                    pos_vec = type('Vec3', (), {'x': pozisyon[0], 'y': pozisyon[1] if len(pozisyon) > 1 else -2, 'z': pozisyon[2] if len(pozisyon) > 2 else 0})()
                    vel_vec = type('Vec3', (), {'x': 0, 'y': 0, 'z': 0})()
                    
                    def set_method(key, val):
                        if key == 'rol':
                            rov.role = int(val)
                    
                    def get_method(key):
                        if key == 'batarya':
                            return rov.battery
                        elif key == 'gps':
                            return np.array([rov.x, rov.y, rov.z])
                        elif key == 'hiz':
                            return np.array([0, 0, 0])
                        elif key == 'rol':
                            return rov.role
                        elif key == 'sonar':
                            return -1
                        return None
                    
                    def move_method(komut, guc=1.0):
                        pass
                    
                    rov = type('ROV', (), {
                        'id': i,
                        'position': pos_vec,
                        'velocity': vel_vec,
                        'battery': 1.0,
                        'role': 0,
                        'environment_ref': self.ortam,
                        'x': pozisyon[0],
                        'y': pozisyon[1] if len(pozisyon) > 1 else -2,
                        'z': pozisyon[2] if len(pozisyon) > 2 else 0,
                        'set': set_method,
                        'get': get_method,
                        'move': move_method
                    })()
                
                self.ortam.rovs.append(rov)
            
        # 5. Filo Kurulumu (sadece ilk kurulumda veya ROV sayısı değiştiğinde)
        if not hasattr(self, 'filo') or self.filo is None or len(self.ortam.rovs) != len(getattr(self.filo, 'sistemler', [])):
            self.filo = Filo()
            self.filo.otomatik_kurulum(
                rovs=self.ortam.rovs,
                lider_id=0,
                ortam_ref=self.ortam,
                modem_ayarlari=modem_ayarlari,
                baslangic_hedefleri={},  # Boş dict = formasyon hesaplaması yapılmasın
                sensor_ayarlari=sensor_ayarlari
            )
            self.ortam.filo = self.filo
            self.filo.ortam_ref = self.ortam  # Filo'ya ortam referansını ekle
        
        # 6. Aktif durumu
        self.aktif = True
        
        # 7. Başlangıçta bir kez dağıt (güvenli pozisyonlara yerleştir)
        self._nesneleri_yeniden_dagit()
        # Yeni ortam oluşturulduğunda her zaman göster (verbose kontrolü yok)
        print(f"✅ Yeni senaryo oluşturuldu: {n_rovs} ROV, {n_engels} Engel, Havuz: {havuz_genisligi}x{havuz_genisligi}")
        return self
    
    def guncelle(self, delta_time=0.016):
        """
        Senaryo ortamını bir adım günceller (simülasyon adımı).
        
        Args:
            delta_time (float): Geçen süre (saniye, varsayılan: 0.016 = ~60 FPS)
        
        Örnek:
            senaryo.guncelle(0.016)  # 1 frame güncelle
        """
        if not self.aktif:
            print("⚠️ Senaryo aktif değil. Önce senaryo.uret() çağırın.")
            return
        
        # Ursina time.dt'yi ayarla (eğer varsa)
        try:
            import time as ursina_time
            if hasattr(ursina_time, 'dt'):
                ursina_time.dt = delta_time
        except:
            pass
        
        # ROV'ları güncelle (sadece update metodu varsa)
        for rov in self.ortam.rovs:
            try:
                if hasattr(rov, 'update') and callable(getattr(rov, 'update', None)):
                    rov.update()
                else:
                    # Minimal ROV için basit fizik güncellemesi
                    if hasattr(rov, 'velocity'):
                        v = rov.velocity
                        # Hızı pozisyona uygula
                        if hasattr(v, 'x'):
                            # Pozisyonu güncelle (x, y, z attribute'ları üzerinden)
                            rov.x += v.x * delta_time
                            rov.y += v.y * delta_time
                            rov.z += v.z * delta_time
                            
                            # Position objesini de güncelle (varsa)
                            if hasattr(rov, 'position'):
                                if hasattr(rov.position, 'x'):
                                    rov.position.x = rov.x
                                    rov.position.y = rov.y
                                    rov.position.z = rov.z
                            
                            # Sürtünme (basit)
                            v.x *= 0.95
                            v.y *= 0.95
                            v.z *= 0.95
            except Exception as e:
                # Update hatası görmezden gel (headless mod)
                pass
        
        # Filo sistemini güncelle (GAT kodları olmadan, sadece fizik)
        if self.filo:
            # GAT kodları olmadan güncelle (varsayılan: 0 = OK)
            tahminler = np.zeros(len(self.ortam.rovs), dtype=int)
            self.filo.guncelle_hepsi(tahminler)
    
    def get(self, rov_id, veri_tipi):
        """
        ROV sensör verilerine erişim (Filo üzerinden).
        
        Args:
            rov_id (int): ROV ID'si
            veri_tipi (str): Veri tipi
                - "batarya": Batarya seviyesi (0-1)
                - "gps": GPS koordinatları [x, y, z]
                - "hiz": Hız vektörü [vx, vy, vz]
                - "sonar": Sonar mesafesi
                - "rol": ROV rolü (0=takipçi, 1=lider)
                - "engel_mesafesi": Engel tespit mesafesi
                - "iletisim_menzili": İletişim menzili
        
        Returns:
            Veri tipine göre değer veya None
        
        Örnek:
            batarya = senaryo.get(0, "batarya")
            gps = senaryo.get(0, "gps")
            sonar = senaryo.get(0, "sonar")
        """
        if not self.aktif:
            print("⚠️ Senaryo aktif değil. Önce senaryo.uret() çağırın.")
            return None
        
        if not self.filo:
            print("⚠️ Filo sistemi kurulmamış.")
            return None
        
        # Filo üzerinden veri al
        veri = self.filo.get(rov_id, veri_tipi)
        
        # Eğer filo None döndürdüyse, direkt ROV'tan al (fallback)
        if veri is None and rov_id < len(self.ortam.rovs):
            rov = self.ortam.rovs[rov_id]
            if hasattr(rov, 'get'):
                veri = rov.get(veri_tipi)
        
        return veri
    
    def set(self, rov_id, ayar_adi, deger):
        """
        ROV ayarlarını değiştirir.
        
        Args:
            rov_id (int): ROV ID'si
            ayar_adi (str): Ayar adı
            deger: Ayar değeri
        
        Örnek:
            senaryo.set(0, "engel_mesafesi", 25.0)
        """
        if not self.aktif:
            print("⚠️ Senaryo aktif değil.")
            return
        
        if self.filo:
            self.filo.set(rov_id, ayar_adi, deger)
    
    def git(self, rov_id, x, z, y=None, ai=True):
        """
        ROV'a hedef atar.
        
        Args:
            rov_id (int): ROV ID'si
            x (float): X koordinatı
            z (float): Z koordinatı
            y (float, optional): Y koordinatı (derinlik)
            ai (bool): AI aktif mi?
        
        Örnek:
            senaryo.git(0, 50, 60, -10)  # ROV-0'a hedef atar
        """
        if not self.aktif:
            print("⚠️ Senaryo aktif değil.")
            return
        
        if self.filo:
            self.filo.git(rov_id, x, z, y, ai)
    
    def temizle(self):
        """
        Senaryo ortamını temizler ve kaynakları serbest bırakır.
        """
        if self.ortam:
            # ROV'ları temizle
            for rov in self.ortam.rovs:
                if hasattr(rov, 'destroy'):
                    rov.destroy()
            self.ortam.rovs = []
            
            # Engelleri temizle
            for engel in self.ortam.engeller:
                if hasattr(engel, 'destroy'):
                    engel.destroy()
            self.ortam.engeller = []
        
        self.filo = None
        self.ortam = None
        self.aktif = False
        
        print("✅ Senaryo temizlendi")


# Global fonksiyonlar (kolay kullanım için)
def uret(n_rovs=3, n_engels=15, havuz_genisligi=200, verbose=False, **kwargs):
    """
    Senaryo oluşturur (global fonksiyon).
    
    Args:
        n_rovs (int): ROV sayısı
        n_engels (int): Engel sayısı
        havuz_genisligi (float): Havuz genişliği
        verbose (bool): Log mesajlarını göster (varsayılan: False)
        **kwargs: Diğer parametreler (engel_tipleri, baslangic_pozisyonlari, vb.)
    
    Returns:
        Senaryo: Senaryo instance'ı
    
    Örnek:
        senaryo.uret(n_rovs=4, n_engels=20)
    """
    global _senaryo_instance
    if _senaryo_instance is None:
        _senaryo_instance = Senaryo(verbose=verbose)
    return _senaryo_instance.uret(n_rovs=n_rovs, n_engels=n_engels, 
                                   havuz_genisligi=havuz_genisligi, verbose=verbose, **kwargs)


# Global instance (kolay erişim için)
def _get_instance():
    """Global senaryo instance'ını döndürür."""
    global _senaryo_instance
    if _senaryo_instance is None:
        _senaryo_instance = Senaryo()
    return _senaryo_instance


# Module-level functions (kolay erişim için)
def get(rov_id, veri_tipi):
    """ROV verisine erişim."""
    instance = _get_instance()
    return instance.get(rov_id, veri_tipi) if instance.aktif else None


def set(rov_id, ayar_adi, deger):
    """ROV ayarını değiştirir."""
    instance = _get_instance()
    if instance.aktif:
        instance.set(rov_id, ayar_adi, deger)


def git(rov_id, x, z, y=None, ai=True):
    """ROV'a hedef atar."""
    instance = _get_instance()
    if instance.aktif:
        instance.git(rov_id, x, z, y, ai)


def guncelle(delta_time=0.016):
    """Senaryo ortamını günceller."""
    instance = _get_instance()
    if instance.aktif:
        instance.guncelle(delta_time)


def temizle():
    """Senaryo ortamını temizler."""
    global _senaryo_instance
    if _senaryo_instance:
        _senaryo_instance.temizle()
        _senaryo_instance = None


# Module-level filo erişimi için __getattr__ kullan
def __getattr__(name):
    """Module-level attribute erişimi (senaryo.filo için)."""
    if name == 'filo':
        instance = _get_instance()
        return instance.filo if instance.aktif else None
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
