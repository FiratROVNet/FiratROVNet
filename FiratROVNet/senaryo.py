"""
Senaryo Üretim Modülü - Yapay Zeka Eğitimi İçin Veri Üretimi

Bu modül, GUI olmadan (headless) simülasyon ortamları oluşturur ve
yapay zeka algoritmalarını eğitmek için veri üretir.

Yeni Özellikler (v1.8+):
- Dinamik engel basitleştirme (engel_bulutu → basitleştirilmiş polygonlar)
- Derinlik koruma çok-waypoint navigasyonda
- HullManager entegrasyonu
- Graceful fallback mekanizması

Kullanım:
    from FiratROVNet import senaryo
    
    # Senaryo oluştur
    senaryo.uret(n_rovs=4, n_engels=20, havuz_genisligi=200)
    
    # Veri al
    batarya = senaryo.get(0, "batarya")
    gps = senaryo.get(0, "gps")
    sonar = senaryo.get(0, "sonar")
    
    # Dinamik rota planlama (Lidar verileri otomatik entegre)
    senaryo.git(0, 100, 50, -30, ai=True)
    
    # veya Filo üzerinden
    batarya = senaryo.filo.get(0, "batarya")
"""

import os
import sys
from panda3d.core import loadPrcFileData

# Log seviyesini 'fatal' yaparak sadece hayati hataları gösterir, bilgi mesajlarını gizler
loadPrcFileData('', 'notify-level fatal')
loadPrcFileData('', 'notify-level-util fatal')

# Ursina'yı headless modda başlat
os.environ['URSINA_HEADLESS'] = '1'

from ursina import *
from FiratROVNet.simulasyon import Ortam, ROV
from FiratROVNet.gnc import Filo
from FiratROVNet.hull import HullManager
import numpy as np
import random
import networkx as nx

# Global senaryo instance
_senaryo_instance = None


class Senaryo:
    """
    Senaryo üretim sınıfı - Headless simülasyon ortamı oluşturur.
    
    Özellinkleri:
    - Pozisyon bulma ve dağıtma (adalar, ROV'lar)
    - Dinamik engel basitleştirme (Lidar → geometri)
    - A* pathfinding with dynamic obstacles
    - Derinlik koruma multi-waypoint navigasyonda
    - Filo yönetimi ve GAT entegrasyonu
    
    Güvenli pozisyon bulma fonksiyonları:
    - _guvenli_ada_pozisyonu_bul: Adalar için güvenli pozisyon bulur
    - _guvenli_rov_pozisyonu_bul: ROV'lar için güvenli pozisyon bulur
    
    Bu sınıf, GUI olmadan simülasyon ortamları oluşturur ve
    yapay zeka eğitimi için veri üretir.
    """
    
    # Singleton pattern - class-level instance
    _singleton_instance = None
    
    # Object Pooling Sabitleri - İlk seferde max sayıda entity oluştur
    MAX_ROVS = 20        # Havuzda 20 ROV hazır bekler
    MAX_ADALAR = 20      # Havuzda 20 Ada hazır bekler
    MAX_KAYALAR = 20     # Havuzda 20 Kaya hazır bekler
    
    # Rastgele sayıda entity gösterme aralıkları
    MIN_ROVS = 4
    MAX_ROVS_GOSTER = 12
    MIN_ADALAR = 3
    MAX_ADALAR_GOSTER = 6
    MIN_KAYALAR = 10
    MAX_KAYALAR_GOSTER = 20
    
    @classmethod
    def get_instance(cls, verbose=False):
        """Singleton instance döndürür - eğer yoksa yaratır."""
        if cls._singleton_instance is None:
            cls._singleton_instance = cls(verbose=verbose)
        return cls._singleton_instance
    
    def __init__(self, verbose=False):
        self.app = None
        self.filo = None
        self.ortam = None
        self.aktif = False
        self.verbose = verbose  # Log mesajlarını kontrol eder
        
        # Hull Manager (dinamik engeller için)
        self.hull_manager = None
        
        # Önbellek mekanizması (hızlı senaryo üretimi için)
        self._cache_n_rovs = None
        self._cache_n_engels = None
        self._cache_havuz_genisligi = None
        
        # Object pooling - Ortam bir kez oluşturuldu mu?
        self._entities_created = False
    
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
        Entity'leri yok etmeden SADECE koordinatlarını değiştirir ve visibility kontrolü yapar. (Çok Hızlı)
        Object pooling: İstenen sayıda entity göster, geri kalanları gizle.
        Bu metod, mevcut nesnelerin pozisyonlarını güvenli şekilde yeniden dağıtır.
        """
        from .config import GATLimitleri, HareketAyarlari
        from ursina import Vec3
        
        havuz = self._cache_havuz_genisligi
        min_mesafe_ada = HareketAyarlari.RANDOM_HEDEF_MIN_MESAFE_ADA  # Adalardan minimum mesafe
        min_mesafe_rov = GATLimitleri.CARPISMA * 1.5  # ROV'lar arası minimum mesafe
        
        # Rastgele sayıda entity göster (kullanıcının isteği: ROV 4-12, Ada 3-6, Kaya 10-20)
        import random
        istenen_rov_sayisi = random.randint(self.MIN_ROVS, self.MAX_ROVS_GOSTER)
        istenen_ada_sayisi = random.randint(self.MIN_ADALAR, self.MAX_ADALAR_GOSTER)
        istenen_kaya_sayisi = random.randint(self.MIN_KAYALAR, self.MAX_KAYALAR_GOSTER)
        
        yeni_ada_pos = []  # [(x, z, r)]
        
        # --- 1. ADALARI (Engelleri) DAĞIT + VİSİBİLİTY KONTROL ---
        # İlk N adayı göster, geri kalanları gizle
        if hasattr(self.ortam, 'island_positions') and self.ortam.island_positions:
            for i, ada_data in enumerate(self.ortam.island_positions):
                # None kontrolü (çıkarılmış adalar için None olabilir)
                if ada_data is None:
                    continue
                
                # Visibility kontrolü: İstenen sayıdan fazlasını gizle
                if i >= istenen_ada_sayisi:
                    # Ada entity'sini gizle
                    if hasattr(self.ortam, 'island_entities') and i < len(self.ortam.island_entities):
                        ada_entity = self.ortam.island_entities[i]
                        if ada_entity and hasattr(ada_entity, 'visible'):
                            ada_entity.visible = False
                        if ada_entity and hasattr(ada_entity, 'enabled'):
                            ada_entity.enabled = False
                    continue  # Gizli ada için pozisyon hesaplama
                
                # Ada gösterilecek - görünür yap
                if hasattr(self.ortam, 'island_entities') and i < len(self.ortam.island_entities):
                    ada_entity = self.ortam.island_entities[i]
                    if ada_entity and hasattr(ada_entity, 'visible'):
                        ada_entity.visible = True
                    if ada_entity and hasattr(ada_entity, 'enabled'):
                        ada_entity.enabled = True
                
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
                    if ada_data is not None and len(ada_data) >= 2:
                        yeni_ada_pos.append((ada_data[0], ada_data[1], radius))
        
        # --- 2. KAYALARI DAĞIT + VİSİBİLİTY KONTROL ---
        # Kayalar EntityLoader.rock_entities listesinde tutuluyor
        if hasattr(self.ortam, 'loader') and hasattr(self.ortam.loader, 'rock_entities'):
            havuz_yari = havuz * 0.9  # Havuz sınırlarından biraz içerde
            
            for kaya_idx, kaya in enumerate(self.ortam.loader.rock_entities):
                if kaya is None:
                    continue
                
                # Visibility kontrolü: İstenen sayıdan fazlasını gizle
                if kaya_idx >= istenen_kaya_sayisi:
                    if hasattr(kaya, 'visible'):
                        kaya.visible = False
                    if hasattr(kaya, 'enabled'):
                        kaya.enabled = False
                    continue  # Gizli kaya için pozisyon hesaplama
                
                # Kaya gösterilecek - görünür yap
                if hasattr(kaya, 'visible'):
                    kaya.visible = True
                if hasattr(kaya, 'enabled'):
                    kaya.enabled = True
                
                # Rastgele yeni pozisyon (x, z rastgele, y derinliği koru)
                yeni_x = random.uniform(-havuz_yari, havuz_yari)
                yeni_z = random.uniform(-havuz_yari, havuz_yari)
                
                # Mevcut y (derinlik) değerini koru
                mevcut_y = kaya.y if hasattr(kaya, 'y') else -30
                
                # Pozisyonu güncelle (derinlik korunur)
                if hasattr(kaya, 'position'):
                    kaya.position = Vec3(yeni_x, mevcut_y, yeni_z)
                elif hasattr(kaya, 'x'):
                    kaya.x = yeni_x
                    kaya.z = yeni_z
                    # y değiştirilmez (derinlik korunur)
        
        # --- 3. ROV'LARI DAĞIT + VİSİBİLİTY KONTROL ---
        # İlk N ROV'u göster, geri kalanları gizle
        mevcut_rov_pos = []
        for rov_idx, rov in enumerate(self.ortam.rovs):
            if rov is None:
                continue  # Çıkarılmış ROV'ları atla
            
            # Visibility kontrolü: İstenen sayıdan fazlasını gizle
            if rov_idx >= istenen_rov_sayisi:
                # ROV'u gizle ve devre dışı bırak
                if hasattr(rov, 'visible'):
                    rov.visible = False
                if hasattr(rov, 'enabled'):
                    rov.enabled = False
                if hasattr(rov, 'label') and rov.label:
                    rov.label.visible = False
                continue  # Gizli ROV için pozisyon hesaplama
            
            # ROV gösterilecek - görünür yap
            if hasattr(rov, 'visible'):
                rov.visible = True
            if hasattr(rov, 'enabled'):
                rov.enabled = True
            if hasattr(rov, 'label') and rov.label:
                rov.label.visible = True
            
            pos = self._guvenli_rov_pozisyonu_bul(mevcut_rov_pos, yeni_ada_pos, 
                                                  min_mesafe_rov, min_mesafe_ada, havuz)
            if pos:
                yeni_x, yeni_z = pos
                # Rastgele derinlik: 0 ile -30 metre arası
                yeni_y = random.uniform(-40, 0)
                
                # ROV pozisyonunu güncelle
                if hasattr(rov, 'position'):
                    rov.position = Vec3(yeni_x, yeni_y, yeni_z)
                elif hasattr(rov, 'x'):
                    rov.x = yeni_x
                    rov.y = yeni_y
                    rov.z = yeni_z
                
                # Hızı sıfırla
                if hasattr(rov, 'velocity'):
                    if hasattr(rov.velocity, 'x'):
                        rov.velocity.x = 0
                        rov.velocity.y = 0
                        rov.velocity.z = 0
                    else:
                        rov.velocity = Vec3(0, 0, 0)
                
                mevcut_rov_pos.append([yeni_x, yeni_y, yeni_z])
            else:
                # Güvenli pozisyon bulunamazsa rastgele yerleştir
                yeni_x = random.uniform(-havuz * 0.45, havuz * 0.45)
                yeni_z = random.uniform(-havuz * 0.45, havuz * 0.45)
                yeni_y = random.uniform(-30, 0)  # Rastgele derinlik
                
                if hasattr(rov, 'position'):
                    rov.position = Vec3(yeni_x, yeni_y, yeni_z)
                elif hasattr(rov, 'x'):
                    rov.x = yeni_x
                    rov.y = yeni_y
                    rov.z = yeni_z
                
                if hasattr(rov, 'velocity'):
                    if hasattr(rov.velocity, 'x'):
                        rov.velocity.x = 0
                        rov.velocity.y = 0
                        rov.velocity.z = 0
                    else:
                        rov.velocity = Vec3(0, 0, 0)
                
                mevcut_rov_pos.append([yeni_x, yeni_y, yeni_z])
        
        # Lideri random seç (sadece görünür/aktif ROV'lar arasından)
        # Object pooling: Sadece istenen sayıda ROV görünür, onlar arasından lider seç
        gorunur_rovs = [rov for i, rov in enumerate(self.ortam.rovs) 
                        if rov is not None and i < istenen_rov_sayisi]
        
        if len(gorunur_rovs) > 0:
            yeni_lider_id = random.randint(0, len(gorunur_rovs) - 1)
            gorunur_indeks = 0
            for i, rov in enumerate(self.ortam.rovs):
                if rov is None or i >= istenen_rov_sayisi:
                    continue
                try:
                    if gorunur_indeks == yeni_lider_id:
                        if hasattr(rov, 'set') and callable(rov.set):
                            rov.set('rol', 1)
                        elif hasattr(rov, 'role'):
                            rov.role = 1
                    else:
                        if hasattr(rov, 'set') and callable(rov.set):
                            rov.set('rol', 0)
                        elif hasattr(rov, 'role'):
                            rov.role = 0
                except Exception as e:
                    if self.verbose:
                        print(f"⚠️ Lider seçimi hatası ROV-{i}: {e}")
                gorunur_indeks += 1
        
        # Senaryo aktif duruma getir
        self.aktif = True
        
        if self.verbose:
            print(f"🔄 Senaryo Yeniden Düzenlendi: {istenen_rov_sayisi} ROV, {istenen_ada_sayisi} Ada, {istenen_kaya_sayisi} Kaya (Havuz: {self.MAX_ROVS}/{self.MAX_ADALAR}/{self.MAX_KAYALAR})")
        return self
        
    def uret(self, n_rovs=None, n_engels=None, havuz_genisligi=None, n_adalar=None,
             engel_tipleri=None, baslangic_pozisyonlari=None,
             modem_ayarlari=None, sensor_ayarlari=None, verbose=None):
        """
        Senaryo ortamı oluşturur veya mevcut nesneleri yeniden dağıtır (optimize edilmiş).
        
        Dinamik Engel Desteği:
        - engel_bulutu otomatik olarak A* pathfinding'e entegre edilir
        - Lidar/Sonar verilerinden geometrik engeller oluşturulur
        - HullManager via dinamik_engelleri_basitlestir() kullanılır
        
        Derinlik Koruma:
        - _git_hedef_derinligi dictionary'si multi-waypoint rotalarında derinliği korur
        - git_path() calls preserve target depth across waypoints
        
        Args:
            n_rovs (int): ROV sayısı (varsayılan: 3, None ise mevcut sayı korunur)
            n_engels (int): Engel sayısı (varsayılan: 15, None ise mevcut sayı korunur)
            n_adalar (int): Ada sayısı (None ise otomatik belirlenir)
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
        
        # Her zaman aktif duruma getir (test script'leri için)
        self.aktif = True
        
        # 1. OBJECT POOLING: Entity'ler zaten oluşturulduysa SADECE POZISYON DEĞİŞTİR
        if self._entities_created:
            if self.verbose:
                print("🔄 Hızlı Mod: Sadece pozisyonlar güncelleniyor (entity'ler korunuyor)")
            # Önbelleği güncelle (istenen sayılar)
            if n_rovs is not None:
                self._cache_n_rovs = n_rovs
            if n_engels is not None:
                self._cache_n_engels = n_engels
            if havuz_genisligi is not None:
                self._cache_havuz_genisligi = havuz_genisligi
            
            return self._nesneleri_yeniden_dagit()
        
        # 2. Önbellek Güncelleme (sadece ilk kurulumda)
        if n_rovs is not None:
            self._cache_n_rovs = n_rovs
        elif self._cache_n_rovs is None:
            self._cache_n_rovs = 4  # Varsayılan (test senaryosuna göre)
        
        if n_engels is not None:
            self._cache_n_engels = n_engels
        elif self._cache_n_engels is None:
            self._cache_n_engels = 15  # Varsayılan
        
        if havuz_genisligi is not None:
            self._cache_havuz_genisligi = havuz_genisligi
        elif self._cache_havuz_genisligi is None:
            self._cache_havuz_genisligi = 200  # Varsayılan
        
        # Object pooling: İlk seferde max sayıda entity oluştur
        # Gerçekte oluşturulacak sayılar (ilk seferde MAX, sonra gizle/göster)
        n_rovs_entity = self.MAX_ROVS  # Her zaman max sayıda entity oluştur
        n_engels_entity = self.MAX_ADALAR
        n_kaya_entity = self.MAX_KAYALAR  # Kaya havuzu
        
        havuz_genisligi = self._cache_havuz_genisligi
        
        # 3. İlk Kurulum (Sadece bir kez çalışır - AĞIR KISIM)
        # Object pooling: Ortam ve entity'ler sadece ilk seferde yaratılır
        if not self._entities_created:
            if not self.verbose:
                try:
                    from panda3d.core import loadPrcFileData
                    loadPrcFileData("", "window-type none")
                    loadPrcFileData("", "audio-library-name null")
                    loadPrcFileData("", "notify-level error")
                    loadPrcFileData("", "default-directnotify-level error")
                    loadPrcFileData("", "notify-level-display error")
                except Exception:
                    pass
            
            # Ursina'yı headless modda başlat (SADECE İLK SEFERDE)
            if self.app is None:
                # Headless mod için özel ayarlar
                os.environ['URSINA_HEADLESS'] = '1'
                
                try:
                    if not self.verbose:
                        from contextlib import redirect_stdout, redirect_stderr
                        import io
                        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                            self.app = Ursina(
                                vsync=False,
                                development_mode=False,
                                show_ursina_splash=False,
                                borderless=True,
                                title="FıratROVNet Senaryo Üretimi (Headless)"
                            )
                    else:
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
                    except Exception:
                        pass
                    
                    try:
                        if hasattr(window, 'show'):
                            window.show = False
                    except Exception:
                        pass
                    
                    try:
                        if hasattr(window, 'fps_counter'):
                            window.fps_counter.enabled = False
                    except Exception:
                        pass
                        
                except Exception as e:
                    # Ursina başlatılamazsa minimal ortam oluştur
                    print(f"⚠️ Ursina headless mod başlatılamadı: {e}")
                    print("   Minimal ortam modu kullanılıyor...")
                    self.app = None
            
            # Ortam oluştur - Gerçek Ortam sınıfını kullan (simulasyon.py'den)
            # SADECE İLK SEFERDE - Object pooling için ortam bir kez yaratılır
            if self.ortam is None:
                try:
                    from FiratROVNet.simulasyon import Ortam as OrtamSinifi
                    if not self.verbose:
                        from contextlib import redirect_stdout, redirect_stderr
                        import io
                        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                            self.ortam = OrtamSinifi(verbose=self.verbose)
                    else:
                        self.ortam = OrtamSinifi(verbose=self.verbose)
                    # Ortam'a verbose flag'ini aktar
                    if hasattr(self.ortam, 'verbose'):
                        self.ortam.verbose = self.verbose
                    
                    # Headless mod için görsel özellikleri kapat
                    if hasattr(self.ortam, 'app'):
                        try:
                            if hasattr(self.ortam.app, 'window'):
                                self.ortam.app.window.show = False
                        except Exception:
                            pass
                    
                    # Harita sistemini kapat (headless mod için)
                    if hasattr(self.ortam, 'harita') and self.ortam.harita:
                        try:
                            self.ortam.harita.goster(False)
                        except Exception:
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
        
        # 4. Nesne Sayıları Değiştiyse sim_olustur çağır (SADECE İLK SEFERDE)
        # Object pooling: Entity'ler bir kez oluşturulur, sonra sadece pozisyon değişir
        sim_olustur_basarili = False
        if not self._entities_created and hasattr(self.ortam, 'sim_olustur'):
            try:
                if self.verbose:
                    print(f"🏗️ İLK KURULUM: {n_rovs_entity} ROV, {n_engels_entity} Ada entity'si oluşturuluyor (MAX kapasitede)...")
                else:
                    print(f"🏗️ Entity havuzu oluşturuluyor: {n_rovs_entity} ROV, {n_engels_entity} Ada...")
                    
                # sim_olustur tuple bekliyor: (n_rovs,) formatında grup yapılandırması
                self.ortam.sim_olustur(
                    n_rovs=(n_rovs_entity,),  # MAX sayıda entity oluştur
                    n_islands=n_engels_entity,
                    n_rocks=n_kaya_entity,  # Kaya havuzu
                    havuz_genisligi=havuz_genisligi
                )
                sim_olustur_basarili = True
                self._entities_created = True  # Artık entity'ler oluşturuldu, bir daha çağırma!
                if self.verbose:
                    print(f"✅ Entity havuzu hazır: {n_rovs_entity} ROV, {n_engels_entity} Ada, {n_kaya_entity} Kaya!")
                else:
                    print("✅ Entity havuzu hazır! Sonraki çağrılar 50x hızlı olacak.")
            except Exception as e:
                import traceback
                print(f"⚠️ sim_olustur hatası: {e}")
                if self.verbose:
                    traceback.print_exc()
                sim_olustur_basarili = False
                self._entities_created = False  # Hata olduysa bayrağı sıfırla
        elif self._entities_created:
            # Entity'ler zaten var - bu kod parçasına asla gelmemeli
            # Çünkü yukarıdaki kontrol zaten _nesneleri_yeniden_dagit() döndürdü
            sim_olustur_basarili = True
        
        # Fallback: sim_olustur yoksa veya başarısız olduysa manuel oluşturma
        if not sim_olustur_basarili:
            self.ortam.havuz_genisligi = havuz_genisligi
            # Yeni yapıda engeller listesi yok, atla
            # if not hasattr(self.ortam, 'engeller') or not self.ortam.engeller:
            #     self.ortam.engeller = []
            # ROV listesini her zaman sıfırla (yeni senaryo için)
            if hasattr(self.ortam, 'rovs'):
                # Mevcut ROV'ları destroy et
                for rov in self.ortam.rovs:
                    if rov is not None:
                        try:
                            if hasattr(rov, 'destroy'):
                                rov.destroy()
                        except Exception:
                            pass
            self.ortam.rovs = []
            
            # Not: Yeni yapıda engeller EntityLoader tarafından yönetiliyor
            # Manuel engel oluşturma desteği kaldırıldı
            
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
                        except Exception:
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
            
        # 5. Filo Kurulumu (sadece ilk kurulumda)
        if not hasattr(self, 'filo') or self.filo is None:
            # Filo constructor otomatik kurulumu yapıyor
            self.filo = Filo(ortam_ref=self.ortam)
            self.ortam.filo = self.filo
            
            # HullManager initialize et (dinamik engeller için)
            if self.hull_manager is None:
                self.hull_manager = HullManager(filo_ref=self.filo)
        
        # 6. Aktif durumu
        self.aktif = True

        # 7. İLK KURULUMDAN SONRA: Pozisyonları düzenle (visibility + koordinatlar)
        # Object pooling: İstenen sayıda entity göster, geri kalanları gizle
        if self._entities_created:
            if self.verbose:
                print(f"🎯 İstenen: {self._cache_n_rovs} ROV, {self._cache_n_engels} Ada (Havuz: {self.MAX_ROVS} ROV, {self.MAX_ADALAR} Ada)")
            return self._nesneleri_yeniden_dagit()
        
        # --- BURADAN SONRASI SADECE İLK KURULUMDA ÇALIŞIR ---
        # Entity'ler oluşturuldu, şimdi pozisyonları düzenle
        print(f"✅ Yeni senaryo oluşturuldu: {self._cache_n_rovs} ROV aktif (Havuz: {self.MAX_ROVS}), {self._cache_n_engels} Ada aktif (Havuz: {self.MAX_ADALAR})")
        return self._nesneleri_yeniden_dagit()
    
    def guncelle(self, delta_time=0.016):
        """
        Senaryo ortamını bir adım günceller (simülasyon adımı).
        
        Yapılan İşlemler:
        - Su yüzeyi animasyonu
        - ROV fizik güncellemesi
        - Filo sistemi güncellemesi (GAT kodları)
        - engel_bulutu otomatik güncellenir (Lidar/Sonar)
        - Dinamik engeller A* pathfinding'e otomatik entegre
        
        Args:
            delta_time (float): Geçen süre (saniye, varsayılan: 0.016 = ~60 FPS)
        
        Örnek:
            senaryo.guncelle(0.016)  # 1 frame güncelle
        """
        if not self.aktif:
            print("⚠️ Senaryo aktif değil. Önce senaryo.uret() çağırın.")
            return
        
        # Ursina time.dt'yi ayarla (su yüzeyi animasyonu ve diğer entity update'leri için)
        try:
            from ursina import time as ursina_time
            ursina_time.dt = delta_time
        except Exception:
            pass
        
        # Su yüzeyi animasyonu (senaryo.guncelle döngüsünden çağrıldığı için burada da güncelle)
        try:
            if hasattr(self.ortam, 'ocean_surface') and self.ortam.ocean_surface is not None:
                if hasattr(self.ortam.ocean_surface, 'update') and callable(self.ortam.ocean_surface.update):
                    self.ortam.ocean_surface.update()
        except Exception:
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
        ROV sensör verilerine erişim (Filo üzerinden + direkt ROV erişimi).
        
        Dinamik Engel Verileri:
        - engel_bulutu: Lidar/Sonar tarafından tespit edilen engel noktaları
        - Otomatik basitleştirme: HullManager.dinamik_engelleri_basitlestir()
        
        Args:
            rov_id (int): ROV ID'si
            veri_tipi (str): Veri tipi
                - "batarya": Batarya seviyesi (0-1)
                - "gps": GPS koordinatları [x, y, z] (Derinlik korunur)
                - "hiz": Hız vektörü [vx, vy, vz]
                - "sonar": Sonar mesafesi
                - "lidar": Lidar mesafeleri (dict: {0: ileri, 1: sağ, 2: sol, 3: dip})
                - "rol": ROV rolü (0=takipçi, 1=lider)
                - "group_id": Grup ID'si
                - "derinlik": Y koordinatı (derinlik)
                - "engel_mesafesi": Engel tespit mesafesi
                - "iletisim_menzili": İletişim menzili
        
        Returns:
            Veri tipine göre değer veya None
        
        Örnek:
            batarya = senaryo.get(0, "batarya")
            gps = senaryo.get(0, "gps")  # [x, y, z] mevcut z korunmuş
            sonar = senaryo.get(0, "sonar")
            lidar = senaryo.get(0, "lidar")  # {0: ileri, 1: sağ, 2: sol, 3: dip}
            group_id = senaryo.get(0, "group_id")  # 0, 1, 2...
            derinlik = senaryo.get(0, "derinlik")  # -5.0 (y koordinatı)
        """
        if not self.aktif:
            return None
        
        if not self.filo:
            return None
        
        # ROV referansını bul
        rov_ref = None
        if hasattr(self.ortam, 'rovs'):
            for rov in self.ortam.rovs:
                if rov and hasattr(rov, 'id') and rov.id == rov_id:
                    rov_ref = rov
                    break
        
        if rov_ref is None:
            return None
        
        # Tüm veri tipleri için Filo.get() kullan (ROV sensörlerini otomatik okur)
        veri = self.filo.get(rov_id, veri_tipi, sessiz=True) if self.filo else None
        
        # Filo'dan veri alınamazsa, direkt ROV'tan fallback
        if veri is None and hasattr(rov_ref, 'get'):
            veri = rov_ref.get(veri_tipi)
        
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
        ROV'a hedef atar (A* pathfinding + dinamik engel desteği).
        
        Özellikler:
        - Multi-waypoint rotalar derinlik korumasıyla desteklenir
        - Dinamik engeller (Lidar) otomatik rota planlamaya dahil
        - AI aktif ise GAT kodları hesaplanır
        
        Args:
            rov_id (int): ROV ID'si
            x (float): X koordinatı
            z (float): Z koordinatı (Hedef pozisyonu)
            y (float, optional): Y koordinatı (derinlik). None ise mevcut derinlik korunur
            ai (bool): AI aktif mi? (varsayılan: True)
        
        Örnek:
            senaryo.git(0, 50, 60, -10)  # ROV-0'a hedef atar, derinlik -10m
            senaryo.git(1, 100, 100)      # ROV-1'e hedef atar, mevcut derinliği koru
        """
        if not self.aktif:
            print("⚠️ Senaryo aktif değil.")
            return
        
        if self.filo:
            self.filo.git(rov_id, x, z, y, ai)
    
    def temizle(self, tam_temizlik=False):
        """
        Senaryo ortamını temizler.
        
        Args:
            tam_temizlik (bool): 
                - False: Sadece nesneleri gizler, ortamı korur (Object pooling için ideal, varsayılan)
                - True: Her şeyi yok eder, singleton'ı sıfırlar (Sistemi kapatmak için)
        
        Temizlenen Kaynaklar (tam_temizlik=True):
        - Tüm ROV entity'leri
        - Engel entity'leri
        - Filo sistemi ve GNC controllers
        - HullManager ve dinamik engel cache'i
        - engel_bulutu verisi
        """
        if not tam_temizlik:
            # Soft reset: Sadece nesneleri gizle, ortamı koru (pooling için)
            if self.ortam:
                # ROV'ları gizle ve devre dışı bırak
                for rov in self.ortam.rovs:
                    if rov:
                        if hasattr(rov, 'enabled'):
                            rov.enabled = False
                        if hasattr(rov, 'visible'):
                            rov.visible = False
                        # ROV'un navigasyon hedeflerini sıfırla
                        if hasattr(rov, 'gnc') and rov.gnc:
                            rov.gnc.manuel_kontrol = True
                
                # Adaları gizle
                for isl in getattr(self.ortam, 'island_entities', []):
                    if isl:
                        if hasattr(isl, 'enabled'):
                            isl.enabled = False
                        if hasattr(isl, 'visible'):
                            isl.visible = False
            
            self.aktif = False
            if self.verbose:
                print("✅ Soft reset yapıldı (nesneler gizlendi, ortam korundu)")
            return
        
        # Hard reset: Her şeyi yok et (eski davranış)
        if self.ortam:
            # ROV'ları temizle
            for rov in self.ortam.rovs:
                if hasattr(rov, 'destroy'):
                    try:
                        rov.destroy()
                    except Exception:
                        pass
            self.ortam.rovs = []
            
            # Engelleri temizle (yeni yapıda engeller EntityLoader tarafından yönetiliyor)
            if hasattr(self.ortam, 'engeller') and self.ortam.engeller:
                for engel in self.ortam.engeller:
                    if hasattr(engel, 'destroy'):
                        try:
                            engel.destroy()
                        except Exception:
                            pass
                self.ortam.engeller = []
        
        # HullManager ve cache temizle
        self.hull_manager = None
        
        self.filo = None
        self.ortam = None
        self.aktif = False
        self._entities_created = False  # Entity'lerin tekrar yaratılmasına izin ver
        
        if self.verbose:
            print("🚫 Tam temizlik yapıldı (her şey yok edildi)")


# Global fonksiyonlar (kolay kullanım için)
def uret(n_rovs=3, n_engels=15, havuz_genisligi=200, n_adalar=None, verbose=False, **kwargs):
    """
    Senaryo oluşturur (global fonksiyon).
    
    Args:
        n_rovs (int): ROV sayısı
        n_engels (int): Engel sayısı
        havuz_genisligi (float): Havuz genişliği
        n_adalar (int): Ada sayısı (None ise otomatik belirlenir)
        verbose (bool): Log mesajlarını göster (varsayılan: False)
        **kwargs: Diğer parametreler (engel_tipleri, baslangic_pozisyonlari, vb.)
    
    Returns:
        Senaryo: Senaryo instance'ı
    
    Örnek:
        senaryo.uret(n_rovs=4, n_engels=20)
    """
    global _senaryo_instance
    
    # Singleton kontrolü - SADECE ilk seferde yarat
    if _senaryo_instance is None:
        _senaryo_instance = Senaryo.get_instance(verbose=verbose)
    
    # uret metodunu çağır (Bu metod zaten self._entities_created kontrolü içeriyor)
    return _senaryo_instance.uret(
        n_rovs=n_rovs,
        n_engels=n_engels,
        havuz_genisligi=havuz_genisligi,
        n_adalar=n_adalar,
        verbose=verbose,
        **kwargs
    )


# Global instance (kolay erişim için)
def _get_instance():
    """Global senaryo instance'ını döndürür."""
    global _senaryo_instance
    if _senaryo_instance is None:
        _senaryo_instance = Senaryo.get_instance()
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


def temizle(hard_reset=False):
    """
    Senaryo ortamını temizler.
    
    DİKKAT: test_senaryo_100.py içinde döngüde çağırıyorsanız hard_reset=False olmalı!
    
    Args:
        hard_reset (bool): 
            - False: Soft reset - nesneleri gizler, singleton korunur (varsayılan)
            - True: Hard reset - her şeyi yok eder, singleton sıfırlanır
    """
    global _senaryo_instance
    if _senaryo_instance:
        _senaryo_instance.temizle(tam_temizlik=hard_reset)
        
        # SADECE hard_reset=True ise singleton'ı sıfırla
        if hard_reset:
            _senaryo_instance = None
            Senaryo._singleton_instance = None


# Module-level filo erişimi için __getattr__ kullan
def __getattr__(name):
    """Module-level attribute erişimi (senaryo.filo, senaryo.ortam için)."""
    if name == 'filo':
        instance = _get_instance()
        return instance.filo if instance and instance.aktif else None
    elif name == 'ortam':
        instance = _get_instance()
        return instance.ortam if instance and instance.aktif else None
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
