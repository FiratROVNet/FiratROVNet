import sys
import matplotlib
# Matplotlib Backend Ayarı (Kritik: Diğer importlardan önce olmalı)
# TkAgg, Python thread'leri ile en uyumlu çalışan backend'dir
# Hem Windows hem Linux'ta çökme riskini en aza indirir
try:
    matplotlib.use('TkAgg', force=True)  # force=True ile kesin ayarla
except Exception as e:
    print(f"⚠️ [HARITA] Backend ayarlanamadı: {e}")
    pass  # Fallback için devam et

from panda3d.core import loadPrcFileData

# Log seviyesini 'fatal' yaparak sadece hayati hataları gösterir, bilgi mesajlarını gizler
loadPrcFileData('', 'notify-level fatal')
loadPrcFileData('', 'notify-level-util fatal')

from ursina import *
from ursina import Vec3, window, application  # Vec3, window, application doğrudan import
import numpy as np
import random
import threading
import code
import torch
from math import sin, cos, atan2, degrees, radians, pi
import os
from typing import Tuple, List, Optional

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import queue

# Global Interactive Mode - Bir kez aç (thread-safe)
try:
    plt.ion()
except Exception:
    pass  # Zaten açıksa devam et
    
from .config import (
    cfg,
    SensorAyarlari,
    GATLimitleri,
    HareketAyarlari,
    FizikSabitleri,
    SimulasyonSabitleri
)
from .simulasyon_yardimci import (
    kayalari_olustur,
    sim_to_ursina,
    ursina_to_sim
)



class ROV(Entity):
    def __init__(self, rov_id, **kwargs):
        super().__init__()
        
        # FBX model kontrolü (Windows uyumlu path - geliştirilmiş)
        # Models-3D klasörü proje kök dizininde (CWD'de)
        # Ursina relative path'i tercih eder, bu yüzden önce relative path dene
        
        # 1. Relative path (CWD'den)
        rov_model_path_rel = "Models-3D/water/my_models/submarine/submarine1.fbx"
        
        # 2. Absolute path (script'in bulunduğu yerden)
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        rov_model_path_abs = os.path.join(script_dir, "Models-3D", "water", "my_models", "submarine", "submarine1.fbx")
        
        # 3. CWD'den absolute path (Windows için ek kontrol)
        cwd = os.getcwd()
        rov_model_path_cwd_abs = os.path.join(cwd, "Models-3D", "water", "my_models", "submarine", "submarine1.fbx")
        
        # Path seçimi: Önce relative path'i kontrol et (Ursina için tercih edilen)
        rov_model_path = None
        model_found = False
        
        # Kontrol sırası: relative -> script_abs -> cwd_abs
        if os.path.exists(rov_model_path_rel):
            rov_model_path = rov_model_path_rel
            model_found = True
        elif os.path.exists(rov_model_path_abs):
            # Absolute path varsa, CWD'ye göre relative path'e çevir
            try:
                rov_model_path = os.path.relpath(rov_model_path_abs, cwd)
                # Eğer relative path oluşturulamazsa veya dosya yoksa, absolute kullan
                if not os.path.exists(rov_model_path):
                    rov_model_path = rov_model_path_abs
                model_found = True
            except (ValueError, OSError):
                # Relative path oluşturulamazsa absolute kullan
                rov_model_path = rov_model_path_abs
                model_found = True
        elif os.path.exists(rov_model_path_cwd_abs):
            # CWD'den absolute path varsa kullan
            try:
                rov_model_path = os.path.relpath(rov_model_path_cwd_abs, cwd)
                if not os.path.exists(rov_model_path):
                    rov_model_path = rov_model_path_cwd_abs
                model_found = True
            except (ValueError, OSError):
                rov_model_path = rov_model_path_cwd_abs
                model_found = True
        
        # Hiçbiri bulunamazsa relative path'i kullan (Ursina kendi path çözümlemesini yapacak)
        if not model_found:
            rov_model_path = rov_model_path_rel
        
        # Ursina için path'i normalize et (forward slash kullan)
        if os.path.sep == '\\':  # Windows
            rov_model_path = rov_model_path.replace('\\', '/')
        
        # Model dosyası kontrolü (en az bir yerde varsa kullan)
        model_exists = os.path.exists(rov_model_path) if rov_model_path else False
        
        # Windows için ek path kontrolü: normalize edilmiş path'leri de kontrol et
        if not model_exists and os.path.sep == '\\':  # Windows
            # Windows'ta path'leri normalize et ve tekrar kontrol et
            normalized_paths = [
                rov_model_path_rel.replace('/', '\\'),
                rov_model_path_rel.replace('\\', '/'),
                rov_model_path_abs.replace('/', '\\'),
                rov_model_path_abs.replace('\\', '/'),
                rov_model_path_cwd_abs.replace('/', '\\'),
                rov_model_path_cwd_abs.replace('\\', '/'),
            ]
            for norm_path in normalized_paths:
                if os.path.exists(norm_path):
                    rov_model_path = norm_path.replace('\\', '/')  # Ursina için forward slash
                    model_exists = True
                    break
        
        if model_exists or os.path.exists(rov_model_path_rel) or os.path.exists(rov_model_path_abs) or os.path.exists(rov_model_path_cwd_abs):
            # FBX model kullan - Model çok büyük olduğu için yaklaşık 1000 kat küçültülüyor
            # Ursina için path'i normalize et (forward slash kullan)
            if os.path.sep == '\\':  # Windows
                rov_model_path = rov_model_path.replace('\\', '/')
            
            # Model dosyasının gerçekten var olduğundan emin ol (Windows için ek kontrol)
            final_model_path = rov_model_path
            if not os.path.exists(final_model_path):
                # Windows'ta backslash ile de dene
                final_model_path_win = final_model_path.replace('/', '\\')
                if os.path.exists(final_model_path_win):
                    final_model_path = final_model_path_win.replace('\\', '/')  # Ursina için forward slash
                else:
                    # Model bulunamadı, fallback kullan (sessizce)
                    self.model = 'cube'
                    self.scale = (1.5, 0.8, 2.5)
                    self.color = color.orange
                    self.collider = 'box'
                    self.unlit = True
                    self.gat_kodu = 0
                    return  # Erken çıkış, aşağıdaki FBX ayarlarını atla
            
            self.model = final_model_path
            self.scale = (0.01, 0.01, 0.01)  # FBX model için çok küçük scale (1000 kat küçültme)
            # Mesh collider intersects() ile çalışmaz, bu yüzden box collider kullanıyoruz
            # Görsel model mesh, ama çarpışma kontrolü için box kullanılıyor
            self.collider = 'box'  # Primitive collider (intersects() için gerekli)
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
        
        # Rotation'ı başlangıçta ayarla (yaw rotasyonu için)
        self.rotation = Vec3(0, 0, 0) 
        
        # Sensör ayarları config.py'den alınır (GAT limitleri ile tutarlı)
        from .config import SensorAyarlari
        self.sensor_config = SensorAyarlari.VARSAYILAN.copy()
        self.environment_ref = None
        
        # --- GÜVENLİK ALANI (Trigger/Overlap) ---
        # ROV'un etrafında görünmez bir küre: "Yakınlık Sensörü"
        # Bu alan içindeki objeleri tespit etmek için kullanılır
        # NOT: collider=None yapıldı - intersects() çarpışma sorunlarını önlemek için
        safety_zone_radius = self.sensor_config.get("engel_mesafesi", 20.0) / 2.0  # Yarıçap = menzil / 2
        self.safety_zone = Entity(
            parent=self,
            model='sphere',
            scale=safety_zone_radius * 2,  # Çap = yarıçap * 2
            collider=None,  # Collider kaldırıldı - çarpışma sorunlarını önlemek için
            color=color.rgba(255, 0, 0, 50),  # Debug için hafif kırmızı (görünür değil)
            visible=True,  # Normalde kapalı
            unlit=True
        )
        
        # --- SENSÖR CACHE (Thread-Safe) ---
        # Fiziksel raycast işlemleri sadece Ana Thread'de (update içinde) yapılır
        # Konsol thread'i sadece bu cache'lenmiş değerleri okur
        self.son_sonar_mesafesi = -1  # Sonar mesafesi cache
        self.son_lidar_mesafeleri = {0: -1, 1: -1, 2: -1}  # Lidar mesafeleri cache (ön, sağ, sol)
        
        # Manuel hareket kontrolü (sürekli hareket için)
        self.manuel_hareket = {
            'yon': None,  # 'ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur', 'yaw'
            'guc': 0.0    # 0.0 - 1.0 arası güç
        }
        # Eşzamanlı hareket: her eksendeki güç (sadece ilgili eksen move() ile güncellenir)
        # surge: ileri/geri, sway: sağ/sol, heave: çık/bat, yaw: dönüş hızı (-1..1)
        self.active_forces = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0, 'yaw': 0.0}
        
        # Engel tespit bilgisi (kesikli çizgi için)
        self.tespit_edilen_engel = None  # En yakın engel referansı
        self.engel_mesafesi = 999.0  # En yakın engel mesafesi
        self.engel_cizgi = None  # Kesikli çizgi entity'si
        
        # Sonar iletişim bilgisi (ROV'lar arası kesikli çizgi için)
        self.iletisim_rovlari = {}  # {rov_id: {'mesafe': float, 'cizgi': Entity, 'yuzey_iletisimi': bool}}
        
        # İletişim durumu (liderle iletişim var mı?)
        self.lider_ile_iletisim = False  # Liderle iletişim durumu
        self.yuzeyde = False  # Yüzeyde mi? (z_depth >= 0, yani derinlik pozitif) 

    def ekle(self, konum=None):
        """
        ROV'u simülasyona ekler (eğer henüz eklenmemişse).
        
        Args:
            konum: (x, y, z) tuple veya None (varsayılan pozisyon kullanılır)
        
        Returns:
            bool: İşlem başarılı ise True, aksi halde False
        
        Örnek:
            rov = ROV(5, position=(10, -5, 20))
            rov.ekle()  # Varsayılan pozisyonda ekle
            rov.ekle((50, -10, 30))  # Belirtilen pozisyonda ekle
        """
        # Environment referansı kontrolü
        if not hasattr(self, 'environment_ref') or self.environment_ref is None:
            print(f"⚠️ ROV-{self.id} eklenemedi: environment_ref bulunamadı")
            return False
        
        ortam = self.environment_ref
        
        # ROV listesi kontrolü
        if not hasattr(ortam, 'rovs'):
            ortam.rovs = []
        
        # Eğer bu ID'de zaten ROV varsa, ekleme yapma
        if self.id < len(ortam.rovs) and ortam.rovs[self.id] is not None:
            print(f"⚠️ ROV-{self.id} zaten mevcut. Önce çıkarmak için: rov.cikar()")
            return False
        
        # Pozisyon ayarla
        if konum is not None:
            if isinstance(konum, (tuple, list)) and len(konum) == 3:
                x, y, z = konum  # Simülasyon koordinat sistemi: (x_2d, y_2d, z_depth)
                # ROV için koordinat dönüşümü: sim_to_ursina(x, z, y) -> (x, z_depth, y_2d)
                ursina_x, ursina_y, ursina_z = sim_to_ursina(x, z, y)
                self.position = Vec3(ursina_x, ursina_y, ursina_z)
        
        # Environment referansını ayarla
        self.environment_ref = ortam
        
        # Filo referansını ayarla (varsa)
        if hasattr(ortam, 'filo'):
            self.filo_ref = ortam.filo
        
        # ROV listesine ekle (ID'ye göre yerleştir)
        while len(ortam.rovs) <= self.id:
            ortam.rovs.append(None)
        ortam.rovs[self.id] = self
        
        # Verbose kontrolü
        verbose = False
        if hasattr(ortam, 'verbose'):
            verbose = ortam.verbose
        
        if verbose:
            pos_str = f"({konum[0]}, {konum[1]}, {konum[2]})" if konum else "varsayılan"
            print(f"✅ ROV-{self.id} eklendi: {pos_str}")
        
        return True
    
    def cikar(self):
        """
        ROV'u simülasyondan çıkarır.
        ROV çıkarıldıktan sonra sistem otomatik olarak kalan ROV'ların ID'lerini 
        0'dan başlayarak yeniden numaralandırır.
        
        Returns:
            bool: İşlem başarılı ise True, aksi halde False
        
        Örnek:
            rov.cikar()  # ROV'u simülasyondan çıkar ve ID'leri yeniden numaralandır
        """
        # Environment referansı kontrolü
        if not hasattr(self, 'environment_ref') or self.environment_ref is None:
            print(f"⚠️ ROV çıkarılamadı: environment_ref bulunamadı")
            return False
        
        ortam = self.environment_ref
        
        # ROV listesi kontrolü
        if not hasattr(ortam, 'rovs') or not ortam.rovs:
            print(f"⚠️ ROV bulunamadı (ROV listesi boş)")
            return False
        
        # ROV'un listede olup olmadığını kontrol et
        rov_bulundu = False
        for i, rov in enumerate(ortam.rovs):
            if rov is self:
                rov_bulundu = True
                break
        
        if not rov_bulundu:
            print(f"⚠️ ROV listede bulunamadı")
            return False
        
        # ROV entity'sini destroy et
        try:
            # Görünürlüğü kapat
            if hasattr(self, 'visible'):
                self.visible = False
            if hasattr(self, 'enabled'):
                self.enabled = False
            
            # Label'ı destroy et
            if hasattr(self, 'label') and self.label is not None:
                try:
                    if hasattr(self.label, 'destroy'):
                        self.label.destroy()
                except Exception:
                    pass
            
            # Safety zone'u destroy et
            if hasattr(self, 'safety_zone') and self.safety_zone is not None:
                try:
                    if hasattr(self.safety_zone, 'destroy'):
                        self.safety_zone.destroy()
                except Exception:
                    pass
            
            # Engel çizgisini destroy et
            if hasattr(self, 'engel_cizgi') and self.engel_cizgi is not None:
                try:
                    if hasattr(self.engel_cizgi, 'destroy'):
                        self.engel_cizgi.destroy()
                except Exception:
                    pass
            
            # İletişim çizgilerini destroy et
            if hasattr(self, 'iletisim_rovlari') and self.iletisim_rovlari:
                for rov_id, iletisim_info in self.iletisim_rovlari.items():
                    if isinstance(iletisim_info, dict) and 'cizgi' in iletisim_info:
                        cizgi = iletisim_info['cizgi']
                        if cizgi is not None and hasattr(cizgi, 'destroy'):
                            try:
                                cizgi.destroy()
                            except Exception:
                                pass
            
            # Parent'ı kaldır
            if hasattr(self, 'parent'):
                self.parent = None
            
            # Entity'yi destroy et
            if hasattr(self, 'destroy'):
                self.destroy()
            
            # Scene'den de kaldırmayı dene (eğer mümkünse)
            try:
                from ursina import scene
                if hasattr(scene, 'entities') and self in scene.entities:
                    scene.entities.remove(self)
            except Exception:
                pass
        except Exception as e:
            # Hata olsa bile devam et
            verbose = False
            if hasattr(ortam, 'verbose'):
                verbose = ortam.verbose
            if verbose:
                eski_id = getattr(self, 'id', '?')
                print(f"⚠️ ROV-{eski_id} destroy hatası: {e}")
        finally:
            # ROV listesinden çıkar (self referansını bul ve None yap)
            for i, rov in enumerate(ortam.rovs):
                if rov is self:
                    ortam.rovs[i] = None
                    break
        
        # Eski ID'yi sakla (yeniden numaralandırmadan önce)
        eski_id = getattr(self, 'id', '?')
        
        # ROV ID'lerini yeniden numaralandır (0'dan başlayarak)
        ortam._rov_id_yeniden_numaralandir()
        
        # Verbose kontrolü
        verbose = False
        if hasattr(ortam, 'verbose'):
            verbose = ortam.verbose
        
        if verbose:
            print(f"✅ ROV-{eski_id} çıkarıldı ve ID'ler yeniden numaralandırıldı")
        
        return True

    def update(self):
        # Manuel hareket kontrolü (sürekli hareket için) — active_forces ile eşzamanlı hareket
        if self.manuel_hareket['yon'] is not None:
            if self.manuel_hareket['yon'] == 'dur':
                # Tüm kuvvetleri sıfırla ve yavaşça dur
                if hasattr(self, 'active_forces'):
                    for k in self.active_forces:
                        self.active_forces[k] = 0.0
                self.velocity *= FizikSabitleri.VELOCITY_DURMA_CARPANI
                if self.velocity.length() < FizikSabitleri.VELOCITY_DURMA_ESIGI:
                    self.velocity = Vec3(0, 0, 0)
                    self.manuel_hareket['yon'] = None
                    self.manuel_hareket['guc'] = 0.0
            elif self.manuel_hareket['yon'] == 'yaw':
                # Yaw: active_forces['yaw'] ile sürekli dönme (derece/saniye)
                yaw_guc = self.active_forces.get('yaw', 0.0)
                if abs(yaw_guc) > 0:
                    yaw_hizi = abs(yaw_guc) * 90.0
                    yaw_delta = yaw_hizi * time.dt
                    if not hasattr(self, 'rotation') or self.rotation is None:
                        self.rotation = Vec3(0, 0, 0)
                    elif not isinstance(self.rotation, Vec3):
                        self.rotation = Vec3(self.rotation[0], self.rotation[1], self.rotation[2]) if isinstance(self.rotation, (tuple, list)) and len(self.rotation) >= 3 else Vec3(0, 0, 0)
                    current_x = self.rotation.x if isinstance(self.rotation, Vec3) else 0
                    current_y = self.rotation.y if isinstance(self.rotation, Vec3) else 0
                    current_z = self.rotation.z if isinstance(self.rotation, Vec3) else 0
                    new_y = current_y + (yaw_delta if yaw_guc > 0 else -yaw_delta)
                    while new_y >= 360:
                        new_y -= 360
                    while new_y < 0:
                        new_y += 360
                    self.rotation = Vec3(current_x, new_y, current_z)
            elif self.manuel_hareket['guc'] > 0 or any(self.active_forces.get(k, 0) != 0 for k in ('surge', 'sway', 'heave')):
                # Hibrit: Manuel kuvvet velocity'e eklenir (AI çıktısı guncelle() ile zaten eklenmiş olabilir)
                # Final_Force = AI_Output + Manual_Input (vektörel toplama)
                # Eşzamanlı hareket: active_forces'tan vektör hesapla (surge, sway, heave)
                af = getattr(self, 'active_forces', {'surge': 0.0, 'sway': 0.0, 'heave': 0.0})
                surge = af.get('surge', 0.0)
                sway = af.get('sway', 0.0)
                heave = af.get('heave', 0.0)
                yaw_acisi = 0.0
                if hasattr(self, 'rotation') and self.rotation is not None:
                    yaw_acisi = self.rotation.y if isinstance(self.rotation, Vec3) else (self.rotation[1] if isinstance(self.rotation, (tuple, list)) and len(self.rotation) >= 2 else 0)
                yaw_rad = radians(yaw_acisi)
                # Lokal eksenler (Ursina: X sağ, Z ileri, Y yukarı). İleri = (sin(yaw), 0, cos(yaw)), Sağ = (cos(yaw), 0, -sin(yaw))
                ileri_x = sin(yaw_rad)
                ileri_z = cos(yaw_rad)
                sag_x = cos(yaw_rad)
                sag_z = -sin(yaw_rad)
                # Global hareket vektörü = surge*ileri + sway*sağ + heave*yukarı
                hareket_x = surge * ileri_x + sway * sag_x
                hareket_z = surge * ileri_z + sway * sag_z
                hareket_y = heave
                # Büyüklük 1.0'ı geçerse normalize et (çaprazda motor limitini aşmasın)
                toplam_len = (hareket_x ** 2 + hareket_z ** 2 + hareket_y ** 2) ** 0.5
                if toplam_len > 1.0:
                    hareket_x /= toplam_len
                    hareket_z /= toplam_len
                    hareket_y /= toplam_len
                max_guc = 100.0
                thrust = max_guc * time.dt * HareketAyarlari.MOTOR_GUC_KATSAYISI
                self.velocity.x += hareket_x * thrust
                self.velocity.z += hareket_z * thrust
                self.velocity.y += hareket_y * thrust
                if self.velocity.length() > max_guc:
                    self.velocity = self.velocity.normalized() * max_guc
        
        # --- SENSÖR GÜNCELLEME (Ana Thread'de - Thread-Safe) ---
        # Tüm fiziksel raycast işlemleri burada yapılır
        # Konsol thread'i get() çağırdığında sadece cache'lenmiş değerleri okur
        if self.environment_ref:
            self._sensorleri_guncelle()
        
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
        
        # --- OTOMATİK ÇARPIŞMA TEPKİSİ (Intersects) ---
        # Not: intersects() sadece primitive collider'lar (box, sphere, capsule) ile çalışır
        # Mesh collider'lar sadece çarpışmaları "alabilir" ama intersects() yapamaz
        try:
            collision = self.intersects(ignore=(self.safety_zone,))
            if collision.hit:
                # Geri sekme efekti (daha güçlü)
                self.velocity = -self.velocity * FizikSabitleri.CARPISMA_HIZ_YANSIMA
                
                # İç içe geçmeyi önlemek için pozisyonu daha güçlü it
                if hasattr(collision, 'world_normal') and collision.world_normal:
                    # Normal vektörü kullanarak daha güçlü itme
                    push_distance = FizikSabitleri.CARPISMA_ITME_MESAFESI
                    self.position += collision.world_normal * push_distance
                elif hasattr(collision, 'entity') and collision.entity:
                    # Engel varsa, engelden uzaklaş
                    fark_vektoru = (self.position - collision.entity.position)
                    mesafe = fark_vektoru.length()
                    if mesafe > 0.001:
                        fark_vektoru = fark_vektoru.normalized()
                        push_distance = FizikSabitleri.CARPISMA_ITME_MESAFESI
                        self.position += fark_vektoru * push_distance
                    else:
                        # Çok yakınsa rastgele yöne it
                        push_distance = FizikSabitleri.CARPISMA_ITME_MESAFESI
                        self.position += Vec3(1, 0, 0) * push_distance
                
                # Hızı sıfırla (çarpışma sonrası dur)
                if self.velocity.length() < FizikSabitleri.CARPISMA_HIZ_SIFIRLAMA_ESIGI:
                    self.velocity = Vec3(0, 0, 0)
                
                if self.environment_ref and self.environment_ref.verbose:
                    print(f"💥 ROV-{self.id} Çarpışma: {collision.entity if hasattr(collision, 'entity') else 'Bilinmeyen'}")
        except Exception as e:
            # Mesh collider hatası veya başka bir hata durumunda
            # Manuel çarpışma kontrolüne geri dön (_carpisma_kontrolu zaten var)
            pass
        
        # Fizik
        self.position += self.velocity * time.dt
        self.velocity *= FizikSabitleri.SURTUNME_KATSAYISI
        
        # Simülasyon sınır kontrolü (ROV'ların dışarı çıkmasını önle)
        # Sınırlar: +-havuz_genisligi (yani +-200 birim)
        # 10 metre güvenlik mesafesi: ROV'lar sınırlardan 10 metre içeride kalmalı
        HAVUZ_GUVENLIK_MESAFESI = 10.0  # Metre cinsinden güvenlik mesafesi
        if self.environment_ref:
            havuz_genisligi = getattr(self.environment_ref, 'havuz_genisligi', 200)
            havuz_sinir = havuz_genisligi  # +-havuz_genisligi
            guvenli_sinir = havuz_sinir - HAVUZ_GUVENLIK_MESAFESI  # 10 metre içerideki sınır
            
            # X ve Z sınırları (10 metre güvenlik mesafesi ile)
            if abs(self.x) > guvenli_sinir:
                self.x = np.sign(self.x) * guvenli_sinir
                self.velocity.x = 0  # Güvenlik sınırında durdur
            
            if abs(self.z) > guvenli_sinir:
                self.z = np.sign(self.z) * guvenli_sinir
                self.velocity.z = 0  # Güvenlik sınırında durdur
        
        if self.role == 1: # Lider
            if self.y < 0:
                self.velocity.y += FizikSabitleri.KALDIRMA_KUVVETI * time.dt
                if self.y > FizikSabitleri.LIDER_YUZEY_YAKINLIK:
                    self.velocity.y *= FizikSabitleri.LIDER_YUZEY_HIZ_CARPANI
            if self.y < FizikSabitleri.LIDER_YUZEY_ALT_SINIR:
                self.y = FizikSabitleri.LIDER_YUZEY_ALT_SINIR
            if self.y > FizikSabitleri.LIDER_YUZEY_UST_SINIR: 
                self.y = FizikSabitleri.LIDER_YUZEY_UST_SINIR
                self.velocity.y = 0
        else: # Takipçi
            if self.y > FizikSabitleri.TAKIPCI_YUZEY_SINIRI: 
                self.y = FizikSabitleri.TAKIPCI_YUZEY_SINIRI
                self.velocity.y = 0
            if self.y < FizikSabitleri.TAKIPCI_MAX_DERINLIK: 
                self.y = FizikSabitleri.TAKIPCI_MAX_DERINLIK
                self.velocity.y = 0

        if self.velocity.length() > 0.01: 
            self.battery -= FizikSabitleri.BATARYA_SOMURME_KATSAYISI * time.dt
        
        # Çarpışma kontrolü
        if self.environment_ref:
            self._carpisma_kontrolu()

    def move(self, komut, guc=1.0):
        # Batarya bitmişse hareket ettirme
        if self.battery <= 0:
            return
        thrust = guc * FizikSabitleri.HIZLANMA_CARPANI * time.dt

        # ROV'un yaw rotasyonunu al (Y ekseni etrafında dönme açısı - derece)
        yaw_acisi = 0.0
        if hasattr(self, 'rotation') and self.rotation is not None:
            if isinstance(self.rotation, Vec3):
                yaw_acisi = self.rotation.y
            elif isinstance(self.rotation, (tuple, list)) and len(self.rotation) >= 2:
                yaw_acisi = self.rotation[1]
        
        # Yaw açısını radyana çevir
        yaw_radyan = radians(yaw_acisi)
        
        # Yatay hareket komutları için (ileri, geri, sağ, sol)
        # ROV'un yönüne göre hareket vektörünü hesapla
        if komut == "ileri":
            # İleri: ROV'un baktığı yön (Z ekseni pozitif yönü, yaw açısına göre döndürülmüş)
            hareket_x = sin(yaw_radyan) * thrust
            hareket_z = cos(yaw_radyan) * thrust
            self.velocity.x += hareket_x
            self.velocity.z += hareket_z
        elif komut == "geri":
            # Geri: ROV'un arkası (Z ekseni negatif yönü, yaw açısına göre döndürülmüş)
            hareket_x = -sin(yaw_radyan) * thrust
            hareket_z = -cos(yaw_radyan) * thrust
            self.velocity.x += hareket_x
            self.velocity.z += hareket_z
        elif komut == "sag":
            # Sağ: ROV'un sağ tarafı (X ekseni pozitif yönü, yaw açısına göre döndürülmüş)
            hareket_x = cos(yaw_radyan) * thrust
            hareket_z = -sin(yaw_radyan) * thrust
            self.velocity.x += hareket_x
            self.velocity.z += hareket_z
        elif komut == "sol":
            # Sol: ROV'un sol tarafı (X ekseni negatif yönü, yaw açısına göre döndürülmüş)
            hareket_x = -cos(yaw_radyan) * thrust
            hareket_z = sin(yaw_radyan) * thrust
            self.velocity.x += hareket_x
            self.velocity.z += hareket_z
        elif komut == "cik":
            # Yukarı: Y ekseni pozitif (yaw'dan etkilenmez)
            self.velocity.y += thrust 
        elif komut == "bat":
            # Aşağı: Y ekseni negatif (yaw'dan etkilenmez)
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
                if hasattr(self, 'ortam') and hasattr(self.ortam, 'verbose') and self.ortam.verbose:
                    print(f"✅ ROV-{self.id} artık LİDER.")
            else:
                self.color = color.orange
                self.label.text = f"ROV-{self.id}"
                if hasattr(self, 'ortam') and hasattr(self.ortam, 'verbose') and self.ortam.verbose:
                    print(f"✅ ROV-{self.id} artık TAKİPÇİ.")
        elif ayar_adi == "yaw":
            # Yaw açısını derece olarak ayarla (Y ekseni etrafında dönme)
            yaw_derece = float(deger)
            # 0-360 arası normalize et
            while yaw_derece >= 360:
                yaw_derece -= 360
            while yaw_derece < 0:
                yaw_derece += 360
            
            # Mevcut rotation değerini al
            if not hasattr(self, 'rotation') or self.rotation is None:
                self.rotation = Vec3(0, 0, 0)
            elif not isinstance(self.rotation, Vec3):
                # Tuple veya list ise Vec3'e dönüştür
                if isinstance(self.rotation, (tuple, list)) and len(self.rotation) >= 3:
                    self.rotation = Vec3(self.rotation[0], self.rotation[1], self.rotation[2])
                else:
                    self.rotation = Vec3(0, 0, 0)
            
            # Yaw açısını güncelle (sadece Y ekseni)
            current_x = self.rotation.x if hasattr(self.rotation, 'x') else 0
            current_z = self.rotation.z if hasattr(self.rotation, 'z') else 0
            self.rotation = Vec3(current_x, yaw_derece, current_z)
        elif ayar_adi in self.sensor_config: 
            self.sensor_config[ayar_adi] = deger

    def get(self, veri_tipi, taraf=None):
        if veri_tipi == "gps": 
            return np.array([self.x, self.y, self.z])
        elif veri_tipi == "hiz": 
            return np.array([self.velocity.x, self.velocity.y, self.velocity.z])
        elif veri_tipi == "batarya": 
            return self.battery
        elif veri_tipi == "yaw":
            # Yaw açısını derece olarak döndür (Y ekseni etrafında dönme açısı)
            if hasattr(self, 'rotation') and self.rotation is not None:
                # Vec3 kontrolü için type() kullan (isinstance yerine)
                rotation_type = type(self.rotation).__name__
                if rotation_type == 'Vec3' or hasattr(self.rotation, 'y'):
                    # Vec3 tipinde veya y özelliği varsa
                    return float(self.rotation.y)
                elif isinstance(self.rotation, (tuple, list)) and len(self.rotation) >= 2:
                    return float(self.rotation[1])
            return 0.0  # Varsayılan: 0 derece
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
            """
            Sonar sensörü: En yakın engeli tespit eder. Thread-Safe cache'lenmiş değer döner.
            Maksimum algılama menzili: sensor_config["engel_mesafesi"] (varsayılan 10 m).
            Engel bu menzil dışındaysa veya tespit yoksa -1 döner.
            """
            # Konsol thread'i sadece cache'lenmiş değeri okur (raycast yapmaz!)
            return self.son_sonar_mesafesi
        elif veri_tipi == "lidar":
            """
            Lidar sensörü: Yöne göre en yakın engeli tespit eder. Thread-Safe cache kullanır.
            Maksimum algılama menzili: sensor_config["engel_mesafesi"] (varsayılan 10 m).
            Engel yoksa veya menzil dışındaysa -1 döner.
            taraf: 0=Ön, 1=Sol, 2=Sağ; None=Ön.
            """
            # Konsol thread'i sadece cache'lenmiş değeri okur (raycast yapmaz!)
            t = taraf if taraf is not None else 0
            return self.son_lidar_mesafeleri.get(t, -1)
        return None
    
    def _sensorleri_guncelle(self):
        """
        Tüm ağır fiziksel raycast işlemlerini Ana Thread'de güvenli bölgede yap.
        Bu fonksiyon sadece update() içinde (Ana Thread'de) çağrılır.
        Konsol thread'i get() çağırdığında sadece cache'lenmiş değerleri okur.
        """
        import math
        
        if not self.environment_ref:
            return
        
        engel_mesafesi_limit = self.sensor_config.get("engel_mesafesi", SensorAyarlari.VARSAYILAN["engel_mesafesi"])
        lidar_menzil = engel_mesafesi_limit
        lidar_acisi = math.radians(60)  # 60 derece görüş açısı
        
        # Engel tespiti (her zaman çalışır, manuel kontrol olsun olmasın)
        # Bu fonksiyon self.engel_mesafesi'ni günceller
        self._engel_tespiti()
        
        # Sonar verisini güncelle (8 yönlü tarama yerine en yakın engeli kullan)
        # _engel_tespiti() zaten en yakın engeli buluyor
        if self.tespit_edilen_engel and self.engel_mesafesi < SimulasyonSabitleri.ENGEL_TESPITI_MIN_MESAFE:
            if self.engel_mesafesi < engel_mesafesi_limit:
                self.son_sonar_mesafesi = self.engel_mesafesi
            else:
                self.son_sonar_mesafesi = -1
        else:
            self.son_sonar_mesafesi = -1
        
        # ROV'un yön vektörlerini hesapla
        if hasattr(self, 'forward') and self.forward:
            forward_vec = Vec3(self.forward.x, 0, self.forward.z).normalized()
        else:
            forward_vec = Vec3(0, 0, 1)
        
        # Sol ve sağ vektörleri hesapla
        left_vec = Vec3(-forward_vec.z, 0, forward_vec.x).normalized()
        right_vec = Vec3(forward_vec.z, 0, -forward_vec.x).normalized()
        
        # Raycast origin: ROV'un kendi box collider'ından dışarı kaydır
        raycast_origin = self.world_position + Vec3(0, 0.5, 0)
        
        # Ignore tuple'ı döngü dışında oluştur
        ignore_list = [self]
        if hasattr(self, 'safety_zone') and self.safety_zone:
            ignore_list.append(self.safety_zone)
        ignore_tuple = tuple(ignore_list)
        
        # Lidar verilerini güncelle (Ön: 0, Sol: 1, Sağ: 2)
        yonler = {
            0: forward_vec,  # Ön
            1: left_vec,     # Sol
            2: right_vec     # Sağ
        }
        
        raycast_sayisi = SimulasyonSabitleri.LIDAR_RAYCAST_SAYISI  # Her yön için raycast (koni taraması)
        
        for yon_id, yon_temel in yonler.items():
            min_dist = -1
            
            # Koni taraması (60 derece açı içinde)
            for i in range(raycast_sayisi):
                # Açı ofsetini hesapla
                if raycast_sayisi > 1:
                    angle = (i / (raycast_sayisi - 1) - 0.5) * lidar_acisi
                else:
                    angle = 0
                
                # Yönü döndür (Sadece yatay düzlemde)
                cos_a = math.cos(angle)
                sin_a = math.sin(angle)
                
                # Basit vektör döndürme formülü
                rot_yon = Vec3(
                    yon_temel.x * cos_a - yon_temel.z * sin_a,
                    0,
                    yon_temel.x * sin_a + yon_temel.z * cos_a
                )
                
                # Normalize et (güvenli)
                rot_yon_length = (rot_yon.x**2 + rot_yon.z**2)**0.5
                if rot_yon_length < 0.001:
                    rot_yon = Vec3(0, 0, 1)
                else:
                    rot_yon = Vec3(rot_yon.x / rot_yon_length, 0, rot_yon.z / rot_yon_length)
                
                # Raycast origin'i yönüne doğru kaydır
                yon_origin = raycast_origin + (yon_temel * 1.5)
                
                try:
                    # Ursina Raycast çağrısı
                    hit_info = raycast(
                        yon_origin,
                        rot_yon,
                        distance=lidar_menzil,
                        ignore=ignore_tuple,
                        debug=False
                    )
                    
                    if hit_info and hasattr(hit_info, 'hit') and hit_info.hit:
                        if hasattr(hit_info, 'distance'):
                            dist = hit_info.distance
                            if min_dist == -1 or dist < min_dist:
                                min_dist = dist
                except Exception:
                    continue
            
            # Cache'e kaydet
            self.son_lidar_mesafeleri[yon_id] = min_dist if min_dist >= 0 else -1
    
    def _engel_tespiti(self):
        """
        FİZİK MOTORU TABANLI: Raycast kullanarak en yakın engeli tespit eder.
        Çizgiyi engelin yüzeyine (raycast hit noktasına) çizer.
        Havuz sınırları da engel olarak algılanır.
        OPTİMİZE EDİLMİŞ: Segmentation fault önleme için origin kaydırıldı ve tuple döngü dışında oluşturuldu.
        """
        if not self.environment_ref:
            return
        
        engel_mesafesi_limit = self.sensor_config.get("engel_mesafesi", SensorAyarlari.VARSAYILAN["engel_mesafesi"])
        min_mesafe = SimulasyonSabitleri.ENGEL_TESPITI_MIN_MESAFE
        en_yakin_engel = None
        en_yakin_nokta = None  # Raycast hit noktası
        
        # ROV'un yönünü al (forward vektörü)
        if hasattr(self, 'forward') and self.forward:
            forward_vec = Vec3(self.forward.x, 0, self.forward.z).normalized()
        else:
            # Varsayılan yön (z ekseni pozitif yönü - ileri)
            forward_vec = Vec3(0, 0, 1)
            
        # Raycast origin: ROV'un kendi box collider'ından dışarı kaydır (segfault önleme)
        # ROV merkezinden 1.5 birim ileri kaydırıyoruz
        raycast_origin = self.world_position + Vec3(0, 0.5, 0) + (forward_vec * 1.5)
        
        # Ignore tuple'ı döngü dışında oluştur (Bellek yönetimi için kritik)
        ignore_tuple = (self, self.safety_zone) if hasattr(self, 'safety_zone') and self.safety_zone else (self,)
        
        # Önce ön yönde raycast yap (en önemli yön)
        hit_info = raycast(
            raycast_origin,
            forward_vec,
            distance=engel_mesafesi_limit,
            ignore=ignore_tuple,
            debug=False  # Segfault riskini azaltmak için debug'ı kapatın
        )
        
        if hit_info.hit:
            mesafe = hit_info.distance
            if mesafe < min_mesafe:
                min_mesafe = mesafe
                en_yakin_engel = hit_info.entity if hasattr(hit_info, 'entity') else None
                en_yakin_nokta = hit_info.world_point if hasattr(hit_info, 'world_point') else None
        
        # Eğer ön yönde engel yoksa, diğer yönleri de kontrol et (sağ, sol, arka)
        if not hit_info.hit or min_mesafe >= engel_mesafesi_limit:
            # Sağ yön
            right_vec = Vec3(forward_vec.z, 0, -forward_vec.x).normalized()
            hit_info = raycast(
                raycast_origin,
                right_vec,
                distance=engel_mesafesi_limit,
                ignore=ignore_tuple,
                debug=False
            )
            if hit_info.hit and hit_info.distance < min_mesafe:
                min_mesafe = hit_info.distance
                en_yakin_engel = hit_info.entity if hasattr(hit_info, 'entity') else None
                en_yakin_nokta = hit_info.world_point if hasattr(hit_info, 'world_point') else None
            
            # Sol yön
            left_vec = Vec3(-forward_vec.z, 0, forward_vec.x).normalized()
            hit_info = raycast(
                raycast_origin,
                left_vec,
                distance=engel_mesafesi_limit,
                ignore=ignore_tuple,
                debug=False
            )
            if hit_info.hit and hit_info.distance < min_mesafe:
                min_mesafe = hit_info.distance
                en_yakin_engel = hit_info.entity if hasattr(hit_info, 'entity') else None
                en_yakin_nokta = hit_info.world_point if hasattr(hit_info, 'world_point') else None
            
            # Arka yön
            back_vec = -forward_vec
            hit_info = raycast(
                raycast_origin,
                back_vec,
                distance=engel_mesafesi_limit,
                ignore=ignore_tuple,
                debug=False
            )
            if hit_info.hit and hit_info.distance < min_mesafe:
                min_mesafe = hit_info.distance
                en_yakin_engel = hit_info.entity if hasattr(hit_info, 'entity') else None
                en_yakin_nokta = hit_info.world_point if hasattr(hit_info, 'world_point') else None
        
        # Havuz sınırlarını da kontrol et (fallback - raycast duvarları algılamazsa)
            if hasattr(self.environment_ref, 'havuz_genisligi'):
                havuz_genisligi = self.environment_ref.havuz_genisligi
                havuz_sinir = havuz_genisligi
                
            x_mesafe_sag = havuz_sinir - self.position.x
            x_mesafe_sol = self.position.x - (-havuz_sinir)
            z_mesafe_on = havuz_sinir - self.position.z
            z_mesafe_arka = self.position.z - (-havuz_sinir)
            
            en_yakin_sinir_mesafe = min(x_mesafe_sag, x_mesafe_sol, z_mesafe_on, z_mesafe_arka)
            
            if en_yakin_sinir_mesafe < min_mesafe and en_yakin_sinir_mesafe < engel_mesafesi_limit:
                min_mesafe = en_yakin_sinir_mesafe
                en_yakin_engel = "havuz_siniri"
                
                # En yakın sınırın pozisyonunu hesapla
                if en_yakin_sinir_mesafe == x_mesafe_sag:
                    en_yakin_nokta = Vec3(havuz_sinir, self.position.y, self.position.z)
                elif en_yakin_sinir_mesafe == x_mesafe_sol:
                    en_yakin_nokta = Vec3(-havuz_sinir, self.position.y, self.position.z)
                elif en_yakin_sinir_mesafe == z_mesafe_on:
                    en_yakin_nokta = Vec3(self.position.x, self.position.y, havuz_sinir)
                else:
                    en_yakin_nokta = Vec3(self.position.x, self.position.y, -havuz_sinir)

        # Tespit Sonucu
        if en_yakin_engel and min_mesafe < engel_mesafesi_limit:
            self.tespit_edilen_engel = en_yakin_engel
            self.engel_mesafesi = min_mesafe
            
            # Çizgi fonksiyonuna raycast hit noktasını gönder
            if en_yakin_nokta:
                self._kesikli_cizgi_ciz(en_yakin_nokta, min_mesafe)
        else:
            self.tespit_edilen_engel = None
            self.engel_mesafesi = SimulasyonSabitleri.ENGEL_TESPITI_MIN_MESAFE
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
        parca_uzunlugu = SimulasyonSabitleri.KESIKLI_CIZGI_PARCA_UZUNLUGU
        bosluk_uzunlugu = SimulasyonSabitleri.KESIKLI_CIZGI_BOSLUK_UZUNLUGU
        
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
            # None kontrolü (çıkarılmış ROV'ları atla)
            if diger_rov is None:
                continue
            
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
        
        # Kesikli çizgi parçaları (iletişim çizgisi için)
        parca_uzunlugu = SimulasyonSabitleri.ILETISIM_CIZGI_PARCA_UZUNLUGU
        bosluk_uzunlugu = SimulasyonSabitleri.ILETISIM_CIZGI_BOSLUK_UZUNLUGU
        
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
            # None kontrolü (çıkarılmış ROV'ları atla)
            if rov is None:
                continue
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
    
    def _carpisma_kontrolu(self):
        """
        FİZİK MOTORU TABANLI: Intersects kullanarak çarpışma kontrolü yapar.
        Update() fonksiyonunda zaten intersects() kullanılıyor, bu fonksiyon
        ek manuel kontrol için (eski kod uyumluluğu) tutuluyor.
        """
        # Not: Ana çarpışma kontrolü update() fonksiyonunda intersects() ile yapılıyor
        # Bu fonksiyon sadece ek kontrol için (eski kod uyumluluğu)
        if not self.environment_ref:
            return
        
        # ROV kütlesi (basitleştirilmiş)
        rov_kutlesi = FizikSabitleri.ROV_KUTLESI
        
        # Diğer ROV'larla çarpışma (intersects zaten kontrol ediyor, burada sadece momentum hesaplaması)
        for diger_rov in self.environment_ref.rovs:
            # None kontrolü (çıkarılmış ROV'ları atla)
            if diger_rov is None:
                continue
            if diger_rov.id == self.id:
                continue
            
            mesafe = distance(self.position, diger_rov.position)
            min_mesafe = FizikSabitleri.ROV_MINIMUM_MESAFE  # ROV boyutlarına göre minimum mesafe
            
            if mesafe < min_mesafe:
                # Çarpışma tespit edildi - momentum korunumu hesapla
                carpisma_yonu = (self.position - diger_rov.position).normalized()
                goreceli_hiz = self.velocity - diger_rov.velocity
                goreceli_hiz_buyuklugu = goreceli_hiz.length()
                
                if goreceli_hiz_buyuklugu > 0.1:
                    diger_rov_kutlesi = FizikSabitleri.ROV_KUTLESI
                    nokta_carpim = goreceli_hiz.dot(carpisma_yonu)
                    
                    if nokta_carpim < 0:  # Birbirine yaklaşıyorlar
                        # Momentum korunumu
                        carpan1 = (2 * diger_rov_kutlesi / (rov_kutlesi + diger_rov_kutlesi)) * nokta_carpim
                        self.velocity = self.velocity - carpisma_yonu * carpan1
                        
                        carpan2 = (2 * rov_kutlesi / (rov_kutlesi + diger_rov_kutlesi)) * (-nokta_carpim)
                        diger_rov.velocity = diger_rov.velocity - (-carpisma_yonu) * carpan2
                        
                        # Pozisyonları ayır (daha güçlü)
                        ayirma_mesafesi = (min_mesafe - mesafe) + 3.0  # Artırıldı: 2.0 -> 3.0
                        self.position += carpisma_yonu * ayirma_mesafesi
                        diger_rov.position -= carpisma_yonu * ayirma_mesafesi
                        
        # Ada entity'leri ile çarpışma kontrolü (manuel - mesh collider intersects() yapamaz)
        # Tüm adaları kontrol et
        if hasattr(self.environment_ref, 'island_entities') and self.environment_ref.island_entities:
            for island_idx, island_entity in enumerate(self.environment_ref.island_entities):
                if not island_entity or not hasattr(island_entity, 'position') or island_entity.position is None:
                    continue
                
                # Ada yarıçapını bul
                island_radius = SimulasyonSabitleri.ADA_VARSAYILAN_RADIUS / 2.0  # Varsayılan (yarıçap)
                if hasattr(self.environment_ref, 'island_positions') and self.environment_ref.island_positions:
                    if island_idx < len(self.environment_ref.island_positions):
                        island_data = self.environment_ref.island_positions[island_idx]
                        if len(island_data) >= 3:
                            island_radius = island_data[2]
                
                # Yatay mesafe (Y eksenini yok say - ada su yüzeyinin üstünde)
                dx = self.position.x - island_entity.position.x
                dz = self.position.z - island_entity.position.z
                yatay_mesafe = (dx**2 + dz**2)**0.5
                
                # Ada yüzeyine mesafe
                yuzey_mesafesi = yatay_mesafe - island_radius
                
                # Çok yakınsa veya içindeyse it (güçlü itme)
                if yuzey_mesafesi < 3.0:  # 3 metre güvenlik mesafesi
                    if yatay_mesafe > 0.001:
                        itme_yonu = Vec3(dx / yatay_mesafe, 0, dz / yatay_mesafe)
                    else:
                        itme_yonu = Vec3(1, 0, 0)  # Varsayılan yön
                    
                    # Güçlü itme (içindeyse daha güçlü)
                    if yuzey_mesafesi < 0:
                        itme_mesafesi = abs(yuzey_mesafesi) + 5.0  # İçindeyse 5 metre daha it
                    else:
                        itme_mesafesi = (3.0 - yuzey_mesafesi) + 2.0  # Yakınsa 2 metre it
                    
                    self.position += itme_yonu * itme_mesafesi
                    self.velocity = -self.velocity * 0.3  # Hızı güçlü yansıt
                    
                    # Hız çok düşükse sıfırla
                    if self.velocity.length() < 0.5:
                        self.velocity = Vec3(0, 0, 0)

# ============================================================
# MİNİMAP SİSTEMİ (Ursina UI - Ekran Üzerinde)
# ============================================================
from ursina import *
import numpy as np

class Minimap(Entity):
    """
    Gelişmiş HUD Radar ve Navigasyon Haritası.
    Matplotlib yerine Ursina UI kullanır (GPU Tabanlı, 0 FPS kaybı).
    scale_carpan: 1 = taban boyut (0.45), 2 = 2 katı, 0.1 = onda biri vb.
    """
    BASE_SCALE = 0.50  # Taban boyut (Ursina UI birimi); scale_carpan=1 iken kullanılır

    def __init__(self, ortam_ref, scale_carpan=1, pozisyon='bottom_right', **kwargs):
        self._scale_carpan = float(scale_carpan)
        effective = self.BASE_SCALE * self._scale_carpan
        # Konumlandırma Mantığı
        if pozisyon == 'bottom_right':
            pos = window.bottom_right + Vec2(-effective/2 - 0.02, effective/2 + 0.02)
        else:
            pos = (0, 0)

        super().__init__(
            parent=camera.ui,
            model='quad',
            color=color.rgba(255, 255, 255, 0.01),  # Beyaz, daha şeffaf arka plan
            scale=(effective, effective),
            position=pos,
            origin=(0, 0),
            **kwargs
        )

        self.ortam_ref = ortam_ref
        self.havuz_genisligi = getattr(ortam_ref, 'havuz_genisligi', 200)
        self._pozisyon = pozisyon
        # grid_sayisi: None = varsayılan (GRID_UNIT m); N = toplam N aralık (her grid = (2*havuz)/N m)
        self._grid_sayisi = kwargs.pop('grid', None)
        
        # --- Katmanlar (Z-Order) ---
        # Arka Plan (-0.0) -> Grid (-0.1) -> Adalar (-0.2) -> Yollar (-0.3) -> ROV (-0.4) -> Engeller (-0.5)
        
        # Sınır Çizgisi (Border)
        self.border = Entity(parent=self, model='quad', color=color.white, scale=(1.02, 1.02), z=0.01, alpha=0.5)
        
        # Dinamik Nesne Referansları
        self.rov_ikonlari = {}      # {id: Entity}
        self.engel_noktalari = []   # [Entity, ...]
        self.statik_nesneler = []   # [Entity, ...] (Adalar, Grid)
        
        # Çizgi Meshleri (A* ve Hull için)
        self.path_entity = None
        self.hull_entity = None
        
        # Başlangıç Kurulumu
        self._grid_olustur()
        self._adalari_ciz()
        
        # Durum
        self.visible = False

    def _apply_scale(self):
        """Çarpana göre entity scale ve pozisyonu günceller (filo.minimap(scale=2) vb.)."""
        effective = self.BASE_SCALE * self._scale_carpan
        self.scale = (effective, effective)
        if self._pozisyon == 'bottom_right':
            self.position = window.bottom_right + Vec2(-effective/2 - 0.02, effective/2 + 0.02)

    def dunya_to_harita(self, x, z):
        """
        Dünya koordinatlarını (Metre) Harita lokal koordinatlarına (-0.5, 0.5) çevirir.
        Ursina UI'da Y ekseni yukarıdır, bu yüzden Dünya Z'si Harita Y'si olur.
        """
        # Ölçekleme faktörü: Havuzun toplam genişliği (genislik * 2)
        factor = 1.0 / (self.havuz_genisligi * 2)
        mx = x * factor
        my = z * factor
        return Vec3(mx, my, 0)

    # Grid: varsayılan her çizgi bu kadar metre (grid_sayisi verilmezse)
    GRID_UNIT = 50

    def _grid_step_metre(self):
        """Grid başına mesafe (m). grid_sayisi verilmişse (2*havuz)/N, yoksa GRID_UNIT."""
        half = self.havuz_genisligi
        if self._grid_sayisi is not None and self._grid_sayisi > 0:
            return (2.0 * half) / self._grid_sayisi
        return float(self.GRID_UNIT)

    def _grid_olustur(self):
        """Radar görünümü için grid çizgileri ve eksenleri oluşturur. Merkez (0,0) harita ortasındadır."""
        factor = 1.0 / (self.havuz_genisligi * 2)  # dünya -> lokal (-0.5, 0.5)
        grid_z = -0.1
        grid_color = color.rgba(255, 255, 255, 50)
        line_thick = 0.004

        # Ana Eksenler (X ve Y) — merkezde 0,0
        self.statik_nesneler.append(Entity(parent=self, model='quad', scale=(1, 0.005), color=color.rgba(255,255,255,100), z=grid_z))
        self.statik_nesneler.append(Entity(parent=self, model='quad', scale=(0.005, 1), color=color.rgba(255,255,255,100), z=grid_z))

        # Grid adımı: grid_sayisi varsa (2*havuz)/N m, yoksa GRID_UNIT m
        half = self.havuz_genisligi
        step = self._grid_step_metre()
        label_z = grid_z - 0.05
        label_scale = 1  # metin boyutu (minimap lokal biriminde)
        # Dikey çizgiler (sabit dünya X): lokal x = world_x * factor — x ekseni tarafında mesafe etiketi
        world_x = -half
        while world_x <= half:
            local_x = world_x * factor
            if world_x != 0:  # merkez eksenini tekrar çizme
                self.statik_nesneler.append(Entity(
                    parent=self, model='quad',
                    position=(local_x, 0, grid_z),
                    scale=(line_thick, 1),
                    color=grid_color
                ))
            # X grid: sadece ilk ve son noktada mesafe etiketi (altta)
            if world_x != -half:
                lbl = Text(
                    text=str(int(world_x)),
                    parent=self,
                    position=(local_x, -0.50, label_z),
                    scale=label_scale,
                    color=color.rgba(0, 0, 0, 1),
                    origin=(0.5, 0.5),
                    z=label_z
                )
                self.statik_nesneler.append(lbl)
            world_x += step
        # Yatay çizgiler (sabit dünya Z/Y): lokal y = world_z * factor — y ekseni tarafında mesafe etiketi
        world_z = -half
        while world_z <= half:
            local_y = world_z * factor
            if world_z != 0:
                self.statik_nesneler.append(Entity(
                    parent=self, model='quad',
                    position=(0, local_y, grid_z),
                    scale=(1, line_thick),
                    color=grid_color
                ))
            # Y grid: sadece ilk ve son noktada mesafe etiketi (solda)
            if True:           
                lbl = Text(
                    text=str(int(world_z)),
                    parent=self,
                    position=(-0.52, local_y, label_z),
                    scale=label_scale,
                    color=color.rgba(0, 0, 0, 1),
                    origin=(0.5, 0.5),
                    z=label_z
                )
                self.statik_nesneler.append(lbl)
            world_z += step

        # Grid bilgisi: 1 grid = X m, ölçek (türev: harita birimi başına metre)
        step_m = self._grid_step_metre()
        toplam_metre = 2 * half
        olcek_metre_birim = toplam_metre  # 1 harita birimi (-0.5..0.5) = toplam_metre m
        info_z = label_z - 0.02
        self.statik_nesneler.append(Text(
            parent=self,
            text=f"1 grid={step_m:.0f}m | 1 birim={olcek_metre_birim:.0f}m",
            position=(0, -0.54, info_z),
            scale=0.7,
            color=color.rgba(0, 0, 0, 1),
            origin=(0.5, 0.5),
            z=info_z
        ))

        # Dairesel Menzil Çizgileri (%33, %66, %100)
        for r in [0.33, 0.66, 1.0]:
            self.statik_nesneler.append(Entity(
                parent=self,
                model=Circle(resolution=60, radius=0.5 * r, mode='line', thickness=2),
                scale=1,
                color=color.rgba(255,255,0,50),
                z=grid_z
            ))

    def _adalari_ciz(self):
        """Simülasyondaki adaları haritada kahverengi daireler olarak gösterir."""
        if hasattr(self.ortam_ref, 'island_positions') and self.ortam_ref.island_positions:
            for pos in self.ortam_ref.island_positions:
                # Format: [x, z, radius] veya [x, z]
                x, z = pos[0], pos[1]
                r = pos[2] if len(pos) > 2 else 10.0
                
                # Konum ve Ölçek Hesapla
                map_pos = self.dunya_to_harita(x, z)
                # Yarıçap ölçeği (Harita 1 birim = 2*havuz_genisligi)
                map_scale = (r * 2) / (self.havuz_genisligi * 2)
                
                ada = Entity(
                    parent=self,
                    model='circle',
                    color=color.hex('#8B5A3C'), # Kahverengi
                    position=(map_pos.x, map_pos.y, -0.2),
                    scale=(map_scale, map_scale),
                    alpha=0.8
                )
                self.statik_nesneler.append(ada)

    def _statik_yeniden_ciz(self):
        """Grid ve adalar dahil statik nesneleri siler ve yeniden çizer (grid değişince kullanılır)."""
        for e in self.statik_nesneler:
            try:
                destroy(e)
            except Exception:
                pass
        self.statik_nesneler = []
        self._grid_olustur()
        self._adalari_ciz()

    def update_hull(self, points):
        """
        Convex Hull (Güvenlik Alanı) çizgilerini çizer.
        points: Numpy array veya liste [[x, z], ...]
        """
        if self.hull_entity:
            destroy(self.hull_entity)
            self.hull_entity = None
            
        if points is None or len(points) < 3:
            return

        # Noktaları harita koordinatlarına çevir
        verts = []
        for p in points:
            # Gelen veri formatı kontrolü
            px = p[0]
            pz = p[1] if len(p) > 1 else 0
            
            # Local koordinata çevir
            mp = self.dunya_to_harita(px, pz)
            verts.append((mp.x, mp.y, -0.25)) # Z-index: Adaların üstünde, ROV'un altında
            
        # Çizgiyi kapat (Son noktayı ilke bağla)
        verts.append(verts[0])

        self.hull_entity = Entity(
            parent=self,
            model=Mesh(vertices=verts, mode='line', thickness=2),
            color=color.cyan,
            alpha=0.6
        )

    def update_path(self, path_points):
        """
        A* Yolunu çizer (Yeşil Çizgi).
        path_points: [(x1, z1), (x2, z2), ...]
        """
        if self.path_entity:
            destroy(self.path_entity)
            self.path_entity = None
            
        if not path_points or len(path_points) < 2:
            return

        verts = []
        for p in path_points:
            mp = self.dunya_to_harita(p[0], p[1])
            verts.append((mp.x, mp.y, -0.3)) # Z-index: Hull'un üstünde

        self.path_entity = Entity(
            parent=self,
            model=Mesh(vertices=verts, mode='line', thickness=3),
            color=color.lime,
            alpha=0.9
        )
        
        # Başlangıç ve Bitiş Noktaları
        # (Bu örnekte basit tutmak için sadece çizgi çiziyoruz)

    def _rov_renk_al(self, rov):
        """3D ROV'un rengini alır; minimap ikonunda aynı renk kullanılır."""
        c = getattr(rov, 'color', None)
        if c is not None and hasattr(c, 'r') and hasattr(c, 'g') and hasattr(c, 'b'):
            return c
        return color.orange

    def update(self):
        """Her karede çalışır: ROV konumlarını günceller."""
        if not self.visible or not self.ortam_ref:
            return

        # ROV'ları Güncelle
        if hasattr(self.ortam_ref, 'rovs'):
            active_ids = set()
            for rov in self.ortam_ref.rovs:
                rid = getattr(rov, 'id', id(rov))
                active_ids.add(rid)
                rov_renk = self._rov_renk_al(rov)
                # Yön oku: beyaz (her ROV renginde okunaklı)
                ok_renk = color.white

                target_pos = self.dunya_to_harita(rov.x, rov.z)
                target_pos.z = -0.4 # En üstte (Z negatif bize yakın demek UI'da)

                if rid not in self.rov_ikonlari:
                    # ROV Gövdesi (Daire) - 3D ile aynı renk
                    govde = Entity(parent=self, model='circle', scale=0.04, color=rov_renk, position=target_pos)
                    # Yön Oku (yön göstergesi, açık tonda)
                    ok = Entity(parent=govde, model='quad', scale=(0.2, 0.8), y=0.4, color=ok_renk)
                    # ROV ID etiketi (ikonun üstünde)
                    Text(parent=govde, text=str(rid), position=(0, 1.5, 0), scale=50, color=rov_renk, origin=(0.5, 0.5))
                    self.rov_ikonlari[rid] = govde
                else:
                    current_entity = self.rov_ikonlari[rid]
                    current_entity.color = rov_renk
                    if current_entity.children:
                        current_entity.children[0].color = ok_renk

                # Pozisyonu yumuşak güncelle (Lerp)
                current_entity = self.rov_ikonlari[rid]
                current_entity.position = lerp(current_entity.position, target_pos, time.dt * 15)
                
                # Rotasyonu güncelle (Simülasyon Rotation Y -> Harita Rotation Z)
                # Simülasyon 0 derece = İleri (Z+). Harita 0 derece = Yukarı (Y+). Uyumlular.
                # Ancak Ursina dönüş yönleri farklı olabilir, -rotasyon genelde çözer.
                current_entity.rotation_z = -rov.rotation_y

            # Silinenleri Temizle
            for rid in list(self.rov_ikonlari.keys()):
                if rid not in active_ids:
                    destroy(self.rov_ikonlari[rid])
                    del self.rov_ikonlari[rid]

    def engel_ekle(self, x, z):
        """engel_bul() fonksiyonundan gelen tespitleri kırmızı nokta olarak ekler."""
        if not self.visible: return
        
        pos = self.dunya_to_harita(x, z)
        # Harita sınırları içinde mi? (-0.5 ile 0.5 arası)
        if abs(pos.x) > 0.5 or abs(pos.y) > 0.5: return
        
        # Nokta Ekle
        nokta = Entity(
            parent=self,
            model='circle',
            scale=0.015,
            color=color.red,
            position=(pos.x, pos.y, -0.35),
            alpha=0.8
        )
        self.engel_noktalari.append(nokta)
        
        # Performans: Çok fazla nokta varsa eskileri sil (FIFO)
        if len(self.engel_noktalari) > 150:
            eski = self.engel_noktalari.pop(0)
            destroy(eski)

    def temizle_engeller(self):
        for e in self.engel_noktalari:
            destroy(e)
        self.engel_noktalari.clear()

    def goster(self, durum=True, convex=False, a_star=False, scale=None, grid=None):
        """Görünürlüğü ayarlar. scale: çarpan; grid: grid sayısı (None=varsayılan GRID_UNIT m). convex/a_star Harita.update() ile senkronize edilir."""
        if grid is not None:
            n = int(grid)
            if n > 0 and n != self._grid_sayisi:
                self._grid_sayisi = n
                self._statik_yeniden_ciz()
        if scale is not None:
            self._scale_carpan = float(scale)
            self._apply_scale()
        self.visible = durum
        # Çocukları da gizle/göster (Ursina bazen bunu otomatik yapmayabilir parent UI ise)
        for child in self.children:
            child.enabled = durum
        # Açılırken mevcut A* yolunu ve convex çevresini çiz
        if durum and hasattr(self.ortam_ref, 'harita') and self.ortam_ref.harita:
            harita = self.ortam_ref.harita
            path = getattr(harita, 'a_star_yolu', None)
            if path and len(path) >= 2:
                try:
                    self.update_path(path)
                except Exception:
                    pass
            # Hesaplanmış convex hull varsa uygun renkte göster (cyan)
            hull_data = getattr(harita, 'convex_hull_data', None)
            if hull_data and isinstance(hull_data, dict):
                points = hull_data.get('points')
                if points is not None and len(points) >= 3:
                    try:
                        import numpy as np
                        pts = np.asarray(points)
                        if len(pts.shape) == 2 and pts.shape[1] >= 2:
                            pts_2d = pts[:, :2].tolist() if pts.shape[1] > 2 else pts.tolist()
                            self.update_hull(pts_2d)
                    except Exception:
                        pass
# ============================================================
# HARİTA SİSTEMİ (Matplotlib - Ayrı Pencere)
# ============================================================
class Harita:
    """
    Google Maps benzeri harita sistemi (Matplotlib ile ayrı pencerede).
    ROV'ları ok şeklinde, adaları ve engelleri gösterir.
    """
    def __init__(self, ortam_ref, pencere_boyutu=(800, 800), filo_ref=None):
        """
        Args:
            ortam_ref: Ortam sınıfı referansı
            pencere_boyutu: Harita penceresi boyutu (genişlik, yükseklik)
            filo_ref: Filo referansı (A* için ada çevre noktalarını almak için)
        """
        self.hedef_pozisyon = None  # Hedef pozisyonu (x, y) formatında
        self.ortam_ref = ortam_ref
        self.filo_ref = filo_ref  # Filo referansı (ada_cevre için)
        self.pencere_boyutu = pencere_boyutu
        self.manuel_engeller = []  # Elle eklenen engeller [(x_2d, y_2d), ...]
        self.tespit_edilen_engeller = []  # engel_bul ile tespit edilen noktalar [(x_2d, y_2d), ...] (kırmızı nokta)
        
        # Durum Değişkenleri
        self.gorunur = False
        self.fig = None
        self.ax = None
        
        # Thread Güvenliği İçin İstek Bayrakları
        self._ac_istegi = False
        self._kapat_istegi = False
        
        # Convex Hull görüntüleme
        self.convex_hull_data = None  # {'hull': ConvexHull, 'points': array, 'center': tuple}
        self.goster_convex = False  # Convex hull'u göster/gizle
        
        # A* Yol Planlama
        self.a_star_yolu = None  # [(x1, y1), (x2, y2), ...] formatında A* yolu
        self.goster_a_star = False  # A* yolunu göster/gizle
        
        # Havuz genişliği
        self.havuz_genisligi = getattr(ortam_ref, 'havuz_genisligi', 200)
        
        if getattr(self.ortam_ref, "verbose", False):
            print("✅ Harita sistemi hazır. Kullanım: harita.goster(True)")
    
    def _setup_figure(self):
        """Bu fonksiyon mutlaka ANA THREAD içinde çağrılmalıdır."""
        try:
            # Thread kontrolü - sadece ana thread'de çalış
            import threading
            if threading.current_thread() is not threading.main_thread():
                print("⚠️ [HARITA] _setup_figure() ana thread dışında çağrıldı - atlanıyor")
                return
            
            # Interactive mode'u doğrula (global seviyede zaten açıldı)
            try:
                if not plt.isinteractive():
                    plt.ion()
            except Exception:
                pass
            
            # Eğer fig zaten varsa kapat
            if self.fig is not None:
                try:
                    plt.close(self.fig)
                except Exception:
                    pass
            
            # Pencere boyutu kontrolü - minimum boyut garantisi
            min_figsize = 6.0  # Minimum 6 inç
            fig_width = max(self.pencere_boyutu[0]/100, min_figsize)
            fig_height = max(self.pencere_boyutu[1]/100, min_figsize)
            
            # Yeni pencere oluştur
            self.fig, self.ax = plt.subplots(figsize=(fig_width, fig_height))
            self.fig.canvas.manager.set_window_title('ROV Haritasi')
            
            # Pencere kapatıldığında algıla
            try:
                self.fig.canvas.mpl_connect('close_event', self._on_close)
            except Exception:
                pass
            
            # Pencereyi göster - ÖNCE GÖSTER, SONRA ÇİZ
            try:
                plt.show(block=False)
            except Exception as e:
                print(f"⚠️ [HARITA] plt.show() hatası: {e}")
                return
            
            # İlk çizimi yap (pencere açıldıktan sonra)
            try:
                self._ciz()
            except Exception as e:
                print(f"⚠️ [HARITA] İlk çizim hatası: {e}")
            
            # Çizimi güncelle ve pencereyi öne getir
            try:
                # ÖNEMLİ: canvas.draw() çağrısı - pencereyi güncelle
                self.fig.canvas.draw()
                
                # Pencereyi öne getir (TkAgg için)
                if hasattr(self.fig.canvas, 'manager') and hasattr(self.fig.canvas.manager, 'window'):
                    window = self.fig.canvas.manager.window
                    try:
                        window.lift()
                        window.attributes('-topmost', True)
                        window.after_idle(window.attributes, '-topmost', False)
                    except Exception:
                        pass
            except Exception as win_e:
                print(f"⚠️ [HARITA] Pencere güncellenirken hata: {win_e}")
        except Exception as e:
            print(f"❌ [HARITA] Harita penceresi başlatılamadı: {e}")
            import traceback
            traceback.print_exc()

    def _on_close(self, event):
        """Pencere çarpıdan kapatıldığında."""
        self.gorunur = False
        self.fig = None
        self.ax = None
    
    def _ciz_gps_pin(self, x, y, renk, yon=None, rov_id=None):
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
        
        # ROV ID'sini yazdır
        if rov_id is not None:
            self.ax.text(x, y - pin_boyut * 1.5, f'{rov_id}', 
                        fontsize=9, ha='center', va='top', 
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', 
                                edgecolor='black', alpha=0.8, linewidth=0.5),
                        zorder=12)

    def _ciz_ada_sekli(self, ada_id):
        """
        Ada şeklini filo.ada_cevre() ve Ada() bilgilerini kullanarak dinamik olarak çizer.
        Gerçek ada konumu ve yarıçapını kullanır.
        
        Args:
            ada_id: Ada ID'si
        """
        from matplotlib import patches
        
        # Ada merkez pozisyonunu al
        if not hasattr(self, 'ortam_ref') or not self.ortam_ref:
            return
        
        ada_konum = self.ortam_ref.Ada(ada_id)
        if ada_konum is None:
            return
        
        ada_x, ada_y = ada_konum
        
        # Ada yarıçapını island_positions'dan al (gerçek değer)
        ada_radius = None
        if hasattr(self.ortam_ref, 'island_positions') and ada_id < len(self.ortam_ref.island_positions):
            island_data = self.ortam_ref.island_positions[ada_id]
            if len(island_data) >= 3:
                ada_radius = float(island_data[2])
        
        # Ada çevre noktalarını al (filo.ada_cevre() ile)
        if not self.filo_ref or not hasattr(self.filo_ref, 'ada_cevre'):
            # Fallback: Dairesel çizim
            if ada_radius is None:
                ada_radius = self.havuz_genisligi * 0.08
            
            ada_sekli = patches.Ellipse(
                (ada_x, ada_y), 
                width=ada_radius * 2.0,
                height=ada_radius * 2.0,
                facecolor='#8B5A3C', 
                edgecolor='black', 
                linewidth=2,
                alpha=0.8, 
                zorder=4
            )
            self.ax.add_patch(ada_sekli)
            # Ada ID'sini ada üzerine yaz
            self.ax.text(ada_x, ada_y, f'Ada-{ada_id}', 
                        fontsize=10, fontweight='bold', 
                        color='white', ha='center', va='center',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.6, edgecolor='white', linewidth=1),
                        zorder=6)
            return
        
        try:
            # Ada çevre noktalarını al (offset=0 ile tam çevre)
            ada_cevre_noktalari = self.filo_ref.ada_cevre(offset=0.0)
            
            if not ada_cevre_noktalari or len(ada_cevre_noktalari) == 0:
                # Fallback: Dairesel çizim
                if ada_radius is None:
                    ada_radius = self.havuz_genisligi * 0.08
                
                ada_sekli = patches.Ellipse(
                    (ada_x, ada_y), 
                    width=ada_radius * 2.0,
                    height=ada_radius * 2.0,
                    facecolor='#8B5A3C', 
                    edgecolor='black', 
                    linewidth=2,
                    alpha=0.8, 
                    zorder=4
                )
                self.ax.add_patch(ada_sekli)
                return
            
            # Her ada için nokta sayısını dinamik olarak hesapla
            # ada_cevre() her ada için 12 nokta döndürüyor (30° aralıklarla 0-330°)
            if len(ada_cevre_noktalari) > 0:
                # Ada sayısını island_positions'dan al
                if hasattr(self.ortam_ref, 'island_positions') and self.ortam_ref.island_positions:
                    gercek_ada_sayisi = len(self.ortam_ref.island_positions)
                    nokta_sayisi_per_ada = len(ada_cevre_noktalari) // gercek_ada_sayisi
                else:
                    # Fallback: 12 nokta varsay (ada_cevre() varsayılanı)
                    nokta_sayisi_per_ada = 36
            else:
                nokta_sayisi_per_ada = 36
            ada_sayisi = len(ada_cevre_noktalari) // nokta_sayisi_per_ada
            
            if ada_id >= ada_sayisi:
                # Fallback: Dairesel çizim
                if ada_radius is None:
                    ada_radius = self.havuz_genisligi * 0.1
                
                ada_sekli = patches.Ellipse(
                    (ada_x, ada_y), 
                    width=ada_radius * 2.0,
                    height=ada_radius * 2.0,
                    facecolor='#8B5A3C', 
                    edgecolor='black', 
                    linewidth=2,
                    alpha=0.8, 
                    zorder=4
                )
                self.ax.add_patch(ada_sekli)
                return
            
            # Bu ada için noktaları al
            baslangic_idx = ada_id * nokta_sayisi_per_ada
            bitis_idx = baslangic_idx + nokta_sayisi_per_ada
            
            if bitis_idx > len(ada_cevre_noktalari):
                bitis_idx = len(ada_cevre_noktalari)
            
            ada_noktalari = ada_cevre_noktalari[baslangic_idx:bitis_idx]
            
            # Minimum 3 nokta gerekli (polygon için)
            if len(ada_noktalari) >= 3:
                # Polygon koordinatlarını hazırla
                polygon_xy = []
                for n in ada_noktalari:
                    # Nokta formatı kontrolü: (x, y, z) veya (x, y)
                    if isinstance(n, (list, tuple)) and len(n) >= 2:
                        try:
                            polygon_xy.append((float(n[0]), float(n[1])))
                        except (ValueError, TypeError, IndexError):
                            continue
                
                # Yeterli nokta varsa çiz
                if len(polygon_xy) >= 3:
                    # Kapalı polygon için ilk noktayı sona ekle
                    polygon_xy.append(polygon_xy[0])
                    
                    # Ada şeklini polygon olarak çiz
                    ada_polygon = patches.Polygon(
                        polygon_xy,
                        facecolor='#8B5A3C',
                        edgecolor='black',
                        linewidth=2,
                        alpha=0.8,
                        zorder=4
                    )
                    self.ax.add_patch(ada_polygon)
                    
                    # Ada yarıçapını hesapla (eğer island_positions'dan alınamadıysa)
                    if ada_radius is None:
                        # Noktaların merkeze ortalama uzaklığından hesapla
                        import math
                        uzakliklar = []
                        for px, py in polygon_xy[:-1]:  # Son nokta tekrar olduğu için atla
                            uzaklik = math.sqrt((px - ada_x)**2 + (py - ada_y)**2)
                            uzakliklar.append(uzaklik)
                        ada_radius = sum(uzakliklar) / len(uzakliklar) if uzakliklar else 20.0
                    # Ada ID'sini ada üzerine yaz
                    self.ax.text(ada_x, ada_y, f'Ada-{ada_id}', 
                                fontsize=10, fontweight='bold', 
                                color='white', ha='center', va='center',
                                bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.6, edgecolor='white', linewidth=1),
                                zorder=6)
                    # Ada üzerinde küçük detaylar (ağaç/tepe gibi)
                    detay_positions = [
                        (0.3, 0.4), (-0.4, 0.2), (0.2, -0.3), (-0.3, -0.2), (0.0, 0.5)
                    ]
                    for dx, dy in detay_positions:
                        detay_x = ada_x + dx * ada_radius * 0.6
                        detay_y = ada_y + dy * ada_radius * 0.6
                        self.ax.plot(detay_x, detay_y, 'o', color='#654321', markersize=3, zorder=5)
            else:
                # Yeterli nokta yoksa dairesel çizim
                if ada_radius is None:
                    ada_radius = self.havuz_genisligi * 0.08
                
                ada_sekli = patches.Ellipse(
                    (ada_x, ada_y), 
                    width=ada_radius * 2.0,
                    height=ada_radius * 2.0,
                    facecolor='#8B5A3C', 
                    edgecolor='black', 
                    linewidth=2,
                    alpha=0.8, 
                    zorder=4
                )
                self.ax.add_patch(ada_sekli)
        except Exception as e:
            # Hata durumunda fallback: dairesel çizim
            if ada_radius is None:
                ada_radius = self.havuz_genisligi * 0.1
            
            ada_sekli = patches.Ellipse(
                (ada_x, ada_y), 
                width=ada_radius * 2.0,
                height=ada_radius * 2.0,
                facecolor='#8B5A3C', 
                edgecolor='black', 
                linewidth=2,
                alpha=0.8, 
                zorder=4
            )
            self.ax.add_patch(ada_sekli)
    
    def _ciz(self):
        """Eksenleri temizle ve her şeyi yeniden çiz."""
        if self.ax is None or self.fig is None:
            return
        
        # Pencere kapatılmış olabilir kontrolü
        try:
            self.ax.clear()
        except Exception:
            # Pencere kapatılmış, temizle
            self.fig = None
            self.ax = None
            self.gorunur = False
            return
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
                
                # ROV ID'sini al
                rov_id = None
                if hasattr(rov, 'id'):
                    rov_id = rov.id
                elif hasattr(rov, 'rov_id'):
                    rov_id = rov.rov_id
                
                self._ciz_gps_pin(x_2d, y_2d, renk, yon, rov_id)

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
        
        # Adaları Çiz - ada_cevre() ile gerçek çevre noktalarını kullan
        # Hata toleranslı: Eğer ada_cevre() başarısız olursa fallback (dairesel) çizim kullan
        ada_cevre_basarili = False
        if self.filo_ref and hasattr(self.filo_ref, 'ada_cevre'):
            try:
                # Ada çevre noktalarını al (offset=0 ile tam çevre)
                ada_cevre_noktalari = self.filo_ref.ada_cevre(offset=0.0)
                
                if ada_cevre_noktalari and len(ada_cevre_noktalari) > 0:
                    from matplotlib import patches
                    nokta_sayisi_per_ada = 12
                    
                    # Hata toleranslı hesaplama
                    try:
                        ada_sayisi = len(ada_cevre_noktalari) // nokta_sayisi_per_ada
                    except (ZeroDivisionError, TypeError):
                        ada_sayisi = 0
                    
                    # Her ada için çizim
                    for ada_idx in range(ada_sayisi):
                        try:
                            baslangic_idx = ada_idx * nokta_sayisi_per_ada
                            bitis_idx = baslangic_idx + nokta_sayisi_per_ada
                            
                            # Liste sınır kontrolü
                            if bitis_idx > len(ada_cevre_noktalari):
                                bitis_idx = len(ada_cevre_noktalari)
                            
                            ada_noktalari = ada_cevre_noktalari[baslangic_idx:bitis_idx]
                            
                            # Minimum 3 nokta gerekli (polygon için)
                            if len(ada_noktalari) >= 3:
                                try:
                                    # Polygon olarak çiz (gerçek şekil)
                                    polygon_xy = []
                                    for n in ada_noktalari:
                                        # Nokta formatı kontrolü
                                        if isinstance(n, (list, tuple)) and len(n) >= 2:
                                            try:
                                                polygon_xy.append((float(n[0]), float(n[1])))
                                            except (ValueError, TypeError, IndexError):
                                                continue
                                    
                                    # Yeterli nokta varsa çiz
                                    if len(polygon_xy) >= 3:
                                        # Kapalı polygon için ilk noktayı sona ekle
                                        polygon_xy.append(polygon_xy[0])
                                        
                                        ada_polygon = patches.Polygon(
                                            polygon_xy,
                                            facecolor='#8B5A3C',
                                            edgecolor='black',
                                            linewidth=1.5,
                                            alpha=0.7,
                                            zorder=4
                                        )
                                        self.ax.add_patch(ada_polygon)
                                        ada_cevre_basarili = True
                                except Exception as poly_e:
                                    # Bu ada için polygon çizimi başarısız, devam et
                                    continue
                        except Exception as ada_e:
                            # Bu ada için hata, devam et
                            continue
            except Exception as e:
                print(f"⚠️ [HARITA] Ada çevre noktaları çizilirken hata: {e}")
        
        # Fallback: Eğer ada_cevre() başarısız olduysa veya hiç çizilmediyse dairesel çizim kullan
        if not ada_cevre_basarili:
            # Fallback: Eski yöntem (dairesel)
            if hasattr(self.ortam_ref, 'island_positions') and self.ortam_ref.island_positions:
                    from matplotlib import patches
                    for is_pos in self.ortam_ref.island_positions:
                        if len(is_pos) == 3:
                            rad = is_pos[2]
                        else:
                            rad = self.havuz_genisligi * 0.08
                        
                        ada = patches.Ellipse(
                            (is_pos[0], is_pos[1]), 
                            width=rad * 4.0,
                            height=rad * 3.6,
                            facecolor='#8B5A3C', 
                            edgecolor='black', 
                            alpha=0.7, 
                            zorder=4
                        )
                        self.ax.add_patch(ada)
        else:
            # Fallback: Eski yöntem (dairesel)
            if hasattr(self.ortam_ref, 'island_positions') and self.ortam_ref.island_positions:
                from matplotlib import patches
                for is_pos in self.ortam_ref.island_positions:
                    if len(is_pos) == 3:
                        rad = is_pos[2]
                    else:
                        rad = self.havuz_genisligi * 0.08
                    
                    ada = patches.Ellipse(
                        (is_pos[0], is_pos[1]), 
                        width=rad * 4.0,
                        height=rad * 3.6,
                        facecolor='#8B5A3C', 
                        edgecolor='black', 
                        alpha=0.7, 
                        zorder=4
                    )
                    self.ax.add_patch(ada)

        # Manuel Engeller
        if self.manuel_engeller:
            ex = [p[0] for p in self.manuel_engeller]
            ey = [p[1] for p in self.manuel_engeller]
            self.ax.scatter(ex, ey, c='red', marker='X', s=80, label="Engel", zorder=10,
                          edgecolors='darkred', linewidths=2)

        # Tespit edilen engeller (engel_bul sonucu — kırmızı noktalar)
        if self.tespit_edilen_engeller:
            tx = [p[0] for p in self.tespit_edilen_engeller]
            ty = [p[1] for p in self.tespit_edilen_engeller]
            self.ax.scatter(tx, ty, c='red', marker='o', s=40, label="Tespit", zorder=10,
                          edgecolors='darkred', linewidths=1, alpha=0.9)
        
        # Convex Hull Çizimi
        if self.goster_convex:
            if self.convex_hull_data is None:
                # Debug: convex_hull_data None ise
                pass  # Henüz hull oluşturulmamış
            elif self.convex_hull_data.get('hull') is None:
                # Debug: hull None ise
                pass  # Hull oluşturulamadı
            else:
                try:
                    hull = self.convex_hull_data['hull']
                    points = self.convex_hull_data['points']
                    
                    # Hull'un boyutunu kontrol et (2D veya 3D)
                    if points is not None and len(points) > 0:
                        hull_dim = points.shape[1] if len(points.shape) > 1 else 0
                        
                        if hull_dim == 2:
                            # 2D hull - (x, y) formatında
                            # Harita (x, y) kullanıyor, direkt çiz
                            # Genişletilmiş points array'i kullan (her 5 metrede bir nokta içerir)
                            # Points array'i zaten sıralı olmalı (kenarlar üzerinde interpolasyon yapıldı)
                            if len(points) > 0:
                                # Points array'i zaten sıralı ve genişletilmiş
                                # Kapalı çokgen için ilk noktayı sona ekle
                                hull_points_2d_closed = np.vstack([points, points[0]])
                                self.ax.plot(hull_points_2d_closed[:, 0], hull_points_2d_closed[:, 1], 
                                           'b-', linewidth=2, alpha=0.7, label='Convex Hull', zorder=8)
                        elif hull_dim == 3:
                            # 3D hull - 2D projeksiyon (x-y düzlemi)
                            # Points: (x, y, z) formatında
                            # Harita (x, y) kullanıyor, bu yüzden (x, y) -> (x, y) çiziyoruz
                            if hasattr(hull, 'vertices') and len(hull.vertices) > 0:
                                hull_vertices_3d = points[hull.vertices]
                                # x ve y koordinatlarını al (z derinlik, haritada gösterilmez)
                                hull_points_2d = hull_vertices_3d[:, [0, 1]]  # x ve y koordinatları
                                # Kapalı çokgen için ilk noktayı sona ekle
                                if len(hull_points_2d) > 0:
                                    hull_points_2d_closed = np.vstack([hull_points_2d, hull_points_2d[0]])
                                    self.ax.plot(hull_points_2d_closed[:, 0], hull_points_2d_closed[:, 1], 
                                               'b-', linewidth=2, alpha=0.7, label='Convex Hull', zorder=8)
                        
                        # Hull merkezini göster
                        center = self.convex_hull_data.get('center')
                        if center:
                            if len(center) == 3:
                                # 3D center -> 2D (x, y)
                                self.ax.plot(center[0], center[1], 'bo', markersize=8, 
                                           markeredgecolor='darkblue', markeredgewidth=2, 
                                           label='Hull Merkezi', zorder=9)
                            elif len(center) == 2:
                                # 2D center (x, y) -> haritada (x, y)
                                self.ax.plot(center[0], center[1], 'bo', markersize=8, 
                                           markeredgecolor='darkblue', markeredgewidth=2, 
                                           label='Hull Merkezi', zorder=9)
                except Exception as e:
                    print(f"⚠️ [HARITA] Convex hull çizilirken hata: {e}")
                    import traceback
                    traceback.print_exc()
        
        # A* Yolu Çizimi
        if self.goster_a_star and self.a_star_yolu and len(self.a_star_yolu) > 0:
            try:
                # Yolu çiz (yeşil çizgi)
                path_x = [p[0] for p in self.a_star_yolu]
                path_y = [p[1] for p in self.a_star_yolu]
                self.ax.plot(path_x, path_y, 'g-', linewidth=3, alpha=0.8, 
                           label='A* Yolu', zorder=7)
                
                # Başlangıç noktasını işaretle (yeşil daire)
                if len(self.a_star_yolu) > 0:
                    self.ax.plot(path_x[0], path_y[0], 'go', markersize=10, 
                               markeredgecolor='darkgreen', markeredgewidth=2, 
                               label='Başlangıç', zorder=10)
                
                # Hedef noktasını işaretle (kırmızı daire)
                if len(self.a_star_yolu) > 1:
                    self.ax.plot(path_x[-1], path_y[-1], 'ro', markersize=10, 
                               markeredgecolor='darkred', markeredgewidth=2, 
                               label='Hedef', zorder=10)
            except Exception as e:
                print(f"⚠️ [HARITA] A* yolu çizilirken hata: {e}")
                import traceback
                traceback.print_exc()
        
        # Legend (engeller ve convex hull için)
        legend_items = []
        if self.manuel_engeller:
            legend_items.append('Engel')
        if self.tespit_edilen_engeller:
            legend_items.append('Tespit')
        if self.goster_convex and self.convex_hull_data and self.convex_hull_data.get('hull') is not None:
            legend_items.append('Convex Hull')
            legend_items.append('Hull Merkezi')
        if self.goster_a_star and self.a_star_yolu and len(self.a_star_yolu) > 0:
            legend_items.append('A* Yolu')
            legend_items.append('Başlangıç')
            legend_items.append('Hedef')
        
        if legend_items:
            self.ax.legend(loc='upper right', fontsize=9)

        # Thread-safe çizim
        try:
            import threading
            if threading.current_thread() is threading.main_thread() and self.fig is not None:
                self.fig.canvas.draw_idle()
        except Exception:
            # Hata durumunda sessizce devam et
            pass
    
    def goster(self, durum=None, convex=False, a_star=False):
        """
        Konsoldan (Shell Thread) çağrılır. 
        Sadece istek bırakır, işlemi update() (Main Thread) yapar.
        
        Args:
            durum: True/False - Haritayı aç/kapat
            convex: True/False - Convex hull'u göster/gizle
            a_star: True/False - A* yolunu göster/gizle
        """
        # Eğer sadece convex parametresi verilmişse
        if durum is None:
            self.goster_convex = convex if isinstance(convex, bool) else (str(convex).lower() == "true")
            if getattr(self.ortam_ref, "verbose", False):
                print(f"✅ [HARITA] Convex hull görüntüleme: {self.goster_convex}")
            return
        
        # String gelme ihtimaline karşı kontrol ("True" -> True)
        if isinstance(durum, str):
            durum = durum.lower() == "true"
        
        if isinstance(convex, str):
            convex = convex.lower() == "true"
            
        if durum:
            self._ac_istegi = True
            self._kapat_istegi = False
        else:
            self._kapat_istegi = True
            self._ac_istegi = False
        
        # Convex hull görüntüleme ayarı
        self.goster_convex = convex
        if convex and getattr(self.ortam_ref, "verbose", False):
            print(f"✅ [HARITA] Convex hull görüntüleme aktif: {self.goster_convex}")
            if self.convex_hull_data:
                print(f"   Hull data mevcut: {self.convex_hull_data.get('hull') is not None}")
            else:
                print(f"   ⚠️ Hull data henüz oluşturulmamış. formasyon_sec() veya guvenlik_hull_olustur() çağırın.")
        
        # A* yol görüntüleme ayarı
        if isinstance(a_star, str):
            a_star = a_star.lower() == "true"
        self.goster_a_star = a_star
        if a_star:
            print(f"✅ [HARITA] A* yol görüntüleme aktif: {self.goster_a_star}")
            if self.a_star_yolu:
                print(f"   A* yolu mevcut: {len(self.a_star_yolu)} nokta")
            else:
                print(f"   ⚠️ A* yolu henüz hesaplanmamış. a_star_yolu_hesapla() çağırın.")

    def a_star_yolu_hesapla(self, start: Tuple[float, float], goal: Tuple[float, float],
                            safety_margin: float = 2.0) -> Optional[List[Tuple[float, float]]]:
        """
        A* algoritması kullanarak başlangıçtan hedefe yol hesaplar.
        
        Args:
            start: (x, y) başlangıç koordinatları (metre)
            goal: (x, y) hedef koordinatları (metre)
            safety_margin: Engel etrafında güvenlik mesafesi (metre, varsayılan: 2.0)
        
        Returns:
            Optional[List[Tuple[float, float]]]: Bulunan yol [(x1, y1), (x2, y2), ...] veya None
        """
        try:
            from .a_star import AStarPlanner
            
            # Harita sınırlarını al
            min_x = -self.havuz_genisligi
            max_x = self.havuz_genisligi
            min_y = -self.havuz_genisligi
            max_y = self.havuz_genisligi
            map_bounds = (min_x, max_x, min_y, max_y)
            
            # Engelleri topla
            obstacles = []
            
            # Manuel engeller
            for engel in self.manuel_engeller:
                if len(engel) >= 2:
                    # Engel formatı: (x, y) veya (x, y, radius)
                    if len(engel) >= 3:
                        obstacles.append((engel[0], engel[1], engel[2]))
                    else:
                        # Varsayılan yarıçap
                        obstacles.append((engel[0], engel[1], 5.0))
            
            # Adalar - ada_cevre() fonksiyonunu kullanarak çevre noktalarını al
            # Bu, adaların gerçek şeklini daha doğru temsil eder
            polygon_obstacles = []  # Polygon engeller (ada çevre noktaları)
            
            if self.filo_ref and hasattr(self.filo_ref, 'ada_cevre'):
                try:
                    # Ada çevre noktalarını al (offset=0 ile tam çevre)
                    ada_cevre_noktalari = self.filo_ref.ada_cevre(offset=0.0)
                    
                    # Her ada için çevre noktalarını polygon olarak ekle
                    # ada_cevre() her ada için 12 nokta döndürür
                    if ada_cevre_noktalari and len(ada_cevre_noktalari) > 0:
                        nokta_sayisi_per_ada = 12
                        ada_sayisi = len(ada_cevre_noktalari) // nokta_sayisi_per_ada
                        
                        for ada_idx in range(ada_sayisi):
                            baslangic_idx = ada_idx * nokta_sayisi_per_ada
                            bitis_idx = baslangic_idx + nokta_sayisi_per_ada
                            ada_noktalari = ada_cevre_noktalari[baslangic_idx:bitis_idx]
                            
                            if len(ada_noktalari) >= 3:
                                # Polygon olarak ekle (sadece x, y koordinatları)
                                polygon = [(n[0], n[1]) for n in ada_noktalari]
                                polygon_obstacles.append(polygon)
                                
                                # Ayrıca dairesel engel olarak da ekle (fallback için)
                                # Ada konumunu al
                                if hasattr(self.ortam_ref, 'Ada'):
                                    try:
                                        ada_konum = self.ortam_ref.Ada(ada_idx)
                                        if ada_konum:
                                            ada_x, ada_y = ada_konum
                                            # Yarıçapı çevre noktalarından hesapla
                                            import math
                                            max_radius = 0.0
                                            for nokta in ada_noktalari:
                                                dist = math.sqrt((nokta[0] - ada_x)**2 + (nokta[1] - ada_y)**2)
                                                max_radius = max(max_radius, dist)
                                            obstacles.append((ada_x, ada_y, max_radius))
                                    except:
                                        pass
                except Exception as e:
                    print(f"⚠️ [HARITA] Ada çevre noktaları alınırken hata: {e}")
                    import traceback
                    traceback.print_exc()
            
            # Fallback: Eski yöntem (sadece merkez ve yarıçap) - polygon yoksa
            if not polygon_obstacles and hasattr(self.ortam_ref, 'island_positions') and self.ortam_ref.island_positions:
                for is_pos in self.ortam_ref.island_positions:
                    if len(is_pos) >= 3:
                        # Güvenlik mesafesi ile genişletilmiş yarıçap
                        obstacles.append((is_pos[0], is_pos[1], is_pos[2] + safety_margin))
            
            # A* planner oluştur
            planner = AStarPlanner(grid_size=1.0)  # 1 metre grid çözünürlüğü
            
            # Yolu hesapla (polygon engelleri ile)
            path = planner.find_path(start, goal, obstacles, map_bounds, safety_margin, 
                                   polygon_obstacles=polygon_obstacles if polygon_obstacles else None)
            
            if path:
                self.a_star_yolu = path
                print(f"✅ [HARITA] A* yolu hesaplandı: {len(path)} nokta")
                # Minimap açıksa A* rotasını hemen çiz (filo.minimap() ile açılan haritada)
                if hasattr(self.ortam_ref, 'minimap') and self.ortam_ref.minimap and getattr(self.ortam_ref.minimap, 'visible', False):
                    try:
                        self.ortam_ref.minimap.update_path(path)
                    except Exception:
                        pass
                return path
            else:
                self.a_star_yolu = None
                print(f"❌ [HARITA] A* yolu bulunamadı!")
                return None
                
        except Exception as e:
            print(f"❌ [HARITA] A* yolu hesaplanırken hata: {e}")
            import traceback
            traceback.print_exc()
            return None
    
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
                try:
                    self._setup_figure()
                    self.gorunur = True
                    print("✅ Harita açıldı.")
                    # Pencereyi öne getir ve görünür yap
                    if self.fig is not None:
                        try:
                            # TkAgg backend için pencereyi öne getir
                            if hasattr(self.fig.canvas, 'manager') and hasattr(self.fig.canvas.manager, 'window'):
                                window = self.fig.canvas.manager.window
                                if hasattr(window, 'lift'):
                                    window.lift()
                                if hasattr(window, 'wm_attributes'):
                                    window.wm_attributes('-topmost', True)
                                    window.wm_attributes('-topmost', False)
                        except Exception as e:
                            print(f"⚠️ [HARITA] Pencere öne getirilemedi: {e}")
                except Exception as e:
                    print(f"❌ [HARITA] Harita açılırken hata: {e}")
                    import traceback
                    traceback.print_exc()

        # 2.5 Minimap convex hull senkronizasyonu (ana harita kapalı olsa bile minimap açıksa güncelle)
        if hasattr(self.ortam_ref, 'minimap') and self.ortam_ref.minimap and getattr(self.ortam_ref.minimap, 'visible', False) and self.convex_hull_data:
            if not hasattr(self, '_minimap_hull_cnt'):
                self._minimap_hull_cnt = 0
            self._minimap_hull_cnt += 1
            if self._minimap_hull_cnt >= 30:
                self._minimap_hull_cnt = 0
                try:
                    points = self.convex_hull_data.get('points')
                    if points is not None and len(points) >= 3:
                        pts = np.asarray(points)
                        if len(pts.shape) == 2 and pts.shape[1] >= 2:
                            pts_2d = pts[:, :2] if pts.shape[1] > 2 else pts
                            self.ortam_ref.minimap.update_hull(pts_2d)
                except Exception:
                    pass

        # 3. Rutin Çizim Güncellemesi
        if self.gorunur and self.fig is not None:
            # Thread kontrolü - sadece ana thread'de çalış
            import threading
            if threading.current_thread() is not threading.main_thread():
                return  # Ana thread dışında çalışma
            
            # Çizim performansı için sayaç mekanizması - Her 30 frame'de bir güncelle
            if not hasattr(self, '_up_cnt'):
                self._up_cnt = 0
            if not hasattr(self, '_update_interval'):
                self._update_interval = 30  # 30 frame'de bir güncelle
            
            self._up_cnt += 1
            
            # Çizim güncellemesi - 30 karede bir (Performans)
            if self._up_cnt >= self._update_interval:
                self._up_cnt = 0
                try:
                    # Havuz genişliğini güncelle (sim_olustur'da değişebilir)
                    self.havuz_genisligi = getattr(self.ortam_ref, 'havuz_genisligi', 200)
                    self._ciz()
                    
                    # Çizimi güncelle (her 30 frame'de bir - performans için)
                    try:
                        # draw_idle() non-blocking ama bazen yeterince hızlı güncellemez
                        # Bu yüzden her 30 frame'de bir draw() kullanıyoruz
                        self.fig.canvas.draw()
                    except Exception:
                        # Pencere kapatılmış olabilir
                        self.fig = None
                        self.ax = None
                        self.gorunur = False
                        return
                    
                    # Minimap'i senkronize et (eğer varsa ve görünürse)
                    if hasattr(self.ortam_ref, 'minimap') and self.ortam_ref.minimap and self.ortam_ref.minimap.visible:
                        try:
                            # Convex hull'u güncelle
                            if self.goster_convex and self.convex_hull_data:
                                points = self.convex_hull_data.get('points')
                                if points is not None and len(points) > 0:
                                    if len(points.shape) > 1 and points.shape[1] == 2:
                                        self.ortam_ref.minimap.update_hull(points)
                                    elif len(points.shape) > 1 and points.shape[1] == 3:
                                        points_2d = points[:, [0, 1]]
                                        self.ortam_ref.minimap.update_hull(points_2d)
                            
                            # A* yolunu güncelle
                            if self.goster_a_star and self.a_star_yolu:
                                self.ortam_ref.minimap.update_path(self.a_star_yolu)
                        except Exception:
                            pass  # Minimap güncelleme hatası - sessizce devam et
                except Exception:
                    # Pencere harici bir sebeple kapandıysa
                    self.fig = None
                    self.ax = None
                    self.gorunur = False
                    return
            
            # ÖNEMLİ: flush_events() her karede çağrılmalı (pencere donmasını önlemek için)
            # Bu, çizim güncellemesinden bağımsız olarak GUI olay döngüsünü canlı tutar
            try:
                if hasattr(self.fig.canvas, 'flush_events'):
                    self.fig.canvas.flush_events()
            except Exception:
                # Pencere kapatılmış veya hata - sessizce devam et
                pass
    
    def ekle(self, x_2d, y_2d=None, tip='engel'):
        """
        Haritaya elle engel/nesne ekler.
        
        Args:
            x_2d: 2D düzlem X koordinatı VEYA dizi şeklinde noktalar [(x, y), ...] veya [(x, y, z), ...]
            y_2d: 2D düzlem Y koordinatı (x_2d dizi ise None olabilir)
            tip: Nesne tipi ('engel', 'hedef', vb.)
        
        Returns:
            bool: Başarılı ise True
        """
        # Dizi kontrolü: x_2d bir dizi/liste ise tüm noktaları işle
        if isinstance(x_2d, (list, tuple, np.ndarray)):
            noktalar = x_2d
            basarili_sayisi = 0
            
            for nokta in noktalar:
                # Her nokta 2D (x, y) veya 3D (x, y, z) olabilir
                if isinstance(nokta, (list, tuple, np.ndarray)) and len(nokta) >= 2:
                    x = float(nokta[0])
                    y = float(nokta[1])
                    # z varsa yok sayılır (harita 2D)
                    
                    # Tek nokta ekleme işlemi
                    if tip == 'engel':
                        # Engel listesine ekle
                        self.manuel_engeller.append((x, y))
                        
                        # Ortam'a engel entity'si ekle
                        if hasattr(self.ortam_ref, 'engeller'):
                            # Engel entity'si oluştur (görünmez hitbox)
                            engel = Entity(
                                model='icosphere',
                                position=sim_to_ursina(x, y, self.ortam_ref.SEA_FLOOR_Y),
                                scale=(20, 20, 20),
                                visible=False,
                                collider='sphere',
                                color=color.red,
                                unlit=True
                            )
                            self.ortam_ref.engeller.append(engel)
                        
                        basarili_sayisi += 1
            
            # Haritayı güncelle (sadece görünürse ve pencere varsa)
            if self.gorunur and self.fig is not None and basarili_sayisi > 0:
                try:
                    self._ciz()
                    # Çizimi hemen güncelle
                    if self.fig is not None:
                        self.fig.canvas.draw()
                except Exception as e:
                    print(f"⚠️ [HARITA] Ekleme sonrası çizim hatası: {e}")
            
            if basarili_sayisi > 0:
                print(f"✅ {basarili_sayisi} nokta eklendi (toplam {len(noktalar)} nokta)")
                return True
            return False
        
        # Tek nokta ekleme (eski davranış)
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
                try:
                    self._ciz()
                    # Çizimi hemen güncelle
                    if self.fig is not None:
                        self.fig.canvas.draw()
                except Exception as e:
                    print(f"⚠️ [HARITA] Ekleme sonrası çizim hatası: {e}")
            print(f"✅ Engel eklendi: ({x_2d:.1f}, {y_2d:.1f})")
            return True
        
        return False

    def tespit_engelleri_guncelle(self, noktalar_2d, debug=False):
        """
        engel_bul ile tespit edilen engelleri haritada kırmızı noktalar olarak günceller.
        debug=True ise önce eski tespit noktalarını siler, sonra yenilerini ekler.
        noktalar_2d: [(x, y), ...] simülasyon 2D koordinatları (x = Ursina x, y = Ursina z).
        """
        if debug:
            self.tespit_edilen_engeller.clear()
        if noktalar_2d:
            new_list = [(float(p[0]), float(p[1])) for p in noktalar_2d if len(p) >= 2]
            if debug:
                self.tespit_edilen_engeller = new_list
            else:
                self.tespit_edilen_engeller.extend(new_list)
        if self.gorunur and self.fig is not None:
            try:
                self._ciz()
                self.fig.canvas.draw_idle()
            except Exception as e:
                print(f"⚠️ [HARITA] Tespit engelleri çizilirken hata: {e}")

    def tespit_engelleri_temizle(self):
        """Haritadaki tespit edilen engel noktalarını (kırmızı noktalar) siler."""
        self.tespit_edilen_engeller.clear()
        if self.gorunur and self.fig is not None:
            try:
                self._ciz()
                self.fig.canvas.draw_idle()
            except Exception as e:
                print(f"⚠️ [HARITA] Tespit engelleri temizlenirken hata: {e}")
    
    def temizle(self):
        """Haritayı temizler (elle eklenen engelleri siler)"""
        self.manuel_engeller = []
        if self.gorunur and self.fig is not None:
            self._ciz()
        print("Harita temizlendi (elle eklenen engeller silindi)")
    
    def kapat(self):
        """Harita penceresini tamamen kapatır"""
        self.goster(False)


def _load_obj_as_mesh(obj_path):
    """
    OBJ dosyasını yükler, quad yüzleri üçgene çevirir; Ursina 7.x mesh uyumsuzluğunu giderir.
    Cinema 4D / v/vt formatı ve quad face'leri destekler.
    Returns:
        Ursina Mesh veya None (hata/boş dosya).
    """
    if not obj_path or not os.path.exists(obj_path):
        return None
    v_list = []
    vt_list = []
    vertices = []
    uvs = []
    triangles = []
    try:
        with open(obj_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if not parts:
                    continue
                if parts[0] == 'v' and len(parts) >= 4:
                    v_list.append((float(parts[1]), float(parts[2]), float(parts[3])))
                elif parts[0] == 'vt' and len(parts) >= 3:
                    vt_list.append((float(parts[1]), float(parts[2])))
                elif parts[0] == 'f':
                    face_verts = []
                    for i in range(1, len(parts)):
                        seg = parts[i].split('/')
                        try:
                            vi = int(seg[0])
                        except (ValueError, IndexError):
                            continue
                        if vi < 0:
                            vi = len(v_list) + vi
                        else:
                            vi -= 1
                        vti = 0
                        if len(seg) > 1 and seg[1].strip():
                            try:
                                vti = int(seg[1])
                            except ValueError:
                                pass
                        if vti < 0:
                            vti = len(vt_list) + vti
                        elif vti > 0:
                            vti -= 1
                        if vi < 0 or vi >= len(v_list):
                            continue
                        u, v = vt_list[vti] if vti < len(vt_list) else (0.0, 0.0)
                        face_verts.append((vi, u, v))
                    if len(face_verts) == 3:
                        base = len(vertices)
                        for vi, u, v in face_verts:
                            vertices.append(v_list[vi])
                            uvs.append((u, v))
                        triangles.append((base, base + 1, base + 2))
                    elif len(face_verts) == 4:
                        base = len(vertices)
                        for vi, u, v in face_verts:
                            vertices.append(v_list[vi])
                            uvs.append((u, v))
                        triangles.append((base, base + 1, base + 2))
                        triangles.append((base, base + 2, base + 3))
    except Exception:
        return None
    if not vertices or not triangles:
        return None
    try:
        return Mesh(vertices=vertices, triangles=triangles, uvs=uvs, mode='triangle', static=True)
    except Exception:
        return None


class Ortam:
    def __init__(self, verbose=False):
        self.verbose = verbose  # Log mesajlarını kontrol eder
        # --- Ursina Ayarları ---
        self.app = Ursina(
            vsync=False,
            development_mode=False,
            show_ursina_splash=False,
            borderless=False,
            title="FıratROVNet Simülasyonu"
        )
        
        if not self.verbose:
            from contextlib import redirect_stdout, redirect_stderr
            import io
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                window.fullscreen = False
                window.exit_button.visible = False
                window.fps_counter.enabled = True
                window.size = (1280, 720)  # Daha geniş pencere boyutu (16:9 aspect ratio)
                window.center_on_screen()
                application.run_in_background = True
                window.color = color.rgb(10, 30, 50)  # Arka plan
        else:
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
        
        # Su yüzeyi texture: önce repo_root, yoksa cwd ile dene
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        water_texture_path = os.path.join(repo_root, "Models-3D", "water", "my_models", "water4.jpg")
        normal_map_path = os.path.join(repo_root, "Models-3D", "water", "my_models", "map", "water4_normal.png")
        if not os.path.exists(water_texture_path):
            water_texture_path = os.path.join(os.getcwd(), "Models-3D", "water", "my_models", "water4.jpg")
        if not os.path.exists(normal_map_path):
            normal_map_path = os.path.join(os.getcwd(), "Models-3D", "water", "my_models", "map", "water4_normal.png")
        water_texture = None
        if os.path.exists(water_texture_path):
            try:
                water_texture = Texture(water_texture_path)
            except Exception:
                water_texture = None
        normals_texture = None
        if os.path.exists(normal_map_path):
            try:
                normals_texture = Texture(normal_map_path)
            except Exception:
                normals_texture = None
        self.ocean_surface = Entity(
            model="plane",
            scale=(500, 1, 500),
            position=(0, self.WATER_SURFACE_Y_BASE, 0),
            texture=water_texture,
            texture_scale=(1, 1),  # Tekrarlı su dokusu görünür olsun
            normals=normals_texture,
            double_sided=True,
            color=color.rgb(0.5, 0.65, 0.9),
            alpha=0.25,  # Texture net görünsün (senaryo.guncelle ile animasyon)
            transparent=True,
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
        # Hem Ursina ana döngüsü hem de senaryo.guncelle() ile çağrılabilir.
        def update_ocean():
            try:
                dt = getattr(time, 'dt', None)
                if dt is None or not (dt > 0):
                    dt = 0.016
            except Exception:
                dt = 0.016
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
                collider='box',
                unlit=False,
                alpha=1.0,
                transparent=True,
                render_queue=0
            )
        else:
            self.ocean_taban = None


        
        # Ada modeli (su yüzeyinin üstünde, deniz tabanına değen)
        # 1-5 arasında random ada oluştur
        # Models-3D klasörü proje kök dizininde (CWD'de)
        # Ursina relative path'i tercih eder, bu yüzden önce relative path dene
        # Windows uyumlu path
        island_model_path_rel = "Models-3D/lowpoly-island/source/island1_design2_c4d.obj"
        island_texture_path_rel = "Models-3D/lowpoly-island/textures/textureSurface_Color_2.jpg"
        
        # Absolute path'i de kontrol et (script'in bulunduğu yerden)
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        island_model_path_abs = os.path.join(script_dir, "Models-3D", "lowpoly-island", "source", "island1_design2_c4d.obj")
        island_texture_path_abs = os.path.join(script_dir, "Models-3D", "lowpoly-island", "textures", "textureSurface_Color_2.jpg")
        
        # Path seçimi: Önce relative path'i kontrol et (Ursina için tercih edilen)
        island_model_path = None
        island_texture_path = None
        
        if os.path.exists(island_model_path_rel):
            island_model_path = island_model_path_rel
        elif os.path.exists(island_model_path_abs):
            # Absolute path varsa, CWD'ye göre relative path'e çevir
            try:
                cwd = os.getcwd()
                island_model_path = os.path.relpath(island_model_path_abs, cwd)
                # Eğer relative path oluşturulamazsa veya dosya yoksa, absolute kullan
                if not os.path.exists(island_model_path):
                    island_model_path = island_model_path_abs
            except (ValueError, OSError):
                # Relative path oluşturulamazsa absolute kullan
                island_model_path = island_model_path_abs
        else:
            # Hiçbiri yoksa relative path'i kullan (Ursina kendi path çözümlemesini yapacak)
            island_model_path = island_model_path_rel
        
        if os.path.exists(island_texture_path_rel):
            island_texture_path = island_texture_path_rel
        elif os.path.exists(island_texture_path_abs):
            # Absolute path varsa, CWD'ye göre relative path'e çevir
            try:
                cwd = os.getcwd()
                island_texture_path = os.path.relpath(island_texture_path_abs, cwd)
                # Eğer relative path oluşturulamazsa veya dosya yoksa, absolute kullan
                if not os.path.exists(island_texture_path):
                    island_texture_path = island_texture_path_abs
            except (ValueError, OSError):
                # Relative path oluşturulamazsa absolute kullan
                island_texture_path = island_texture_path_abs
        else:
            # Hiçbiri yoksa relative path'i kullan (Ursina kendi path çözümlemesini yapacak)
            island_texture_path = island_texture_path_rel
        
        # Ursina için path'leri normalize et (forward slash kullan)
        if os.path.sep == '\\':  # Windows
            if island_model_path:
                island_model_path = island_model_path.replace('\\', '/')
            if island_texture_path:
                island_texture_path = island_texture_path.replace('\\', '/')
        
        # =============================
        # ADA BOYUT AYARLARI (TEK YER)
        # =============================
        ISLAND_BASE_SCALE = 0.25      # Genel ada boyutu
        ISLAND_SCALE_RANDOM = (0.7, 1.1)  # Random çeşitlilik
        
        # Ada pozisyonlarını sakla (ROV yerleştirme için)
        self.island_positions = []
        # Ada entity'lerini sakla (çarpışma kontrolü için)
        self.island_entities = []
        
        # Havuz genişliği (varsayılan 200, sim_olustur'da güncellenebilir)
        self.havuz_genisligi = 200
        
        # Model ve texture kontrolü (en az bir yerde varsa kullan)
        model_exists = os.path.exists(island_model_path) if island_model_path else False
        texture_exists = os.path.exists(island_texture_path) if island_texture_path else False
        
        if model_exists or os.path.exists(island_model_path_rel) or os.path.exists(island_model_path_abs):
            # ============================================================
            # ADA OLUŞTURMA AYARLARI
            # ============================================================
            n_islands = random.randint(2, 4)  # Azaltıldı: 1-3 arası random ada sayısı
            
            # Engel listesini hazırla (eğer yoksa oluştur)
            if not hasattr(self, 'engeller'):
                self.engeller = []
            
            # Ada Y pozisyonu (su yüzeyinin üstünde sabit)
            max_wave_height = self.WATER_SURFACE_Y_BASE + 1.5
            island_y_position = max_wave_height + 5
            
            # Havuz sınırları: +-havuz_genisligi (yani +-200 birim)
            # X ve Z eksenleri random, Y ekseni su yüzeyinin üstünde sabit
            # Güvenlik payı: Adaların yarıçapı olduğu için kenarlara yerleşen adalar
            # havuz dışına taşmaması için güvenlik payı ekleniyor (tahmini maksimum ada yarıçapı: 90.0)
            guvenli_sinir = max(10.0, self.havuz_genisligi - 15.0)
            min_x = -guvenli_sinir
            max_x = guvenli_sinir
            min_z = -guvenli_sinir
            max_z = guvenli_sinir
            
            # Mevcut ada pozisyonları (çakışma kontrolü için)
            placed_island_positions = []
            
            # ============================================================
            # HER ADA İÇİN OLUŞTURMA DÖNGÜSÜ
            # ============================================================
            for island_idx in range(n_islands):
                # --- 1. ÖLÇEK HESAPLAMA (TEK PARAMETRE) ---
                scale_factor = ISLAND_BASE_SCALE * random.uniform(*ISLAND_SCALE_RANDOM)
                visual_scale = (scale_factor, scale_factor, scale_factor)
                
                # Tahmini yarıçap (pozisyon hesaplamak için, sonra gerçek değerle güncellenecek)
                # Varsayılan model genişliği 140 birim (fallback)
                estimated_radius = (140.0 * scale_factor) / 2
                
                # Minimum mesafe (tahmini ada yarıçapı kadar)
                min_distance_between_islands = estimated_radius
                
                # --- 2. GÜVENLİ POZİSYON BULMA (X ve Z random, Y sabit) ---
                # Ada yarıçapını hesaba katarak havuz sınırlarını daralt
                # Ada kenarlarının havuz sınırları içinde kalması için
                min_x_safe = min_x + estimated_radius
                max_x_safe = max_x - estimated_radius
                min_z_safe = min_z + estimated_radius
                max_z_safe = max_z - estimated_radius
                
                # Eğer ada çok büyükse ve havuz sınırlarına sığmıyorsa, merkeze yerleştir
                if min_x_safe >= max_x_safe or min_z_safe >= max_z_safe:
                    # Ada çok büyük, merkeze yerleştir
                    island_x = 0.0
                    island_z = 0.0
                else:
                    island_x, island_z = self._find_safe_island_position(
                        placed_island_positions=placed_island_positions,
                        min_x=min_x_safe,
                        max_x=max_x_safe,
                        min_z=min_z_safe,
                        max_z=max_z_safe,
                        min_distance=min_distance_between_islands,
                        max_attempts=100
                    )
                
                # --- 3. ADA ENTITY OLUŞTUR ---
                # Ursina 7.x: OBJ quad'ları bazen triangles:0 verir; önce doğrudan, olmazsa _load_obj_as_mesh ile üçgenleyip yükle
                island = None
                island_radius = estimated_radius
                island_model_for_entity = None
                try:
                    island = Entity(
                        model=island_model_path,
                        position=(island_x, island_y_position, island_z),
                        scale=visual_scale,
                        texture=island_texture_path if (texture_exists or os.path.exists(island_texture_path_rel) or os.path.exists(island_texture_path_abs)) else None,
                        collider='box',
                        unlit=False,
                        double_sided=True,
                        color=color.white,
                        alpha=1.0,
                        transparent=True,
                        render_queue=0
                    )
                    if hasattr(island, 'model') and island.model:
                        tri = getattr(island.model, 'triangles', None)
                        idx = getattr(island.model, 'indices', None)
                        n_tri = (len(tri) if tri is not None else 0) or (len(idx) // 3 if idx is not None else 0)
                        if n_tri == 0:
                            destroy(island)
                            island = None
                except Exception:
                    island = None
                if island is None and island_model_path:
                    island_model_for_entity = _load_obj_as_mesh(island_model_path)
                if island is None and island_model_for_entity is not None:
                    try:
                        island = Entity(
                            model=island_model_for_entity,
                            position=(island_x, island_y_position, island_z),
                            scale=visual_scale,
                            texture=island_texture_path if (texture_exists or os.path.exists(island_texture_path_rel) or os.path.exists(island_texture_path_abs)) else None,
                            collider='box',
                            unlit=False,
                            double_sided=True,
                            color=color.white,
                            alpha=1.0,
                            transparent=True,
                            render_queue=0
                        )
                    except Exception:
                        island = None
                if island is None:
                    if island_idx == 0:
                        print("⚠️ Ada OBJ modeli yüklenemedi (Ursina mesh formatı uyumsuz), cube kullanılıyor.")
                    island = Entity(
                        model='cube',
                        position=(island_x, island_y_position, island_z),
                        scale=visual_scale,
                        collider='box',
                        color=color.white,
                        alpha=1.0
                    )
                    island_radius = estimated_radius

                if island is not None:
                    # --- 4. GERÇEK YARIÇAP HESAPLAMA (Modelden otomatik) ---
                    try:
                        if island_radius is None or (hasattr(island, 'model') and hasattr(island.model, 'bounds') and island.model.bounds):
                            if hasattr(island.model, 'bounds') and island.model.bounds:
                                min_b, max_b = island.model.bounds
                                model_size = max_b - min_b
                                island_radius = max(model_size.x, model_size.z) * island.world_scale.x / 2
                            else:
                                island_radius = estimated_radius
                    except Exception:
                        island_radius = estimated_radius
                if island_radius is None:
                    island_radius = estimated_radius
 
  
                
                # İlk adayı self.island olarak sakla (geriye uyumluluk için)
                if island_idx == 0:
                    self.island = island
                
                # Ada entity'lerini sakla (çarpışma kontrolü için)
                if not hasattr(self, 'island_entities'):
                    self.island_entities = []
                self.island_entities.append(island)
                
                # Ada pozisyonunu ve yarıçapını sakla
                # Koordinat sistemi: (x_2d, y_2d, radius) - z_depth her zaman aynı (su yüzeyinin üstünde)
                # radius: Ada yarıçapı (harita çizimi için)
                self.island_positions.append((island_x, island_z, island_radius))  # (x_2d, y_2d, radius)
                
                # Yerleştirilen ada pozisyonunu kaydet (sonraki adalar için çakışma kontrolü)
                placed_island_positions.append((island_x, island_z))
        else:
            # Fallback: Ada yoksa None
            self.island = None
            self.island_positions = []
            self.island_entities = []
        
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
        # engeller listesi oluşturuldu (ada varsa)
        # Eğer ada yoksa veya engeller listesi oluşturulmadıysa, şimdi oluştur
        if not hasattr(self, 'engeller'):
            self.engeller = []

        # Konsol verileri
        self.konsol_verileri = {}
        
        # Harita sistemi (Matplotlib - ayrı pencere)
        try:
            # Filo referansını al (varsa)
            filo_ref = getattr(self, 'filo', None)
            self.harita = Harita(ortam_ref=self, pencere_boyutu=(800, 800), filo_ref=filo_ref)
            if self.verbose:
                print("✅ Harita sistemi başarıyla oluşturuldu (Matplotlib penceresi)")
        except Exception as e:
            print(f"❌ Harita oluşturulurken hata: {e}")
            import traceback
            traceback.print_exc()
            self.harita = None
        
        # Minimap sistemi (Ursina UI - ekran üzerinde)
        try:
            # Filo referansını al (varsa)
            filo_ref = getattr(self, 'filo', None)
            self.minimap = Minimap(ortam_ref=self, filo_ref=filo_ref, visible=False)
            if self.verbose:
                print("✅ Minimap sistemi başarıyla oluşturuldu")
        except Exception as e:
            print(f"❌ Minimap oluşturulurken hata: {e}")
            import traceback
            traceback.print_exc()
            self.minimap = None
    
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
        # İlk ada ise, güvenli sınırlar içinde rastgele yerleştir
        if not placed_island_positions:
            # Sınırlar zaten ada yarıçapı hesaba katılarak daraltılmış (min_x_safe, max_x_safe vb.)
            return (
                random.uniform(min_x, max_x),
                random.uniform(min_z, max_z)
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
    
    # ============================================================
    # SİMÜLASYON OLUŞTURMA
    # ============================================================
    def sim_olustur(self, n_rovs=3, n_engels=15, havuz_genisligi=200):
        """
        Simülasyon ortamını oluşturur: ROV'lar, kayalar, havuz sınırları.
        
        Args:
            n_rovs: Oluşturulacak ROV sayısı (varsayılan: 3)
            n_engels: Oluşturulacak kaya sayısı (varsayılan: 15)
            havuz_genisligi: Havuz genişliği (varsayılan: 200)
        """
        # Havuz genişliğini güncelle
        self.havuz_genisligi = havuz_genisligi
        
        # ============================================================
        # GÖRSEL BOYUTLANDIRMA: havuz_genisligi'ne göre dinamik ayarlama
        # ============================================================
        # Yeni görsel boyut hesapla
        # havuz_genisligi yarıçap gibi kullanılıyor (merkezden kenara), 
        # toplam genişlik = havuz_genisligi * 2
        # Görsel nesne toplam genişliğe eşit olmalı (kenar boşluğu için minimal çarpan)
        yeni_boyut = havuz_genisligi * 2.0
        
        # Görsel nesnelerin X ve Z scale'lerini güncelle (Y eksenini koru)
        if hasattr(self, 'ocean_surface') and self.ocean_surface:
            # Y eksenini koru (mevcut scale.y değeri)
            mevcut_y = self.ocean_surface.scale.y if hasattr(self.ocean_surface.scale, 'y') else self.ocean_surface.scale[1]
            self.ocean_surface.scale = (yeni_boyut, mevcut_y, yeni_boyut)
        
        if hasattr(self, 'water_volume') and self.water_volume:
            # Y eksenini koru (mevcut scale.y değeri)
            mevcut_y = self.water_volume.scale.y if hasattr(self.water_volume.scale, 'y') else self.water_volume.scale[1]
            self.water_volume.scale = (yeni_boyut, mevcut_y, yeni_boyut)
        
        if hasattr(self, 'seabed') and self.seabed:
            # Y eksenini koru (mevcut scale.y değeri)
            mevcut_y = self.seabed.scale.y if hasattr(self.seabed.scale, 'y') else self.seabed.scale[1]
            self.seabed.scale = (yeni_boyut, mevcut_y, yeni_boyut)
        
        if hasattr(self, 'cimen_katmani') and self.cimen_katmani:
            # Y eksenini koru (mevcut scale.y değeri)
            mevcut_y = self.cimen_katmani.scale.y if hasattr(self.cimen_katmani.scale, 'y') else self.cimen_katmani.scale[1]
            self.cimen_katmani.scale = (yeni_boyut, mevcut_y, yeni_boyut)
        
        # ocean_taban için orantılı scale (orijinal 500'e göre)
        if hasattr(self, 'ocean_taban') and self.ocean_taban:
            # Orijinal scale: (2.2 * (500 / 500), 1, 1.8 * (500 / 500)) = (2.2, 1, 1.8)
            # Orijinal boyut: 500
            # Yeni boyut: yeni_boyut
            # Oran: yeni_boyut / 500
            oran = yeni_boyut / 500.0
            mevcut_y = self.ocean_taban.scale.y if hasattr(self.ocean_taban.scale, 'y') else self.ocean_taban.scale[1]
            self.ocean_taban.scale = (2.2 * oran, mevcut_y, 1.8 * oran)
        # ============================================================
        
        # Ada pozisyonlarını koru (eğer varsa, None değerleri hariç)
        ada_positions_backup = []
        if hasattr(self, 'island_positions') and self.island_positions:
            # None değerlerini filtrele
            ada_positions_backup = [ada for ada in self.island_positions if ada is not None]
        
        # Engeller (Kayalar) - Listeyi sıfırla
        self.engeller = []
        
        # Ada pozisyonlarını geri yükle (eğer varsa)
        if ada_positions_backup:
            self.island_positions = ada_positions_backup
        
        # ============================================================
        # KAYA OLUŞTURMA (Güvenli Pozisyonlama)
        # ============================================================
        # Kayalar havuz sınırlarına değmeyecek şekilde pozisyonlanır
        # Çaplarıyla orantılı olarak 8 metre güvenlik birimiyle içerde oluşur
        self.engeller = kayalari_olustur(
            n_engels=n_engels,
            havuz_genisligi=self.havuz_genisligi,
            sea_floor_y=self.SEA_FLOOR_Y,
            water_surface_y_base=self.WATER_SURFACE_Y_BASE,
            guvenlik_mesafesi=8.0,  # 8 metre güvenlik mesafesi
            min_boyut=15,
            max_boyut=40,
            max_z_boyut=60
        )

        # ============================================================
        # ROV YERLEŞTİRME (Adaların dışına - Ada radyuslarına göre)
        # ============================================================
        # ROV listesini temizle (eski ROV'ları sil)
        # #region agent log
        eski_rov_sayisi = len(self.rovs) if hasattr(self, 'rovs') and self.rovs else 0
        debug_log_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".cursor", "debug.log"))
        try:
            os.makedirs(os.path.dirname(debug_log_path), exist_ok=True)
            with open(debug_log_path, 'a', encoding='utf-8') as f:
                f.write(f'{{"sessionId":"debug-session","runId":"run1","hypothesisId":"C","location":"simulasyon.py:2712","message":"ROV listesi temizlenmeden önce","data":{{"eski_rov_sayisi":{eski_rov_sayisi},"hedef_rov_sayisi":{n_rovs}}},"timestamp":{int(__import__("time").time()*1000)}}}\n')
        except OSError:
            pass
        # #endregion
        
        # Eski ROV'ları destroy et (Entity oldukları için)
        if hasattr(self, 'rovs') and self.rovs:
            for rov in self.rovs:
                try:
                    if hasattr(rov, 'destroy'):
                        rov.destroy()
                    elif hasattr(rov, '__del__'):
                        del rov
                except:
                    pass
        
        # ROV listesini sıfırla
        self.rovs = []
        
        # #region agent log
        try:
            os.makedirs(os.path.dirname(debug_log_path), exist_ok=True)
            with open(debug_log_path, 'a', encoding='utf-8') as f:
                f.write(f'{{"sessionId":"debug-session","runId":"run1","hypothesisId":"C","location":"simulasyon.py:2730","message":"ROV listesi temizlendikten sonra","data":{{"yeni_rov_sayisi":{len(self.rovs)},"hedef_rov_sayisi":{n_rovs}}},"timestamp":{int(__import__("time").time()*1000)}}}\n')
        except OSError:
            pass
        # #endregion
        
        # Havuz sınırları: +-havuz_genisligi (yani +-200 birim)
        # 10 metre güvenlik mesafesi: ROV'lar sınırlardan 10 metre içeride olmalı
        HAVUZ_GUVENLIK_MESAFESI = 10.0  # Metre cinsinden güvenlik mesafesi
        havuz_sinir = self.havuz_genisligi  # +-havuz_genisligi
        min_x = -havuz_sinir + HAVUZ_GUVENLIK_MESAFESI
        max_x = havuz_sinir - HAVUZ_GUVENLIK_MESAFESI
        min_z = -havuz_sinir + HAVUZ_GUVENLIK_MESAFESI
        max_z = havuz_sinir - HAVUZ_GUVENLIK_MESAFESI
        
        # Güvenlik payı (ada radyusuna ek olarak bırakılacak minimum mesafe)
        GUVENLIK_PAYI = SimulasyonSabitleri.ADA_GUVENLIK_PAYI
        
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
                    varsayilan_radius = SimulasyonSabitleri.ADA_VARSAYILAN_RADIUS  # Güvenli varsayılan değer
                    ada_bilgileri.append({
                        'x': island_x_2d,
                        'y': island_y_2d,
                        'radius': varsayilan_radius,
                        'min_safe_distance': varsayilan_radius + GUVENLIK_PAYI
                    })
        
        # Lider ROV ID'sini al (varsayılan: 0)
        lider_id = 0
        if hasattr(self, 'filo') and self.filo and hasattr(self.filo, 'orijinal_lider_id'):
            lider_id = self.filo.orijinal_lider_id
        
        for i in range(n_rovs):
            max_attempts = SimulasyonSabitleri.ROV_YERLESTIRME_MAX_DENEME
            placed = False
            
            # Tüm ROV'lar (Lider veya Takipçi fark etmeksizin) -10m ile -20m arasında doğsun
            z_depth = random.uniform(-20.0, -10.0)
            
            # Güvenli pozisyon bul (maksimum deneme sayısı kadar)
            for attempt in range(max_attempts):
                # Random pozisyon (havuz sınırları içinde)
                # Koordinat sistemi: (x_2d, y_2d, z_depth)
                x_2d = random.uniform(min_x, max_x)
                y_2d = random.uniform(min_z, max_z)  # Not: min_z/max_z aslında y_2d sınırları
                
                # Derinlik zaten yukarıda belirlendi (-10 ile -20 metre arası)
                # Her denemede aynı derinliği kullan (veya istersen her denemede değiştir)
                
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
                    x, z, y = sim_to_ursina(x_2d, y_2d, z_depth)
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
                
                # Tüm ROV'lar (Lider dahil) -10 ile -20 metre arasında doğsun
                z_depth = random.uniform(-20.0, -10.0)
                x, y, z = sim_to_ursina(x_2d, y_2d, z_depth)
                new_rov = ROV(rov_id=i, position=(x, y, z))
                new_rov.environment_ref = self
                if hasattr(self, 'filo'):
                    new_rov.filo_ref = self.filo
                self.rovs.append(new_rov)
                print(f"⚠️ ROV-{i} zorla yerleştirildi (ada kontrolü başarısız)")

        # ============================================================
        # HAVUZ SINIRLARI (Görünmez Duvarlar - Raycast için)
        # ============================================================
        # Raycast'in duvarları algılaması için görünmez boxlar eklemek en iyisidir
        havuz_sinir = self.havuz_genisligi
        duvar_kalinligi = SimulasyonSabitleri.DUVAR_KALINLIGI
        duvar_yuksekligi = SimulasyonSabitleri.DUVAR_YUKSEKLIGI
        
        # Sağ duvar (+X)
        Entity(
            model='cube',
            position=(havuz_sinir + duvar_kalinligi/2, 0, 0),
            scale=(duvar_kalinligi, duvar_yuksekligi, havuz_sinir * 2),
            collider='box',
            visible=False,  # Oyuncuya görünmez ama sensöre takılır
            color=color.clear
        )
        
        # Sol duvar (-X)
        Entity(
            model='cube',
            position=(-havuz_sinir - duvar_kalinligi/2, 0, 0),
            scale=(duvar_kalinligi, duvar_yuksekligi, havuz_sinir * 2),
            collider='box',
            visible=False,
            color=color.clear
        )
        
        # Ön duvar (+Z)
        Entity(
            model='cube',
            position=(0, 0, havuz_sinir + duvar_kalinligi/2),
            scale=(havuz_sinir * 2, duvar_yuksekligi, duvar_kalinligi),
            collider='box',
            visible=False,
            color=color.clear
        )
        
        # Arka duvar (-Z)
        Entity(
            model='cube',
            position=(0, 0, -havuz_sinir - duvar_kalinligi/2),
            scale=(havuz_sinir * 2, duvar_yuksekligi, duvar_kalinligi),
            collider='box',
            visible=False,
            color=color.clear
        )

        if self.verbose:
            print(f"🌊 Simülasyon Hazır: {n_rovs} ROV, {n_engels} Gri Kaya.")
    
    # --- Ada ve ROV Konum Yönetimi (Senaryo Modülü İçin) ---
    def Ada(self, ada_id, x=None, y=None):
        """
        Ada pozisyonunu değiştirir, konumunu döndürür, yeni ada ekler veya var olan adayı çıkarır.
        Z (derinlik) değeri mevcut değerinden korunur, değiştirilmez.
        
        Args:
            ada_id: Ada ID'si
            x: Yeni X koordinatı veya "ekle"/"cikar" komutu
                - "ekle": Yeni ada ekle (y parametresi konum tuple'ı olmalı: (x, y) veya (x, y, radius))
                - "cikar": Var olan adayı çıkar
                - None: Mevcut konumu döndürür
                - Sayı: Yeni X koordinatı (y parametresi de verilmeli)
            y: Yeni Y koordinatı (Simülasyon koordinat sistemi - İleri-Geri) veya konum tuple'ı (x="ekle" ise)
        
        Returns:
            tuple: (x, y) koordinatları veya None veya True/False (işlem başarılı/başarısız)
        
        Örnek:
            # Ada konumunu değiştir (z değeri korunur)
            app.Ada(0, 50, 60)  # X=50, Y=60, Z değeri mevcut değerinden korunur
            
            # Ada konumunu al
            konum = app.Ada(0)  # (x, y) tuple döner
            
            # Yeni ada ekle
            app.Ada(5, "ekle", (50, 60))  # Ada ID=5, X=50, Y=60, varsayılan radius
            app.Ada(6, "ekle", (70, 80, 40.0))  # Ada ID=6, X=70, Y=80, radius=40.0
            
            # Var olan adayı çıkar
            app.Ada(5, "cikar")  # Ada ID=5'i çıkar
        """
        # Ada ekleme işlemi
        if x == "ekle":
            if not isinstance(y, (tuple, list)) or len(y) < 2:
                if getattr(self, 'verbose', False):
                    print(f"⚠️ Ada ekleme hatası: Konum tuple'ı (x, y) veya (x, y, radius) formatında olmalı")
                return False
            
            # Konum bilgisini al
            konum_x, konum_y = y[0], y[1]
            radius_verildi = len(y) >= 3
            if radius_verildi:
                radius = y[2]
            else:
                radius = None  # Scale'den hesaplanacak
            
            # Ada pozisyonları ve entity listelerini başlat
            if not hasattr(self, 'island_positions'):
                self.island_positions = []
            if not hasattr(self, 'island_entities'):
                self.island_entities = []
            
            # Ada ID'si için yeterli kapasite yoksa genişlet
            while len(self.island_positions) <= ada_id:
                self.island_positions.append(None)  # Placeholder
            while len(self.island_entities) <= ada_id:
                self.island_entities.append(None)  # Placeholder
            
            # Eğer bu ID'de zaten ada varsa, ekleme yapma
            if ada_id < len(self.island_entities) and self.island_entities[ada_id] is not None:
                if getattr(self, 'verbose', False):
                    print(f"⚠️ Ada-{ada_id} zaten mevcut. Önce çıkarmak için: Ada({ada_id}, 'cikar')")
                return False
            
            if ada_id < len(self.island_positions) and self.island_positions[ada_id] is not None:
                if getattr(self, 'verbose', False):
                    print(f"⚠️ Ada-{ada_id} zaten mevcut. Önce çıkarmak için: Ada({ada_id}, 'cikar')")
                return False
            
            # Ada modeli ve texture yolları (Windows uyumlu path)
            island_model_path_rel = "Models-3D/lowpoly-island/source/island1_design2_c4d.obj"
            island_texture_path_rel = "Models-3D/lowpoly-island/textures/textureSurface_Color_2.jpg"
            
            # Absolute path'i de kontrol et (script'in bulunduğu yerden)
            script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            island_model_path_abs = os.path.join(script_dir, "Models-3D", "lowpoly-island", "source", "island1_design2_c4d.obj")
            island_texture_path_abs = os.path.join(script_dir, "Models-3D", "lowpoly-island", "textures", "textureSurface_Color_2.jpg")
            
            # Path seçimi: Önce relative path'i kontrol et
            island_model_path = None
            island_texture_path = None
            
            if os.path.exists(island_model_path_rel):
                island_model_path = island_model_path_rel
            elif os.path.exists(island_model_path_abs):
                try:
                    cwd = os.getcwd()
                    island_model_path = os.path.relpath(island_model_path_abs, cwd)
                    if not os.path.exists(island_model_path):
                        island_model_path = island_model_path_abs
                except (ValueError, OSError):
                    island_model_path = island_model_path_abs
            else:
                island_model_path = island_model_path_rel
            
            if os.path.exists(island_texture_path_rel):
                island_texture_path = island_texture_path_rel
            elif os.path.exists(island_texture_path_abs):
                try:
                    cwd = os.getcwd()
                    island_texture_path = os.path.relpath(island_texture_path_abs, cwd)
                    if not os.path.exists(island_texture_path):
                        island_texture_path = island_texture_path_abs
                except (ValueError, OSError):
                    island_texture_path = island_texture_path_abs
            else:
                island_texture_path = island_texture_path_rel
            
            # Ursina için path'leri normalize et (forward slash kullan)
            if os.path.sep == '\\':  # Windows
                if island_model_path:
                    island_model_path = island_model_path.replace('\\', '/')
                if island_texture_path:
                    island_texture_path = island_texture_path.replace('\\', '/')
            
            # Ada Y pozisyonu (su yüzeyinin üstünde sabit)
            if hasattr(self, 'WATER_SURFACE_Y_BASE'):
                max_wave_height = self.WATER_SURFACE_Y_BASE + 1.5
            else:
                max_wave_height = 0.0
            island_y_position = max_wave_height + 5
            
            # Ada ölçek faktörü (normal sistemdeki gibi)
            # Normal sistemde: ISLAND_BASE_SCALE * random.uniform(0.7, 1.1)
            ISLAND_BASE_SCALE = 0.25
            ISLAND_SCALE_RANDOM = (0.7, 1.1)
            
            # Scale ve radius hesaplama (normal sistemdeki mantık)
            if radius_verildi:
                # Radius verilmiş, scale'i radius'a göre hesapla
                # Normal sistemde: estimated_radius = (140.0 * scale_factor) / 2
                # Tersine: scale_factor = (radius * 2) / 140.0
                estimated_scale = (radius * 2) / 140.0
                # Normal sistemdeki sınırlar içinde tut
                min_scale = ISLAND_BASE_SCALE * ISLAND_SCALE_RANDOM[0]  # 0.175
                max_scale = ISLAND_BASE_SCALE * ISLAND_SCALE_RANDOM[1]  # 0.275
                scale_factor = max(min_scale, min(max_scale, estimated_scale))
                # Scale'e göre gerçek radius'u hesapla (normal sistemdeki gibi)
                radius = (140.0 * scale_factor) / 2
            else:
                # Radius verilmemiş, normal sistemdeki gibi random scale kullan
                scale_factor = ISLAND_BASE_SCALE * random.uniform(*ISLAND_SCALE_RANDOM)
                # Scale'den radius hesapla (normal sistemdeki gibi)
                radius = (140.0 * scale_factor) / 2
            
            visual_scale = (scale_factor, scale_factor, scale_factor)
            
            # Simülasyon koordinat sisteminden Ursina'ya dönüştür
            # Sim: (x, y_forward, z_depth) -> Ursina: (x, z_depth, y_forward)
            # Ada için z_depth = 0 (su yüzeyinin üstünde)
            z_depth = 0
            ursina_x, ursina_y, ursina_z = sim_to_ursina(konum_x, konum_y, z_depth)
            
            # Ada entity oluştur
            # Model ve texture kontrolü (en az bir yerde varsa kullan)
            model_exists_ada = os.path.exists(island_model_path) if island_model_path else False
            texture_exists_ada = os.path.exists(island_texture_path) if island_texture_path else False

            if model_exists_ada or os.path.exists(island_model_path_rel) or os.path.exists(island_model_path_abs):
                texture_path_final = None
                if texture_exists_ada or os.path.exists(island_texture_path_rel) or os.path.exists(island_texture_path_abs):
                    texture_path_final = island_texture_path
                island_radius = radius
                island = None
                try:
                    island = Entity(
                        model=island_model_path,
                        position=(ursina_x, island_y_position, ursina_z),
                        scale=visual_scale,
                        texture=texture_path_final,
                        collider='box',
                        unlit=False,
                        double_sided=True,
                        color=color.white,
                        alpha=1.0,
                        transparent=True,
                        render_queue=0
                    )
                    if hasattr(island, 'model') and island.model:
                        tri = getattr(island.model, 'triangles', None)
                        idx = getattr(island.model, 'indices', None)
                        n_tri = (len(tri) if tri is not None else 0) or (len(idx) // 3 if idx is not None else 0)
                        if n_tri == 0:
                            destroy(island)
                            island = None
                except Exception:
                    island = None
                if island is None and island_model_path:
                    island_mesh = _load_obj_as_mesh(island_model_path)
                    if island_mesh is not None:
                        try:
                            island = Entity(
                                model=island_mesh,
                                position=(ursina_x, island_y_position, ursina_z),
                                scale=visual_scale,
                                texture=texture_path_final,
                                collider='box',
                                unlit=False,
                                double_sided=True,
                                color=color.white,
                                alpha=1.0,
                                transparent=True,
                                render_queue=0
                            )
                        except Exception:
                            island = None
                if island is None:
                    island = Entity(
                        model='cube',
                        position=(ursina_x, island_y_position, ursina_z),
                        scale=visual_scale,
                        collider='box',
                        color=color.white,
                        alpha=1.0
                    )
                    island_radius = radius
                else:
                    try:
                        if hasattr(island.model, 'bounds') and island.model.bounds:
                            min_b, max_b = island.model.bounds
                            model_size = max_b - min_b
                            island_radius = max(model_size.x, model_size.z) * island.world_scale.x / 2
                    except Exception:
                        pass
            else:
                # Fallback: Cube model
                island = Entity(
                    model='cube',
                    position=(ursina_x, island_y_position, ursina_z),
                    scale=(radius * 2, 10, radius * 2),
                    color=color.green,
                    collider='box',
                    unlit=True
                )
                island_radius = radius
            
            # İlk adayı self.island olarak sakla (geriye uyumluluk için)
            if ada_id == 0:
                self.island = island
            
            # Ada entity'sini ve pozisyonunu sakla
            self.island_entities[ada_id] = island
            self.island_positions[ada_id] = (konum_x, konum_y, island_radius)
            
            # Verbose kontrolü
            verbose = False
            if hasattr(self, 'ortam') and hasattr(self.ortam, 'verbose'):
                verbose = self.ortam.verbose
            elif hasattr(self, 'verbose'):
                verbose = self.verbose
            
            if verbose:
                print(f"✅ Ada-{ada_id} eklendi: ({konum_x}, {konum_y}), radius={island_radius:.2f}")
            
            return True
        
        # Ada çıkarma işlemi
        elif x == "cikar":
            # Verbose kontrolü (başta yapılmalı)
            verbose = False
            if hasattr(self, 'ortam') and hasattr(self.ortam, 'verbose'):
                verbose = self.ortam.verbose
            elif hasattr(self, 'verbose'):
                verbose = self.verbose
            
            # Ada pozisyonları kontrolü
            if not hasattr(self, 'island_positions') or not self.island_positions:
                print(f"⚠️ Ada-{ada_id} bulunamadı (ada listesi boş)")
                return False
            
            if ada_id >= len(self.island_positions) or self.island_positions[ada_id] is None:
                print(f"⚠️ Ada-{ada_id} bulunamadı")
                return False
            
            # Ada entity'sini destroy et
            if hasattr(self, 'island_entities') and self.island_entities:
                if ada_id < len(self.island_entities) and self.island_entities[ada_id] is not None:
                    island_entity = self.island_entities[ada_id]
                    try:
                        # Görünürlüğü kapat
                        if hasattr(island_entity, 'visible'):
                            island_entity.visible = False
                        if hasattr(island_entity, 'enabled'):
                            island_entity.enabled = False
                        # Parent'ı kaldır
                        if hasattr(island_entity, 'parent'):
                            island_entity.parent = None
                        # Entity'yi destroy et
                        if hasattr(island_entity, 'destroy'):
                            island_entity.destroy()
                        # Scene'den de kaldırmayı dene (eğer mümkünse)
                        try:
                            from ursina import scene
                            if hasattr(scene, 'entities') and island_entity in scene.entities:
                                scene.entities.remove(island_entity)
                        except Exception:
                            pass
                    except Exception as e:
                        # Hata olsa bile devam et
                        if verbose:
                            print(f"⚠️ Ada-{ada_id} destroy hatası: {e}")
                    finally:
                        self.island_entities[ada_id] = None
            
            # Ada hitbox'larını sil (eğer varsa)
            # Not: Hitbox'ları destroy etmek yeterli, listeyi değiştirmeye gerek yok
            # Çünkü hitbox'lar None kontrolü ile zaten atlanır
            if hasattr(self, 'island_hitboxes') and self.island_hitboxes:
                ada_hitbox_start = ada_id * 5
                for hitbox_idx in range(ada_hitbox_start, min(ada_hitbox_start + 5, len(self.island_hitboxes))):
                    if hitbox_idx < len(self.island_hitboxes):
                        hitbox = self.island_hitboxes[hitbox_idx]
                        if hitbox is not None:
                            try:
                                if hasattr(hitbox, 'visible'):
                                    hitbox.visible = False
                                if hasattr(hitbox, 'enabled'):
                                    hitbox.enabled = False
                                if hasattr(hitbox, 'parent'):
                                    hitbox.parent = None
                                if hasattr(hitbox, 'destroy'):
                                    hitbox.destroy()
                            except Exception:
                                pass
                        self.island_hitboxes[hitbox_idx] = None
            
            # Pozisyon listesinden çıkar
            self.island_positions[ada_id] = None
            
            # İlk ada silindiyse self.island'ı None yap
            if ada_id == 0 and hasattr(self, 'island'):
                self.island = None
            
            if verbose:
                print(f"✅ Ada-{ada_id} çıkarıldı")
            
            return True
        
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
            # Ada pozisyonunu güncelle (z değerini koru)
            radius = self.island_positions[ada_id][2] if len(self.island_positions[ada_id]) > 2 else 50.0
            old_pos = self.island_positions[ada_id]
            # Mevcut z değerini entity'den al (eğer varsa), yoksa 0 kullan
            z = 0  # Varsayılan
            if hasattr(self, 'island_entities') and ada_id < len(self.island_entities):
                island_entity = self.island_entities[ada_id]
                if hasattr(island_entity, 'position') and hasattr(island_entity.position, 'y'):
                    # Ursina koordinat sisteminden simülasyon koordinat sistemine dönüştür
                    # Ursina: (x, y_up, z_forward) -> Sim: (x, z_forward, y_up)
                    # z değeri Ursina'da y ekseninde (y_up), Sim'de z ekseninde (z_depth)
                    _, _, z = ursina_to_sim(island_entity.position.x, island_entity.position.y, island_entity.position.z)
                elif hasattr(island_entity, 'y'):
                    # Ursina koordinat sisteminden simülasyon koordinat sistemine dönüştür
                    _, _, z = ursina_to_sim(island_entity.x, island_entity.y, island_entity.z)
            
            # island_positions formatı: (x, y, radius) - z bilgisi entity'de saklanıyor
            self.island_positions[ada_id] = (x, y, radius)
            
            # Ada entity'sini güncelle (Ursina koordinat sistemine dönüştür)
            # Sim: (x, y, z) -> Ursina: (x, z, y)
            if hasattr(self, 'island_entities') and ada_id < len(self.island_entities):
                island_entity = self.island_entities[ada_id]
                # Simülasyon koordinat sisteminden Ursina'ya dönüştür
                # Sim: (x, y_forward, z_depth) -> Ursina: (x, z_depth, y_forward)
                ursina_x, ursina_y, ursina_z = sim_to_ursina(x, y, z)
                
                if hasattr(island_entity, 'position'):
                    island_entity.position = Vec3(ursina_x, ursina_y, ursina_z)
                elif hasattr(island_entity, 'x'):
                    island_entity.x = ursina_x
                    island_entity.y = ursina_y
                    island_entity.z = ursina_z
            
            # Ada hitbox'larını güncelle (eğer varsa)
            if hasattr(self, 'island_hitboxes') and self.island_hitboxes:
                # Her ada için 5 hitbox var (varsayılan)
                ada_hitbox_start = ada_id * 5
                # Simülasyon koordinat sisteminden Ursina'ya dönüştür
                ursina_x, ursina_y, ursina_z = sim_to_ursina(x, y, z)
                for hitbox_idx in range(ada_hitbox_start, min(ada_hitbox_start + 5, len(self.island_hitboxes))):
                    hitbox = self.island_hitboxes[hitbox_idx]
                    if hasattr(hitbox, 'position'):
                        hitbox.position = Vec3(ursina_x, ursina_y, ursina_z)
                    elif hasattr(hitbox, 'x'):
                        hitbox.x = ursina_x
                        hitbox.y = ursina_y
                        hitbox.z = ursina_z
            
            # Verbose kontrolü için ortam referansı gerekli
            verbose = False
            if hasattr(self, 'ortam') and hasattr(self.ortam, 'verbose'):
                verbose = self.ortam.verbose
            elif hasattr(self, 'verbose'):
                verbose = self.verbose
            
            if verbose:
                print(f"✅ Ada-{ada_id} pozisyonu güncellendi: ({x}, {y}, z=0)")
            return (x, y)
        else:
            # Mevcut konumu döndür
            if ada_id < len(self.island_positions):
                ada_pos = self.island_positions[ada_id]
                return (ada_pos[0], ada_pos[1])
            else:
                return None
    
    def _rov_id_yeniden_numaralandir(self):
        """
        ROV ID'lerini 0'dan başlayarak yeniden numaralandırır.
        Çıkarılmış ROV'ların yerini doldurur ve tüm ID'leri sırayla yeniden atar.
        """
        if not hasattr(self, 'rovs') or not self.rovs:
            return
        
        # Aktif ROV'ları bul (None olmayanlar)
        aktif_rovs = [r for r in self.rovs if r is not None]
        
        if not aktif_rovs:
            # Hiç aktif ROV yoksa listeyi temizle
            self.rovs = []
            return
        
        # ID eşleştirmesi: eski_id -> yeni_id
        id_eslestirme = {}
        yeni_rovs_listesi = []
        
        # Her aktif ROV için yeni ID ata (0'dan başlayarak)
        for yeni_id, rov in enumerate(aktif_rovs):
            eski_id = rov.id
            id_eslestirme[eski_id] = yeni_id
            
            # ROV'un ID'sini güncelle
            rov.id = yeni_id
            
            # Label'ı güncelle
            if hasattr(rov, 'label') and rov.label is not None:
                try:
                    rov.label.text = f"ROV-{yeni_id}"
                except Exception:
                    pass
            
            # Yeni listeye ekle
            yeni_rovs_listesi.append(rov)
        
        # ROV listesini güncelle
        self.rovs = yeni_rovs_listesi
        
        # Filo sistemindeki referansları güncelle (eğer varsa)
        if hasattr(self, 'filo') and self.filo is not None:
            # Filo sistemlerini yeniden düzenle (ROV ID'lerine göre sırala)
            # Sistemler listesi ROV ID'leriyle eşleşmeli (sistemler[rov_id] = sistem)
            
            # Önce mevcut sistemleri ROV ID'lerine göre eşleştir
            sistem_eslestirme = {}
            for sistem in self.filo.sistemler:
                if hasattr(sistem, 'rov') and sistem.rov is not None:
                    eski_id = sistem.rov.id
                    if eski_id in id_eslestirme:
                        yeni_id = id_eslestirme[eski_id]
                        sistem_eslestirme[yeni_id] = sistem
                        
                        # GNC sistemindeki rov_id referanslarını güncelle
                        if hasattr(sistem, 'rov_id') and sistem.rov_id is not None:
                            sistem.rov_id = yeni_id
            
            # Sistemler listesini yeniden düzenle (yeni ID'lere göre, ROV ID'leriyle eşleşecek şekilde)
            # Sistemler listesi ROV ID'leriyle indekslenmiş olmalı: sistemler[rov_id] = sistem
            yeni_sistemler = [None] * len(aktif_rovs)
            for yeni_id, sistem in sistem_eslestirme.items():
                if 0 <= yeni_id < len(yeni_sistemler):
                    yeni_sistemler[yeni_id] = sistem
            
            # None olmayan sistemleri filtrele ve sırala
            # Ama ROV ID'leriyle eşleşmesi için None'ları da tutmalıyız
            # Aslında sistemler listesi ROV ID'leriyle eşleşmeli, bu yüzden None'ları kaldırmamalıyız
            # Ama liste uzunluğu aktif ROV sayısına eşit olmalı
            self.filo.sistemler = yeni_sistemler
            
            # Filo'daki lider ID'sini güncelle
            if hasattr(self.filo, 'orijinal_lider_id'):
                if self.filo.orijinal_lider_id in id_eslestirme:
                    self.filo.orijinal_lider_id = id_eslestirme[self.filo.orijinal_lider_id]
                elif self.filo.orijinal_lider_id >= len(aktif_rovs):
                    # Lider çıkarılmışsa, yeni lideri bul (ilk ROV lider olur)
                    if len(aktif_rovs) > 0:
                        self.filo.orijinal_lider_id = 0
                        # İlk ROV'u lider yap
                        if len(self.rovs) > 0:
                            self.rovs[0].role = 1
            
            # Filo'daki formasyon hedefleri ve git hedeflerini güncelle
            if hasattr(self.filo, '_formasyon_hedefleri'):
                yeni_formasyon_hedefleri = {}
                for eski_id, hedef in self.filo._formasyon_hedefleri.items():
                    if eski_id in id_eslestirme:
                        yeni_formasyon_hedefleri[id_eslestirme[eski_id]] = hedef
                self.filo._formasyon_hedefleri = yeni_formasyon_hedefleri
            
            if hasattr(self.filo, '_git_hedef_yaw'):
                yeni_git_hedef_yaw = {}
                for eski_id, yaw in self.filo._git_hedef_yaw.items():
                    if eski_id in id_eslestirme:
                        yeni_git_hedef_yaw[id_eslestirme[eski_id]] = yaw
                self.filo._git_hedef_yaw = yeni_git_hedef_yaw
            
            if hasattr(self.filo, '_git_nokta_listesi'):
                yeni_git_nokta_listesi = {}
                for eski_id, nokta_listesi in self.filo._git_nokta_listesi.items():
                    if eski_id in id_eslestirme:
                        yeni_git_nokta_listesi[id_eslestirme[eski_id]] = nokta_listesi
                self.filo._git_nokta_listesi = yeni_git_nokta_listesi
            
            if hasattr(self.filo, '_git_mevcut_nokta_indeksi'):
                yeni_git_mevcut_nokta_indeksi = {}
                for eski_id, indeks in self.filo._git_mevcut_nokta_indeksi.items():
                    if eski_id in id_eslestirme:
                        yeni_git_mevcut_nokta_indeksi[id_eslestirme[eski_id]] = indeks
                self.filo._git_mevcut_nokta_indeksi = yeni_git_mevcut_nokta_indeksi
        
        # Verbose kontrolü
        verbose = False
        if hasattr(self, 'verbose'):
            verbose = self.verbose
        
        if verbose:
            print(f"✅ ROV ID'leri yeniden numaralandırıldı: {len(aktif_rovs)} aktif ROV")
    
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
        engeller = self.engeller
        
        # None olmayan ROV'ları filtrele (çıkarılmış ROV'ları atla)
        aktif_rovs = [r for r in self.rovs if r is not None]
        
        if not aktif_rovs:
            # Hiç aktif ROV yoksa boş veri döndür (GAT modeli 9 özellik bekliyor)
            class MiniData:
                def __init__(self, x, edge_index):
                    self.x, self.edge_index = x, edge_index
            return MiniData(x=torch.zeros((0, 9), dtype=torch.float), edge_index=torch.zeros((2, 0), dtype=torch.long))

        n = len(aktif_rovs)
        x = torch.zeros((n, 9), dtype=torch.float)
        positions = [r.position for r in aktif_rovs]
        sources, targets = [], []

        L = {'LEADER': 60.0, 'DISCONNECT': 35.0, 'OBSTACLE': 20.0, 'COLLISION': 8.0}

        # Mesafe matrisi (min_rov_dist ve lider_dist için)
        dist_matrix = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                if i != j:
                    dist_matrix[i][j] = distance(positions[i], positions[j])

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
            x[i][1] = aktif_rovs[i].battery
            x[i][2] = 0.9
            x[i][3] = min(1.0, abs(aktif_rovs[i].y) / 100.0)
            x[i][4] = getattr(aktif_rovs[i].velocity, 'x', 0.0)
            x[i][5] = getattr(aktif_rovs[i].velocity, 'z', 0.0)
            x[i][6] = 1.0 if getattr(aktif_rovs[i], 'role', 0) == 1 else 0.0
            # [7] En yakın ROV mesafesi (normalize 0–100 -> 0–1), GAT ile uyumlu
            if n > 1:
                min_rov_d = min(dist_matrix[i][j] for j in range(n) if j != i)
                x[i][7] = float(min(min_rov_d / 100.0, 1.0))
            # [8] Lider mesafesi (takipçiler için, normalize 0–100 -> 0–1)
            if i != 0 and n > 0:
                x[i][8] = float(min(dist_matrix[i][0] / 100.0, 1.0))

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
