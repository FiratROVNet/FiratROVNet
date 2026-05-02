"""
Camera Manager Module
ROV kamera sistemlerinin yönetimi, ayarları ve YOLO Entegrasyonu (Ursina UI Destekli).
"""

import builtins
import cv2
import numpy as np
from direct.task import Task
from PIL import Image
from ursina import Entity, camera, Texture, destroy, color  # Ursina UI için eklendi

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️ UYARI: 'ultralytics' kütüphanesi bulunamadı. YOLO özellikleri devre dışı.")


class CameraManager:
    def __init__(self, filo_ref=None):
        self.filo_ref = filo_ref
        self.aktif_kameralar = {}  
        
        self.yolo_modelleri = {}   
        self.aktif_yolo_gorevleri = {} 
        self.yolo_ui_ekranlari = {} # Yeni: Oyun içi YOLO HUD ekranları
        
    def kamera_ekle(self, rov_id=0, mesafe=(0, -40, 120), aci=(0, 0, 0), fov=75, bolge=(0.02, 0.20, 0.80, 0.98)):
        if not hasattr(builtins, 'base'):
            return None
        b = builtins.base  
        if rov_id in self.aktif_kameralar:
            self.kamera_kaldir(rov_id)

        cam_np = b.makeCamera(b.win)
        cam_node = cam_np.node()
        
        try:
            if not self.filo_ref: raise ValueError("Filo referansı bulunamadı")
            target_rov = self.filo_ref.find_rov_by_id(rov_id)
            if not target_rov: raise ValueError(f"ROV-{rov_id} bulunamadı")
            cam_np.reparentTo(target_rov)
        except Exception as e:
            print(f"❌ HATA: {e}")
            return None
        
        cam_np.setPos(mesafe[0], mesafe[1], mesafe[2])
        cam_np.setHpr(aci[0], aci[1], aci[2])
        cam_node.getLens().setFov(fov)
        region = cam_node.get_display_region(0)
        region.set_dimensions(bolge[0], bolge[1], bolge[2], bolge[3])
        region.set_sort(10)  
        cam_node.set_camera_mask(1) 

        self.aktif_kameralar[rov_id] = cam_np
        print(f"🎥 ROV-{rov_id} FPV Kamera Aktif (Bölge: {bolge})")
        return cam_np

    # ==========================================
    # YOLO ENTEGRASYON BÖLÜMÜ (UI DESTEKLİ)
    # ==========================================

    def yolo_baslat(self, rov_id, model_path='yolov8n.pt', islem_hizi=3):
            if not YOLO_AVAILABLE: return False
            if rov_id not in self.aktif_kameralar: return False
            if rov_id in self.aktif_yolo_gorevleri: return True

            if model_path not in self.yolo_modelleri:
                print(f"🧠 YOLO Modeli yükleniyor: {model_path}...")
                self.yolo_modelleri[model_path] = YOLO(model_path)
                
            if rov_id not in self.yolo_ui_ekranlari:
                # 1. ANA EKRAN (Sola kaydırıldı ve daha şeffaf)
                self.yolo_ui_ekranlari[rov_id] = Entity(
                    parent=camera.ui,
                    model='quad',
                    scale=(0.3, 0.2),      
                    position=(0.5, 0.39),   # <-- FPS panelinden kurtarmak için Sola ve biraz aşağıya kaydırıldı
                    z=0,                    
                    color=color.rgba(255, 255, 255, 130), # <-- Daha şeffaf (Cam etkisi artırıldı)
                    unlit=True
                )
                
                # 2. ŞIK BİR ÇERÇEVE (Kalınlık azaltıldı, zarif neon çizgi)
                self.yolo_ui_ekranlari[rov_id].cerceve = Entity(
                    parent=self.yolo_ui_ekranlari[rov_id],
                    model='quad',
                    scale=(1.02, 1.03), # <-- Daha ince çerçeve
                    color=color.rgba(0, 255, 255, 150), # Saydam siber-mavi
                    z=0.01 
                )
                
                # 3. KÜÇÜK BİR BAŞLIK (Yazı çerçeveye hizalandı)
                from ursina import Text
                self.yolo_ui_ekranlari[rov_id].baslik = Text(
                    parent=self.yolo_ui_ekranlari[rov_id],
                    text=f"ROV-{rov_id} AI VISION",
                    origin=(0, 0),
                    position=(0, 0.55), # <-- Çerçevenin tam üst hizasına milimetrik oturtuldu
                    scale=2.2,          # <-- Yazı boyutu dengelendi
                    color=color.rgba(0, 255, 255, 220)
                )
                
            b = builtins.base
            cam_node = self.aktif_kameralar[rov_id].node()
            region = cam_node.get_display_region(0)
            
            task_name = f"yolo_task_rov_{rov_id}"
            task = b.taskMgr.add(
                self._yolo_guncelle_task, task_name,
                extraArgs=[rov_id, region, self.yolo_modelleri[model_path], islem_hizi],
                appendTask=True
            )
            task.frame_counter = 0 
            self.aktif_yolo_gorevleri[rov_id] = task
            print(f"🎯 ROV-{rov_id} için Oyun İçi YOLO Tespit Sistemi Başlatıldı!")
            return True

    def _yolo_guncelle_task(self, rov_id, region, model, islem_hizi, task):
            task.frame_counter += 1
            if task.frame_counter % islem_hizi != 0: return Task.cont
                
            tex = region.getScreenshot()
            if not tex: return Task.cont
                
            data = tex.getRamImageAs("RGB")
            if not data: return Task.cont
                
            img_np = np.frombuffer(data, np.uint8)
            img_np = img_np.reshape((tex.getYSize(), tex.getXSize(), 3))
            
            # Görüntüyü Panda3D formatından BGR'ye (OpenCV için) çevir
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            img_bgr = cv2.flip(img_bgr, 0)
            
            # YOLO Nesne Tespiti (Çizim işlemi plot() ile yapılır)
            results = model.predict(source=img_bgr, conf=0.5, verbose=False)
            annotated_bgr = results[0].plot()
            
            # BGR'den RGB'ye geri çevir
            annotated_rgb = cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB)
            height, width, _ = annotated_rgb.shape
            
            ui_entity = self.yolo_ui_ekranlari.get(rov_id)
            if not ui_entity: return Task.cont

            # PANDA3D DONANIMSAL TEXTURE GÜNCELLEMESİ (Hızlı ve stabil)
            if not hasattr(ui_entity, '_p3d_tex') or ui_entity._p3d_tex.getXSize() != width:
                from panda3d.core import Texture as P3DTexture
                pt = P3DTexture("yolo_tex")
                pt.setup2dTexture(width, height, P3DTexture.T_unsigned_byte, P3DTexture.F_rgb)
                ui_entity._p3d_tex = pt
                
                # --- DÜZELTİLEN SATIR BURASI ---
                # Ursina objesi yerine doğrudan Panda3D objesine texture ataması yapıyoruz
                ui_entity.model.setTexture(pt, 1) 
                
            # Texture verisini doğrudan RAM'e yazdır (Flicker/titreme sorununu çözer)
            annotated_rgb_flipped = cv2.flip(annotated_rgb, 0) # OpenGL standardı gereği
            ui_entity._p3d_tex.setRamImage(annotated_rgb_flipped.tobytes())
            
            return Task.cont

    def yolo_durdur(self, rov_id):
        if rov_id in self.aktif_yolo_gorevleri:
            b = builtins.base
            b.taskMgr.remove(self.aktif_yolo_gorevleri[rov_id])
            del self.aktif_yolo_gorevleri[rov_id]
            
            # Ursina UI Panelini de sahneden sil
            if rov_id in self.yolo_ui_ekranlari:
                destroy(self.yolo_ui_ekranlari[rov_id])
                del self.yolo_ui_ekranlari[rov_id]
                
            print(f"🛑 ROV-{rov_id} için YOLO durduruldu.")
            return True
        return False

    # ==========================================
    # MEVCUT (ESKİ) YARDIMCI METOTLAR
    # ==========================================

    def kamera_kaldir(self, rov_id):
        if rov_id not in self.aktif_kameralar:
            return False
        try:
            self.yolo_durdur(rov_id) # Kamerayı silerken YOLO'yu da durdurur
            
            b = builtins.base
            eski_cam = self.aktif_kameralar[rov_id]
            b.win.removeDisplayRegion(eski_cam.node().getDisplayRegion(0))
            eski_cam.removeNode()
            del self.aktif_kameralar[rov_id]
            return True
        except Exception as e:
            return False
            
    def kamera_guncelle(self, rov_id, mesafe=None, aci=None, fov=None):
        if rov_id not in self.aktif_kameralar:
            return False
        try:
            cam_np = self.aktif_kameralar[rov_id]
            if mesafe is not None: cam_np.setPos(mesafe[0], mesafe[1], mesafe[2])
            if aci is not None: cam_np.setHpr(aci[0], aci[1], aci[2])
            if fov is not None: cam_np.node().getLens().setFov(fov)
            return True
        except Exception as e:
            return False
            
    def tum_kameralari_kaldir(self):
        kaldirilan = 0
        for rov_id in list(self.aktif_kameralar.keys()):
            if self.kamera_kaldir(rov_id):
                kaldirilan += 1
        return kaldirilan

    def kamera_var_mi(self, rov_id):
        return rov_id in self.aktif_kameralar

    def kamera_bilgisi(self, rov_id):
        if rov_id not in self.aktif_kameralar:
            return None
        try:
            cam_np = self.aktif_kameralar[rov_id]
            pos = cam_np.getPos()
            hpr = cam_np.getHpr()
            fov = cam_np.node().getLens().getFov()
            return {
                'pozisyon': (pos.x, pos.y, pos.z),
                'aci': (hpr.x, hpr.y, hpr.z),
                'fov': fov[0] if isinstance(fov, tuple) else fov
            }
        except Exception as e:
            return None

    def aktif_kamera_listesi(self):
        return list(self.aktif_kameralar.keys())

    def kamera_ayarla(self, rov_id=0, mesafe=(0, -40, 120), aci=(0, 0, 0), fov=75, bölge=(0.02, 0.20, 0.80, 0.98)):
        return self.kamera_ekle(rov_id, mesafe, aci, fov, bölge)