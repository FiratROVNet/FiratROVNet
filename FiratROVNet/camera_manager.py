"""
Camera Manager Module
ROV kamera sistemlerinin yönetimi, ayarları ve YOLO Entegrasyonu (Ursina UI Destekli).
"""

import builtins
import cv2
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from direct.task import Task
from ursina import destroy, time
from FiratROVNet.config import PerformansAyarlari
from FiratROVNet.kutuphane.moduls.Panels import YOLOVisionPanel
from FiratROVNet.model_paths import YOLOV8N_MODEL, path_str

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
        self.yolo_panelleri = {}
        self._yolo_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="rov-yolo")
        self._yolo_futures = {}
        self.yolo_son_tespitler = {}
        
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

    def yolo_baslat(self, rov_id, model_path=None, islem_hizi=3):
            if not YOLO_AVAILABLE: return False
            if rov_id not in self.aktif_kameralar: return False
            if rov_id in self.aktif_yolo_gorevleri: return True
            model_path = path_str(YOLOV8N_MODEL) if model_path is None else model_path

            if model_path not in self.yolo_modelleri:
                print(f"🧠 YOLO Modeli yükleniyor: {model_path}...")
                self.yolo_modelleri[model_path] = YOLO(model_path)
                
            if rov_id not in self.yolo_panelleri:
                self.yolo_panelleri[rov_id] = YOLOVisionPanel(rov_id)
                
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
            task.capture_accum = 0.0
            self.aktif_yolo_gorevleri[rov_id] = task
            print(f"🎯 ROV-{rov_id} için Oyun İçi YOLO Tespit Sistemi Başlatıldı!")
            return True

    def _yolo_predict_worker(self, model, img_bgr):
            results = model.predict(source=img_bgr, conf=0.5, verbose=False)
            tespitler = []
            names = getattr(model, "names", {}) or {}
            boxes = getattr(results[0], "boxes", None)
            if boxes is not None:
                for box in boxes:
                    try:
                        xyxy = box.xyxy[0].tolist()
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        tespitler.append({
                            "bbox": tuple(float(v) for v in xyxy),
                            "class_id": cls_id,
                            "class_name": str(names.get(cls_id, cls_id)),
                            "confidence": conf,
                        })
                    except Exception:
                        continue
            annotated_bgr = results[0].plot()
            return cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB), tespitler

    def _yolo_texture_guncelle(self, rov_id, annotated_rgb):
            ui_entity = self.yolo_panelleri.get(rov_id)
            if not ui_entity:
                return
            ui_entity.set_rgb_frame(annotated_rgb)

    def _yolo_guncelle_task(self, rov_id, region, model, islem_hizi, task):
            task.frame_counter += 1
            future = self._yolo_futures.get(rov_id)
            if future is not None and future.done():
                try:
                    annotated_rgb, tespitler = future.result()
                    self.yolo_son_tespitler[rov_id] = tespitler
                    self._yolo_texture_guncelle(rov_id, annotated_rgb)
                except Exception as exc:
                    print(f"⚠️ YOLO worker hatası: {exc}")
                self._yolo_futures.pop(rov_id, None)

            if rov_id in self._yolo_futures:
                return Task.cont

            dt = getattr(time, "dt", 0.016) or 0.016
            task.capture_accum = getattr(task, "capture_accum", 0.0) + dt
            hz = float(getattr(PerformansAyarlari, "YOLO_CAPTURE_HZ", 5.0) or 5.0)
            interval = 1.0 / hz if hz > 0 else max(1, int(islem_hizi)) * 0.016
            if task.frame_counter % max(1, int(islem_hizi)) != 0 and task.capture_accum < interval:
                return Task.cont
            task.capture_accum = 0.0
                
            tex = region.getScreenshot()
            if not tex: return Task.cont
                
            data = tex.getRamImageAs("RGB")
            if not data: return Task.cont
                
            img_np = np.frombuffer(data, np.uint8)
            img_np = img_np.reshape((tex.getYSize(), tex.getXSize(), 3))
            
            # Görüntüyü Panda3D formatından BGR'ye (OpenCV için) çevir
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            img_bgr = cv2.flip(img_bgr, 0)
            self._yolo_futures[rov_id] = self._yolo_executor.submit(self._yolo_predict_worker, model, img_bgr.copy())
            
            return Task.cont

    def yolo_durdur(self, rov_id):
        if rov_id in self.aktif_yolo_gorevleri:
            b = builtins.base
            b.taskMgr.remove(self.aktif_yolo_gorevleri[rov_id])
            del self.aktif_yolo_gorevleri[rov_id]
            future = self._yolo_futures.pop(rov_id, None)
            if future is not None:
                future.cancel()
            
            # Ursina UI Panelini de sahneden sil
            if rov_id in self.yolo_panelleri:
                destroy(self.yolo_panelleri[rov_id].root)
                del self.yolo_panelleri[rov_id]
                
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
