"""
Camera Manager Module
ROV kamera sistemlerinin yönetimi, ayarları ve YOLO Entegrasyonu (Ursina UI Destekli).
"""

import builtins
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from direct.task import Task
from ursina import destroy, time
from FiratROVNet.config import PerformansAyarlari, TespitAyarlari

try:
    from panda3d.core import (
        FrameBufferProperties, GraphicsOutput, GraphicsPipe,
        Texture as P3DCamTexture, WindowProperties,
    )
    OFFSCREEN_AVAILABLE = True
except Exception:
    OFFSCREEN_AVAILABLE = False

try:
    from FiratROVNet.model_paths import YOLOV8N_MODEL, path_str
except Exception:
    YOLOV8N_MODEL = "yolov8n.pt"

    def path_str(path):
        return str(path)

try:
    import cv2
    CV2_AVAILABLE = True
except Exception as exc:
    cv2 = None
    CV2_AVAILABLE = False
    print(f"⚠️ UYARI: OpenCV yüklenemedi. YOLO kamera analizi devre dışı: {exc}")

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except Exception as exc:
    YOLO_AVAILABLE = False
    print(f"⚠️ UYARI: 'ultralytics' yüklenemedi. YOLO özellikleri devre dışı: {exc}")

try:
    from FiratROVNet.kutuphane.moduls.Panels import YOLOVisionPanel
except Exception as exc:
    YOLOVisionPanel = None
    print(f"⚠️ UYARI: YOLO paneli yüklenemedi: {exc}")


class CameraManager:
    def __init__(self, filo_ref=None):
        self.filo_ref = filo_ref
        self.aktif_kameralar = {}        # rov_id → CameraNodePath (display kamera)
        self._offscreen_buffers = {}     # rov_id → GraphicsOutput (offscreen)
        self._offscreen_textures = {}    # rov_id → Texture (RAM kopyası)
        self._offscreen_cams = {}        # rov_id → CameraNodePath (offscreen)

        self.yolo_modelleri = {}
        self.aktif_yolo_gorevleri = {}
        self.yolo_panelleri = {}
        self.yolo_ui_ekranlari = self.yolo_panelleri
        self._yolo_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="rov-yolo")
        self._yolo_futures = {}
        self.yolo_son_tespitler = {}
        self.yolo_conf: dict = {}
        # Tespit modu: 'renk' | 'model' | 'hibrit'
        self.tespit_modu: dict = {}
        # Global tespit durumu (R tuşuyla geçişlerde taşınır)
        self._global_tespit_aktif: bool = False
        self._global_tespit_mod: str = 'hibrit'
        self._global_tespit_conf: float = 0.5
        
    def kamera_ekle(self, rov_id=0, mesafe=(0, -40, 120), aci=(0, 0, 0), fov=75, bolge=(0.02, 0.20, 0.80, 0.98)):
        if not hasattr(builtins, 'base'):
            return None
        b = builtins.base
        if rov_id in self.aktif_kameralar:
            self.kamera_kaldir(rov_id)

        try:
            if not self.filo_ref:
                raise ValueError("Filo referansı bulunamadı")
            target_rov = self.filo_ref.find_rov_by_id(rov_id)
            if not target_rov:
                raise ValueError(f"ROV-{rov_id} bulunamadı")
        except Exception as e:
            print(f"❌ HATA: {e}")
            return None

        # ── Görüntü kamerası (ekrana yansıtılır) ──────────────────────────
        cam_np = b.makeCamera(b.win)
        cam_node = cam_np.node()
        cam_np.reparentTo(target_rov)
        cam_np.setPos(mesafe[0], mesafe[1], mesafe[2])
        cam_np.setHpr(aci[0], aci[1], aci[2])
        cam_node.getLens().setFov(fov)
        region = cam_node.get_display_region(0)
        region.set_dimensions(bolge[0], bolge[1], bolge[2], bolge[3])
        region.set_sort(10)
        cam_node.set_camera_mask(0b01)   # maske 1: ROV modeli görünmesin
        self.aktif_kameralar[rov_id] = cam_np

        # ── Offscreen render buffer (YOLO/renk filtresi için) ──────────────
        self._offscreen_baslat(rov_id, target_rov, mesafe, aci, fov)

        print(f"🎥 ROV-{rov_id} FPV Kamera Aktif (Bölge: {bolge})")
        return cam_np

    def _offscreen_baslat(self, rov_id, target_rov, mesafe, aci, fov):
        """Tespit için ayrı bir offscreen buffer + kamera oluşturur."""
        if not OFFSCREEN_AVAILABLE or not hasattr(builtins, 'base'):
            return
        b = builtins.base
        w = int(TespitAyarlari.KAMERA_GENISLIK)
        h = int(TespitAyarlari.KAMERA_YUKSEKLIK)
        try:
            fb_props = FrameBufferProperties()
            fb_props.setRgbColor(True)
            fb_props.setDepthBits(16)
            win_props = WindowProperties()
            win_props.setSize(w, h)
            buf = b.graphicsEngine.makeOutput(
                b.pipe, f"rov_offscreen_{rov_id}", -100,
                fb_props, win_props,
                GraphicsPipe.BFRefuseWindow | GraphicsPipe.BFResizeable,
                b.win.getGsg(), b.win,
            )
            if buf is None:
                print(f"⚠️ ROV-{rov_id} offscreen buffer oluşturulamadı, screenshot fallback kullanılacak.")
                return
            tex = P3DCamTexture(f"rov_tex_{rov_id}")
            buf.addRenderTexture(tex, GraphicsOutput.RTMCopyRam)
            off_cam = b.makeCamera(buf)
            off_cam.reparentTo(target_rov)
            off_cam.setPos(mesafe[0], mesafe[1], mesafe[2])
            off_cam.setHpr(aci[0], aci[1], aci[2])
            off_cam.node().getLens().setFov(fov)
            off_cam.node().set_camera_mask(0b01)
            self._offscreen_buffers[rov_id] = buf
            self._offscreen_textures[rov_id] = tex
            self._offscreen_cams[rov_id] = off_cam
            print(f"🖥️  ROV-{rov_id} offscreen buffer hazır ({w}×{h})")
        except Exception as exc:
            print(f"⚠️ ROV-{rov_id} offscreen hata: {exc}")

    def _goruntu_yakala(self, rov_id):
        """Offscreen texture'dan BGR görüntü döndürür. Yoksa display region screenshot'ına düşer."""
        tex = self._offscreen_textures.get(rov_id)
        if tex is not None:
            try:
                data = tex.getRamImageAs("RGB")
                if data:
                    img = np.frombuffer(bytes(data), np.uint8).reshape(
                        (tex.getYSize(), tex.getXSize(), 3)
                    )
                    return cv2.flip(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), 0)
            except Exception:
                pass
        # Fallback: eski screenshot yöntemi
        cam_node = self.aktif_kameralar.get(rov_id)
        if cam_node is None:
            return None
        try:
            region = cam_node.node().get_display_region(0)
            tex2 = region.getScreenshot()
            if not tex2:
                return None
            data2 = tex2.getRamImageAs("RGB")
            if not data2:
                return None
            img2 = np.frombuffer(data2, np.uint8).reshape((tex2.getYSize(), tex2.getXSize(), 3))
            return cv2.flip(cv2.cvtColor(img2, cv2.COLOR_RGB2BGR), 0)
        except Exception:
            return None

    # ==========================================
    # YOLO ENTEGRASYON BÖLÜMÜ (UI DESTEKLİ)
    # ==========================================

    # ==========================================
    # TESPİT MODU YÖNETİMİ
    # ==========================================

    def tespit_modu_sec(self, rov_id, mod: str):
        """
        Tespit modunu çalışma anında değiştirir.
        mod: 'renk' | 'model' | 'hibrit'
        """
        mod = str(mod).strip().lower()
        if mod not in ('renk', 'model', 'hibrit'):
            print(f"⚠️ Geçersiz mod: '{mod}'. Seçenekler: renk | model | hibrit")
            return False
        self.tespit_modu[rov_id] = mod
        panel = self.yolo_panelleri.get(rov_id)
        if panel and hasattr(panel, 'mod_guncelle'):
            panel.mod_guncelle(mod)
        print(f"🔧 ROV-{rov_id} tespit modu: {mod}")
        return True

    def _renk_filtresi_isle(self, img_bgr):
        """
        HSV renk maskesi ile nesne tespiti yapar.
        Döndürür: (annotated_bgr, tespitler_listesi)
        """
        if cv2 is None:
            return img_bgr, []
        araliklar = list(TespitAyarlari.RENK_ARALIKLAR)
        min_alan = int(TespitAyarlari.MIN_KONTUR_ALANI)
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        annotated = img_bgr.copy()
        tespitler = []

        # Kırmızı iki aralıkta wrap-around yapar — birleştir
        kirmizi_mask = None
        islendi = set()
        for isim, lower, upper in araliklar:
            if isim == 'kirmizi2':
                continue
            if isim == 'kirmizi':
                mask1 = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
                mask2 = np.zeros_like(mask1)
                for isim2, l2, u2 in araliklar:
                    if isim2 == 'kirmizi2':
                        mask2 = cv2.inRange(hsv, np.array(l2, np.uint8), np.array(u2, np.uint8))
                        break
                mask = cv2.bitwise_or(mask1, mask2)
                kirmizi_mask = mask
                islendi.add('kirmizi')
            else:
                mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            konturer, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # BGR renk gösterimi için lower HSV → BGR
            bgr_renk = cv2.cvtColor(np.array([[[lower[0], lower[1], lower[2]]]], dtype=np.uint8), cv2.COLOR_HSV2BGR)[0][0].tolist()
            for cnt in konturer:
                alan = cv2.contourArea(cnt)
                if alan < min_alan:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(annotated, (x, y), (x + w, y + h), bgr_renk, 2)
                cv2.putText(annotated, f"{isim} ({int(alan)}px)",
                            (x, max(y - 6, 10)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, bgr_renk, 1, cv2.LINE_AA)
                tespitler.append({
                    "bbox": (float(x), float(y), float(x + w), float(y + h)),
                    "class_id": -1,
                    "class_name": isim,
                    "confidence": float(alan) / (img_bgr.shape[0] * img_bgr.shape[1]),
                })
        return annotated, tespitler

    def yolo_baslat(self, rov_id, model_path=None, islem_hizi=3, conf: float = 0.5,
                    mod=None):
            if not CV2_AVAILABLE or YOLOVisionPanel is None:
                print("⚠️ Tespit başlatılamadı: OpenCV/YOLO paneli hazır değil.")
                return False
            if rov_id not in self.aktif_kameralar and self.kamera_ekle(rov_id=rov_id) is None:
                print(f"⚠️ Tespit başlatılamadı: ROV-{rov_id} için kamera açılamadı.")
                return False
            if rov_id in self.aktif_yolo_gorevleri:
                return True
            maks = int(getattr(PerformansAyarlari, 'MAX_YOLO_AKTIF', 2))
            if len(self.aktif_yolo_gorevleri) >= maks:
                print(f"⚠️ Tespit: Eş zamanlı limit ({maks}) doldu. ROV-{rov_id} için başlatılmadı.")
                return False

            # Tespit modu belirle
            aktif_mod = mod if mod is not None else getattr(TespitAyarlari, 'MOD', 'hibrit')
            self.tespit_modu[rov_id] = aktif_mod
            self.yolo_conf[rov_id] = float(conf)
            # Global durumu güncelle
            self._global_tespit_aktif = True
            self._global_tespit_mod   = aktif_mod
            self._global_tespit_conf  = float(conf)

            # Model sadece renk modunda değilse yükle
            yolo_model = None
            if aktif_mod in ('model', 'hibrit'):
                if not YOLO_AVAILABLE:
                    print(f"⚠️ ROV-{rov_id}: YOLO mevcut değil, renk moduna geçiliyor.")
                    self.tespit_modu[rov_id] = 'renk'
                    aktif_mod = 'renk'
                else:
                    model_path = path_str(YOLOV8N_MODEL) if model_path is None else model_path
                    if model_path not in self.yolo_modelleri:
                        print(f"🧠 YOLO Modeli yükleniyor: {model_path}...")
                        self.yolo_modelleri[model_path] = YOLO(model_path)
                    yolo_model = self.yolo_modelleri[model_path]

            if rov_id not in self.yolo_panelleri:
                self.yolo_panelleri[rov_id] = YOLOVisionPanel(rov_id)
            panel = self.yolo_panelleri[rov_id]
            if hasattr(panel, 'mod_guncelle'):
                panel.mod_guncelle(aktif_mod)

            b = builtins.base
            task_name = f"yolo_task_rov_{rov_id}"
            rov_conf = self.yolo_conf.get(rov_id, 0.5)
            task = b.taskMgr.add(
                self._yolo_guncelle_task, task_name,
                extraArgs=[rov_id, yolo_model, islem_hizi, rov_conf],
                appendTask=True
            )
            task.frame_counter = 0
            task.capture_accum = 0.0
            self.aktif_yolo_gorevleri[rov_id] = task
            print(f"🎯 ROV-{rov_id} Tespit Sistemi Başlatıldı! Mod: {aktif_mod}")
            return True

    def _yolo_predict_worker(self, model, img_bgr, conf: float = 0.5):
            try:
                results = model.predict(source=img_bgr, conf=float(conf), verbose=False)
            except Exception as cuda_err:
                err_str = str(cuda_err)
                if "CUDA" in err_str or "cuda" in err_str:
                    try:
                        model.to("cpu")
                        print("⚠️ YOLO CUDA hatası, CPU'ya geçildi.")
                        results = model.predict(source=img_bgr, conf=float(conf), verbose=False, device="cpu")
                    except Exception as cpu_err:
                        print(f"⚠️ YOLO CPU fallback hatası: {cpu_err}")
                        raise
                else:
                    raise
            tespitler = []
            names = getattr(model, "names", {}) or {}
            boxes = getattr(results[0], "boxes", None)
            if boxes is not None:
                for box in boxes:
                    try:
                        xyxy = box.xyxy[0].tolist()
                        cls_id = int(box.cls[0])
                        c = float(box.conf[0])
                        tespitler.append({
                            "bbox": tuple(float(v) for v in xyxy),
                            "class_id": cls_id,
                            "class_name": str(names.get(cls_id, cls_id)),
                            "confidence": c,
                        })
                    except Exception:
                        continue
            annotated_bgr = results[0].plot()
            return cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB), tespitler

    def _hibrit_worker(self, model, img_bgr, conf: float = 0.5):
        """Hibrit mod: YOLO çalıştır; düşük güvende renk filtresini de ekle."""
        try:
            annotated_rgb, tespitler = self._yolo_predict_worker(model, img_bgr, conf)
            min_conf = float(getattr(TespitAyarlari, 'HIBRIT_MIN_CONF', 0.40))
            # Hiç tespit yoksa veya en yüksek güven düşükse renk filtresi ekle
            max_guven = max((t['confidence'] for t in tespitler), default=0.0)
            if max_guven < min_conf:
                annotated_bgr = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)
                _, renk_tespitler = self._renk_filtresi_isle(img_bgr)
                # Renk tespitlerini orijinal annotated üzerine çiz
                for t in renk_tespitler:
                    x1, y1, x2, y2 = (int(v) for v in t['bbox'])
                    cv2.rectangle(annotated_bgr, (x1, y1), (x2, y2), (0, 255, 200), 2)
                    cv2.putText(annotated_bgr, f"[renk] {t['class_name']}",
                                (x1, max(y1 - 6, 10)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.45, (0, 255, 200), 1, cv2.LINE_AA)
                tespitler = tespitler + renk_tespitler
                annotated_rgb = cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB)
            return annotated_rgb, tespitler
        except Exception:
            # YOLO hata → sadece renk filtresi
            annotated_bgr, tespitler = self._renk_filtresi_isle(img_bgr)
            return cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB), tespitler

    def _yolo_texture_guncelle(self, rov_id, annotated_rgb):
            panel = self.yolo_panelleri.get(rov_id)
            if not panel:
                return
            panel.set_rgb_frame(annotated_rgb)

    def _renk_filtresi_rgb_worker(self, img_bgr):
        """Renk filtresi sonucunu RGB olarak döndürür (worker thread için)."""
        annotated_bgr, tespitler = self._renk_filtresi_isle(img_bgr)
        if cv2 is not None:
            return cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB), tespitler
        return annotated_bgr[:, :, ::-1], tespitler

    def _yolo_guncelle_task(self, rov_id, model, islem_hizi, conf, task):
            task.frame_counter += 1
            future = self._yolo_futures.get(rov_id)
            if future is not None and future.done():
                try:
                    annotated_rgb, tespitler = future.result()
                    self.yolo_son_tespitler[rov_id] = tespitler
                    self._yolo_texture_guncelle(rov_id, annotated_rgb)
                    task._hata_sayaci = 0
                except Exception as exc:
                    hata_sayaci = getattr(task, '_hata_sayaci', 0) + 1
                    task._hata_sayaci = hata_sayaci
                    if hata_sayaci == 1 or hata_sayaci % 30 == 0:
                        print(f"⚠️ Tespit worker hatası (#{hata_sayaci}): {exc}")
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

            img_bgr = self._goruntu_yakala(rov_id)
            if img_bgr is None:
                return Task.cont

            mod = self.tespit_modu.get(rov_id, getattr(TespitAyarlari, 'MOD', 'hibrit'))
            if mod == 'renk':
                self._yolo_futures[rov_id] = self._yolo_executor.submit(
                    self._renk_filtresi_rgb_worker, img_bgr.copy()
                )
            elif mod == 'model' and model is not None:
                self._yolo_futures[rov_id] = self._yolo_executor.submit(
                    self._yolo_predict_worker, model, img_bgr.copy(), conf
                )
            elif mod == 'hibrit' and model is not None:
                self._yolo_futures[rov_id] = self._yolo_executor.submit(
                    self._hibrit_worker, model, img_bgr.copy(), conf
                )
            else:
                # Model yok, sadece renk
                self._yolo_futures[rov_id] = self._yolo_executor.submit(
                    self._renk_filtresi_rgb_worker, img_bgr.copy()
                )
            return Task.cont

    def yolo_durdur(self, rov_id, _global_sifirla: bool = True):
        if rov_id in self.aktif_yolo_gorevleri:
            b = builtins.base
            b.taskMgr.remove(self.aktif_yolo_gorevleri[rov_id])
            del self.aktif_yolo_gorevleri[rov_id]
            future = self._yolo_futures.pop(rov_id, None)
            if future is not None:
                future.cancel()
            self.yolo_conf.pop(rov_id, None)
            
            # Ursina UI Panelini de sahneden sil
            if rov_id in self.yolo_panelleri:
                destroy(self.yolo_panelleri[rov_id].root)
                del self.yolo_panelleri[rov_id]

            if _global_sifirla:
                self._global_tespit_aktif = False
                
            print(f"🛑 ROV-{rov_id} için YOLO durduruldu.")
            return True
        return False

    def kapat(self) -> None:
        """Tüm kameraları kaldırır ve thread pool'u temizler. Simülasyon kapanınca çağrılmalı."""
        self.tum_kameralari_kaldir()
        self._yolo_executor.shutdown(wait=False)

    def __del__(self):
        try:
            self._yolo_executor.shutdown(wait=False)
        except Exception:
            pass

    # ==========================================
    # MEVCUT (ESKİ) YARDIMCI METOTLAR
    # ==========================================

    def kamera_kaldir(self, rov_id):
        if rov_id not in self.aktif_kameralar:
            return False
        try:
            self.yolo_durdur(rov_id)
            b = builtins.base
            eski_cam = self.aktif_kameralar[rov_id]
            try:
                b.win.removeDisplayRegion(eski_cam.node().getDisplayRegion(0))
            except Exception:
                pass
            eski_cam.removeNode()
            del self.aktif_kameralar[rov_id]
            # Offscreen buffer temizle
            off_cam = self._offscreen_cams.pop(rov_id, None)
            if off_cam is not None:
                try:
                    off_cam.removeNode()
                except Exception:
                    pass
            buf = self._offscreen_buffers.pop(rov_id, None)
            if buf is not None:
                try:
                    b.graphicsEngine.removeWindow(buf)
                except Exception:
                    pass
            self._offscreen_textures.pop(rov_id, None)
            return True
        except Exception as e:
            print(f"⚠️ [CAMERA] ROV-{rov_id} kamera kaldırma hatası: {e}")
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
