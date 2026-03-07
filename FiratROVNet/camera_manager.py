"""
Camera Manager Module - Saf Panda3D DisplayRegion Odaklı
ROV kamera sistemleri ve ekrandaki projeksiyon bölgeleri (DisplayRegion)
Panda3D NodePath ile yönetilir; Ursina update döngüsüne girmez.
"""

import builtins


class CameraManager:
    """
    ROV kamera sistemlerini ve ekrandaki DisplayRegion (Projeksiyon) bölgelerini
    saf Panda3D performansı ile yönetir.
    """

    def __init__(self, filo_ref=None):
        self.filo_ref = filo_ref
        self.aktif_kameralar = {}  # {rov_id: camera_nodepath}
        # Varsayılan kamera ayarları
        self.default_rov_id = 0
        self.default_mesafe = (0, -40, 120)
        self.default_aci = (0, 0, 0)
        self.default_fov = 75
        self.default_bolge = (0, 0.20, 0.80, 1.0)

    def kamera_ekle(self, rov_id=None, mesafe=None, aci=None, fov=None, bolge=None):
        """
        Kamerayı oluşturur ve ekranın belirli bir bölgesine yansıtır.
        bolge: (sol, sağ, alt, üst) -> 0.0 - 1.0 arası ekran koordinatları.
        """
        if rov_id is None:
            rov_id = self.default_rov_id
        if mesafe is None:
            mesafe = self.default_mesafe
        if aci is None:
            aci = self.default_aci
        if fov is None:
            fov = self.default_fov
        if bolge is None:
            bolge = self.default_bolge

        b = getattr(builtins, "base", None)
        if b is None:
            print("❌ HATA: Simülasyon henüz başlatılmadığı için kamera oluşturulamaz.")
            return None

        if rov_id in self.aktif_kameralar:
            self.kamera_kaldir(rov_id)

        # 1. Saf Panda3D Kamerası Oluştur
        cam_np = b.makeCamera(b.win)
        cam_node = cam_np.node()

        # 2. ROV'a Bağla (Panda3D hiyerarşisiyle)
        try:
            filo = self.filo_ref
            if filo is None:
                raise ValueError("Filo referansı yok")
            target_rov = filo.find_rov_by_id(rov_id)
            if not target_rov:
                raise ValueError(f"ROV-{rov_id} bulunamadı")
            # Ursina Entity → Panda3D NodePath (Ursina'da Entity bazen .entity ile NodePath verir)
            parent_np = getattr(target_rov, "entity", target_rov)
            cam_np.reparentTo(parent_np)
        except Exception as e:
            print(f"❌ HATA: Kamera ROV'a bağlanamadı: {e}")
            b.win.removeDisplayRegion(cam_node.getDisplayRegion(0))
            cam_np.removeNode()
            return None

        # 3. 3D Konum ve Lens Ayarı (Panda3D transform – update döngüsüne gerek yok)
        cam_np.setPos(mesafe[0], mesafe[1], mesafe[2])
        cam_np.setHpr(aci[0], aci[1], aci[2])  # H(Yaw), P(Pitch), R(Roll)
        cam_node.getLens().setFov(fov)

        # 4. Ekran Projeksiyon Bölgesi (DisplayRegion)
        region = cam_node.getDisplayRegion(0)
        region.set_dimensions(bolge[0], bolge[1], bolge[2], bolge[3])
        region.set_sort(100)  # Sahne üstünde dursun

        self.aktif_kameralar[rov_id] = cam_np
        print(f"🎥 ROV-{rov_id} FPV Kamera Aktif (saf Panda3D).")
        return cam_np

    def ekran_bolgesi_ayarla(self, rov_id, sol=0.0, sag=1.0, alt=0.0, ust=1.0):
        """
        Kameranın ekrana yansıyan görüntüsünün yerini ve boyutunu değiştirir.
        sol, sag, alt, ust: 0.0 - 1.0 arası ekran koordinatları.
        """
        cam_np = self.aktif_kameralar.get(rov_id)
        if cam_np is None:
            return False
        region = cam_np.node().getDisplayRegion(0)
        region.set_dimensions(sol, sag, alt, ust)
        return True

    def tam_ekran_yap(self, rov_id):
        """Kamerayı tüm ekrana yayar (0, 1, 0, 1)."""
        return self.ekran_bolgesi_ayarla(rov_id, 0.0, 1.0, 0.0, 1.0)

    def kamera_guncelle(self, rov_id, mesafe=None, aci=None, fov=None):
        """Kamera pozisyonunu veya lensini Panda3D üzerinden günceller."""
        cam_np = self.aktif_kameralar.get(rov_id)
        if cam_np is None:
            return False
        if mesafe is not None:
            cam_np.setPos(mesafe[0], mesafe[1], mesafe[2])
        if aci is not None:
            cam_np.setHpr(aci[0], aci[1], aci[2])
        if fov is not None:
            cam_np.node().getLens().setFov(fov)
        return True

    def kamera_kaldir(self, rov_id):
        """Kamerayı ve DisplayRegion'ı kaldırır."""
        if rov_id not in self.aktif_kameralar:
            return False
        b = getattr(builtins, "base", None)
        if b is None:
            return False
        try:
            cam_np = self.aktif_kameralar[rov_id]
            b.win.removeDisplayRegion(cam_np.node().getDisplayRegion(0))
            cam_np.removeNode()
            del self.aktif_kameralar[rov_id]
            print(f"🎥 ROV-{rov_id} kamerası kaldırıldı.")
            return True
        except Exception as e:
            print(f"❌ HATA: Kamera kaldırılırken sorun: {e}")
            return False

    def kamera_bilgisi(self, rov_id):
        """Kamera bilgilerini Panda3D pozisyon/açı/FOV olarak döner."""
        cam_np = self.aktif_kameralar.get(rov_id)
        if cam_np is None:
            return None
        pos = cam_np.getPos()
        hpr = cam_np.getHpr()
        return {
            "pozisyon": (pos.x, pos.y, pos.z),
            "aci": (hpr.x, hpr.y, hpr.z),
            "fov": cam_np.node().getLens().getFov(),
        }

    def aktif_kamera_listesi(self):
        return list(self.aktif_kameralar.keys())

    def kamera_var_mi(self, rov_id):
        """Belirtilen ROV için aktif kamera var mı."""
        return rov_id in self.aktif_kameralar

    def tum_kameralari_kaldir(self):
        for rid in list(self.aktif_kameralar.keys()):
            self.kamera_kaldir(rid)
