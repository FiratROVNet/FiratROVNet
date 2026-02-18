"""
Camera Manager Module
ROV kamera sistemlerinin yönetimi ve ayarları.
"""

import builtins


class CameraManager:
    """
    ROV'lara dinamik FPV kamera bağlama ve yönetme sistemi.
    Modern programlama prensipleriyle tasarlanmıştır.
    """
    
    def __init__(self, filo_ref=None):
        """
        Args:
            filo_ref: Filo referansı (ROV bulma için)
        """
        self.filo_ref = filo_ref
        self.aktif_kameralar = {}  # {rov_id: camera_node_path}
        
    def kamera_ekle(self, rov_id=0, mesafe=(0, -40, 120), aci=(0, 0, 0), fov=75, bolge=(0.02, 0.20, 0.80, 0.98)):
        """
        ROV'a dinamik bir FPV kamera bağlar.
        
        Args:
            rov_id: ROV ID'si
            mesafe: Kameranın ROV'a göre pozisyonu (x, y, z) - Panda3D koordinat sistemi
            aci: Kameranın açısı (heading, pitch, roll)
            fov: Field of View (Görüş açısı)
            bolge: Ekran bölgesi (sol, sağ, alt, üst) - 0-1 arası normalize değerler
            
        Returns:
            camera_node_path veya None (hata durumunda)
        """
        # Simülasyonun çalışıp çalışmadığını kontrol et
        if not hasattr(builtins, 'base'):
            print("❌ HATA: Simülasyon henüz başlatılmadığı için kamera oluşturulamaz.")
            return None
        
        b = builtins.base  # Panda3D ana nesnesi

        # 1. Eğer bu ROV için zaten bir kamera varsa temizle
        if rov_id in self.aktif_kameralar:
            self.kamera_kaldir(rov_id)

        # 2. Yeni Kamera Oluştur
        cam_np = b.makeCamera(b.win)
        cam_node = cam_np.node()
        
        # 3. Kamerayı ROV'a Bağla
        try:
            if not self.filo_ref:
                raise ValueError("Filo referansı bulunamadı")
                
            target_rov = self.filo_ref.find_rov_by_id(rov_id)
            if not target_rov:
                raise ValueError(f"ROV-{rov_id} bulunamadı")
            cam_np.reparentTo(target_rov)
        except Exception as e:
            print(f"❌ HATA: ROV-{rov_id} nesnesine ulaşılamadı: {e}")
            return None
        
        # 4. Konum ve Açı (Panda3D: X sağ, Y ileri, Z yukarı)
        cam_np.setPos(mesafe[0], mesafe[1], mesafe[2])
        cam_np.setHpr(aci[0], aci[1], aci[2])
        
        # 5. Lens ve Ekran Bölgesi
        cam_node.getLens().setFov(fov)
        region = cam_node.get_display_region(0)
        region.set_dimensions(bolge[0], bolge[1], bolge[2], bolge[3])
        region.set_sort(10)  # En üstte çizilmesi için
        
        # Minimap ve UI'yı bu kameradan gizle (isteğe bağlı)
        cam_node.set_camera_mask(1) 

        self.aktif_kameralar[rov_id] = cam_np
        print(f"🎥 ROV-{rov_id} FPV Kamera Aktif (Bölge: {bolge})")
        return cam_np
    
    def kamera_kaldir(self, rov_id):
        """
        Belirtilen ROV'un kamerasını kaldırır.
        
        Args:
            rov_id: ROV ID'si
            
        Returns:
            bool: Başarılı ise True
        """
        if rov_id not in self.aktif_kameralar:
            return False
            
        try:
            b = builtins.base
            eski_cam = self.aktif_kameralar[rov_id]
            b.win.removeDisplayRegion(eski_cam.node().getDisplayRegion(0))
            eski_cam.removeNode()
            del self.aktif_kameralar[rov_id]
            print(f"🎥 ROV-{rov_id} kamerası kaldırıldı")
            return True
        except Exception as e:
            print(f"❌ HATA: ROV-{rov_id} kamerası kaldırılırken hata: {e}")
            return False
    
    def kamera_guncelle(self, rov_id, mesafe=None, aci=None, fov=None):
        """
        Mevcut kameranın ayarlarını günceller.
        
        Args:
            rov_id: ROV ID'si
            mesafe: Yeni pozisyon (opsiyonel)
            aci: Yeni açı (opsiyonel)
            fov: Yeni FOV (opsiyonel)
            
        Returns:
            bool: Başarılı ise True
        """
        if rov_id not in self.aktif_kameralar:
            print(f"❌ HATA: ROV-{rov_id} için aktif kamera bulunamadı")
            return False
            
        try:
            cam_np = self.aktif_kameralar[rov_id]
            
            if mesafe is not None:
                cam_np.setPos(mesafe[0], mesafe[1], mesafe[2])
                
            if aci is not None:
                cam_np.setHpr(aci[0], aci[1], aci[2])
                
            if fov is not None:
                cam_np.node().getLens().setFov(fov)
                
            print(f"🎥 ROV-{rov_id} kamera ayarları güncellendi")
            return True
        except Exception as e:
            print(f"❌ HATA: ROV-{rov_id} kamera güncellenirken hata: {e}")
            return False
    
    def tum_kameralari_kaldir(self):
        """
        Tüm aktif kameraları kaldırır.
        
        Returns:
            int: Kaldırılan kamera sayısı
        """
        kaldirilan = 0
        for rov_id in list(self.aktif_kameralar.keys()):
            if self.kamera_kaldir(rov_id):
                kaldirilan += 1
        return kaldirilan
    
    def kamera_var_mi(self, rov_id):
        """
        Belirtilen ROV'un kamerası var mı kontrol eder.
        
        Args:
            rov_id: ROV ID'si
            
        Returns:
            bool: Kamera varsa True
        """
        return rov_id in self.aktif_kameralar
    
    def kamera_bilgisi(self, rov_id):
        """
        Belirtilen ROV'un kamera bilgilerini döner.
        
        Args:
            rov_id: ROV ID'si
            
        Returns:
            dict: Kamera bilgileri (pozisyon, açı, fov) veya None
        """
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
            print(f"❌ HATA: ROV-{rov_id} kamera bilgisi alınırken hata: {e}")
            return None
    
    def aktif_kamera_listesi(self):
        """
        Aktif kameraların ROV ID listesini döner.
        
        Returns:
            list: ROV ID'leri
        """
        return list(self.aktif_kameralar.keys())
    
    # Backward compatibility için wrapper metod
    def kamera_ayarla(self, rov_id=0, mesafe=(0, -40, 120), aci=(0, 0, 0), fov=75, bölge=(0.02, 0.20, 0.80, 0.98)):
        """
        Eski API uyumluluğu için wrapper metod.
        kamera_ekle() metodunu çağırır.
        """
        return self.kamera_ekle(rov_id, mesafe, aci, fov, bölge)
