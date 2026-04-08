import math
import numpy as np
from typing import Any, Optional

class DataMixin:
    """Veri erisim fonksiyonlari."""

    filo: Any

    def get(
        self,
        rov_id: Optional[int] = None,
        veri_tipi: Optional[str] = None,
        taraf: Optional[int] = None,
        koordinator=None,
        sessiz: bool = False,
    ):
        """
        ROV bilgilerini guvenli bir sekilde alir.

        Args:
            rov_id: ROV ID (0, 1, 2, ...) veya None (tum ROV'lar icin)
            veri_tipi: 'gps', 'hiz', 'batarya', 'rol', 'lidar', 'engels', 'yaw' vb.
            taraf: Lidar yonu (0: On, 1: Sag, 2: Sol)
            koordinator: Koordinat donusturucu sinifi
            sessiz: Hata mesajlarini gizler
        """
        # 1. Toplu GPS Istegi (Tum ROV'lar)
        if rov_id is None and veri_tipi is None:
            return self.filo._get_all_rovs_positions()

        # 2. Ortam ve Liste Kontrolu
        if not self.filo.ortam_ref or not hasattr(self.filo.ortam_ref, 'rovs'):
            if not sessiz:
                print("❌ [HATA] Ortam veya ROV listesi bulunamadi!")
            return None

        rov_listesi = self.filo.ortam_ref.rovs
        if len(rov_listesi) == 0:
            if not sessiz:
                print("❌ [HATA] Henuz hic ROV yok.")
            return None

        # 3. ID Kontrolu
        if rov_id is None or not isinstance(rov_id, int) or rov_id < 0:
            if not sessiz:
                print(f"❌ [HATA] Gecersiz ROV ID: {rov_id}")
            return None

        # 4. ROV Nesnesini Al ve Canlilik Kontrolu
        rov = self.filo.find_rov_by_id(rov_id) if hasattr(self.filo, 'find_rov_by_id') else None
        if rov is None:
            return None

        # Eger ROV listede None ise veya Ursina tarafinda silindiyse (is_destroyed)
        if rov is None or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
            if not sessiz and veri_tipi:
                print(f"⚠️ [GET] ROV-{rov_id} ulasilamaz (silinmis veya patlamis).")
            return None

        try:
            val = None

            # --- OZEL VERI TIPI ISLEMLERI ---

            if veri_tipi == "lidar":
                # Doğrudan ROV'dan lidar dict'i al
                val = rov.get("lidar")

            elif veri_tipi == "sonar":
                # Doğrudan ROV'dan sonar scalar'ını al
                val = rov.get("sonar")

            elif veri_tipi == "gps":
                # ROV'dan ham (Ursina) GPS al
                ursina_gps = rov.get("gps")
                if ursina_gps is not None:
                    # Numpy array gelirse tuple'a cevir
                    if isinstance(ursina_gps, np.ndarray):
                        ursina_gps = tuple(ursina_gps.tolist())

                    # Simulasyon koordinatina donustur
                    if koordinator:
                        val = koordinator.ursina_to_sim(*ursina_gps)
                    else:
                        val = ursina_gps
                else:
                    val = None

            elif veri_tipi == "engels":
                # Engel hesaplama fonksiyonunu cagir
                val = self.compute_obstacle_positions(rov_id)

            elif veri_tipi == "mod":
                # GNC'den mod bilgisi (TemelGNC.mod)
                gnc = getattr(rov, 'gnc', None)
                if gnc and hasattr(gnc, 'mod'):
                    val = gnc.mod
                else:
                    val = None

            elif veri_tipi in ("gps_sinyal", "gps_signal"):
                # Önce yeni sensor paketine bak, yoksa eski GNC alanına dön.
                sensor = getattr(rov, 'sensor', None)
                if sensor and hasattr(sensor, 'gps_signal'):
                    val = sensor.gps_signal
                else:
                    val = None
                if val is not None:
                    return val
                gnc = getattr(rov, 'gnc', None)
                if gnc and hasattr(gnc, 'gps_sinyal'):
                    val = gnc.gps_sinyal
                else:
                    val = None

            elif veri_tipi == "sensor":
                val = getattr(rov, 'sensor', None)

            elif veri_tipi == "imu":
                sensor = getattr(rov, 'sensor', None)
                val = sensor.imu if sensor and hasattr(sensor, 'imu') else None

            elif veri_tipi == "bar":
                sensor = getattr(rov, 'sensor', None)
                val = sensor.bar if sensor and hasattr(sensor, 'bar') else None

            elif veri_tipi == "sicaklik":
                sensor = getattr(rov, 'sensor', None)
                val = sensor.sicaklik if sensor and hasattr(sensor, 'sicaklik') else None

            else:
                # Diger standart veriler (batarya, rol, hiz, yaw, sonar...)
                # ROV.get() metoduna yonlendir
                val = rov.get(veri_tipi)

            if val is None and not sessiz:
                # Debug amacli log (Sik cagiran verilerde kapatilabilir)
                pass

            return val

        except Exception as e:
            # Obje o an silindiyse Panda3D hatasi verebilir, bunu sessizce yutuyoruz
            if "!is_empty()" not in str(e):
                print(f"❌ [HATA] Helper.get hatasi ({veri_tipi}): {e}")
            return None

    def points(self) -> list:
        """
        Tum ROV koordinatlarini ve tum engel koordinatlarini birlestirip dondurur.
        """
        all_points = []
        try:
            rovs_positions = self.filo._get_all_rovs_positions()
            for _, position in rovs_positions.items():
                if position is not None:
                    all_points.append(position)

            for rov_id in rovs_positions.keys():
                engels = self.filo._compute_obstacle_positions(rov_id)
                if engels:
                    all_points.extend(engels)
        except Exception as e:
            print(f"❌ [HATA] Points hesaplanirken hata: {e}")
            import traceback
            traceback.print_exc()

        return all_points

    def compute_obstacle_positions(self, rov_id: int) -> list:
        """
        ROV'un tum lidar sensorlerinden engel koordinatlarini hesaplar.
        Simulasyon formatinda (X: Sag-Sol, Y: Ileri-Geri, Z: Derinlik) calisir.
        L0: İleri, L1: Sağ, L2: Sol, L3: Dip
        """
        LIDAR_OFFSETS = {
            0: 0,     # ileri
            1: -90,   # sag
            2: 90     # sol
            # L3 (dip) offset yok, Z ekseninde işlem
        }
        obstacles = []

        try:
            gps = self.filo.get(rov_id, "gps")
            if gps is None:
                return []

            x0, y0, z0 = gps[0], gps[1], gps[2]
            yaw_deg = self.filo.get(rov_id, "yaw") or 0.0

            # L0, L1, L2: Horizontal lidarlar (İleri, Sağ, Sol)
            for lidar_indis in [0, 1, 2]:
                distance = self.filo.get(rov_id, "lidar", lidar_indis)
                if distance is not None and distance > 0 and distance != -1:
                    offset = LIDAR_OFFSETS[lidar_indis]
                    theta_rad = math.radians(yaw_deg + offset)
                    ox = x0 + distance * math.sin(theta_rad)
                    oy = y0 + distance * math.cos(theta_rad)
                    oz = z0
                    obstacles.append((ox, oy, oz))
            
            # L3: Dip lidar (vertical - Z ekseni)
            distance_dip = self.filo.get(rov_id, "lidar", 3)
            if distance_dip is not None and distance_dip > 0 and distance_dip != -1:
                # Aşağı bakıyor, Z ekseninde değişim
                ox = x0
                oy = y0
                oz = z0 - distance_dip  # Simulasyon: Z negatif = derinlik
                obstacles.append((ox, oy, oz))
                
        except Exception as e:
            print(f"❌ [HATA] Engel koordinatlari hesaplanirken hata: {e}")
            import traceback
            traceback.print_exc()

        return obstacles
