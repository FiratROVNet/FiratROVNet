import math
import numpy as np # type: ignore[import-not-found]
from collections import deque
from typing import cast
from ursina import Vec3, time # type: ignore[import-not-found]
from FiratROVNet.config import Hidrodinamik, GATLimitleri, BasitKalmanFiltresi, KalmanAyarlari, PIDAyarlari # type: ignore[import-not-found]
from FiratROVNet.kutuphane.helper.gnc_helper.mixins.formation import Formasyon # type: ignore[import-not-found]
from FiratROVNet.kutuphane.moduls.PID import PID # type: ignore[import-not-found]
from panda3d.core import Vec3 as P3Vec # type: ignore[import-not-found]



class TemelGNCHelper:
    """Tekil ROV fizikleri ve kontrol mantigi."""

    APF_GUC_GECMISI_LIMITI = 150

    def __init__(self, rov_entity, filo_ref=None, gnc_ref=None):
        self.rov = rov_entity
        self.filo_ref = filo_ref
        self.gnc_ref = gnc_ref

        # Opsiyonel: APF yerine alternatif vektor kaynagi icin callable.
        self.hareket_vektor_kaynagi = None
        self._koordinator = None
        self._motor_kalman_filtreleri: list[BasitKalmanFiltresi] = []

        self.pid_depth = PID(Kp=PIDAyarlari.DEPTH_Kp, Ki=PIDAyarlari.DEPTH_Ki, Kd=PIDAyarlari.DEPTH_Kd,
                             out_min=PIDAyarlari.OUT_MIN, out_max=PIDAyarlari.OUT_MAX)
        self.pid_roll = PID(Kp=PIDAyarlari.STAB_Kp, Ki=PIDAyarlari.STAB_Ki, Kd=PIDAyarlari.STAB_Kd,
                            out_min=PIDAyarlari.OUT_MIN, out_max=PIDAyarlari.OUT_MAX)
        self.pid_pitch = PID(Kp=PIDAyarlari.STAB_Kp, Ki=PIDAyarlari.STAB_Ki, Kd=PIDAyarlari.STAB_Kd,
                             out_min=PIDAyarlari.OUT_MIN, out_max=PIDAyarlari.OUT_MAX)
        self._son_depth_gucu = 0.0

    def _vec3_gecerli_mi(self, vec) -> bool:
        try:
            return math.isfinite(float(vec.x)) and math.isfinite(float(vec.y)) and math.isfinite(float(vec.z))
        except (AttributeError, TypeError, ValueError):
            return False

    def fizik_uygula(self):
        physics_node = getattr(self.rov, 'physics_node', None)
        if physics_node is None: return

        # A. LİNEER SÖNÜMLEME (DAHA MAKUL)
        mevcut_hiz = physics_node.getLinearVelocity()
        if mevcut_hiz is None:
            mevcut_hiz = Vec3(0, 0, 0)
        hiz_buyuklugu = mevcut_hiz.length()
        f_drag = Vec3(0, 0, 0)
        
        if hiz_buyuklugu > 0.001:
            # Quadratic drag - sadece çok yüksek hızlarda etkin
            drag_quadratic = 0.5 * Hidrodinamik.SU_YOGUNLUGU * Hidrodinamik.DRAG_KATSAYISI_CD * Hidrodinamik.ON_YUZEY_ALANI * (hiz_buyuklugu ** 1.5)  # Kare yerine 1.5 üs
            drag_linear = hiz_buyuklugu * 1.0  # 2.0 yerine 1.0
            yon_vec = Vec3(-mevcut_hiz.x, -mevcut_hiz.y, -mevcut_hiz.z).normalized()
            if self._vec3_gecerli_mi(yon_vec):
                f_drag = yon_vec * (drag_quadratic + drag_linear)

        # B. AÇISAL SÖNÜMLEME
        mevcut_acisal_hiz = physics_node.getAngularVelocity()
        if mevcut_acisal_hiz is None:
            mevcut_acisal_hiz = Vec3(0, 0, 0)
            
        acisal_hiz_buyuklugu = mevcut_acisal_hiz.length()
        
        # Spin kill - hala gerekli
        if acisal_hiz_buyuklugu > 10.0:
            physics_node.setAngularVelocity(Vec3(0, 0, 0))
            mevcut_acisal_hiz = Vec3(0, 0, 0)
            acisal_hiz_buyuklugu = 0.0
            
        t_drag = Vec3(0, 0, 0)
        if acisal_hiz_buyuklugu > 0.001:
            damping_roll_pitch = 8.0
            damping_yaw = 30.0
            t_drag = Vec3(-mevcut_acisal_hiz.x * damping_roll_pitch,
                         -mevcut_acisal_hiz.y * damping_yaw,
                         -mevcut_acisal_hiz.z * damping_roll_pitch)

        # C. YERCEKIMI VE KALDIRMA KUVVETI
        batma = getattr(self.rov.gnc, 'batma_orani', 1.0)
        su_icindeki_hacim = Hidrodinamik.HACIM * batma

        f_yercekimi = Vec3(0, -Hidrodinamik.KUTLE * Hidrodinamik.YER_CEKIMI, 0)
        f_kaldirma  = Vec3(0, su_icindeki_hacim * Hidrodinamik.SU_YOGUNLUGU * Hidrodinamik.YER_CEKIMI, 0)

        physics_np = getattr(self.rov, 'physics_np', None)
        if physics_np is not None:
            cob_world = physics_np.getQuat().xform(
                P3Vec(0.0, Hidrodinamik.COB_YUKSEKLIGI, 0.0)
            )
        else:
            cob_world = P3Vec(0.0, Hidrodinamik.COB_YUKSEKLIGI, 0.0)
        physics_node.applyForce(
            P3Vec(f_kaldirma.x, f_kaldirma.y, f_kaldirma.z),
            cob_world
        )
        f_merkez = f_yercekimi + f_drag
        f_merkez = f_merkez or Vec3(0, 0, 0)

        # D. FİZİK MOTORUNA AKTARIM
        if self._vec3_gecerli_mi(f_merkez):
            limit = 20000.0
            clamped_force = P3Vec(
                max(-limit, min(limit, f_merkez.x)),
                max(-limit, min(limit, f_merkez.y)),
                max(-limit, min(limit, f_merkez.z))
            )
            physics_node.applyCentralForce(clamped_force)
        
        if t_drag is not None and self._vec3_gecerli_mi(t_drag):
            limit = 10000.0
            clamped_torque = P3Vec(
                max(-limit, min(limit, t_drag.x)),
                max(-limit, min(limit, t_drag.y)),
                max(-limit, min(limit, t_drag.z))
            )
            physics_node.applyTorque(clamped_torque)

    def vektor_to_motor_sim(self, veriler: dict[str, Vec3 | float]):
        """
        APF'den gelen 3B hareket vektörünü BlueROV2 benzeri 6 motor
        (4 yatay, 2 dikey) için güç komutlarına dönüştürür.
        """
        if self.rov is None or self.filo_ref is None:
            return
        
        # 2. ÖNCE ÇEVRESEL FİZİĞİ UYGULA (Suyun ve Dünyanın Etkisi)
        self.fizik_uygula()

        hedef_vektor = veriler.get('hedef_vektor', Vec3(0, 0, 0))
        engel_vektor = veriler.get('engel_vektor', Vec3(0, 0, 0))
        rov_vektor = veriler.get('rov_vektor', Vec3(0, 0, 0))
        if isinstance(rov_vektor, Vec3):
            # ROV-ROV kaçınması derinlik kanalını sürmemeli; dikey kontrol PID/dip lidar tarafında kalır.
            rov_vektor = Vec3(rov_vektor.x, rov_vektor.y, 0.0)


        try:
            hedef_gucu = max(0.0, min(1.0, float(veriler.get('hedef_gucu', 0.0))))
            engel_gucu = max(0.0, min(1.0, float(veriler.get('engel_gucu', 0.0))))
            rov_gucu = max(0.0, min(1.0, float(veriler.get('rov_gucu', 0.0))))
        except (TypeError, ValueError):
            hedef_gucu = 0.0
            engel_gucu = 0.0
            rov_gucu = 0.0

        aktif_formasyon = getattr(self.filo_ref, "aktif_formasyon", {}) if self.filo_ref is not None else {}
        grup_id = getattr(self.rov, "group_id", 0)
        takipci_formasyon_modu = (
            isinstance(aktif_formasyon, dict)
            and bool(aktif_formasyon.get(grup_id))
            and getattr(self.rov, "role", 0) != 1
            and getattr(getattr(self.rov, "gnc", None), "mod", 1) != 0
        )
        if takipci_formasyon_modu:
            hedef_gucu *= max(0.0, min(1.0, 1.0 - (rov_gucu)))

        def yon_vektoru_al(v):
            if not isinstance(v, Vec3):
                return Vec3(0, 0, 0)
            yon = v.normalized() if v.length() > 0.001 else Vec3(0, 0, 0)
            return Vec3(yon.x, -yon.z, yon.y)

        v_hedef = yon_vektoru_al(hedef_vektor)
        v_engel = yon_vektoru_al(engel_vektor)
        v_rov = yon_vektoru_al(rov_vektor)

        motorlar = getattr(self.rov, "motorlar", [])
        birlesik = [float(getattr(motor, "guc", 0.0)) for motor in motorlar]
        if not motorlar:
            return

        # Hedef için güçleri hesapla
        h_hareket = self.filo_ref.tum_motorlarin_guclerini_hesapla(self.rov.id, v_hedef, hedef_gucu)
        h_yaw, _ = self.filo_ref.yaw_gucleri_hesapla(self.rov, v_hedef, hedef_gucu)

        # Engel için güçleri hesapla
        e_hareket = self.filo_ref.tum_motorlarin_guclerini_hesapla(self.rov.id, v_engel, engel_gucu)
        e_yaw, _ = self.filo_ref.yaw_gucleri_hesapla(self.rov, v_engel, engel_gucu)

        # ROV (sürü) için güçleri hesapla
        r_hareket = self.filo_ref.tum_motorlarin_guclerini_hesapla(self.rov.id, v_rov, rov_gucu)
        r_yaw, _ = self.filo_ref.yaw_gucleri_hesapla(self.rov, v_rov, rov_gucu)

        h = 0.9
        y = 0.1
        


        ham_hedef = 1 * hedef_gucu
        ham_engel = 1 * engel_gucu
        ham_rov = 1 * rov_gucu

        toplam = ham_hedef + ham_engel + ham_rov

        if toplam > 0:
            ham_hedef /= toplam
            ham_engel /= toplam
            ham_rov /= toplam


        if isinstance(hedef_vektor, Vec3) and isinstance(engel_vektor, Vec3) and isinstance(rov_vektor, Vec3):
            self._bileske_vektoru_minimapte_ciz(self.rov.id, hedef_vektor*ham_hedef, engel_vektor*ham_engel, rov_vektor*ham_rov)

        if self.rov.group_id==0 and False:
            if rov_gucu > 0.1:
                print(f"ROV-id:{self.rov.id}, ROV-Guc:{rov_gucu}, Ham-ROV-Guc:{ham_rov}")

        
        apf_guc_kuyruklari_aktif = self._apf_guc_kuyruklari_aktif_mi()

        for i, _motor in enumerate(motorlar):
            # Hedef bileşkesi
            hareket_h = h_hareket[i] if i < len(h_hareket) else birlesik[i]
            yaw_h = h_yaw[i] if i < len(h_yaw) else birlesik[i]
            hedef_toplam = hareket_h * h + yaw_h * y

            # Engel bileşkesi
            hareket_e = e_hareket[i] if i < len(e_hareket) else birlesik[i]
            yaw_e = e_yaw[i] if i < len(e_yaw) else birlesik[i]
            engel_toplam = hareket_e * h + yaw_e * y

            # ROV bileşkesi
            hareket_r = r_hareket[i] if i < len(r_hareket) else birlesik[i]
            yaw_r = r_yaw[i] if i < len(r_yaw) else birlesik[i]
            rov_toplam = hareket_r * h + yaw_r * y
            
            # Katsayılarla yeni bileşke:
            # DİKKAT: Katsayıların toplamı KESİNLİKLE 1.0 olmalıdır (w_engel + w_rov + hedef_katsayisi = 1.0)
            # Aksi takdirde dikey motorlar cached_power üzerinden beslendiği için üstel olarak (NaN) patlar!
            engel_katki = engel_toplam * ham_engel
            rov_katki = rov_toplam * ham_rov
            hedef_katki = hedef_toplam * ham_hedef

            birlesik[i] = engel_katki + rov_katki + hedef_katki

        filtrelenmis_gucler = self._motor_guclerini_kalman_filtrele(birlesik)
        if not filtrelenmis_gucler:
            filtrelenmis_gucler = birlesik
        if apf_guc_kuyruklari_aktif:
            self._apf_guc_kuyruklarini_guncelle(hedef_gucu, engel_gucu, rov_gucu)
        self.filo_ref.motorlari_calistir(self.rov.id, birlesik)

    def _apf_guc_kuyruklari_aktif_mi(self) -> bool:
        hud = getattr(self.filo_ref, "apf_guc_hud", None) if self.filo_ref is not None else None
        return bool(getattr(hud, "visible", False))

    def _apf_guc_kuyruklarini_guncelle(self, hedef_guc: float, engel_guc: float, rov_guc: float):
        gnc = getattr(self.rov, "gnc", None)
        if gnc is None:
            return

        def kuyruk_al(attr: str):
            mevcut = getattr(gnc, attr, None)
            if isinstance(mevcut, deque) and mevcut.maxlen == self.APF_GUC_GECMISI_LIMITI:
                return mevcut
            try:
                yeni = deque(
                    list(mevcut)[-self.APF_GUC_GECMISI_LIMITI:] if mevcut is not None else [],
                    maxlen=self.APF_GUC_GECMISI_LIMITI,
                )
            except TypeError:
                yeni = deque(maxlen=self.APF_GUC_GECMISI_LIMITI)
            setattr(gnc, attr, yeni)
            return yeni

        for attr, deger in (
            ("hedef_guc", hedef_guc),
            ("engel_guc", engel_guc),
            ("rov_guc", rov_guc),
        ):
            try:
                temiz_deger = float(deger)
            except (TypeError, ValueError):
                temiz_deger = 0.0
            kuyruk_al(attr).append(max(0.0, min(1.0, temiz_deger)))

    def _motor_kalman_filtrelerini_hazirla(self, motor_sayisi: int):
        if motor_sayisi <= 0:
            self._motor_kalman_filtreleri = []
            return
        if len(self._motor_kalman_filtreleri) == motor_sayisi:
            return

        self._motor_kalman_filtreleri = [
            BasitKalmanFiltresi(
                R=KalmanAyarlari.MOTOR_R,
                Q=KalmanAyarlari.MOTOR_Q,
                baslangic_degeri=0.0,
            )
            for _ in range(motor_sayisi)
        ]

    def _motor_guclerini_kalman_filtrele(self, gucler: list[float]) -> list[float]:
        if not gucler:
            return []

        self._motor_kalman_filtrelerini_hazirla(len(gucler))
        filtrelenmis: list[float] = []
        for filtre, guc in zip(self._motor_kalman_filtreleri, gucler):
            filtre.ayarla(R=KalmanAyarlari.MOTOR_R, Q=KalmanAyarlari.MOTOR_Q)
            filtrelenmis.append(float(filtre.guncelle(float(guc))))
        return filtrelenmis

    def _guncelle_kontroller(self):
        """
        Temel kontrolleri yapar: gnc_ref, rov ve manuel_kontrol kontrolu.
        """
        if self.gnc_ref is None:
            return False
        if self.rov is None:
            return False
        if self.gnc_ref.manuel_kontrol:
            return False
        return True

    def _koordinator_al(self):
        """Koordinator'u lazy import ile alir (circular import onleme)."""
        if self._koordinator is None:
            from FiratROVNet.gnc import Koordinator # type: ignore[import-not-found]
            self._koordinator = Koordinator
        return self._koordinator

    def _guc_orani_hesapla(self, mesafe: float, limit=(np.sqrt(3) * 400)):
        if mesafe < 2:
            return 0.0
        if mesafe < GATLimitleri.CARPISMA:
            guc = mesafe / limit
            guc = np.log(guc * 10 + 1) / np.log(11)
        else:
            guc = 1.0
        return guc
    def _formasyon_dinamik_guncelle(self, rov_id: int):
        """
        Eger aktif bir formasyon varsa, takipcilerin hedeflerini
        liderin o anki konumuna ve yonune gore gunceller.
        """
        filo = self.filo_ref
        if filo is None:
            return
        aktif = filo.aktif_formasyon.get(self.rov.group_id, False)
        if not aktif:
            return

        grup_rovs = filo.g_rovs.get(self.rov.group_id) if hasattr(filo, 'g_rovs') else None
        lider = next((r for r in (grup_rovs or []) if r and r.role == 1), None)
        if not lider:
            return

        if self.rov.role == 1:
            f_obj = Formasyon(filo)
            lider_pos_sim = (lider.x, lider.z, lider.y)
            yeni_pozisyonlar = f_obj.pozisyonlar(
                aktif['id'],
                aktif['aralik'],
                is_3d=aktif['is_3d'],
                lider_koordinat=lider_pos_sim,
                g_id=self.rov.group_id,
            )
            if not hasattr(filo, 'yeni_pozisyonlar') or not isinstance(filo.yeni_pozisyonlar, dict):
                filo.yeni_pozisyonlar = {}
            filo.yeni_pozisyonlar[self.rov.group_id] = yeni_pozisyonlar
            return

        if self.rov.gnc.mod == 0:
            return

        yeni_pozisyonlar = filo.yeni_pozisyonlar
        if yeni_pozisyonlar is None:
            return

        if isinstance(yeni_pozisyonlar, dict):
            yeni_pozisyonlar = yeni_pozisyonlar.get(self.rov.group_id)
            if yeni_pozisyonlar is None:
                return

        hedef = None
        if isinstance(yeni_pozisyonlar, dict):
            hedef = yeni_pozisyonlar.get(self.rov.id)
        elif isinstance(yeni_pozisyonlar, list):
            if grup_rovs:
                try:
                    grup_index = next(i for i, r in enumerate(grup_rovs) if r and r.id == self.rov.id)
                except StopIteration:
                    grup_index = None
                if grup_index is not None and grup_index < len(yeni_pozisyonlar):
                    hedef = yeni_pozisyonlar[grup_index]

        if hedef is not None:
            try:
                hx, hy, hz = float(hedef[0]), float(hedef[1]), float(hedef[2])
            except (TypeError, ValueError, IndexError):
                return
            if not math.isfinite(hx) or not math.isfinite(hy) or not math.isfinite(hz):
                return
            filo.hedef((hx, hy, hz), rov_id=self.rov.id)

    def _guncelle_waypoint_takip(self, rov_id: int):
        """
        A* cikisi olan (x, z) noktalarini takip eder.
        """
        filo = self.filo_ref
        if filo is None:
            return None, False
        # Rota listesini al
        nokta_listesi = getattr(filo, '_git_nokta_listesi', {}).get(rov_id)
        if not nokta_listesi:
            return None, False

        mevcut_indeks = getattr(filo, '_git_mevcut_nokta_indeksi', {}).get(rov_id, 0)

        # Rota bitti mi?
        if mevcut_indeks >= len(nokta_listesi):
            filo._git_nokta_listesi.pop(rov_id, None)
            filo._git_mevcut_nokta_indeksi.pop(rov_id, None)
            if hasattr(filo, '_git_hedef_derinligi'):
                filo._git_hedef_derinligi.pop(rov_id, None)
            return None, True

        # Mevcut waypoint
        wp = nokta_listesi[mevcut_indeks]
        current_gps = filo.get(rov_id, "gps")

        target_x = float(wp[0])
        target_y = float(wp[1])
        
        # Hedef derinliği kullan (varsa), yoksa mevcut derinliği koru
        target_depth = None
        if hasattr(filo, '_git_hedef_derinligi'):
            target_depth = filo._git_hedef_derinligi.get(rov_id)
        
        if target_depth is not None:
            target_z = target_depth
        else:
            target_z = current_gps[2]

        waypoint_hedef = Vec3(target_x, target_y, target_z)

        dist_x = waypoint_hedef.x - current_gps[0]
        dist_y = waypoint_hedef.y - current_gps[1]
        mesafe_yatay = math.sqrt(dist_x ** 2 + dist_y ** 2)

        if mesafe_yatay < GATLimitleri.CARPISMA:
            filo._git_mevcut_nokta_indeksi[rov_id] = mevcut_indeks + 1
            if mevcut_indeks + 1 < len(nokta_listesi):
                next_wp = nokta_listesi[mevcut_indeks + 1]
                filo.hedef((next_wp[0], next_wp[1], target_z), rov_id=rov_id, ciz=False)
            return waypoint_hedef, False

        return waypoint_hedef, False

    def _hareket_vektor_verisi_al(self, rov_id: int, hedef_koordinat):
        """
        Hareket vektoru verisini ureten kaynak.
        Varsayilan APF; harici kaynaklar icin hareket_vektor_kaynagi atanabilir.
        """
        if callable(self.hareket_vektor_kaynagi):
            return self.hareket_vektor_kaynagi(rov_id=rov_id, hedef_koordinat=hedef_koordinat)
        filo = self.filo_ref
        if filo is None or filo.helper is None:
            return None
        hedefler = getattr(filo, "_rov_hedefleri", None)
        hedefler_dict = hedefler if isinstance(hedefler, dict) else None
        eski_hedef_var = False
        eski_hedef = None
        if hedefler_dict is not None:
            eski_hedef_var = rov_id in hedefler_dict
            if eski_hedef_var:
                eski_hedef = hedefler_dict.get(rov_id)
        if hedef_koordinat is not None and hedefler_dict is not None:
            hedefler_dict[rov_id] = hedef_koordinat
        try:
            return filo.helper.apf(
                rov_id=rov_id,
                hedef=(hedef_koordinat is not None),
                engel=True,
                rov=True,
            )
        finally:
            if hedef_koordinat is not None and hedefler_dict is not None:
                if eski_hedef_var:
                    hedefler_dict[rov_id] = eski_hedef
                else:
                    hedefler_dict.pop(rov_id, None)

    def _log_mesafe_etkisi_hesapla(self, mesafe: float, limit: float) -> float:
        """Mesafeye gore logaritmik etki katsayisi hesaplar (0.0-1.0)."""
        if limit < 1.0:
            limit = 1.0
        if mesafe >= limit:
            return 0.0
        guvenli_mesafe = max(1.0, mesafe)
        guvenli_limit = max(1.000001, limit)
        return max(0.0, 1.0 - float(np.log(guvenli_mesafe)) / float(np.log(guvenli_limit)))

    def _engel_vektoru_isle(self, sonuc):
        engeller = sonuc.get('engeller') or []
        if not engeller:
            return Vec3(0, 0, 0), 0.0
        
        toplam_vektor = Vec3(0, 0, 0)
        max_engel_etkisi = 0.0
        engel_limit = float(GATLimitleri.ENGEL)
        
        for e_info in engeller:
            bv = e_info.get('birim_vektor', [0, 0, 0])
            vx = float(bv[0]) if len(bv) > 0 else 0.0
            vy = float(bv[1]) if len(bv) > 1 else 0.0
            vz = float(bv[2]) if len(bv) > 2 else 0.0
            
            mesafe = float(e_info.get('mesafe', 0.0))
            if mesafe >= engel_limit:
                continue

            etki = self._log_mesafe_etkisi_hesapla(mesafe, engel_limit)
            if etki > max_engel_etkisi:
                max_engel_etkisi = etki
                
            carpan = etki
            toplam_vektor.x += vx * carpan
            toplam_vektor.y += vy * carpan
            toplam_vektor.z += vz * carpan
            
        return toplam_vektor, max_engel_etkisi

    def _rov_vektoru_isle(self, sonuc):
        rovs = sonuc.get('rovs') or []
        if not rovs:
            return Vec3(0, 0, 0), 0.0
            
        toplam_vektor = Vec3(0, 0, 0)
        carpisma_limit = float(getattr(GATLimitleri, "ROV_GUVENLIK_MESAFESI", GATLimitleri.CARPISMA))
        max_carpan = 0.0
        
        for r_info in rovs:
            bv = r_info.get('birim_vektor', [0, 0, 0])
            vx = float(bv[0]) if len(bv) > 0 else 0.0
            vy = float(bv[1]) if len(bv) > 1 else 0.0
            # ROV kaçınması yalnızca yatay düzlemde uygulanır. Derinlik bileşeni ROV'ları
            # birbirinden kaçarken batırıp/çıkarmasın; dikey güvenlik engel APF + depth PID'de kalır.
            yatay_norm = math.sqrt((vx ** 2) + (vy ** 2))
            if yatay_norm <= 1e-6:
                continue
            vx /= yatay_norm
            vy /= yatay_norm
            vz = 0.0
            
            mesafe = float(r_info.get('mesafe', 0.0))
            if mesafe >= carpisma_limit:
                continue

            etki = self._log_mesafe_etkisi_hesapla(mesafe, carpisma_limit)
                
            carpan = etki
            if carpan > max_carpan:
                max_carpan = carpan
            toplam_vektor.x += vx * carpan
            toplam_vektor.y += vy * carpan
            toplam_vektor.z += vz * carpan
            
        return toplam_vektor, max_carpan

    def _hedef_vektoru_isle(self, sonuc, max_engel_etkisi: float):
        h_info = sonuc.get('hedef') or {}
        h_mesafe_raw = h_info.get('mesafe', 0.0)
        try:
            h_mesafe = float(h_mesafe_raw) # type: ignore
        except (TypeError, ValueError):
            h_mesafe = 0.0
        h_birim = Vec3(*h_info.get('birim_vektor', [0, 0, 0]))

        guc0 = self._guc_orani_hesapla(h_mesafe)
        hedef_agirligi = (1.0 - max_engel_etkisi)
        hedef_vektor = h_birim * hedef_agirligi

        return hedef_vektor, guc0

    def _bileske_vektoru_minimapte_ciz(
        self,
        rov_id: int,
        hedef_vektor: Vec3,
        engel_vektor: Vec3,
        rov_vektor: Vec3,
    ) -> None:
        """Net APF bileskesini mavi vektor olarak minimap cizim listesine ekler."""
        filo = self.filo_ref
        helper = getattr(filo, "helper", None) if filo is not None else None
        if helper is None or not hasattr(helper, "vektor"):
            return

        bileske_x = float(getattr(hedef_vektor, "x", 0.0)) + float(getattr(engel_vektor, "x", 0.0)) + float(getattr(rov_vektor, "x", 0.0))
        bileske_y = float(getattr(hedef_vektor, "y", 0.0)) + float(getattr(engel_vektor, "y", 0.0)) + float(getattr(rov_vektor, "y", 0.0))
        yatay_buyukluk = math.sqrt((bileske_x ** 2) + (bileske_y ** 2))
        if yatay_buyukluk <= 0.001:
            return

        cizgi_uzunlugu = min(80.0, yatay_buyukluk * 30.0)
        helper.vektor(
            rov_id_ilk=rov_id,
            vektor=(bileske_x, bileske_y, 0.0),
            renk='m',
            uzunluk=cizgi_uzunlugu,
            ciz=True,
        )

    def _bileske_vektor_hesapla(self, sonuc, rov_id: int) -> dict[str, Vec3 | float]:
        # Temel hedef gucunu al
        h_info = sonuc.get('hedef') or {}
        guc0 = self._guc_orani_hesapla(float(h_info.get('mesafe', 0.0)))
        
        engel_vektor, max_engel_etkisi = self._engel_vektoru_isle(sonuc)
        rov_vektor, rov_carpan = self._rov_vektoru_isle(sonuc)

        # Hedef agirligini yeni etkilerle 1 kere hesapla (kaldirilan cift cagir)
        hedef_vektor, guc1 = self._hedef_vektoru_isle(sonuc, max_engel_etkisi)

        if self.rov.role==1 and self.rov.group_id==0 and False:
            print("engel_vektor", engel_vektor.length())
            print("rov_vektor", rov_vektor.length())
            print("hedef_vektor", hedef_vektor.length())

        return {
            'hedef_vektor': hedef_vektor,
            'engel_vektor': engel_vektor,
            'rov_vektor': rov_vektor,
            'hedef_gucu': guc1,
            'engel_gucu': max_engel_etkisi,
            'rov_gucu': rov_carpan,
        }

    def _guncelle_hareket_uygula(self, rov_id: int):
        """
        APF kullanarak ROV hareketini yonetir.
        Waypoint takip mekanizmasi ile git_path() cagrilarini destekler.
        """
        filo = self.filo_ref
        if filo is None:
            return
        waypoint_hedef, _ = self._guncelle_waypoint_takip(rov_id)
        if waypoint_hedef:
            self.rov.hedef = waypoint_hedef

        self._formasyon_dinamik_guncelle(rov_id)
        hedef_koordinat = filo.hedef(rov_id=rov_id)
        hedef_koordinat = self._rol_derinlik_hedefi_ekle(hedef_koordinat)

        sonuc = self._hareket_vektor_verisi_al(rov_id=rov_id, hedef_koordinat=hedef_koordinat)
        if not sonuc:
            return

        veriler = self._bileske_vektor_hesapla(sonuc, rov_id)

        # Derinlik (Z) bileşenini PID çıktısıyla değiştir.
        # XY yatay hareketi APF'te kalır; sadece dikey kanal PID tarafından yönetilir.
        veriler = self._pid_depth_vektor_duzenle(veriler, hedef_koordinat)

        self.vektor_to_motor_sim(veriler)
        self._pid_stab_motor_uygula()

    def _rol_derinlik_hedefi_ekle(self, hedef_koordinat):
        """
        ROV'un rolüne göre derinlik bandını motorlarla tutması için hedefe Z ekler.
        Açık bir hedef yoksa mevcut X/Y korunur ve sadece derinlik hedefi üretilir.
        """
        filo = self.filo_ref
        if filo is None or self.rov is None or not hasattr(filo, "rol_derinligini_uygula"):
            return hedef_koordinat
        gnc = self.gnc_ref
        gps = getattr(gnc, "gps", None) if gnc is not None else None
        if gps is None or len(gps) < 3:
            return hedef_koordinat

        try:
            mevcut_z = float(gps[2])
            hedef_z = filo.rol_derinligini_uygula(self.rov, mevcut_z)
        except (TypeError, ValueError):
            return hedef_koordinat
        if hedef_z is None:
            return hedef_koordinat

        if hedef_koordinat is None:
            return (float(gps[0]), float(gps[1]), float(hedef_z))
        if len(hedef_koordinat) < 3 or hedef_koordinat[2] is None:
            return (float(hedef_koordinat[0]), float(hedef_koordinat[1]), float(hedef_z))
        return hedef_koordinat

    def _pid_dt(self) -> float:
        """Güvenli dt değeri döner."""
        dt = float(getattr(time, "dt", 0.03) or 0.03)
        return dt if 1e-4 < dt < 1.0 else 0.03

    def _pid_stab_motor_uygula(self):
        """
        Her kare çalışır — sürekli aktif stabilizasyon.
        Roll ve pitch PID çıktılarını mixing matrix ile birleştirip
        her dikey motoru (m4-m7) tam olarak bir kez çağırır.

        Mixing matrix (gövde geometrisinden):
              roll   pitch
          m4:  +1     +1   (sol-ön)
          m5:  -1     +1   (sağ-ön)
          m6:  +1     -1   (sol-arka)
          m7:  -1     -1   (sağ-arka)

        filo.roll() + filo.pitch() yerine bu kullanılmalı;
        iki ayrı çağrı motorları iki kez ateşleyerek kuvvetleri çakıştırırdı.
        """
        if self.rov is None:
            return
        gnc = self.gnc_ref
        if gnc is None:
            return

        b_pitch = float(getattr(gnc, 'bullet_pitch', 0.0))
        b_roll  = float(getattr(gnc, 'bullet_roll',  0.0))

        # [-180, 180] normalize
        b_pitch = ((b_pitch + 180.0) % 360.0) - 180.0
        b_roll  = ((b_roll  + 180.0) % 360.0) - 180.0

        dt = self._pid_dt()

        # Roll, pitch ve derinlik/heave stabilizasyonu — m4-m7.
        # Pozitif heave komutu batma, negatif komut çıkma yönünde çalışır.
        heave = float(getattr(self, "_son_depth_gucu", 0.0))
        guc_roll  = float(self.pid_roll.compute( hedef=0.0, durum=b_roll,  dt=dt, normalize=True))
        guc_pitch = float(self.pid_pitch.compute(hedef=0.0, durum=b_pitch, dt=dt, normalize=True))

        m4 = heave + guc_roll + guc_pitch   # sol-ön
        m5 = heave - guc_roll + guc_pitch   # sağ-ön
        m6 = heave + guc_roll - guc_pitch   # sol-arka
        m7 = heave - guc_roll - guc_pitch   # sağ-arka


        motorlar = getattr(self.rov, 'motorlar', None)
        if motorlar is None or len(motorlar) < 8:
            return

        ESIK = 1e-4

        # m4-m7: heave + roll + pitch (dikey motorlar — doğrudan set)
        for idx, guc in ((4, m4), (5, m5), (6, m6), (7, m7)):
            if abs(guc) > ESIK:
                motorlar[idx].calistir(float(max(-1.0, min(1.0, guc))))


    def _pid_depth_vektor_duzenle(self, veriler: dict, hedef_koordinat) -> dict:
        """
        hedef_vektor'ün Z (derinlik) bileşenini ham APF değeri yerine
        depth PID çıktısıyla değiştirir.

        XY yatay hareketi tamamen APF'te kalır; sadece dikey kanal PID tarafından
        yönetilir. Bu, derinlik salınımlarını bastırır ve steady-state hatasını siler.
        """
        if not hedef_koordinat or len(hedef_koordinat) < 3 or hedef_koordinat[2] is None:
            self._son_depth_gucu = 0.0
            return veriler
        gnc = self.gnc_ref
        if gnc is None:
            self._son_depth_gucu = 0.0
            return veriler
        gps = gnc.gps
        if gps is None or len(gps) < 3:
            self._son_depth_gucu = 0.0
            return veriler

        hedef_z = float(hedef_koordinat[2])
        if self.filo_ref is not None and hasattr(self.filo_ref, "rol_derinligini_uygula"):
            hedef_z = float(self.filo_ref.rol_derinligini_uygula(self.rov, hedef_z))
        mevcut_z = float(gps[2])
        hata = hedef_z - mevcut_z

        # Küçük hatada orijinal APF Z'yi koru (gereksiz PID aktivasyonundan kaçın)
        tolerans = float(getattr(PIDAyarlari, "DEPTH_TOLERANS", 0.15))
        if abs(hata) < tolerans:
            self._son_depth_gucu = 0.0
            return veriler

        pid_z = self.pid_depth.compute(hedef=hedef_z, durum=mevcut_z,
                                       dt=self._pid_dt(), normalize=True)
        depth_gucu = -float(pid_z) * float(getattr(PIDAyarlari, "DEPTH_THRUST_GAIN", 1.0))
        min_guc = float(getattr(PIDAyarlari, "DEPTH_MIN_THRUST", 0.0))
        if abs(depth_gucu) < min_guc:
            depth_gucu = min_guc if depth_gucu >= 0 else -min_guc
        self._son_depth_gucu = max(-1.0, min(1.0, depth_gucu))

        hv = veriler.get('hedef_vektor', Vec3(0, 0, 0))
        if not isinstance(hv, Vec3):
            return veriler

        veriler['hedef_vektor'] = Vec3(hv.x, hv.y, float(pid_z))
        return veriler

    def _pid_depth_uygula(self, hedef_koordinat):
        """
        Hedef derinlik ile mevcut derinlik arasındaki Z hatasını PID ile
        dikey motorlara ek kuvvet olarak uygular.
        """
        if not hedef_koordinat or len(hedef_koordinat) < 3:
            return
        gnc = self.gnc_ref
        if gnc is None:
            return
        filo = self.filo_ref
        if filo is None:
            return

        hedef_z = float(hedef_koordinat[2]) if hedef_koordinat[2] is not None else 0.0
        gps = gnc.gps
        if gps is None or len(gps) < 3:
            return
        mevcut_z = float(gps[2])

        hata = hedef_z - mevcut_z
        if abs(hata) < 0.2:
            return  # Derinlik zaten yeterince doğru

        pid_cikti = self.pid_depth.compute(hedef=0.0, durum=-hata, dt=self._pid_dt(), normalize=True)

        # Dikey motorlara (m4–m7) ek kuvvet uygula
        guc = float(pid_cikti) * 0.5
        for motor_adi in ('m4', 'm5', 'm6', 'm7'):
            motor = getattr(self.rov, motor_adi, None)
            if motor is not None:
                mevcut_guc = float(getattr(motor, 'guc', 0.0))
                yeni_guc = max(-1.0, min(1.0, mevcut_guc + guc))
                motor.calistir(yeni_guc)

    def _pid_stab_uygula(self):
        """
        Roll ve pitch'i sıfırda tutmak için PID bazlı stabilizasyon torku.
        Mevcut roll_koru/pitch_koru'yu tamamlar.
        """
        filo = self.filo_ref
        if filo is None:
            return
        sensor = getattr(self.rov, 'sensor', None)
        if sensor is None:
            return
        imu = getattr(sensor, 'imu', None) or {}
        orientation = imu.get('orientation', {})
        dt = self._pid_dt()

        roll  = float(orientation.get('roll', 0.0))
        pitch = float(orientation.get('pitch', 0.0))

        if abs(roll) > 1.0:
            roll_cikti = self.pid_roll.compute(hedef=0.0, durum=roll, dt=dt, normalize=True)
            if abs(roll_cikti) > 1e-3:
                filo.roll_koru(self.rov, guc_orani=float(abs(roll_cikti)))

        if abs(pitch) > 1.0:
            pitch_cikti = self.pid_pitch.compute(hedef=0.0, durum=pitch, dt=dt, normalize=True)
            if abs(pitch_cikti) > 1e-3:
                filo.pitch_koru(self.rov, guc_orani=float(abs(pitch_cikti)))

    def pid_kazanclari_guncelle(self, depth_Kp=None, depth_Ki=None, depth_Kd=None,
                                stab_Kp=None, stab_Ki=None, stab_Kd=None):
        """
        filo._on_pid_bar_change() tarafından çağrılır.
        Slider değiştiğinde ilgili per-ROV PID nesnelerini günceller.
        """
        if depth_Kp is not None: self.pid_depth.Kp = float(depth_Kp)
        if depth_Ki is not None: self.pid_depth.Ki = float(depth_Ki)
        if depth_Kd is not None: self.pid_depth.Kd = float(depth_Kd)
        if stab_Kp is not None:  self.pid_roll.Kp  = float(stab_Kp);  self.pid_pitch.Kp = float(stab_Kp)
        if stab_Ki is not None:  self.pid_roll.Ki  = float(stab_Ki);  self.pid_pitch.Ki = float(stab_Ki)
        if stab_Kd is not None:  self.pid_roll.Kd  = float(stab_Kd);  self.pid_pitch.Kd = float(stab_Kd)

    def guncelle(self, gat_kodu=None):
        """
        GNC guncelleme: Hedef varsa APF ile vektor hesaplar ve motor komutlarini uygular;
        hedef yoksa sonumler.
        """
        if not self._guncelle_kontroller():
            return

        self._guncelle_hareket_uygula(rov_id=self.rov.id)
