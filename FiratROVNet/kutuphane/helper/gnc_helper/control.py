import math
import numpy as np # type: ignore[import-not-found]
from typing import cast
from ursina import Vec3, time # type: ignore[import-not-found]
from FiratROVNet.config import Hidrodinamik, GATLimitleri, BasitKalmanFiltresi, KalmanAyarlari # type: ignore[import-not-found]
from FiratROVNet.kutuphane.helper.gnc_helper.mixins.formation import Formasyon # type: ignore[import-not-found]
from panda3d.core import Vec3 as P3Vec # type: ignore[import-not-found]



class TemelGNCHelper:
    """Tekil ROV fizikleri ve kontrol mantigi."""

    def __init__(self, rov_entity, filo_ref=None, gnc_ref=None):
        self.rov = rov_entity
        self.filo_ref = filo_ref
        self.gnc_ref = gnc_ref

        # Opsiyonel: APF yerine alternatif vektor kaynagi icin callable.
        self.hareket_vektor_kaynagi = None
        self._koordinator = None
        self._motor_kalman_filtreleri: list[BasitKalmanFiltresi] = []

    def _vec3_gecerli_mi(self, vec) -> bool:
        try:
            return math.isfinite(float(vec.x)) and math.isfinite(float(vec.y)) and math.isfinite(float(vec.z))
        except (AttributeError, TypeError, ValueError):
            return False


    def yaw_uygula(self, hedef_vektor: Vec3 | None = None, guc: float = 0.1):
        if hedef_vektor is None:
            hedef_vektor = Vec3(0, 0, 0)
        V_rov = self.rov.gnc.r_bv
        if V_rov is None:
            V_rov = Vec3(0, 0, 0)
        V_rov = Vec3(V_rov.x, 0, V_rov.z)
        hedef_vektor = Vec3(hedef_vektor.x, 0, hedef_vektor.z)

        len_rov = V_rov.length()
        len_hedef = hedef_vektor.length()
        if len_rov * len_hedef < 1e-9:
            return

        scaler_carpim = V_rov.dot(hedef_vektor)
        
        # Float hassasiyeti nedeniyle domain error almamak için -1.0 ile 1.0 arasına sıkıştır
        oran_degeri = scaler_carpim / (len_rov * len_hedef)
        oran_degeri = max(-1.0, min(1.0, oran_degeri))
        
        radyan = math.acos(oran_degeri)
        aci = math.degrees(radyan)
        oran = aci / 360.0
        filo = self.filo_ref
        if filo is not None:
            filo.yaw(self.rov, oran * guc * 0.1)



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
            damping_factor = 30.0  # 10.0 yerine 5.0 - daha da düşük
            t_drag = Vec3(-mevcut_acisal_hiz.x * damping_factor, 
                        -mevcut_acisal_hiz.y * damping_factor, 
                        -mevcut_acisal_hiz.z * damping_factor)

        # C. YERÇEKİMİ VE KALDIRMA KUVVETİ
        batma = getattr(self.rov.gnc, 'batma_orani', 1.0) 
        su_icindeki_hacim = Hidrodinamik.HACIM * batma
        
        f_yercekimi = Vec3(0, -Hidrodinamik.KUTLE * Hidrodinamik.YER_CEKIMI, 0)
        f_kaldirma = Vec3(0, su_icindeki_hacim * Hidrodinamik.SU_YOGUNLUGU * Hidrodinamik.YER_CEKIMI, 0)
        f_net_env: Vec3 = f_yercekimi + f_kaldirma + f_drag

        # D. FİZİK MOTORUNA AKTARIM
        if f_net_env is not None and self._vec3_gecerli_mi(f_net_env):
            # Limit uygula ama çok agresif olma
            limit = 20000.0
            clamped_force = P3Vec(
                max(-limit, min(limit, f_net_env.x)),
                max(-limit, min(limit, f_net_env.y)),
                max(-limit, min(limit, f_net_env.z))
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
        # Güç oranını sınırla
        try:
            guc_orani = float(veriler.get('guc_orani', 0.0))
        except (TypeError, ValueError):
            guc_orani = 0.0
        guc_orani = max(0.0, min(1.0, guc_orani))

        bileske_vektor = veriler.get('bileske_vektor', Vec3(0, 0, 0))
        if not isinstance(bileske_vektor, Vec3):
            bileske_vektor = Vec3(0, 0, 0)
        final_yon = bileske_vektor.normalized() if bileske_vektor.length() > 0.001 else Vec3(0, 0, 0)
        v_sim_dir = Vec3(final_yon.x, -final_yon.z, final_yon.y)

        if self.rov.role == 1 and self.rov.group_id == 0 and False:
            print(v_sim_dir,v_sim_dir.y)
        
        if self.rov is None or self.filo_ref is None:
            return
        
        # 2. ÖNCE ÇEVRESEL FİZİĞİ UYGULA (Suyun ve Dünyanın Etkisi)
        self.fizik_uygula()
        # 3. MOTOR GÜÇLERİNİ HESAPLA VE UYGULA
        #self.filo_ref.roll_koru(self.rov, guc_orani)
        #self.filo_ref.pitch_koru(self.rov, guc_orani)
        
        motorlar = getattr(self.rov, "motorlar", [])
        birlesik = [float(getattr(motor, "guc", 0.0)) for motor in motorlar]
        if not motorlar:
            return

        hareket_gucleri = self.filo_ref.tum_motorlarin_guclerini_hesapla(self.rov.id, v_sim_dir, guc_orani)
        yaw_gucleri, _ = self.filo_ref.yaw_gucleri_hesapla(self.rov, v_sim_dir, guc_orani)
        #roll_gucleri = self.filo_ref.roll_guclerini_hesapla(self.rov, guc_orani)
        h = 0.85
        y = 0.05
        for i, _motor in enumerate(motorlar):
            hareket = hareket_gucleri[i] if i < len(hareket_gucleri) else birlesik[i]
            yaw = yaw_gucleri[i] if i < len(yaw_gucleri) else birlesik[i]
            #roll = roll_gucleri[i] if i < len(roll_gucleri) else birlesik[i]
            
            birlesik[i] = hareket * h + yaw * y #+ roll * r

        filtrelenmis_gucler = self._motor_guclerini_kalman_filtrele(birlesik)
        self.filo_ref.motorlari_calistir(self.rov.id, birlesik)

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

        if self.rov.gnc.mod == 0:
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
        return filo.helper.apf(
            rov_id=rov_id,
            hedef=(hedef_koordinat is not None),
            engel=True,
            rov=True,
        )

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
        carpisma_limit = float(GATLimitleri.CARPISMA)
        max_carpan = 0.0
        
        for r_info in rovs:
            bv = r_info.get('birim_vektor', [0, 0, 0])
            vx = float(bv[0]) if len(bv) > 0 else 0.0
            vy = float(bv[1]) if len(bv) > 1 else 0.0
            vz = float(bv[2]) if len(bv) > 2 else 0.0
            
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

    def _bileske_vektor_hesapla(self, sonuc, rov_id: int) -> dict[str, Vec3 | float]:
        # Temel hedef gucunu al
        h_info = sonuc.get('hedef') or {}
        guc0 = self._guc_orani_hesapla(float(h_info.get('mesafe', 0.0)))
        
        engel_vektor, max_engel_etkisi = self._engel_vektoru_isle(sonuc)
        rov_vektor, rov_carpan = self._rov_vektoru_isle(sonuc)

        # Hedef agirligini yeni etkilerle 1 kere hesapla (kaldirilan cift cagir)
        hedef_vektor, guc1 = self._hedef_vektoru_isle(sonuc, max_engel_etkisi)

        engel_vektor_agirlikli = engel_vektor * 0.4
        rov_vektor_agirlikli = rov_vektor * 0.4
        hedef_vektor_agirlikli = hedef_vektor * 0.2

        bileske_vektor: Vec3 = engel_vektor_agirlikli + rov_vektor_agirlikli + hedef_vektor_agirlikli
        guc = max(guc1, rov_carpan)
        if guc < 0.01:
            guc = guc0

        return {
            'hedef_vektor': hedef_vektor_agirlikli,
            'engel_vektor': engel_vektor_agirlikli,
            'rov_vektor': rov_vektor_agirlikli,
            'hedef_gucu': guc1,
            'engel_gucu': max_engel_etkisi,
            'rov_gucu': rov_carpan,
            'bileske_vektor': bileske_vektor,
            'guc_orani': guc,
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

        sonuc = self._hareket_vektor_verisi_al(rov_id=rov_id, hedef_koordinat=hedef_koordinat)
        if not sonuc:
            return

        veriler = self._bileske_vektor_hesapla(sonuc, rov_id)
        self.vektor_to_motor_sim(veriler)

    def guncelle(self, gat_kodu=None):
        """
        GNC guncelleme: Hedef varsa APF ile vektor hesaplar ve motor komutlarini uygular;
        hedef yoksa sonumler.
        """
        if not self._guncelle_kontroller():
            return

        self._guncelle_hareket_uygula(rov_id=self.rov.id)
