import math
import numpy as np
from ursina import Vec3, time
from FiratROVNet.config import Hidrodinamik, BasitKalmanFiltresi, GATLimitleri
from .mixins.formation import Formasyon


class TemelGNCHelper:
    """Tekil ROV fizikleri ve kontrol mantigi."""

    HEDEF_TOLERANSI = 0.5
    YAVASLAMA_MESAFESI = 4.0

    def __init__(self, rov_entity, filo_ref=None, gnc_ref=None):
        self.rov = rov_entity
        self.filo_ref = filo_ref
        self.gnc_ref = gnc_ref

        # Opsiyonel: APF yerine alternatif vektor kaynagi icin callable.
        self.hareket_vektor_kaynagi = None

        # Kalman filtreleri
        self.kf_x = BasitKalmanFiltresi(R=0.4, Q=0.01)
        self.kf_y = BasitKalmanFiltresi(R=0.4, Q=0.01)
        self.kf_z = BasitKalmanFiltresi(R=0.4, Q=0.01)

        self.sayac = 0
        self._koordinator = None
        self._last_sim_vektor = None
        self._last_guc = None

    def hiz_hesapla(self, mesafe: float) -> float:
        """
        Hedefe yaklasirken hizi azaltir.
        """
        if mesafe < self.YAVASLAMA_MESAFESI:
            return max(0.1, min(1.0, mesafe / self.YAVASLAMA_MESAFESI))
        return 1.0

    def yaw_ayarla(self, fark_vektoru: Vec3, ani: bool = False):
        """
        Yaw acisini hedefe dogru ayarlar.
        """
        dx, dy = fark_vektoru.x, fark_vektoru.y
        if abs(dx) < 0.01 and abs(dy) < 0.01:
            return

        hedef_yaw = math.degrees(math.atan2(dx, dy)) % 360

        if hasattr(self.rov, 'rotation_y'):
            mevcut = self.rov.rotation_y
            if ani:
                self.rov.rotation_y = hedef_yaw
            else:
                delta = (hedef_yaw - mevcut + 180) % 360 - 180
                max_step = 8.0
                delta = max(-max_step, min(max_step, delta))
                self.rov.rotation_y = (mevcut + delta) % 360

    def waypoint_izdusum(self, rov_id: int, mevcut_waypoint, sonraki_waypoint):
        """
        Waypoint dogrultusuna gore ROV konumunun izdusumunu hesaplar.
        """
        if not mevcut_waypoint or not sonraki_waypoint:
            return None

        mx = float(mevcut_waypoint[0])
        my = float(mevcut_waypoint[1])
        nx = float(sonraki_waypoint[0])
        ny = float(sonraki_waypoint[1])

        dx = nx - mx
        dy = ny - my
        uzunluk = math.hypot(dx, dy)

        if uzunluk < 1e-9:
            eksen = 'x'
        else:
            cos_theta = dx / uzunluk
            sin_theta = dy / uzunluk
            eksen = 'x' if abs(cos_theta) >= abs(sin_theta) else 'y'

        koordinator = self._koordinator_al()
        if koordinator:
            sim_pos = koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z)
        else:
            sim_pos = (self.rov.x, self.rov.z)

        if eksen == 'x':
            proj_rov = (float(sim_pos[0]), 0.0)
            proj_mevcut = (mx, 0.0)
            proj_sonraki = (nx, 0.0)
        else:
            proj_rov = (0.0, float(sim_pos[1]))
            proj_mevcut = (0.0, my)
            proj_sonraki = (0.0, ny)

        return {
            'axis': eksen,
            'proj_rov': proj_rov,
            'proj_mevcut': proj_mevcut,
            'proj_sonraki': proj_sonraki,
        }

    def _waypoint_izdusum_gecildi_mi(self, rov_id: int, mevcut_waypoint, sonraki_waypoint) -> bool:
        """
        ROV'un izdusumunun, mevcut waypoint'e kiyasla sonraki waypoint'e daha yakin olup olmadigini dondurur.
        """
        if not mevcut_waypoint or not sonraki_waypoint:
            return False

        izdusum = self.waypoint_izdusum(rov_id, mevcut_waypoint, sonraki_waypoint)
        if not izdusum:
            return False

        if izdusum['axis'] == 'x':
            rov_fark = abs(izdusum['proj_rov'][0] - izdusum['proj_sonraki'][0])
            mevcut_fark = abs(izdusum['proj_mevcut'][0] - izdusum['proj_sonraki'][0])
        else:
            rov_fark = abs(izdusum['proj_rov'][1] - izdusum['proj_sonraki'][1])
            mevcut_fark = abs(izdusum['proj_mevcut'][1] - izdusum['proj_sonraki'][1])

        return rov_fark < mevcut_fark

    def _kalman_vektor_filtrele(self, v):
        """Gelen ham vektoru Kalman filtresinden gecirir ve temizlenmis vektoru doner."""
        if v is None:
            return Vec3(0, 0, 0)

        yeni_x = self.kf_x.guncelle(v.x)
        yeni_y = self.kf_y.guncelle(v.y)
        yeni_z = self.kf_z.guncelle(v.z)

        return Vec3(yeni_x, yeni_y, yeni_z)

    def batma_orani_hesapla(self):
        """ROV govdesinin suyun icindeki yuzdesini doner (0.0 - 1.0)."""
        su_yuzeyi = 0.0
        rov_yukseklik = self.rov.scale.y * 500
        rov_y = self.rov.y
        en_ust_nokta = rov_y + (rov_yukseklik / 2)
        en_alt_nokta = rov_y - (rov_yukseklik / 2)

        if en_alt_nokta >= su_yuzeyi:
            return 0.0
        if en_ust_nokta <= su_yuzeyi:
            return 1.0

        suyun_altindaki_kisim = su_yuzeyi - en_alt_nokta
        oran = suyun_altindaki_kisim / rov_yukseklik
        return max(0.0, min(1.0, oran))

    def fizik_uygula(self, hedef_yon_ursina: Vec3, guc_orani: float, dt: float, uygula: bool = True):
        if not uygula:
            return hedef_yon_ursina * guc_orani

        f_thrust = hedef_yon_ursina * guc_orani * Hidrodinamik.MAX_ITME_KUVVETI
        mevcut_hiz = self.rov.velocity
        hiz_buyuklugu = mevcut_hiz.length()

        f_drag = Vec3(0, 0, 0)
        if hiz_buyuklugu > 0.001:
            drag_quadratic = 0.5 * Hidrodinamik.SU_YOGUNLUGU * Hidrodinamik.DRAG_KATSAYISI_CD * \
                            Hidrodinamik.ON_YUZEY_ALANI * (hiz_buyuklugu ** 2)
            drag_linear = hiz_buyuklugu * 10.0
            f_drag = -mevcut_hiz.normalized() * (drag_quadratic + drag_linear)

        batma = self.batma_orani_hesapla()
        self.rov.gnc.batma_orani = batma
        su_icindeki_hacim = Hidrodinamik.HACIM * batma
        f_yercekimi = Vec3(0, 0, -Hidrodinamik.KUTLE * Hidrodinamik.YER_CEKIMI)
        f_kaldirma = Vec3(0, 0, su_icindeki_hacim * Hidrodinamik.SU_YOGUNLUGU * Hidrodinamik.YER_CEKIMI)

        f_net = f_thrust + f_yercekimi + f_kaldirma + f_drag
        ivme = f_net / Hidrodinamik.KUTLE
        yeni_hiz = (ivme * dt)

        return yeni_hiz

    def vektor_to_motor_sim(self, v_sim_dir: Vec3, guc_orani: float):
        """
        Fizikten gelen vektoru ROV'un govdesine (pozisyon ve rotasyon) uygular.
        """
        guc_orani = max(0.0, min(1.0, guc_orani))
        dt = time.dt
        if dt > 0.08:
            dt = 0.08
        vim_dir2 = v_sim_dir.normalized() if v_sim_dir.length() > 0.001 else Vec3(0, 0, 0)

        hesaplanan_hiz = self.fizik_uygula(vim_dir2, guc_orani, dt, uygula=True)
        self.rov.velocity = self._kalman_vektor_filtrele(hesaplanan_hiz)

        if guc_orani > 0.01:
            self.yaw_ayarla(self.rov.velocity, ani=False)

        ursina_rov_velocity = Vec3(self.rov.velocity.x, self.rov.velocity.z, self.rov.velocity.y)

        GORSEL_HIZ_CARPANI = 50.0
        self.rov.position += ursina_rov_velocity * dt * GORSEL_HIZ_CARPANI

        if guc_orani > 0.01 and hasattr(self.rov, 'velocity'):
            if hasattr(self, 'filo_ref') and self.filo_ref.helper:
                self.filo_ref.helper.vektor(
                    rov_id_ilk=self.rov.id,
                    vektor=self.rov.velocity,
                    renk='m',
                    ciz=True,
                )
        else:
            self.rov.hedef = None

        return hesaplanan_hiz

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
            from FiratROVNet.gnc import Koordinator
            self._koordinator = Koordinator
        return self._koordinator

    def _rov_pozisyon_sim(self):
        """
        ROV'un mevcut pozisyonunu Sim koordinat sistemine cevirir.
        """
        if self.rov is None:
            return None

        Koordinator = self._koordinator_al()
        current_sim_pos = Vec3(*Koordinator.ursina_to_sim(self.rov.x, self.rov.y, self.rov.z))
        return current_sim_pos

    def _engel_rov_kaçınma_vektörü(self, engel_listesi, rov_listesi):
        """
        Engel ve ROV repulsif vektorlerini birlestirir (hedef yokken kacinma).
        """
        toplam_x, toplam_y = 0.0, 0.0
        for item in (engel_listesi or []) + (rov_listesi or []):
            bv = item.get('birim_vektor')
            m = max(float(item.get('mesafe', 0.0)), 0.5)
            if bv and len(bv) >= 2:
                w = 1.0 / m
                toplam_x += float(bv[0]) * w
                toplam_y += float(bv[1]) * w
        n = math.sqrt(toplam_x * toplam_x + toplam_y * toplam_y)
        if n < 1e-9:
            return None
        return (toplam_x / n, toplam_y / n)

    def _guc_orani_hesapla(self, mesafe: float, limit=(np.sqrt(3) * 400)):
        if mesafe < 2:
            return 0.0

        if mesafe < GATLimitleri.CARPISMA:
            guc = mesafe / limit
            guc = np.log(guc * 10 + 1) / np.log(11)
        else:
            guc = 1.0

        return guc

    def hedef_sifirla(self):
        self.rov.hedef = None

    def _formasyon_dinamik_guncelle(self, rov_id: int):
        """
        Eger aktif bir formasyon varsa, takipcilerin hedeflerini
        liderin o anki konumuna ve yonune gore gunceller.
        """
        aktif = self.filo_ref.aktif_formasyon.get(self.rov.group_id, False)
        if not aktif:
            return

        grup_rovs = self.filo_ref.g_rovs.get(self.rov.group_id) if hasattr(self.filo_ref, 'g_rovs') else None
        lider = next((r for r in (grup_rovs or []) if r and r.role == 1), None)
        if not lider:
            return

        if self.rov.gnc.mod == 0:
            return

        if self.rov.role == 1:
            f_obj = Formasyon(self.filo_ref)
            lider_pos_sim = (lider.x, lider.z, lider.y)
            yeni_pozisyonlar = f_obj.pozisyonlar(
                aktif['id'],
                aktif['aralik'],
                is_3d=aktif['is_3d'],
                lider_koordinat=lider_pos_sim,
                g_id=self.rov.group_id,
            )
            if not hasattr(self.filo_ref, 'yeni_pozisyonlar') or not isinstance(self.filo_ref.yeni_pozisyonlar, dict):
                self.filo_ref.yeni_pozisyonlar = {}
            self.filo_ref.yeni_pozisyonlar[self.rov.group_id] = yeni_pozisyonlar
            return

        yeni_pozisyonlar = self.filo_ref.yeni_pozisyonlar
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
            self.filo_ref.hedef((hedef[0], hedef[1], hedef[2]), rov_id=self.rov.id)

    def _guncelle_waypoint_takip(self, rov_id: int):
        """
        A* cikisi olan (x, z) noktalarini takip eder.
        """
        # Rota listesini al
        nokta_listesi = getattr(self.filo_ref, '_git_nokta_listesi', {}).get(rov_id)
        if not nokta_listesi:
            return None, False

        mevcut_indeks = getattr(self.filo_ref, '_git_mevcut_nokta_indeksi', {}).get(rov_id, 0)

        # Rota bitti mi?
        if mevcut_indeks >= len(nokta_listesi):
            self.filo_ref._git_nokta_listesi.pop(rov_id, None)
            self.filo_ref._git_mevcut_nokta_indeksi.pop(rov_id, None)
            if hasattr(self.filo_ref, '_git_hedef_derinligi'):
                self.filo_ref._git_hedef_derinligi.pop(rov_id, None)
            return None, True

        # Mevcut waypoint
        wp = nokta_listesi[mevcut_indeks]
        current_gps = self.filo_ref.get(rov_id, "gps")

        target_x = float(wp[0])
        target_y = float(wp[1])
        
        # Hedef derinliği kullan (varsa), yoksa mevcut derinliği koru
        target_depth = None
        if hasattr(self.filo_ref, '_git_hedef_derinligi'):
            target_depth = self.filo_ref._git_hedef_derinligi.get(rov_id)
        
        if target_depth is not None:
            target_z = target_depth
        else:
            target_z = current_gps[2]

        waypoint_hedef = Vec3(target_x, target_y, target_z)

        dist_x = waypoint_hedef.x - current_gps[0]
        dist_y = waypoint_hedef.y - current_gps[1]
        mesafe_yatay = math.sqrt(dist_x ** 2 + dist_y ** 2)

        if mesafe_yatay < GATLimitleri.CARPISMA:
            self.filo_ref._git_mevcut_nokta_indeksi[rov_id] = mevcut_indeks + 1
            if mevcut_indeks + 1 < len(nokta_listesi):
                next_wp = nokta_listesi[mevcut_indeks + 1]
                self.filo_ref.hedef((next_wp[0], next_wp[1], target_z), rov_id=rov_id, ciz=False)
            return waypoint_hedef, False

        return waypoint_hedef, False

    def _hareket_vektor_verisi_al(self, rov_id: int, hedef_koordinat):
        """
        Hareket vektoru verisini ureten kaynak.
        Varsayilan APF; harici kaynaklar icin hareket_vektor_kaynagi atanabilir.
        """
        if callable(self.hareket_vektor_kaynagi):
            return self.hareket_vektor_kaynagi(rov_id=rov_id, hedef_koordinat=hedef_koordinat)

        return self.filo_ref.helper.apf(
            rov_id=rov_id,
            hedef=(hedef_koordinat is not None),
            engel=True,
            rov=True,
        )

    def _engel_vektoru_isle(self, sonuc, rov_id: int, guc0: float):
        bileske_vektor = Vec3(0, 0, 0)
        max_engel_etkisi = 0.0
        guc1 = 0.0

        for e_info in sonuc.get('engeller', []):
            bv = Vec3(*e_info.get('birim_vektor', [0, 0, 0]))
            mesafe = float(e_info.get('mesafe', 0.0))

            etki = 1.0 - (mesafe / GATLimitleri.ENGEL)
            #print(f"ROV-{self.rov.id}: Mesafe = {mesafe}, Etki = {etki}")

            max_engel_etkisi = max(max_engel_etkisi, etki)
            guc1 = max(1 - self._guc_orani_hesapla(mesafe, GATLimitleri.ENGEL), guc0)

            

            if etki > 0.2 and self.filo_ref.get(self.rov.id, 'rol') == 1 and e_info.get('yon') != 'asagi_lidar':
                print(self.rov.id,etki,max_engel_etkisi)
                self.filo_ref.formasyon_sec(dinamik=True, tekrar=30, g_id=self.rov.group_id)

            bv_yatay = Vec3(bv.x, bv.y, bv.z)
            bileske_vektor += bv_yatay * etki * 0.4

        return bileske_vektor, max_engel_etkisi, guc1

    def _rov_vektoru_isle(self, sonuc, guc0: float):
        bileske_vektor = Vec3(0, 0, 0)
        max_rov_etkisi = 0.0
        guc2 = 0.0

        for r_info in sonuc.get('rovs', []):
            bv = Vec3(*r_info.get('birim_vektor', [0, 0, 0]))
            mesafe = float(r_info.get('mesafe', 0.0))

            etki = 1 - (mesafe / (GATLimitleri.CARPISMA))
            max_rov_etkisi = max(max_rov_etkisi, etki)

            bv_yatay = Vec3(bv.x, bv.y, bv.z)
            bileske_vektor += bv_yatay * etki * 0.3
            guc2 = max(1 - self._guc_orani_hesapla(mesafe, GATLimitleri.CARPISMA), guc0)

        return bileske_vektor, max_rov_etkisi, guc2

    def _hedef_vektoru_isle(self, sonuc, max_engel_etkisi: float, max_rov_etkisi: float):
        h_info = sonuc.get('hedef') or {}
        h_mesafe = float(h_info.get('mesafe', 0.0))
        h_birim = Vec3(*h_info.get('birim_vektor', [0, 0, 0]))

        guc0 = self._guc_orani_hesapla(h_mesafe)
        hedef_agirligi = (1.0 - max_engel_etkisi) * 0.15 + (1.0 - max_rov_etkisi) * 0.15
        hedef_vektor = h_birim * hedef_agirligi

        return hedef_vektor, guc0

    def _bileske_vektor_hesapla(self, sonuc, rov_id: int):
        hedef_vektor, guc0 = self._hedef_vektoru_isle(sonuc, 0.0, 0.0)

        engel_vektor, max_engel_etkisi, guc1 = self._engel_vektoru_isle(sonuc, rov_id, guc0)
        rov_vektor, max_rov_etkisi, guc2 = self._rov_vektoru_isle(sonuc, guc0)

        # Hedef agirligini yeni etkilerle tekrar hesapla
        hedef_vektor, guc0 = self._hedef_vektoru_isle(sonuc, max_engel_etkisi, max_rov_etkisi)

        bileske_vektor = engel_vektor + rov_vektor + hedef_vektor
        guc = max(guc0, guc1, guc2)

        return bileske_vektor, guc

    def _guncelle_hareket_uygula(self, rov_id: int):
        """
        APF kullanarak ROV hareketini yonetir.
        Waypoint takip mekanizmasi ile git_path() cagrilarini destekler.
        """
        waypoint_hedef, _ = self._guncelle_waypoint_takip(rov_id)
        if waypoint_hedef:
            self.rov.hedef = waypoint_hedef

        self._formasyon_dinamik_guncelle(rov_id)
        hedef_koordinat = self.filo_ref.hedef(rov_id=rov_id)

        sonuc = self._hareket_vektor_verisi_al(rov_id=rov_id, hedef_koordinat=hedef_koordinat)
        if not sonuc:
            return

        bileske_vektor, guc = self._bileske_vektor_hesapla(sonuc, rov_id)
        final_yon = bileske_vektor.normalized() if bileske_vektor.length() > 0.001 else Vec3(0, 0, 0)
        self.vektor_to_motor_sim(final_yon, guc)

    def guncelle(self, gat_kodu=None):
        """
        GNC guncelleme: Hedef varsa APF ile vektor hesaplar ve motor komutlarini uygular;
        hedef yoksa sonumler.
        """
        if not self._guncelle_kontroller():
            return

        self._guncelle_hareket_uygula(rov_id=self.rov.id)
