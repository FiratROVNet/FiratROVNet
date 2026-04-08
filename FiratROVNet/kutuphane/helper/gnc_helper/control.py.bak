import math
import numpy as np
from ursina import Vec3, time
from FiratROVNet.config import Hidrodinamik, GATLimitleri
from .mixins.formation import Formasyon
from panda3d.core import Vec3 as P3Vec



class TemelGNCHelper:
    """Tekil ROV fizikleri ve kontrol mantigi."""

    def __init__(self, rov_entity, filo_ref=None, gnc_ref=None):
        self.rov = rov_entity
        self.filo_ref = filo_ref
        self.gnc_ref = gnc_ref

        # Opsiyonel: APF yerine alternatif vektor kaynagi icin callable.
        self.hareket_vektor_kaynagi = None
        self._koordinator = None


    def yaw_uygula(self,hedef_vektor:Vec3=Vec3(0,0,0),guc:float=0.1):
            V_rov = self.rov.gnc.r_bv
            V_rov = Vec3(V_rov.x, 0,V_rov.z)
            hedef_vektor = Vec3(hedef_vektor.x, 0,hedef_vektor.z)
          

            if V_rov.length()*hedef_vektor.length() == 0:
                return


            scaler_carpim=V_rov.dot(hedef_vektor)
            radyan=math.acos(scaler_carpim/(V_rov.length()*hedef_vektor.length()))
            aci=math.degrees(radyan)
            oran = aci / 360
            filo = self.filo_ref
            if filo is not None:
                filo.yaw(self.rov,oran*guc*0.1)



    def fizik_uygula(self):
            physics_node = getattr(self.rov, 'physics_node', None)
            if physics_node is None: return

            # A. LİNEER SÖNÜMLEME
            mevcut_hiz = physics_node.getLinearVelocity()
            hiz_buyuklugu = mevcut_hiz.length()
            f_drag = Vec3(0, 0, 0)
            if hiz_buyuklugu > 0.001:
                drag_quadratic = 0.5 * Hidrodinamik.SU_YOGUNLUGU * Hidrodinamik.DRAG_KATSAYISI_CD * Hidrodinamik.ON_YUZEY_ALANI * (hiz_buyuklugu ** 2)
                drag_linear = hiz_buyuklugu * 2.0 
                yon_vec = Vec3(-mevcut_hiz.x, -mevcut_hiz.y, -mevcut_hiz.z).normalized()
                f_drag = yon_vec * (drag_quadratic + drag_linear)

            # B. AÇISAL SÖNÜMLEME
            mevcut_acisal_hiz = physics_node.getAngularVelocity()
            acisal_hiz_buyuklugu = mevcut_acisal_hiz.length()
            t_drag = Vec3(0, 0, 0)
            if acisal_hiz_buyuklugu > 0.001:
                a_drag_quad = 0.5 * Hidrodinamik.SU_YOGUNLUGU * Hidrodinamik.DRAG_KATSAYISI_CD * (acisal_hiz_buyuklugu ** 2)
                a_drag_lin = acisal_hiz_buyuklugu * 5
                a_yon_vec = Vec3(-mevcut_acisal_hiz.x, -mevcut_acisal_hiz.y, -mevcut_acisal_hiz.z).normalized()
                t_drag = a_yon_vec * (a_drag_quad + a_drag_lin)

            # C. YERÇEKİMİ VE KALDIRMA KUVVETİ (Merkezi İtme)
            batma = getattr(self.rov.gnc, 'batma_orani', 1.0) 
            su_icindeki_hacim = Hidrodinamik.HACIM * batma
            
            f_yercekimi = Vec3(0, -Hidrodinamik.KUTLE * Hidrodinamik.YER_CEKIMI, 0)
            f_kaldirma = Vec3(0, su_icindeki_hacim * Hidrodinamik.SU_YOGUNLUGU * Hidrodinamik.YER_CEKIMI, 0)
            f_net_env = f_yercekimi + f_kaldirma + f_drag

            # 🔹 D. YENİ: HACİYATMAZ TORKU (Restoring Moment)
            # ROV'un o anki "Yukarı" ekseni ile Dünyanın "Yukarı" (0,1,0) eksenini çarpıyoruz.
            # Bu çarpım, ROV'u her zaman düzeltmeye zorlayan bir tork üretir.
            rov_up = self.rov.up # ROV'un kendi yerel yukarısı
            world_up = Vec3(0, 1, 0) # Dünyanın yukarısı
            
            # Hacıyatmaz Katsayısı: Bu değer ne kadar büyükse araç o kadar zor devrilir
            haciyatmaz_katsayisi = 150.0 * batma # Sadece sudayken çalışır
            t_haciyatmaz = rov_up.cross(world_up) * haciyatmaz_katsayisi

            # E. FİZİK MOTORUNA AKTARIM
            physics_node.applyCentralForce(P3Vec(f_net_env.x, f_net_env.y, f_net_env.z))
            
            # Torkları birleştirip uygula (Sürtünme freni + Hacıyatmaz düzeltmesi)
            toplam_tork = t_drag + t_haciyatmaz
            physics_node.applyTorque(P3Vec(toplam_tork.x, toplam_tork.y, toplam_tork.z))

    def vektor_to_motor_sim(self, v_sim_dir: Vec3, guc_orani: float):
            """
            APF'den gelen 3B hareket vektörünü BlueROV2 benzeri 6 motor
            (4 yatay, 2 dikey) için güç komutlarına dönüştürür.
            """
            # Güç oranını sınırla
            guc_orani = max(0.0, min(1.0, guc_orani))
            v_sim_dir = Vec3(v_sim_dir.x, v_sim_dir.z, v_sim_dir.y)
            

            if self.rov is None:
                return
            filo = self.filo_ref
            if filo is None:
                return
            
            # ============================================================
            # 2. ÖNCE ÇEVRESEL FİZİĞİ UYGULA (Suyun ve Dünyanın Etkisi)
            # ============================================================
            self.fizik_uygula()  # Veya 'self.fizik_uygula(self.rov)' şeklinde tanımlıysa ona göre çağırın

            # ============================================================
            # 3. MOTOR GÜÇLERİNİ HESAPLA
            # ============================================================

            

            



            gucler = filo.tum_motorlarin_guclerini_hesapla(self.rov.id, v_sim_dir, guc_orani)
            tork_gucleri, fark = filo.tork_gucleri_hesapla(self.rov, v_sim_dir, guc_orani)
            g=np.array(tork_gucleri)*0.2+np.array(gucler)*0.8
            g=g.tolist()


            # ============================================================
            # 4. MOTORLARI ÇALIŞTIR VE FİZİKSEL İTKİYİ UYGULA
            # ============================================================
            
            filo.motorlari_calistir(self.rov.id, g)


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

    def _guc_orani_hesapla(self, mesafe: float, limit=(np.sqrt(3) * 400)):
        if mesafe < 2:
            return 0.0
        if mesafe < GATLimitleri.CARPISMA:
            guc = mesafe / limit
            guc = np.log(guc * 10 + 1) / np.log(11)
        else:
            guc = 1.0
        return guc

    def _guc_orani_hesapla_batch(self, mesafeler: np.ndarray, limit: float):
        """Toplu mesafe icin guc orani (NumPy; dongu yok)."""
        if mesafeler.size == 0:
            return np.array([], dtype=np.float64)
        out = np.zeros_like(mesafeler, dtype=np.float64)
        mask_ge2 = mesafeler >= 2
        mask_c = mesafeler < GATLimitleri.CARPISMA
        out[~mask_ge2] = 0.0
        valid = mask_ge2 & mask_c
        if np.any(valid):
            guc = np.clip(mesafeler[valid] / limit * 10 + 1, 1e-9, None)
            out[valid] = np.log(guc) / np.log(11)
        out[mask_ge2 & ~mask_c] = 1.0
        return out

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
            filo.hedef((hedef[0], hedef[1], hedef[2]), rov_id=self.rov.id)

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

    def _engel_vektoru_isle(self, sonuc, rov_id: int, guc0: float):
        engeller = sonuc.get('engeller') or []
        if not engeller:
            return Vec3(0, 0, 0), 0.0, guc0
        # NumPy ile tek seferde: bilesenler (N,3), mesafe (N,)
        n = len(engeller)
        vecs = np.zeros((n, 3), dtype=np.float64)
        mesafeler = np.zeros(n, dtype=np.float64)
        for i, e_info in enumerate(engeller):
            bv = e_info.get('birim_vektor', [0, 0, 0])
            vecs[i] = (bv[0], bv[1], bv[2]) if len(bv) >= 3 else (bv[0], bv[1], 0.0) if len(bv) >= 2 else (0, 0, 0)
            mesafeler[i] = float(e_info.get('mesafe', 0.0))
        engel_limit = float(GATLimitleri.ENGEL)
        etki = np.maximum(0.0, 1.0 - mesafeler / engel_limit)
        agirlikli = vecs * (etki * 0.4)[:, np.newaxis]
        toplam = agirlikli.sum(axis=0)
        max_engel_etkisi = float(np.max(etki))
        guc_vals = 1.0 - self._guc_orani_hesapla_batch(mesafeler, engel_limit)
        guc1 = float(max(guc0, np.max(guc_vals)) if guc_vals.size else guc0)
        return Vec3(toplam[0], toplam[1], toplam[2]), max_engel_etkisi, guc1

    def _rov_vektoru_isle(self, sonuc, guc0: float):
        rovs = sonuc.get('rovs') or []
        if not rovs:
            return Vec3(0, 0, 0), 0.0, guc0
        n = len(rovs)
        vecs = np.zeros((n, 3), dtype=np.float64)
        mesafeler = np.zeros(n, dtype=np.float64)
        for i, r_info in enumerate(rovs):
            bv = r_info.get('birim_vektor', [0, 0, 0])
            vecs[i] = (bv[0], bv[1], bv[2]) if len(bv) >= 3 else (bv[0], bv[1], 0.0) if len(bv) >= 2 else (0, 0, 0)
            mesafeler[i] = float(r_info.get('mesafe', 0.0))
        carpisma = float(GATLimitleri.CARPISMA)
        etki = np.maximum(0.0, 1.0 - mesafeler / carpisma)
        agirlikli = vecs * (etki * 0.35)[:, np.newaxis]
        toplam = agirlikli.sum(axis=0)
        max_rov_etkisi = float(np.max(etki))
        guc_vals = 1.0 - self._guc_orani_hesapla_batch(mesafeler, carpisma)
        guc2 = float(max(guc0, np.max(guc_vals)) if guc_vals.size else guc0)
        return Vec3(toplam[0], toplam[1], toplam[2]), max_rov_etkisi, guc2

    def _hedef_vektoru_isle(self, sonuc, max_engel_etkisi: float, max_rov_etkisi: float):
        h_info = sonuc.get('hedef') or {}
        h_mesafe = float(h_info.get('mesafe', 0.0))
        h_birim = Vec3(*h_info.get('birim_vektor', [0, 0, 0]))

        guc0 = self._guc_orani_hesapla(h_mesafe)
        hedef_agirligi = (1.0 - max_engel_etkisi) * 0.125 + (1.0 - max_rov_etkisi) * 0.125
        hedef_vektor = h_birim * hedef_agirligi

        return hedef_vektor, guc0

    def _bileske_vektor_hesapla(self, sonuc, rov_id: int):
        hedef_vektor, guc0 = self._hedef_vektoru_isle(sonuc, 0.0, 0.0)

        engel_vektor, max_engel_etkisi, guc1 = self._engel_vektoru_isle(sonuc, rov_id, guc0)
        rov_vektor, max_rov_etkisi, guc2 = self._rov_vektoru_isle(sonuc, guc0)

        # Hedef agirligini yeni etkilerle tekrar hesapla
        hedef_vektor, guc0 = self._hedef_vektoru_isle(sonuc, max_engel_etkisi, max_rov_etkisi)

        if self.rov.role == 1 and False:
            print(f"Hedef Vektor: {hedef_vektor}, Guc: {guc0}")
            print(f"Engel Vektor: {engel_vektor}, Guc: {guc1}")
            print(f"ROV Vektor: {rov_vektor}, Guc: {guc2}")

        bileske_vektor = engel_vektor + rov_vektor + hedef_vektor
        guc = max(guc0, guc1, guc2)

        return bileske_vektor, guc

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
