import numpy as np
from FiratROVNet.config import HareketAyarlari


class TrainingMixin:
    """RL ve GAT egitim verisi uretimi."""

    def get_100_samples(self, hull_output=None, sample_count=100):
        """
        yeni_hull cikisindaki noktalari alir ve cevre uzunlugu uzerinden
        sabit sayida (sample_count) ornek nokta dondurur.
        """
        if hull_output is None:
            hull_output = self.filo.yeni_hull(self.filo.ada_cevre())

        points = hull_output.get('points')
        if points is None or len(points) < 2:
            print("⚠️ [SAMPLED] Ornekleme icin yetersiz nokta!")
            return None

        if not np.allclose(points[0], points[-1]):
            points = np.vstack([points, points[0]])

        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt((diffs ** 2).sum(axis=1))
        cumulative_dist = np.concatenate(([0], np.cumsum(segment_lengths)))
        total_perimeter = cumulative_dist[-1]

        if total_perimeter == 0:
            return np.tile(points[0], (sample_count, 1))

        target_dists = np.linspace(0, total_perimeter, sample_count, endpoint=False)
        new_x = np.interp(target_dists, cumulative_dist, points[:, 0])
        new_y = np.interp(target_dists, cumulative_dist, points[:, 1])
        sampled_points = np.column_stack((new_x, new_y))
        return sampled_points

    def uret_rl_egitim_verisi(self):
        """
        RL egitimi icin hizli senaryo uretir ve sabit boyutlu verileri dondurur.
        Yeni senaryo sistemi: Ortam bir kez baslatilir, ROV/ada sayilari dinamik olarak ayarlanir.
        """
        from FiratROVNet import senaryo
        import random

        try:
            n_rov_secenekleri = [4, 6, 8]
            secilen_n = random.choice(n_rov_secenekleri)
            n_engels = random.randint(12, 22)
            n_adalar = random.randint(2, 5)

            senaryo.uret(n_rovs=secilen_n, n_engels=n_engels, n_adalar=n_adalar, havuz_genisligi=200, verbose=False)
            aktif_filo = senaryo.filo
            if not aktif_filo:
                return None

            lider_id = None
            for i in range(secilen_n):
                if i < len(senaryo.filo.sistemler) and senaryo.filo.sistemler[i] is not None:
                    rol = senaryo.get(i, "rol")
                    if rol == 1:
                        lider_id = i
                        break

            if lider_id is None:
                lider_id = 0

            lider_gps = senaryo.get(lider_id, "gps") if lider_id < len(senaryo.filo.sistemler) else None
            lider_yaw = senaryo.get(lider_id, "yaw") if lider_id < len(senaryo.filo.sistemler) else None

            if lider_gps is None:
                lider_gps = np.array([400.0, 400.0, 400.0])
            if lider_yaw is None:
                lider_yaw = 0.0

            rov_filo_gps = []
            for i in range(8):
                if i < secilen_n:
                    try:
                        from FiratROVNet.senaryo import _get_instance
                        senaryo_instance = _get_instance()
                        if senaryo_instance and senaryo_instance.aktif and i < len(senaryo_instance.ortam.rovs):
                            pos = senaryo_instance.get(i, "gps")
                            rov_filo_gps.append(pos if pos is not None else [400.0, 400.0, 400.0])
                        else:
                            rov_filo_gps.append([400.0, 400.0, 400.0])
                    except Exception:
                        rov_filo_gps.append([400.0, 400.0, 400.0])
                else:
                    rov_filo_gps.append([400.0, 400.0, 400.0])
            rov_filo_gps = np.array(rov_filo_gps)

            hull_merkez = np.array([400.0, 400.0])
            hull_noktalar = np.full((100, 2), 400.0)

            ada_cevreleri = aktif_filo.ada_cevre(offset=15.0, sessiz=True)
            hull_dict = aktif_filo.yeni_hull(ada_cevreleri)
            if hull_dict and hull_dict.get('points') is not None:
                center = hull_dict.get('center')
                if center:
                    hull_merkez = np.array([center[0], center[1]])

                samples = self.get_100_samples(hull_dict, 100)
                if samples is not None:
                    hull_noktalar = samples

            out = aktif_filo.helper._formasyon_sec_impl(
                margin=HareketAyarlari.FORMASYON_MESAFESI,
                is_3d=False,
                offset=HareketAyarlari.FORMASYON_OFFSET,
                sessiz=True,
            )

            if out is None:
                return None

            return {
                "output": out,
                "n_rovs": secilen_n,
                "lider_pozisyon": lider_gps,
                "lider_yaw": lider_yaw,
                "rov_filo_gps": rov_filo_gps,
                "hull_merkez": hull_merkez,
                "hull_noktalar": hull_noktalar,
            }
        except Exception as e:
            print(f"❌ [RL_DATA] Veri uretimi sirasinda hata: {e}")
            import traceback
            traceback.print_exc()
            return None

    def lider_sec_veri_uret(self, asil_hedef=None):
        """
        RL egitimi icin lider secim verisi uretir.
        Matematiksel liderlik formulunu 'Label' olarak kullanir.
        Yeni senaryo sistemi: Ortam bir kez baslatilir, ROV/ada sayilari dinamik olarak ayarlanir.
        """
        from FiratROVNet import senaryo
        import random
        from ursina import Vec3

        try:
            n_rov_list = [4, 6, 8]
            secilen_n = random.choice(n_rov_list)
            n_adalar = random.randint(2, 5)
            senaryo.uret(n_rovs=secilen_n, n_engels=random.randint(10, 20), n_adalar=n_adalar, havuz_genisligi=200, verbose=False)

            if not senaryo.filo:
                return None

            hedef = asil_hedef if asil_hedef else Vec3(random.randint(-100, 100), random.randint(-100, 100), 0)

            rov_data = []
            rov_list_for_calc = []
            for i in range(8):
                if i < secilen_n:
                    try:
                        from FiratROVNet.senaryo import _get_instance
                        senaryo_instance = _get_instance()
                        if senaryo_instance and senaryo_instance.aktif and i < len(senaryo_instance.ortam.rovs):
                            bat = senaryo_instance.get(i, "batarya")
                            if bat is None:
                                bat = 1.0
                            bat = bat * 100.0
                            gps = senaryo_instance.get(i, "gps")
                            if gps is None:
                                gps = np.array([400.0, 400.0, 400.0])
                            rov_data.append([bat, gps[0], gps[1], gps[2]])
                            rov_list_for_calc.append({'id': i, 'batarya': bat, 'konum': gps})
                        else:
                            rov_data.append([0.0, 400.0, 400.0, 400.0])
                    except Exception:
                        rov_data.append([0.0, 400.0, 400.0, 400.0])
                else:
                    rov_data.append([0.0, 400.0, 400.0, 400.0])

            from RL_PPO.lider_sec.lider_sec import LiderSecimModulu
            lider_modulu = LiderSecimModulu()
            dogru_lider_id, dogru_skor = lider_modulu.lideri_belirle_ve_yazdir(
                rov_list_for_calc, [hedef.x, hedef.y, hedef.z]
            )

            state = np.array(
                [hedef.x, hedef.y, hedef.z] + list(np.array(rov_data).flatten()),
                dtype=np.float32,
            )

            return {
                "state": state,
                "target_id": dogru_lider_id,
                "target_skor": dogru_skor,
            }
        except Exception as e:
            print(f"❌ Lider veri uretim hatasi: {e}")
            import traceback
            traceback.print_exc()
            return None

    def gat_veri_uret(self):
        """
        GAT egitimi icin senaryo verisi uretir.
        Senaryo.py kullanarak rastgele ROV ve ada ile ortam olusturur.
        """
        from FiratROVNet import senaryo
        from FiratROVNet.senaryo import _get_instance
        import random

        try:
            senaryo_tipi = random.choice([
                'normal',
                'yakin',
                'dagnik',
                'tek_kume',
                'iki_kume',
            ])

            n_rov_secenekleri = [4, 5, 6, 7, 8, 9, 10, 11, 12]
            secilen_n = random.choice(n_rov_secenekleri)
            n_engels = random.randint(10, 20)
            n_adalar = random.randint(2, 6)

            senaryo_instance = _get_instance()
            if senaryo_instance:
                senaryo_instance._cache_n_rovs = None
                senaryo_instance._cache_n_engels = None
                senaryo_instance._cache_n_adalar = None
                senaryo_instance._cache_havuz_genisligi = None

            havuz_genisligi = 200 + random.uniform(-5, 5)

            senaryo.uret(n_rovs=secilen_n, n_engels=n_engels, n_adalar=n_adalar, havuz_genisligi=havuz_genisligi, verbose=False)

            senaryo_instance = _get_instance()
            if not senaryo_instance or not senaryo_instance.aktif:
                return None

            aktif_filo = senaryo_instance.filo
            if not aktif_filo:
                return None

            if hasattr(senaryo_instance.ortam, 'rovs') and senaryo_instance.ortam.rovs:
                from ursina import Vec3
                rovs = [r for r in senaryo_instance.ortam.rovs if r is not None]
                n_rovs_actual = len(rovs)

                if senaryo_tipi == 'yakin':
                    merkez_x = random.uniform(-50, 50)
                    merkez_z = random.uniform(-50, 50)
                    for rov in rovs:
                        offset_x = random.uniform(-4, 4)
                        offset_z = random.uniform(-4, 4)
                        if hasattr(rov, 'position'):
                            rov.position = Vec3(merkez_x + offset_x, -5, merkez_z + offset_z)

                elif senaryo_tipi == 'dagnik':
                    for rov in rovs:
                        pos_x = random.uniform(-80, 80)
                        pos_z = random.uniform(-80, 80)
                        if hasattr(rov, 'position'):
                            rov.position = Vec3(pos_x, -5, pos_z)

                elif senaryo_tipi == 'tek_kume':
                    merkez_x = random.uniform(-40, 40)
                    merkez_z = random.uniform(-40, 40)
                    for rov in rovs:
                        offset_x = random.uniform(-15, 15)
                        offset_z = random.uniform(-15, 15)
                        if hasattr(rov, 'position'):
                            rov.position = Vec3(merkez_x + offset_x, -5, merkez_z + offset_z)

                elif senaryo_tipi == 'iki_kume':
                    merkez1_x, merkez1_z = random.uniform(-60, -20), random.uniform(-60, 60)
                    merkez2_x, merkez2_z = random.uniform(20, 60), random.uniform(-60, 60)
                    for i, rov in enumerate(rovs):
                        if i < n_rovs_actual // 2:
                            merkez_x, merkez_z = merkez1_x, merkez1_z
                        else:
                            merkez_x, merkez_z = merkez2_x, merkez2_z
                        offset_x = random.uniform(-10, 10)
                        offset_z = random.uniform(-10, 10)
                        if hasattr(rov, 'position'):
                            rov.position = Vec3(merkez_x + offset_x, -5, merkez_z + offset_z)

            for _ in range(10):
                senaryo.guncelle(0.016)

            class SenaryoWrapper:
                def __init__(self, instance):
                    self.instance = instance
                    self.filo = instance.filo
                    self.ortam = instance.ortam
                    if hasattr(self.filo, 'helper'):
                        self.filo.helper._sessiz_mod = True

                def get(self, rov_id, veri_tipi):
                    if hasattr(self.instance, 'ortam') and hasattr(self.instance.ortam, 'rovs'):
                        n_rovs = len([r for r in self.instance.ortam.rovs if r is not None])
                        if rov_id >= n_rovs:
                            return None
                    if hasattr(self.filo, 'helper'):
                        self.filo.helper._sessiz_mod = True
                    return self.instance.get(rov_id, veri_tipi)

            return {
                'senaryo': SenaryoWrapper(senaryo_instance),
                'filo': aktif_filo,
                'ortam': senaryo_instance.ortam,
                'n_rovs': secilen_n,
                'n_adalar': n_adalar,
                'n_engels': n_engels,
            }
        except Exception as e:
            print(f"❌ [GAT_DATA] Veri uretimi sirasinda hata: {e}")
            import traceback
            traceback.print_exc()
            return None
