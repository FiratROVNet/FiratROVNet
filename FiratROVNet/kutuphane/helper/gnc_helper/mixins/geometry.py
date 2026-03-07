import math
from FiratROVNet.config import GATLimitleri
from ursina import Vec3, raycast, color, Entity, destroy


class GeometryMixin:
    """Geometri, Hull, APF ve Vektor islemleri."""

    def _engel_radius_al(self, entity, hit_pt_2d):
        """Hit entity veya ortam.island_positions'tan engel yaricapini (metre) dondurur."""
        if entity is not None:
            try:
                sx = getattr(entity, 'scale_x', None)
                if sx is not None:
                    return float(sx) / 2.0
                scale = getattr(entity, 'scale', None)
                if scale is not None:
                    if isinstance(scale, (int, float)):
                        return float(scale) / 2.0
                    s0 = scale[0] if hasattr(scale, '__getitem__') else getattr(scale, 'x', 0)
                    return float(s0) / 2.0 if s0 else 0.0
            except (TypeError, ValueError):
                pass
        ortam = getattr(self.filo, 'ortam_ref', None)
        if ortam and getattr(ortam, 'island_positions', None) and hit_pt_2d:
            hx, hz = float(hit_pt_2d[0]), float(hit_pt_2d[1])
            best_r, best_d = 0.0, float('inf')
            for ip in ortam.island_positions:
                if len(ip) < 3:
                    continue
                ix, iz, ir = float(ip[0]), float(ip[1]), float(ip[2])
                d = math.sqrt((hx - ix) ** 2 + (hz - iz) ** 2)
                if d < best_d:
                    best_d, best_r = d, ir
            return best_r if best_d < float('inf') else 0.0
        return 0.0

    def _engel_bul_cache_sonuc(self, rov, rov_id: int, menzil: float) -> list:
        """
        Ana thread disindan engel_bul cagrildiginda: ROV'un lidar onbelleginden
        (son_lidar_mesafeleri) engel listesi olusturur. Raycast atilmaz.
        
        DEPRECATED: _engel_bul_lidar_isle() kullanın.
        """
        lidar = getattr(rov, 'son_lidar_mesafeleri', None)
        return self._engel_bul_lidar_isle(rov, rov_id, lidar, menzil)
    
    def _engel_bul_lidar_isle(self, rov, rov_id: int, lidar: dict = None, menzil: float = None) -> list:
        """
        🔹 Sadece Lidar verilerinden engel listesi oluşturur (Sonar YOK, Raycast YOK!)
        
        Args:
            rov: ROV Entity nesnesi
            rov_id: ROV ID'si
            lidar: Lidar mesafe dict'i {0: mesafe_ileri, 1: mesafe_sag, 2: mesafe_sol, 3: mesafe_dip}
            menzil: Maksimum algılama mesafesi (None ise GATLimitleri.ENGEL)
        
        Returns:
            List[Dict]: Engel listesi
        """
        try:
            from ursina import Vec3
        except ImportError:
            return []
        
        if menzil is None:
            from FiratROVNet.config import GATLimitleri
            menzil = GATLimitleri.ENGEL
        
        if lidar is None:
            lidar = {}
        
        # Önbellekte geçerli mesafe var mı?
        lidar_0 = lidar.get(0, -1) if isinstance(lidar, dict) else -1  # İleri
        lidar_1 = lidar.get(1, -1) if isinstance(lidar, dict) else -1  # Sağ
        lidar_2 = lidar.get(2, -1) if isinstance(lidar, dict) else -1  # Sol
        lidar_3 = lidar.get(3, -1) if isinstance(lidar, dict) else -1  # Dip
        
        if lidar_0 < 0 and lidar_1 < 0 and lidar_2 < 0 and lidar_3 < 0:
            return []
        
        # ROV konumu ve yaw (derece) — ana engel_bul ile aynı koordinat dönüşümü
        origin = Vec3(rov.world_position.x, rov.world_position.y, rov.world_position.z) + Vec3(0, 0.5, 0)
        
        # 🔹 L3 (dip lidar) için özel origin - ROV gövdesinin ALTINDAN başlat
        origin_l3 = Vec3(rov.world_position.x, rov.world_position.y, rov.world_position.z) + Vec3(0, -8, 0)
        
        yaw_deg = 0.0
        if hasattr(rov, 'rotation') and rov.rotation is not None:
            if hasattr(rov.rotation, 'y'):
                yaw_deg = float(rov.rotation.y)
            elif isinstance(rov.rotation, (tuple, list)) and len(rov.rotation) >= 2:
                yaw_deg = float(rov.rotation[1])
        yaw_rad = math.radians(yaw_deg)
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)

        # Ursina: Z=ileri, X=sag, Y=yukari. Lokal ileri=(0,0,1), sag=(1,0,0), sol=(-1,0,0)
        def global_vektor(lx, ly, lz):
            gx = lx * c + lz * s
            gz = -lx * s + lz * c
            return Vec3(gx, ly, gz).normalized()

        ileri = global_vektor(0, 0, 1)
        sag = global_vektor(1, 0, 0)
        sol = global_vektor(-1, 0, 0)
        asagi = Vec3(0, -1, 0)
        
        sonuclar = []
        
        # Lidar 0 (ön/ileri)
        if lidar_0 > 0 and lidar_0 <= menzil:
            pt = origin + ileri * lidar_0
            sonuclar.append({
                'yon': 'on_lidar',
                'mesafe': lidar_0,
                'koordinat': (pt.x, pt.z, pt.y),
                'ursina_pos': pt,
                'vektor': ileri,
                'radius': 0.0,
            })
        
        # Lidar 1 (sağ)
        if lidar_1 > 0 and lidar_1 <= menzil:
            pt = origin + sag * lidar_1
            sonuclar.append({
                'yon': 'sag_lidar',
                'mesafe': lidar_1,
                'koordinat': (pt.x, pt.z, pt.y),
                'ursina_pos': pt,
                'vektor': sag,
                'radius': 0.0,
            })
        
        # Lidar 2 (sol)
        if lidar_2 > 0 and lidar_2 <= menzil:
            pt = origin + sol * lidar_2
            sonuclar.append({
                'yon': 'sol_lidar',
                'mesafe': lidar_2,
                'koordinat': (pt.x, pt.z, pt.y),
                'ursina_pos': pt,
                'vektor': sol,
                'radius': 0.0,
            })
        
        # Lidar 3 (dip/aşağı) - 🔹 Özel origin kullan (ROV'un üstünden başlat)
        if lidar_3 > 0 and lidar_3 <= menzil:
            pt = origin_l3 + asagi * lidar_3
            sonuclar.append({
                'yon': 'asagi_lidar',
                'mesafe': lidar_3,
                'koordinat': (pt.x, pt.z, pt.y),
                'ursina_pos': pt,
                'vektor': asagi,
                'radius': 0.0,
            })
        
        return sonuclar

    def _statik_engeller_al(self, rov_id: int, menzil: float) -> list:
        """
        ROV'a menzil icinde kalan adalari (statik engeller) APF icin listeler.
        Lidar isini vurmadan once de adalara karsi itme uygulanir; engellere carpmayi azaltir.
        """
        ortam = getattr(self.filo, 'ortam_ref', None)
        if not ortam or not getattr(ortam, 'island_positions', None):
            return []
        rov = self.filo.find_rov_by_id(rov_id) if hasattr(self.filo, 'find_rov_by_id') else None
        if rov is None or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
            return []
        rx, ry, rz = float(rov.x), float(rov.y), float(rov.z)
        sonuclar = []
        for ip in ortam.island_positions:
            if ip is None or len(ip) < 3:
                continue
            ix, iz, ir = float(ip[0]), float(ip[1]), float(ip[2])
            dx, dz = ix - rx, iz - rz
            d_center = math.sqrt(dx * dx + dz * dz)
            if d_center < 1e-6:
                continue
            # ROV'tan adanin yuzeyine mesafe (2D)
            mesafe_yuzey = max(0.0, d_center - ir)
            if mesafe_yuzey >= menzil:
                continue
            # Ada cevresi uzerinde ROV'a en yakin nokta (itme vektoru hedefi)
            scale = (d_center - ir) / d_center if d_center > 1e-6 else 0
            closest_x = rx + dx * scale
            closest_z = rz + dz * scale
            # Koordinat: (x, z, y) Ursina - engel_bul ile ayni format
            sonuclar.append({
                'yon': 'ada',
                'mesafe': mesafe_yuzey,
                'koordinat': (closest_x, closest_z, ry),
                'radius': ir,
            })
        return sonuclar

    def engel_bul(self, rov_id: int, menzil: float = None, debug: bool = False) -> list:
        """
        🔹 ROV için mevcut lidar verilerinden engel listesi oluşturur.
        
        MODÜLER YAPI: filo.get(rov_id, "lidar") kullanarak mevcut sensör 
        verilerini alır. Raycast yapmaz, direkt lidar okumalarını kullanır.
        
        Lidar Yönleri:
            - L0 (0): İleri
            - L1 (1): Sağ
            - L2 (2): Sol
            - L3 (3): Dip/Aşağı
        
        Args:
            rov_id: ROV ID'si
            menzil: Maksimum algılama mesafesi (metre, None ise GATLimitleri.ENGEL)
            debug: Debug modu (şu an kullanılmıyor)
        
        Returns:
            List[Dict]: Engel listesi, her engel:
                {
                    'yon': 'on_lidar' | 'sol_lidar' | 'sag_lidar' | 'asagi_lidar',
                    'mesafe': float (metre),
                    'koordinat': tuple (x, z, y) - sim koordinat,
                    'ursina_pos': Vec3 (ursina global koordinat),
                    'vektor': Vec3 (yön vektörü),
                    'radius': float (engel yarıçapı, şu an 0.0)
                }
        """
        if menzil is None:
            menzil = GATLimitleri.ENGEL

        # ROV nesnesine erişim
        if not self.filo.ortam_ref or not hasattr(self.filo.ortam_ref, 'rovs'):
            return []

        rov = self.filo.find_rov_by_id(rov_id) if hasattr(self.filo, 'find_rov_by_id') else None
        if rov is None:
            return []

        # ROV silinmişse veya None ise işlem yapma
        if rov is None or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
            return []

        # Lidar verisi: filo.get() veya ROV.son_lidar_mesafeleri (her kare guncelle_hepsi 4C'de guncellenir)
        lidar_data = self.filo.get(rov_id, "lidar") if hasattr(self.filo, 'get') else None
        if lidar_data is None:
            lidar_data = getattr(rov, 'son_lidar_mesafeleri', None)
        if not isinstance(lidar_data, dict):
            lidar_data = {}
        
        # Lidar verilerinden engel listesi oluştur (sonar YOK)
        sonuclar = self._engel_bul_lidar_isle(
            rov=rov, 
            rov_id=rov_id, 
            lidar=lidar_data, 
            menzil=menzil
        )
        
        # Cache güncelleme
        rov._son_engeller = sonuclar
        return sonuclar

    def _vektor_arg_norm(self, arg):
        """Tek argumani normalize eder: ROV ID (int) veya nokta (x, z) tuple."""
        if arg is None:
            return None
        try:
            if hasattr(arg, '__len__') and len(arg) >= 2:
                return (float(arg[0]), float(arg[1]))
        except (TypeError, ValueError, IndexError):
            pass
        return int(arg)

    def vektor(self, ilk=None, ikinci=None,
               rov_id_ilk=None, rov_id_ikinci=None,
               baslangic_noktasi=None, bitis_noktasi=None, vektor=None,
               renk='m', uzunluk=20.0, reverse=False, debug=False, ciz=False):
        """
        Gelismis 3D Vektor Metodu (Ursina Koordinat Sistemi Uyumu).
        - Eksenler: x (yatay), z (yatay), y_depth (derinlik/dussey)
        - Format: (x, z, y_depth)
        """
        # Parametre ayarlari
        self._vektor_renk = renk if renk in self.VEKTOR_RENK_KODLARI else 'm'
        self._vektor_reverse = bool(reverse)
        self._vektor_uzunluk_metre = float(uzunluk) if uzunluk is not None else 20.0

        # Baslangic noktasi (pos1) - (x, z, y_depth)
        pos1 = None
        rid1 = rov_id_ilk if rov_id_ilk is not None else (ilk if isinstance(ilk, int) else None)

        if baslangic_noktasi is not None:
            pos1 = (float(baslangic_noktasi[0]), float(baslangic_noktasi[1]), float(baslangic_noktasi[2]))
        elif rid1 is not None:
            # filo.get artik ursina formatinda (x, z, y_depth) donduruyor
            pos1 = self.filo.get(rid1, "gps")

        if pos1 is None:
            return None

        # Yon (birim vektor) hesabi
        ux, uz, uy_depth = 0.0, 0.0, 0.0
        gercek_hedef_pos = None
        gercek_mesafe = 0.0

        # Durum A: Dogrudan vektor verildiginde (vx, vz, vy_depth)
        if vektor is not None:
            try:
                vx, vz, vy_depth = float(vektor[0]), float(vektor[1]), float(vektor[2])
                mag = math.sqrt(vx ** 2 + vz ** 2 + vy_depth ** 2)
                if mag > 1e-9:
                    ux, uz, uy_depth = vx / mag, vz / mag, vy_depth / mag
                gercek_mesafe = self._vektor_uzunluk_metre
                # Sanal bitis noktasi
                gercek_hedef_pos = (pos1[0] + ux * mag, pos1[1] + uz * mag, pos1[2] + uy_depth * mag)
            except Exception:
                return None

        # Durum B: Iki nokta arasi vektor (baslangic -> hedef)
        else:
            rid2 = rov_id_ikinci if rov_id_ikinci is not None else (ikinci if isinstance(ikinci, int) else None)
            if bitis_noktasi is not None:
                gercek_hedef_pos = (float(bitis_noktasi[0]), float(bitis_noktasi[1]), float(bitis_noktasi[2]))
            elif rid2 is not None:
                gercek_hedef_pos = self.filo.get(rid2, "gps")

            if gercek_hedef_pos is not None:
                dx = gercek_hedef_pos[0] - pos1[0]
                dz = gercek_hedef_pos[1] - pos1[1]
                dy_depth = gercek_hedef_pos[2] - pos1[2]

                gercek_mesafe = math.sqrt(dx ** 2 + dz ** 2 + dy_depth ** 2)
                if gercek_mesafe > 1e-9:
                    ux, uz, uy_depth = dx / gercek_mesafe, dz / gercek_mesafe, dy_depth / gercek_mesafe
            else:
                return None

        # Ters cevirme (kuvvet vektorleri icin itme yonu)
        if self._vektor_reverse:
            ux, uz, uy_depth = -ux, -uz, -uy_depth

        # Gorsel cizim bitis noktasi (pos2)
        pos2_cizim = (
            pos1[0] + ux * self._vektor_uzunluk_metre,
            pos1[1] + uz * self._vektor_uzunluk_metre,
            pos1[2] + uy_depth * self._vektor_uzunluk_metre,
        )

        # Cikti verisi (x, z, y_depth)
        ret = {
            'baslangic_3d': pos1,
            'bitis_3d': gercek_hedef_pos,
            'birim_vektor_3d': (ux, uz, uy_depth),
            'uzaklik_metre': float(gercek_mesafe),
        }

        # Minimap cizim listesine ekle
        if ciz:
            if debug:
                self._apf_vektor_list = []

            self._apf_vektor_list.append({
                'baslangic': pos1,
                'bitis': pos2_cizim,
                'renk': self._vektor_renk,
                'uzunluk': self._vektor_uzunluk_metre,
                'rov_id': rid1,
            })

        return ret

    def _vektor_poz_al_3d(self, rov_id, ortam):
        """ROV'un 3D pozisyonunu simulasyon formatinda doner."""
        if not ortam or not hasattr(ortam, 'rovs'):
            return None
        for r in ortam.rovs:
            if r and getattr(r, 'id', None) == rov_id:
                # Ursina (x, y, z) -> Simulasyon (x, y_ileri, z_derinlik)
                return (r.x, r.z, -r.y)
        return None

    def apf(self, rov_id: int, hedef: bool = True, engel: bool = True, rov: bool = True):
        """
        3D APF hesaplama. Engel kacinma duzeltmeleri yapildi.
        """
        self.apf_temizle(rov_id=rov_id)

        toplam_vec = [0.0, 0.0, 0.0]
        out_hedef = None
        out_engeller = []
        out_rovs = []

        def add_vec(res_dict):
            if res_dict and 'birim_vektor_3d' in res_dict:
                bv = res_dict['birim_vektor_3d']
                toplam_vec[0] += bv[0]
                toplam_vec[1] += bv[1]
                toplam_vec[2] += bv[2]
                return True
            return False

        # 1. Hedef vektoru
        if hedef:
            h_koord = self.filo.hedef(rov_id=rov_id)
            if h_koord:
                res = self.vektor(rov_id_ilk=rov_id, bitis_noktasi=h_koord, renk='y', ciz=True)
                if res:
                    add_vec(res)
                    out_hedef = {
                        'birim_vektor': res.get('birim_vektor_3d', (0, 0, 0)),
                        'mesafe': res.get('uzaklik_metre', 0.0),
                    }

        # 2. Engel kacinma: lidar + statik adalar (lidar isini vurmadan once de ada itmesi)
        if engel:
            tespit_edilenler = self.engel_bul(rov_id=rov_id, menzil=GATLimitleri.ENGEL)
            statik = self._statik_engeller_al(rov_id=rov_id, menzil=GATLimitleri.ENGEL)
            for e in tespit_edilenler + statik:
                target = e.get('koordinat')
                sensor_mesafesi = float(e.get('mesafe', 0.0))

                if target:
                    res = self.vektor(
                        rov_id_ilk=rov_id,
                        bitis_noktasi=target,
                        reverse=True,
                        renk='k',
                        ciz=True,
                    )

                    if res and add_vec(res):
                        out_engeller.append({
                            'birim_vektor': res.get('birim_vektor_3d', (0, 0, 0)),
                            'mesafe': sensor_mesafesi,
                            'radius': e.get('radius', 0.0),
                            'yon': e.get('yon'),
                        })

        # 3. Diger ROV'lardan kacinma
        if rov:
            for r in self.rov_vektor(rov_id=rov_id, menzil=GATLimitleri.CARPISMA):
                target = r.get('koordinat')
                gercek_mesafe = float(r.get('mesafe', 0.0))

                if target:
                    res = self.vektor(
                        rov_id_ilk=rov_id,
                        bitis_noktasi=target,
                        reverse=True,
                        renk='t',
                        ciz=True,
                    )

                    if res and add_vec(res):
                        out_rovs.append({
                            'birim_vektor': res.get('birim_vektor_3d', (0, 0, 0)),
                            'mesafe': gercek_mesafe,
                        })

        mag = math.sqrt(sum(v ** 2 for v in toplam_vec))
        birim = (toplam_vec[0] / mag, toplam_vec[1] / mag, toplam_vec[2] / mag) if mag > 1e-9 else (0, 0, 0)

        return {
            'birim_vektor': birim,
            'mesafe': float(mag),
            'hedef': out_hedef,
            'engeller': out_engeller,
            'rovs': out_rovs,
        }

    def apf_temizle(self, rov_id=None) -> None:
        """
        APF vektorlerini temizler. rov_id verilirse sadece o ROV'a ait vektorleri siler;
        bos birakilirsa hepsini temizler.
        Minimap sadece hepsi temizlenirken guncellenir; tek ROV silinirken minimap dokunulmaz
        ki diger ROV'larin vektorleri minimapte kalmaya devam etsin (her ROV icin APF gosterilir).
        """
        if rov_id is None:
            self._apf_vektor_list = []
            ortam = getattr(self.filo, 'ortam_ref', None)
            if ortam and hasattr(ortam, 'minimap') and ortam.minimap is not None:
                try:
                    ortam.minimap._apf_vektorlari_temizle()
                except Exception:
                    pass
        else:
            rid = int(rov_id)
            self._apf_vektor_list = [i for i in self._apf_vektor_list if i.get('rov_id') != rid]

    def apf_guncelle_tum(self) -> None:
        """APF vektorlerini tum ROV'lar icin gunceller (minimap vb.)."""
        self.apf_temizle()

    def hedef_vektor(self, rov_id: int):
        """
        ROV'un hedefine olan 3B vektor bilgisini dondurur (cizim yapmaz).
        """
        hedef_koord = self.filo.hedef(rov_id=rov_id)
        if hedef_koord is None:
            return None
        return self.vektor(
            rov_id_ilk=rov_id,
            bitis_noktasi=hedef_koord,
            renk='y',
            ciz=False,
        )

    def rov_vektor(self, rov_id: int, menzil: float = None):
        """
        ROV'un diger ROV'lara olan 3B kacinma vektorlerini dondurur (cizim yapmaz).
        """
        if menzil is None:
            menzil = GATLimitleri.CARPISMA

        ortam = getattr(self.filo, 'ortam_ref', None)
        if not ortam or not hasattr(ortam, 'rovs'):
            return []

        # Kendi 3D pozisyonumuzu al (Simulasyon formati)
        pos_self = self._vektor_poz_al_3d(rov_id, ortam)
        if not pos_self:
            return []

        result = []
        for r_ent in ortam.rovs:
            if r_ent is None or getattr(r_ent, 'id', None) == rov_id:
                continue
            if getattr(r_ent, 'is_destroyed', False):
                continue

            # Diger ROV pozisyonu (x, z, y_depth) - vektor() ve get("gps") ile ayni format
            pos_other = (r_ent.x, r_ent.z, -r_ent.y)

            dist = math.sqrt(
                (pos_other[0] - pos_self[0]) ** 2 +
                (pos_other[1] - pos_self[1]) ** 2 +
                (pos_other[2] - pos_self[2]) ** 2
            )

            if dist <= menzil:
                vb = self.vektor(
                    baslangic_noktasi=pos_self,
                    bitis_noktasi=pos_other,
                    reverse=True,
                    ciz=False,
                )
                if vb:
                    result.append({
                        'rov_id': int(r_ent.id),
                        'koordinat': pos_other,
                        'vektor_bilgi': vb,
                        'mesafe': dist,
                    })
        return result

    def hull(self, offset=50.0):
        """
        Guvenlik hull olusturur (thread-safe).
        """
        if not self.filo._is_main_thread():
            try:
                from ursina import invoke
                result = [None]

                def wrapper():
                    result[0] = self.filo._guvenlik_hull_olustur_impl(offset)

                invoke(wrapper)
                return result[0] if result[0] is not None else {'hull': None, 'points': None, 'center': None}
            except (ImportError, AttributeError):
                self.filo._command_queue.put(('hull', (offset,), {}))
                return {'hull': None, 'points': None, 'center': None}

        return self.filo._guvenlik_hull_olustur_impl(offset)

    def yeni_hull(self, yasakli_noktalar=None, offset=50.0, buffer_radius=10.0, g_id=0, **kwargs):
        """
        Lider ROV etrafinda dairesel bir guvenli alan (Hull) olusturur.
        """
        base_points = []
        lider_id, lider_gps = self.find_leader_info(sessiz=False, g_id=g_id)
        if lider_gps is None:
            print(f"⚠️ [UYARI] Grup-{g_id} icin Lider ROV bulunamadi! Merkez (0,0) kabul ediliyor.")
            lx, ly = 0.0, 0.0
        else:
            lx, ly = lider_gps[0], lider_gps[1]
        radius = max(15.0, float(offset))
        for i in range(16):
            angle = math.radians(i * 22.5)
            nx = lx + math.cos(angle) * radius
            ny = ly + math.sin(angle) * radius
            base_points.append([nx, ny])
        mgr = None
        if hasattr(self.filo, 'hull_manager') and self.filo.hull_manager:
            mgr = self.filo.hull_manager
        else:
            try:
                from FiratROVNet.hull import HullManager
                mgr = HullManager(self.filo)
            except ImportError:
                print("❌ [KRITIK] HullManager modulu bulunamadi!")
                return {'points': None, 'center': None, 'hull': None}
        sonuc = mgr.yeni_hull(
            base_points=base_points,
            yasakli_noktalar=yasakli_noktalar,
            offset=0.0,
            buffer_radius=buffer_radius,
            g_id=g_id,
        )
        if sonuc.get('points') is None:
            print(f"❌ [DEBUG] HullManager Grup-{g_id} icin sonuc donduremedi!")
        return sonuc

    def ada_cevre(self, offset: float = 15.0, sessiz: bool = False) -> list:
        """
        Simulasyondaki adalari tespit edip 2D (X, Z) duzleminde noktalar dondurur.
        Hull algoritmasi icin [x, z] formatinda cikti verir.
        """
        if not self.filo.ortam_ref:
            if not sessiz:
                print("⚠️ [UYARI] Ortam referansi bulunamadi!")
            return []

        if not hasattr(self.filo.ortam_ref, 'island_positions') or not self.filo.ortam_ref.island_positions:
            if not sessiz:
                print("⚠️ [UYARI] Simulasyonda ada bulunamadi!")
            return []

        tum_noktalar = []
        for island_data in self.filo.ortam_ref.island_positions:
            if island_data is None or len(island_data) < 3:
                continue
            island_x = float(island_data[0])
            island_z = float(island_data[1])
            island_radius = float(island_data[2])
            cevre_mesafesi = island_radius + offset
            for i in range(12):
                aci_rad = math.radians(i * 30)
                nx = island_x + cevre_mesafesi * math.sin(aci_rad)
                nz = island_z + cevre_mesafesi * math.cos(aci_rad)
                tum_noktalar.append([nx, nz])

        if not sessiz and getattr(self.filo.ortam_ref, "verbose", False):
            aktif_ada_sayisi = sum(1 for ada in self.filo.ortam_ref.island_positions if ada is not None)
            print(f"✅ [ADA_CEVRE] {aktif_ada_sayisi} ada icin {len(tum_noktalar)} nokta hesaplandi (offset={offset}m)")
        return tum_noktalar
