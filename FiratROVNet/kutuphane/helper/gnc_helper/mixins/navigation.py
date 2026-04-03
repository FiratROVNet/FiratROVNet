from typing import Any, Optional


class NavigationMixin:
    """Navigasyon fonksiyonlari."""
    filo: Any
    hedef_gorsel_olustur: Any

    def _bar_derinlik_al(self, rov) -> Optional[float]:
        sensor = getattr(rov, 'sensor', None)
        bar = getattr(sensor, 'bar', None)
        if not isinstance(bar, dict):
            return None
        derinlik = bar.get("derinlik")
        if derinlik is None:
            derinlik = bar.get("derinlik_m")
        try:
            return float(derinlik) if derinlik is not None else None
        except (TypeError, ValueError):
            return None

    def _hedef_impl(self, x, y, z, rov_id=None, ciz=True):
        if rov_id is None or not self.filo.ortam_ref:
            return None

        try:
            rov = self.filo.find_rov_by_id(rov_id)
            if not rov:
                return None
        except Exception as e:
            from FiratROVNet.gnc.logs import LogSystem

            LogSystem.log_exception(e)
            return None

        self.filo._rov_hedefleri[rov_id] = (x, y, z)
        self.filo.git(rov_id, x, y, z, ai=True)

        rov = self.filo.find_rov_by_id(rov_id)
        if not rov:
            return None

        if rov.role == 1:
            self.filo.hedef_pozisyon = (x, y, z)
            if ciz:
                self.hedef_gorsel_olustur(x, y, z)
            elif self.filo.hedef_gorsel:
                from ursina import destroy

                destroy(self.filo.hedef_gorsel)
                self.filo.hedef_gorsel = None

        return (x, y, z)

    def _git_impl(self, rov_id: int, x: float, y: float, z: Optional[float] = None, ai: bool = True, sessiz: bool = True) -> None:
        """
        git() fonksiyonunun yeni mimariye (rov.gnc) uyarlanmis implementasyonu.
        ROV objesinden rov.id ile ID tutarliligi saglanir.
        """
        # 1. Ortam ve liste kontrolleri
        if not self.filo.ortam_ref or not hasattr(self.filo.ortam_ref, 'rovs'):
            # Mock ortam durumunda, gnc_sistemleri'nden ara
            if not (hasattr(self.filo, 'ortam_ref') and self.filo.ortam_ref is None and 
                    hasattr(self.filo, 'gnc_sistemleri')):
                if not sessiz:
                    print("❌ [FILO] Ortam referansi bulunamadi.")
                return

        # 2. ROV objesini bul (find_rov_by_id ile)
        rov = None
        if hasattr(self.filo, 'find_rov_by_id'):
            try:
                rov = self.filo.find_rov_by_id(rov_id)
            except Exception:
                rov = None
        
        # Fallback: Mock ortam durumunda
        if rov is None and hasattr(self.filo, 'ortam_ref') and self.filo.ortam_ref is None:
            if hasattr(self.filo, 'gnc_sistemleri') and isinstance(self.filo.gnc_sistemleri, dict):
                gnc_obj = self.filo.gnc_sistemleri.get(rov_id)
                if gnc_obj and hasattr(gnc_obj, 'rov'):
                    rov = gnc_obj.rov
        
        if rov is None:
            if not sessiz:
                print(f"❌ [FILO] Gecersiz ROV ID: {rov_id}")
            return

        # ROV yoksa veya patlamissa islem yapma
        if hasattr(rov, 'is_destroyed') and rov.is_destroyed:
            return

        # ROV'un GNC sistemi var mi?
        if not hasattr(rov, 'gnc') or rov.gnc is None:
            if not sessiz:
                print(f"❌ [FILO] ROV-{rov.id} icin GNC sistemi bulunamadi.")
            return

        # 3. Manuel derinlik hesapla (Z verilmemisse)
        if z is None:
            z = self._bar_derinlik_al(rov)
            if z is None:
                from FiratROVNet.gnc import Koordinator
                mevcut_sim_pos = Koordinator.ursina_to_sim(rov.x, rov.y, rov.z)
                z = mevcut_sim_pos[2]

        # 4. Hedef atama ve ayarlar
        try:
            rov.gnc.manuel_kontrol = False
            rov.gnc.hedef_atama(x, y, z)

            # Filo hafizasina kaydet (ROV objesi ile bagli)
            if not hasattr(self.filo, '_rov_hedefleri'):
                self.filo._rov_hedefleri = {}
            self.filo._rov_hedefleri[rov.id] = (x, y, z)  # rov.id kullan

            if not sessiz:
                print(f"✅ [FILO] ROV-{rov.id} -> Hedef: ({x:.1f}, {y:.1f}, {z:.1f}) | AI: {'ACIK' if ai else 'KAPALI'}")

        except Exception as e:
            if not sessiz:
                print(f"❌ [HATA] Hedef atanamadi: {e}")

    def git(self, rov_id: int, x, y: Optional[float] = None, z: Optional[float] = None, ai: bool = True, sessiz: bool = False) -> None:
        """
        ROV'a hedef koordinati atayan genel fonksiyon.
        Koordinat Formati: (X: Sag-Sol, Y: Ileri-Geri, Z: Derinlik)
        rov_id ile bulunan ROV objesindeki rov.id kullanilir.
        """
        # ROV'u bul (rov.id ile tutarliligi sagla)
        rov = None
        if hasattr(self.filo, 'find_rov_by_id'):
            try:
                rov = self.filo.find_rov_by_id(rov_id)
            except Exception:
                rov = None
        
        # Fallback: Doğru rov ortam_ref olmadan g_rovs'dan ara
        if rov is None and hasattr(self.filo, 'ortam_ref') and self.filo.ortam_ref is None:
            # Mock ortam durumunda, doğrudan gnc_sistemleri kontrol et
            if hasattr(self.filo, 'gnc_sistemleri') and isinstance(self.filo.gnc_sistemleri, dict):
                rov = self.filo.gnc_sistemleri.get(rov_id)
                if rov:
                    # gnc sistemi bulundu, örneğin TemelGNC ise rov_entity'ye eriş
                    if hasattr(rov, 'rov'):
                        rov = rov.rov
        
        if rov is None:
            if not sessiz:
                print(f"❌ [FILO] ROV bulunamadi: {rov_id}")
            return
        
        default_z = self._bar_derinlik_al(rov)
        target_x, target_y, target_z = 0.0, 0.0, (float(z) if z is not None else default_z)

        # 1. Girdi ayrisma (nokta listesi, liste veya float)
        if isinstance(x, (list, tuple)):
            if not x:
                return

            # Durum A: Coklu nokta listesi (rota) -> [[x1,y1], [x2,y2], ...]
            if isinstance(x[0], (list, tuple)):
                self.filo._git_nokta_listesi[rov.id] = [[float(n[0]), float(n[1])] for n in x if len(n) >= 2]
                self.filo._git_mevcut_nokta_indeksi[rov.id] = 0

                # Hedef derinligi tum rota icin sakla
                if not hasattr(self.filo, '_git_hedef_derinligi'):
                    self.filo._git_hedef_derinligi = {}
                self.filo._git_hedef_derinligi[rov.id] = target_z

                # Ilk noktayi hedef olarak al
                ilk_nokta = self.filo._git_nokta_listesi[rov.id][0]
                target_x, target_y= ilk_nokta[0], ilk_nokta[1]
                # target_z zaten z olarak ayarli (satir 68)

            # Durum B: Tekil koordinat listesi -> [x, y] veya [x, y, z]
            else:
                target_x = float(x[0])
                target_y = float(x[1])
                if len(x) >= 3:
                    target_z = float(x[2])
        else:
            # Durum C: Dogrudan float degerler (x, y, z)
            if y is None:
                if not sessiz:
                    print("❌ [FILO] Y koordinati eksik.")
                return
            target_x, target_y = float(x), float(y)

        # 2. Uygulama (implementasyona yonlendir)
        self._git_impl(rov.id, target_x, target_y, target_z, ai, sessiz)

    def git_path(self, rov_id, hedef, ai=True, isaret=True):
        """
        ROV'a bir yol atar ve otomatik moda gecirir (thread-safe).
        ROV'un mevcut derinligini korur.
        isaret=True ise bir sonraki waypoint minimapte gosterilir.
        rov_id ile bulunan ROV objesindeki rov.id kullanilir.
        """
        # ROV'u bul - tutarliligi sagla
        rov = None
        if hasattr(self.filo, 'find_rov_by_id'):
            try:
                rov = self.filo.find_rov_by_id(rov_id)
            except Exception:
                rov = None
        
        # Fallback: Mock ortam durumunda
        if rov is None and hasattr(self.filo, 'ortam_ref') and self.filo.ortam_ref is None:
            if hasattr(self.filo, 'gnc_sistemleri') and isinstance(self.filo.gnc_sistemleri, dict):
                gnc_obj = self.filo.gnc_sistemleri.get(rov_id)
                if gnc_obj and hasattr(gnc_obj, 'rov'):
                    rov = gnc_obj.rov
        
        if rov is None:
            return
        
        # Thread-safe: Ana thread degilse queue'ya ekle
        if not self.filo._is_main_thread():
            self.filo._command_queue.put(('git_path', (rov.id, hedef, ai), {'isaret': isaret}))
            return

        self._git_path_impl(rov.id, hedef, ai, isaret)

    def _git_path_impl(self, rov_id, hedef, ai=True, isaret=False):
        """
        A* ile yol planlar ve ROV'u mevcut derinligini koruyarak o yola sokar.
        rov_id ile bulunan ROV objesindeki rov.id kullanilir.
        """
        # ROV'u bul
        rov = None
        if hasattr(self.filo, 'find_rov_by_id'):
            try:
                rov = self.filo.find_rov_by_id(rov_id)
            except Exception:
                rov = None
        
        # Fallback: Mock ortam durumunda
        if rov is None and hasattr(self.filo, 'ortam_ref') and self.filo.ortam_ref is None:
            if hasattr(self.filo, 'gnc_sistemleri') and isinstance(self.filo.gnc_sistemleri, dict):
                gnc_obj = self.filo.gnc_sistemleri.get(rov_id)
                if gnc_obj and hasattr(gnc_obj, 'rov'):
                    rov = gnc_obj.rov
        
        if rov is None:
            print(f"❌ [FILO] Gecersiz ROV ID: {rov_id}")
            return
        
        if not hasattr(self.filo, '_git_isaret'):
            self.filo._git_isaret = {}
        self.filo._git_isaret[rov.id] = bool(isaret)  # rov.id kullan

        # 1. Mevcut pozisyonu al
        pos = self.filo.get(rov.id, "gps")  # rov.id kullan
        current_x, current_y, current_z = pos[0], pos[1], pos[2]

        current_bar_z = self._bar_derinlik_al(rov)

        # 2. Hedef derinligi belirle (hedefte belirtilmisse onu kullan)
        if isinstance(hedef, (tuple, list)) and len(hedef) >= 3:
            target_z = float(hedef[2])  # Kullanici derinlik belirtmis
        else:
            target_z = current_bar_z if current_bar_z is not None else current_z

        # 3. A* icin 2D baslangic ve hedef (x, y)
        start_2d = (current_x, current_y)

        if isinstance(hedef, (tuple, list)) and len(hedef) >= 2:
            goal_2d = (float(hedef[0]), float(hedef[1]))
        else:
            print(f"❌ [FILO] Hedef formati hatali: {hedef}")
            return

        if not hasattr(self.filo, 'grup_hedefleri'):
            self.filo.grup_hedefleri = {}
        self.filo.grup_hedefleri[getattr(rov, 'group_id', 0)] = (
            float(goal_2d[0]),
            float(goal_2d[1]),
            float(target_z),
        )

        # 4. A* yol planlama
        yol_noktalari = self._a_star_path_planla(start_2d, goal_2d)

        if not yol_noktalari:
            print("⚠️ [FILO] Yol bulunamadi, dogrudan gidiliyor.")
            self.filo.git(rov.id, goal_2d[0], goal_2d[1], target_z, ai=ai)
            return

        # 5. Minimap guncelleme
        ortam = self.filo.ortam_ref
        if ortam and ortam.minimap:
            ortam.minimap.update_path(yol_noktalari)

        # 6. Yolu atama ve baslatma (target_z kullan)
        self.filo.git(rov.id, yol_noktalari, z=target_z, ai=ai)

    def _a_star_path_planla(self, start_2d: tuple, goal_2d: tuple) -> list:
        """
        A* yol planlamasi yapan ortak helper metodu.
        FiratROVNet/a_star.py'deki AStarPlanner sinifini kullanir.
        Ortamdaki engelleri (adalari) ve dinamik engelleri (Lidar bulutunu) 
        otomatik olarak alir.
        """
        try:
            from FiratROVNet.a_star import AStarPlanner
        except ImportError:
            try:
                from ..a_star import AStarPlanner
            except ImportError:
                print("❌ [PATH] AStarPlanner modulu yuklenemedi!")
                return []

        ortam = self.filo.ortam_ref
        if not ortam or not hasattr(ortam, 'island_positions'):
            return []

        # Ortamdaki adalari engel olarak al (X, Z, R)
        adalar = [(p[0], p[1], p[2]) for p in ortam.island_positions if p]
        
        # Dinamik engelleri (Lidar bulutunu) basitlestir
        dinamik_engeller_polygon = []
        try:
            from FiratROVNet.hull import HullManager
            hull_manager = HullManager(filo_ref=self.filo)
            engel_bulutu = getattr(ortam, 'engel_bulutu', [])
            
            if engel_bulutu and len(engel_bulutu) > 0:
                # Lidar bulutunu basitlestir
                poligonlar = hull_manager.dinamik_engelleri_basitlestir(
                    engel_bulutu=engel_bulutu,
                    kume_mesafesi=25.0,
                    buffer_radius=5.0,
                    min_kume_boyutu=3
                )
                
                # Polygon'ları (x, y, radius) formatına dönüştür
                for poly in poligonlar:
                    try:
                        centroid = poly.centroid
                        bounds = poly.bounds  # (minx, miny, maxx, maxy)
                        
                        if bounds:
                            radius = max(
                                bounds[2] - centroid.x,
                                bounds[3] - centroid.y
                            )
                            dinamik_engeller_polygon.append((centroid.x, centroid.y, radius))
                    except Exception:
                        pass
        except ImportError:
            pass
        except Exception as e:
            # Dinamik engel işleme başarısız olsa bile, statik adalarla devam et
            pass
        
        # Tum engelleri birlestir (statik + dinamik)
        tumEngeller = adalar + dinamik_engeller_polygon

        planner = AStarPlanner()
        yol_noktalari = planner.find_path(
            start=start_2d,
            goal=goal_2d,
            obstacles=tumEngeller,
            havuz_genisligi=ortam.havuz_genisligi,
        )

        return yol_noktalari if isinstance(yol_noktalari, list) else []

    def move(self, rov_id: int, yon: str, guc: float = 1.0, sessiz: bool = True) -> None:
        """
        ROV'a guc bazli hareket komutu verir.
        """
        if len(self.filo.ortam_ref.rovs) == 0:
            if not sessiz:
                print("❌ [HATA] GNC sistemleri henuz kurulmamıs!")
                print("   💡 Cozum: filo.ekle() ile GNC sistemleri ekleyin")
            return

        if not isinstance(rov_id, int) or rov_id < 0:
            if not sessiz:
                print(f"❌ [HATA] Gecersiz ROV ID: {rov_id} (pozitif tam sayi olmali)")
                print(f"   Mevcut ROV sayisi: {len(self.filo.ortam_ref.rovs)} (0-{len(self.filo.ortam_ref.rovs)-1} arasi)")
            return

        if self.filo.find_rov_by_id(rov_id) is None:
            if not sessiz:
                print(f"❌ [HATA] ROV ID {rov_id} mevcut degil!")
                print(f"   Mevcut ROV sayisi: {len([r for r in self.filo.ortam_ref.rovs if r])}")
                print("   💡 Cozum: filo.ekle() ile daha fazla GNC sistemi ekleyin")
            return

        gecerli_yonler = ['ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur', 'yaw']
        if yon not in gecerli_yonler:
            if not sessiz:
                print(f"❌ [HATA] Gecersiz hareket yonu: '{yon}'")
                print(f"   Gecerli yonler: {', '.join(gecerli_yonler)}")
            return

        if not isinstance(guc, (int, float)):
            if not sessiz:
                print(f"❌ [HATA] Guc degeri sayi olmali: {guc}")
            return

        if yon == 'yaw':
            guc = max(-1.0, min(1.0, float(guc)))
        else:
            guc = max(0.0, min(1.0, float(guc)))

        try:
            gnc = self.filo.find_rov_by_id(rov_id) if hasattr(self.filo, 'find_rov_by_id') else None
            if gnc is None:
                if not sessiz:
                    print(f"❌ [HATA] ROV-{rov_id} bulunamadi")
                return
            rov = gnc.rov

            if not hasattr(rov, 'active_forces') or rov.active_forces is None:
                rov.active_forces = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0, 'yaw': 0.0}

            if yon == 'dur':
                for k in rov.active_forces:
                    rov.active_forces[k] = 0.0
                if hasattr(rov, 'manuel_hareket'):
                    rov.manuel_hareket['yon'] = 'dur'
                    rov.manuel_hareket['guc'] = 0.0
                if not sessiz:
                    print(f"🛑 [FILO] ROV-{rov_id} manuel giris sifirlandi (otonom devam edebilir)")
                return

            if yon == 'yaw':
                rov.active_forces['yaw'] = guc
                if hasattr(rov, 'manuel_hareket'):
                    rov.manuel_hareket['yon'] = 'yaw'
                    rov.manuel_hareket['guc'] = guc
                if not sessiz:
                    guc_yuzdesi = int(abs(guc) * 100)
                    yon_metni = "saat yonunun tersine" if guc > 0 else "saat yonunde"
                    print(f"🔄 [FILO] ROV-{rov_id} {yon_metni} %{guc_yuzdesi} gucle donduruluyor (yaw)")
                return

            if yon == 'bat' and rov.role == 1:
                if not sessiz:
                    print(f"⚠️ [FILO] ROV-{rov_id} lider, batirilamaz!")
                return

            HAVUZ_GUVENLIK_MESAFESI = 10.0
            if hasattr(rov, 'environment_ref') and rov.environment_ref:
                havuz_genisligi = getattr(rov.environment_ref, 'havuz_genisligi', 200)
                havuz_sinir = havuz_genisligi
                guvenli_sinir = havuz_sinir - HAVUZ_GUVENLIK_MESAFESI

                sinirda_x = abs(rov.x) >= guvenli_sinir * 0.95
                sinirda_z = abs(rov.z) >= guvenli_sinir * 0.95
                sinirda_y_ust = rov.y >= 0.3
                sinirda_y_alt = rov.y <= -95

                if sinirda_x and ((yon == 'sag' and rov.x > 0) or (yon == 'sol' and rov.x < 0)):
                    if not sessiz:
                        print(f"⚠️ [FILO] ROV-{rov_id} havuz sinirinda (X), {yon} yonunde hareket engellendi")
                    return

                if sinirda_z and ((yon == 'ileri' and rov.z > 0) or (yon == 'geri' and rov.z < 0)):
                    if not sessiz:
                        print(f"⚠️ [FILO] ROV-{rov_id} havuz sinirinda (Z), {yon} yonunde hareket engellendi")
                    return

                if sinirda_y_ust and yon == 'cik':
                    if not sessiz:
                        print(f"⚠️ [FILO] ROV-{rov_id} su yuzeyinde, yukari hareket engellendi")
                    return

                if sinirda_y_alt and yon == 'bat':
                    if not sessiz:
                        print(f"⚠️ [FILO] ROV-{rov_id} deniz tabaninda, asagi hareket engellendi")
                    return

            if yon == 'ileri':
                rov.active_forces['surge'] = guc
            elif yon == 'geri':
                rov.active_forces['surge'] = -guc
            elif yon == 'sag':
                rov.active_forces['sway'] = guc
            elif yon == 'sol':
                rov.active_forces['sway'] = -guc
            elif yon == 'cik':
                rov.active_forces['heave'] = guc
            elif yon == 'bat' and rov.role != 1:
                rov.active_forces['heave'] = -guc

            if hasattr(rov, 'manuel_hareket'):
                rov.manuel_hareket['yon'] = yon
                rov.manuel_hareket['guc'] = guc

            if not sessiz:
                guc_yuzdesi = int(abs(guc) * 100)
                print(f"🔵 [FILO] ROV-{rov_id} {yon} yonunde %{guc_yuzdesi} gucle hareket ediyor (eszamanli mod)")
        except AttributeError as e:
            if not sessiz:
                print(f"❌ [HATA] ROV-{rov_id} icin gerekli ozellik bulunamadi: {e}")
                rov_ref = self.filo.find_rov_by_id(rov_id) if hasattr(self.filo, 'find_rov_by_id') else None
                print(f"   💡 Debug: GNC sistemi tipi: {type(rov_ref)}")
                import traceback
                traceback.print_exc()
        except Exception as e:
            if not sessiz:
                print(f"❌ [HATA] Hareket komutu sirasinda hata: {e}")
                print(f"   💡 Debug: ROV ID: {rov_id}, Yon: {yon}, Guc: {guc}")
                import traceback
                traceback.print_exc()
