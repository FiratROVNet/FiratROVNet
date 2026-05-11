from __future__ import annotations

import builtins
import logging
import math
from typing import Any

from ursina import Vec3, application, time  # type: ignore[import]

from ..config import FizikSabitleri, Hidrodinamik, PerformansAyarlari  # type: ignore[import]
from ..lider_sec import liderlik_secimini_baslat  # type: ignore[import]
from .logs import LogSystem  # type: ignore[import]


class FiloInitMixin:
    """Filo kurulum ve merkezi tick parcaciklari."""

    ortam_ref: Any
    world: Any
    motorlar: dict[Any, Any]
    leader_manager: Any
    damage_system: Any
    _ignore_tuple_cache: tuple[Any, ...]
    _ignore_tuple_last_rov_count: int
    mevcut_rov_sayisi: int
    _rov_id_map: dict[Any, Any]
    _rovs_cache: list[Any]
    _rovs_cache_src_len: int

    def BlueROV2_motor_konfigurasyonu(self, rov) -> Any: ...
    def minimap(self, *args, **kwargs) -> Any: ...
    def motor_sema_kaydet(self, *args, **kwargs) -> Any: ...
    def tum_motor_bv_kutuphanelerini_guncelle(self) -> Any: ...
    def kamera_ayarla(self) -> Any: ...
    def _process_command_queue(self) -> Any: ...
    def guncelle_navigasyon_kuyrugu(self) -> Any: ...
    def guncelle_gorseller_ve_renkler(self, tahminler) -> Any: ...
    def aktif_liderlik_hedefleri(self) -> Any: ...
    def entity_patlat(self, hedef_entity, parca_sayisi=60) -> Any: ...

    def _build_ignore_tuple(self):
        """
        🔹 Frame başında bir kere bütün ROV ve parçalarını raycast ignore listesine ekle.
        Böylece her ROV sensörü bağımsız olarak liste inşa etmek yerine, merkezi cache'den okur.
        FPS kazancı: Raycast ignore listesi hesaplaması O(rovs * children) → O(rovs) + O(children) per frame.
        """
        if not self.ortam_ref:
            return

        ortam_rovs = [
            r for r in self.ortam_ref.rovs
            if r and not (hasattr(r, "is_destroyed") and r.is_destroyed)
        ]  # type: ignore[union-attr]
        mevcut_count = len(ortam_rovs)

        if mevcut_count == self._ignore_tuple_last_rov_count and self._ignore_tuple_cache:
            self.ortam_ref.ignore_tuple = self._ignore_tuple_cache
            return

        ignores = []
        for rov in ortam_rovs:
            ignores.append(rov)
            for child in getattr(rov, "children", []):
                ignores.append(child)
                ignores.extend(getattr(child, "children", []))

        self._ignore_tuple_cache = tuple(ignores)  # type: ignore[assignment]
        self._ignore_tuple_last_rov_count = mevcut_count
        self.ortam_ref.ignore_tuple = self._ignore_tuple_cache

    def _hazirla_global_ignore_listesi(self, rov_sayisi):
        """Sistemdeki tüm ROV'ları ve parçalarını tek bir tuple'da toplar."""
        if rov_sayisi != self.mevcut_rov_sayisi:
            self.mevcut_rov_sayisi = rov_sayisi
            ortam = self.ortam_ref
            if ortam is None:
                return

            ortam_rovs = [
                r for r in ortam.rovs
                if r and not (hasattr(r, "is_destroyed") and r.is_destroyed)
            ]  # type: ignore[union-attr]

            ignores = []
            for rov in ortam_rovs:
                ignores.append(rov)
                if hasattr(rov, "children"):
                    for child in rov.children:  # type: ignore[union-attr]
                        ignores.append(child)
                        if hasattr(child, "children"):
                            ignores.extend(child.children)  # type: ignore[union-attr]

            ortam.ignore_tuple = tuple(ignores)

    def _baslatma_tamamla(self):
        """ROV'lar için fiziksel gövdeleri ve motorları kurar. Sadece ilk çalıştırmada tam kurulum yapar."""
        ilk_calisma = not getattr(self, '_baslatma_yapildi', False)

        for rov in (self.ortam_ref.rovs if self.ortam_ref else []):
            if rov is not None:
                self._tek_rov_fizik_kur(rov)

        if ilk_calisma:
            self._baslatma_yapildi = True
            self.minimap(scale=1.0)
            self.motor_sema_kaydet()
            self.tum_motor_bv_kutuphanelerini_guncelle()
            self.kamera_ayarla()

    def _tek_rov_fizik_kur(self, rov):
        """Tek bir ROV için fiziksel gövde, GNC ve motorları kurar."""
        from panda3d.bullet import BulletBoxShape, BulletRigidBodyNode  # type: ignore[import]
        from panda3d.core import Vec3 as PandaVec3  # type: ignore[import]
        from FiratROVNet.gnc import Sensor, TemelGNC

        ortam = self.ortam_ref
        if ortam is None or rov is None:
            return

        render = getattr(getattr(ortam, "app", None), "render", None)
        if render is None:
            render = getattr(getattr(application, "base", None), "render", None)
        if render is None:
            render = getattr(builtins, "render", None)
        if render is None:
            return

        self.mevcut_rov_sayisi = len(ortam.rovs)

        if not getattr(rov, 'gnc', None):
            rov.gnc = TemelGNC(rov, self)
        if not getattr(rov, 'sensor', None):
            rov.sensor = Sensor(rov, self, rov.gnc)
            rov.gnc.sensor = rov.sensor

        if self.motorlar.get(rov.id) is None:
            self.motorlar[rov.id] = []

        # Fizik gövdesi zaten varsa yeniden kurma
        if getattr(rov, 'physics_node', None) is not None:
            return

        node = BulletRigidBodyNode(f"ROV_{rov.id}")
        node.setMass(Hidrodinamik.KUTLE)
        node.setLinearDamping(Hidrodinamik.LINEAR_DAMPING)
        node.setAngularDamping(Hidrodinamik.ANGULAR_DAMPING)

        shape = BulletBoxShape(PandaVec3(1.5, 1.5, 1.5))
        node.addShape(shape)

        rov_np = render.attachNewNode(node)
        rov_np.setPos(rov.position)
        self.world.attachRigidBody(node)

        rov.physics_node = node
        rov.physics_np = rov_np

        try:
            self.BlueROV2_motor_konfigurasyonu(rov)
        except Exception as e:
            logging.warning(
                f"[Filo] ROV-{getattr(rov, 'id', '?')} için motor oluşturulamadı: {e}"
            )

        # O(1) lookup map'e ekle
        if hasattr(self, '_rov_id_map'):
            self._rov_id_map[rov.id] = rov
        # rovs cache invalidate (yeni ROV eklendi)
        if hasattr(self, '_rovs_cache_src_len'):
            self._rovs_cache_src_len = -1

    def _tick_sistem_hazirligi(self):
        """Command queue + physics step."""
        self._process_command_queue()
        dt = time.dt  # type: ignore[attr-defined]

        from math import isnan
        from panda3d.core import Vec3 as PandaVec3  # type: ignore[import]

        if self.ortam_ref and hasattr(self.ortam_ref, "rovs"):
            for rov in self.ortam_ref.rovs:
                if not rov or (hasattr(rov, "is_destroyed") and rov.is_destroyed):  # type: ignore[union-attr]
                    continue
                if getattr(rov, "physics_node", None) and getattr(rov, "physics_np", None):
                    p = rov.physics_np.getPos()  # type: ignore[union-attr]
                    v = rov.physics_node.getLinearVelocity()  # type: ignore[union-attr]
                    if (
                        isnan(p.x) or isnan(p.y) or isnan(p.z) or isnan(v.x) or
                        math.isinf(p.x) or math.isinf(p.y) or math.isinf(p.z) or math.isinf(v.x) or
                        abs(p.x) > 1e6 or abs(p.y) > 1e6 or abs(p.z) > 1e6 or abs(v.x) > 1e6
                    ):
                        print(f"🚨 [HATA YAKALANDI] ROV-{getattr(rov, 'id', '?')} değerleri çöktü!")
                        print(f"   Bozuk Pozisyon: {p}")
                        print(f"   Bozuk Hız: {v}")
                        try:
                            rov.physics_np.setPos(0, 0, 0)  # type: ignore[union-attr]
                            rov.physics_node.setLinearVelocity(PandaVec3(0, 0, 0))  # type: ignore[union-attr]
                            rov.physics_node.setAngularVelocity(PandaVec3(0, 0, 0))  # type: ignore[union-attr]
                            rov.physics_node.clearForces()  # type: ignore[union-attr]
                        except Exception:
                            pass

        from FiratROVNet.kutuphane.moduls.profiler import Profiler

        max_substeps = int(getattr(PerformansAyarlari, "PHYSICS_MAX_SUBSTEPS", 4) or 0)
        physics_step = float(getattr(PerformansAyarlari, "PHYSICS_STEP", 1.0 / 60.0) or (1.0 / 60.0))
        Profiler.start("12_world.doPhysics()")
        self.world.doPhysics(dt, max_substeps, physics_step)
        Profiler.end("12_world.doPhysics()")

    def _tick_navigasyon_ve_gorseller(self, tahminler):
        """Grup bazlı hedef yönetimi + renk/gorsel state."""
        self.guncelle_navigasyon_kuyrugu()
        self.guncelle_gorseller_ve_renkler(tahminler)

    def _tick_lider_yonetimi(self):
        """Lider seçim + leader manager güncellemesi."""
        if not getattr(getattr(self, 'leader_manager', None), 'oto_lider_etkin', True):
            return
        yeni_lider_id, _skor = liderlik_secimini_baslat(self, self.aktif_liderlik_hedefleri())
        self.leader_manager.guncelle_liderler(yeni_lider_id)

    def _tick_rovler(self, tahminler):
        """ROV başına hasar/sensör/gnc + basit limit/batarya güncellemeleri."""
        from FiratROVNet.kutuphane.moduls.profiler import Profiler

        if not self.ortam_ref or not hasattr(self.ortam_ref, "rovs"):
            return

        sea_floor_y = getattr(self.ortam_ref, "SEA_FLOOR_Y", -50.0)
        ortam_rovs = self.ortam_ref.rovs
        tahmin_len = len(tahminler) if tahminler is not None else 0
        dt = time.dt  # type: ignore[attr-defined]

        # _build_ignore_tuple guncelle_hepsi() içinde frame başında çağrılıyor; burada tekrar çağırmaya gerek yok

        for idx, rov in enumerate(ortam_rovs):
            if not rov or (hasattr(rov, "is_destroyed") and rov.is_destroyed):
                continue

            gat_kodu = int(tahminler[idx]) if idx < tahmin_len else 0  # type: ignore[index]

            try:
                p = rov.physics_np.getPos()

                if not (
                    math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z) and
                    abs(p.x) < 1e6 and abs(p.y) < 1e6 and abs(p.z) < 1e6
                ):
                    print(f"🚨 [CRITICAL_NAN_CAUGHT] ROV-{getattr(rov, 'id', '?')} physics_np returned NaN/Inf Position: {p}. Forcing Zero.")
                    rov.physics_np.setPos(0, 0, 0)
                    rov.physics_node.setLinearVelocity(Vec3(0, 0, 0))
                    rov.physics_node.setAngularVelocity(Vec3(0, 0, 0))
                    rov.physics_node.clearForces()
                    p = rov.physics_np.getPos()

                h, pr, r = rov.physics_np.getHpr()
                if not (math.isfinite(h) and math.isfinite(pr) and math.isfinite(r)):
                    print(f"🚨 [CRITICAL_NAN_CAUGHT] ROV-{getattr(rov, 'id', '?')} physics_np returned NaN/Inf Hpr: {h, pr, r}. Forcing Zero.")
                    rov.physics_np.setHpr(0, 0, 0)
                    h, pr, r = 0.0, 0.0, 0.0

                rov.position = Vec3(p.x, p.y, p.z)
                rov.rotation = Vec3(pr, h, r)

                if hasattr(rov, "gnc") and rov.gnc is not None:
                    rov.gnc.bullet_yaw = h
                    rov.gnc.bullet_pitch = pr
                    rov.gnc.bullet_roll = r

                rov._world_mat = rov.physics_np.getNetTransform().getMat()

                if hasattr(rov, "velocity"):
                    v = rov.physics_node.getLinearVelocity()
                    if not (
                        math.isfinite(v.x) and math.isfinite(v.y) and math.isfinite(v.z) and
                        abs(v.x) < 1e6 and abs(v.y) < 1e6 and abs(v.z) < 1e6
                    ):
                        print(f"🚨 [CRITICAL_NAN_CAUGHT] ROV-{getattr(rov, 'id', '?')} getLinearVelocity returned NaN/Inf: {v}. Forcing Zero.")
                        rov.physics_node.setLinearVelocity(Vec3(0, 0, 0))
                        v = Vec3(0, 0, 0)
                    rov.velocity = Vec3(v.x, v.y, v.z)
            except Exception:
                continue

            joule_esigi = 120.0
            state = self.damage_system.rov_hasar_kontrol_direct(rov, joule_esigi=joule_esigi)
            if state:
                self.entity_patlat(rov, parca_sayisi=80)
                continue

            try:
                if hasattr(rov, "_guncelle_sensorler"):
                    Profiler.start("13_rov._guncelle_sensorler()")
                    rov._guncelle_sensorler()
                    Profiler.end("13_rov._guncelle_sensorler()")
            except Exception as e:
                if "!is_empty()" not in str(e):
                    print(f"⚠️ [FİLO] ROV-{rov.id} Sensör Hatası: {e}")
                Profiler.end("13_rov._guncelle_sensorler()")

            try:
                if hasattr(rov, "velocity") and rov.velocity and rov.velocity.length() > 0.01:
                    rov.battery -= FizikSabitleri.BATARYA_SOMURME_KATSAYISI * dt
            except Exception:
                pass

            try:
                if rov.y > 0:
                    rov.y = 0
                if rov.y < sea_floor_y:
                    rov.y = sea_floor_y
            except Exception:
                pass

            try:
                if hasattr(rov, "gnc") and rov.gnc:
                    Profiler.start("14_rov.gnc.guncelle(gat_kodu=gat_kodu)")
                    rov.gnc.guncelle(gat_kodu=gat_kodu)
                    Profiler.end("14_rov.gnc.guncelle(gat_kodu=gat_kodu)")
            except Exception as e:
                if "!is_empty()" not in str(e):
                    print(f"⚠️ [FİLO] ROV-{rov.id} GNC Hatası: {e}")
                LogSystem.log_exception(e)

    def _tick_sistem_guncellemeleri(self, guncelle_gorseller: bool):
        """Queued commands + sonar/minimap + obstacle cloud."""
        from FiratROVNet.kutuphane.moduls.profiler import Profiler

        if self.ortam_ref:
            Profiler.start("15_self.ortam_ref.guncelle_sonar_cizgileri()")
            try:
                self.ortam_ref.guncelle_sonar_cizgileri()
            except Exception as e:
                LogSystem.log_exception(e)
            Profiler.end("15_self.ortam_ref.guncelle_sonar_cizgileri()")

        if not guncelle_gorseller:
            return

        if self.ortam_ref and getattr(self.ortam_ref, "minimap", None):
            try:
                Profiler.start("17_self.ortam_ref.minimap.gorsel_guncelle()")
                self.ortam_ref.minimap.gorsel_guncelle()
                Profiler.end("17_self.ortam_ref.minimap.gorsel_guncelle()")
            except Exception as e:
                LogSystem.log_exception(e)
