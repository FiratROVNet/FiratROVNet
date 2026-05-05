from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable

from FiratROVNet.config import Hidrodinamik


@dataclass(frozen=True)
class GorevHedefi:
    gorev_adi: str
    grup_id: int | None = None
    koordinat: tuple[float, float, float] | None = None
    alan: Any | None = None


@dataclass(frozen=True)
class RovDegerOnerisi:
    rov_id: int
    puan: float
    secilebilir: bool
    gorev: str
    grup_id: int | None
    batarya: float
    mesafe: float
    yeni_enerji: float
    mevcut_gorev_enerjisi: float
    goreve_devam_enerjisi: float
    gorev_degistirme_enerjisi: float
    gorev_degistirme_cezasi: float
    enerji_kazanci: float
    rol_carpani: float
    grup_carpani: float
    neden: str = ""


def gorev_hedefi_coz(gorev_hedefi) -> GorevHedefi:
    if isinstance(gorev_hedefi, GorevHedefi):
        return gorev_hedefi
    if isinstance(gorev_hedefi, dict):
        return GorevHedefi(
            gorev_adi=str(gorev_hedefi.get("gorev_adi") or gorev_hedefi.get("gorev") or "gorev"),
            grup_id=gorev_hedefi.get("grup_id"),
            koordinat=_koordinat_tuple(gorev_hedefi.get("koordinat") or gorev_hedefi.get("hedef")),
            alan=gorev_hedefi.get("alan"),
        )
    if isinstance(gorev_hedefi, (list, tuple)):
        return GorevHedefi(gorev_adi="gorev", koordinat=_koordinat_tuple(gorev_hedefi))
    raise ValueError("gorev_hedefi GorevHedefi, dict veya koordinat tuple/list olmali.")


class RovDegerOnerici:
    """Hedef/gorev icin ROV atama puanlarini hesaplar."""

    GOREV_DEGISTIRME_CEZASI = 25.0

    def __init__(self, filo_ref):
        self.filo = filo_ref

    def deger_havuzu(self, gorev_hedefi) -> list[RovDegerOnerisi]:
        hedef = gorev_hedefi_coz(gorev_hedefi)
        hedef_koord = self._hedef_koordinat_coz(hedef)
        oneriler = []
        for rov in self._tum_rovlari_al():
            oneriler.append(self._rov_puani(rov, hedef, hedef_koord))
        return sorted(oneriler, key=lambda o: o.puan, reverse=True)

    def en_iyi_rovlari_sec(self, gorev_hedefi, gereken_rov_sayisi: int | None = None) -> list[Any]:
        havuz = self.deger_havuzu(gorev_hedefi)
        secilebilir_idler = [o.rov_id for o in havuz if o.secilebilir]
        if gereken_rov_sayisi is not None and gereken_rov_sayisi > 0:
            secilebilir_idler = secilebilir_idler[: int(gereken_rov_sayisi)]
        rovs = []
        for rov_id in secilebilir_idler:
            rov = self.filo.find_rov_by_id(rov_id)
            if rov is not None:
                rovs.append(rov)
        return rovs

    def _tum_rovlari_al(self) -> list[Any]:
        ortam = getattr(self.filo, "ortam_ref", None)
        rovs = getattr(ortam, "rovs", None)
        if rovs is None:
            try:
                rovs = [r for grup in self.filo.g_rovs.values() for r in (grup or [])]
            except Exception:
                rovs = []
        return [r for r in rovs if r and not getattr(r, "is_destroyed", False)]

    def _hedef_koordinat_coz(self, hedef: GorevHedefi) -> tuple[float, float, float]:
        if hedef.koordinat is not None:
            return hedef.koordinat
        if hedef.alan is not None:
            alan = hedef.alan
            if hasattr(alan, "center"):
                cx, cy, cz = alan.center
                return float(cx), float(cy), float(cz)
            if isinstance(alan, dict):
                if {"x_min", "y_min", "x_max", "y_max"}.issubset(alan):
                    z = float(alan.get("z", -20.0))
                    return (
                        (float(alan["x_min"]) + float(alan["x_max"])) * 0.5,
                        (float(alan["y_min"]) + float(alan["y_max"])) * 0.5,
                        z,
                    )
                if {"baslangic", "genislik", "yukseklik"}.issubset(alan):
                    bx, by = alan["baslangic"][:2]
                    z = float(alan.get("z", -20.0))
                    return float(bx) + float(alan["genislik"]) * 0.5, float(by) + float(alan["yukseklik"]) * 0.5, z
            vals = list(alan)
            if len(vals) >= 4:
                z = float(vals[4]) if len(vals) >= 5 else -20.0
                return (float(vals[0]) + float(vals[2])) * 0.5, (float(vals[1]) + float(vals[3])) * 0.5, z
        raise ValueError("Gorev hedefinden koordinat cikarilamadi.")

    def _rov_puani(self, rov, hedef: GorevHedefi, hedef_koord: tuple[float, float, float]) -> RovDegerOnerisi:
        rov_id = int(getattr(rov, "id", -1))
        gnc = getattr(rov, "gnc", None)
        gorev = str(getattr(gnc, "gorev", "idle") or "idle")
        mevcut_hedef = getattr(gnc, "gorev_hedef", None)
        grup_id = getattr(rov, "group_id", None)

        batarya = self._batarya_al(rov)
        rol_carpani = 0.0 if int(getattr(rov, "role", 0)) == 1 else 1.0
        grup_carpani = 1.0 if hedef.grup_id is None or int(grup_id) == int(hedef.grup_id) else -0.35
        mesafe = self._rov_hedef_mesafesi(rov, hedef_koord)
        yeni_enerji = self._enerji_tahmin_et(rov, mesafe)
        devam_enerjisi = self._goreve_devam_enerjisi(rov)
        gorev_degistirme_cezasi = self.GOREV_DEGISTIRME_CEZASI if gorev != "idle" else 0.0
        gorev_degistirme_enerjisi = yeni_enerji + gorev_degistirme_cezasi
        enerji_kazanci = devam_enerjisi - gorev_degistirme_enerjisi
        if devam_enerjisi > 0:
            enerji_skoru = enerji_kazanci / (abs(devam_enerjisi) + 1.0)
        else:
            enerji_skoru = 1.0 / (1.0 + gorev_degistirme_enerjisi)
        mesafe_skoru = 1.0 / (1.0 + mesafe)

        secilebilir = True
        neden = ""
        if gorev == hedef.gorev_adi and self._hedef_ayni_mi(mevcut_hedef, hedef):
            secilebilir = False
            neden = "ayni_gorev_hedefinde"
        elif gorev != "idle":
            neden = f"gorev_degistirme:{gorev}"

        puan = 100.0 * batarya * rol_carpani * grup_carpani * (0.65 * enerji_skoru + 0.35 * mesafe_skoru)
        if gorev != "idle" and secilebilir:
            puan -= gorev_degistirme_cezasi
        if not secilebilir:
            puan = min(puan, -1.0)

        return RovDegerOnerisi(
            rov_id=rov_id,
            puan=float(puan),
            secilebilir=secilebilir,
            gorev=gorev,
            grup_id=int(grup_id) if grup_id is not None else None,
            batarya=batarya,
            mesafe=mesafe,
            yeni_enerji=yeni_enerji,
            mevcut_gorev_enerjisi=devam_enerjisi,
            goreve_devam_enerjisi=devam_enerjisi,
            gorev_degistirme_enerjisi=gorev_degistirme_enerjisi,
            gorev_degistirme_cezasi=gorev_degistirme_cezasi,
            enerji_kazanci=enerji_kazanci,
            rol_carpani=rol_carpani,
            grup_carpani=grup_carpani,
            neden=neden,
        )

    def _batarya_al(self, rov) -> float:
        try:
            return max(0.0, min(1.0, float(getattr(rov, "battery", 0.0))))
        except (TypeError, ValueError):
            return 0.0

    def _rov_hedef_mesafesi(self, rov, hedef: tuple[float, float, float]) -> float:
        gps = self._gps_al(rov)
        return math.sqrt((gps[0] - hedef[0]) ** 2 + (gps[1] - hedef[1]) ** 2 + (gps[2] - hedef[2]) ** 2)

    def _gps_al(self, rov) -> tuple[float, float, float]:
        try:
            gps = self.filo.get(int(rov.id), "gps")
            if gps is not None and len(gps) >= 3:
                return float(gps[0]), float(gps[1]), float(gps[2])
        except Exception:
            pass
        return float(getattr(rov, "x", 0.0)), float(getattr(rov, "z", 0.0)), float(getattr(rov, "y", 0.0))

    def _enerji_tahmin_et(self, rov, mesafe: float) -> float:
        avg_power = self._ortalama_motor_gucu(rov)
        return float(mesafe) * avg_power * float(Hidrodinamik.MAX_ITME_KUVVETI)

    def _ortalama_motor_gucu(self, rov) -> float:
        motorlar = getattr(rov, "motorlar", []) or []
        gucler = []
        for motor in motorlar:
            try:
                gucler.append(abs(float(getattr(motor, "guc", 0.0))))
            except (TypeError, ValueError):
                continue
        if not gucler:
            return 0.25
        return max(0.05, sum(gucler) / len(gucler))

    def _goreve_devam_enerjisi(self, rov) -> float:
        return self._rota_kalan_enerji(rov)

    def _rota_kalan_enerji(self, rov) -> float:
        rov_id = int(getattr(rov, "id", -1))
        gps = self._gps_al(rov)
        route = getattr(self.filo, "_git_nokta_listesi", {}).get(rov_id)
        idx = int(getattr(self.filo, "_git_mevcut_nokta_indeksi", {}).get(rov_id, 0) or 0)
        z_store = getattr(self.filo, "_git_hedef_derinligi", {})
        z = float(z_store.get(rov_id, gps[2])) if isinstance(z_store, dict) else gps[2]

        points: list[tuple[float, float, float]] = []
        if route and idx < len(route):
            for point in route[idx:]:
                if point and len(point) >= 2:
                    points.append((float(point[0]), float(point[1]), z))
        else:
            hedef = getattr(self.filo, "_rov_hedefleri", {}).get(rov_id)
            if hedef and len(hedef) >= 3:
                points.append((float(hedef[0]), float(hedef[1]), float(hedef[2])))

        if not points:
            gnc_hedef = getattr(getattr(rov, "gnc", None), "gorev_hedef", None)
            if gnc_hedef:
                try:
                    points.append(self._hedef_koordinat_coz(gorev_hedefi_coz(gnc_hedef)))
                except Exception:
                    pass

        if not points:
            return 0.0

        toplam_mesafe = 0.0
        prev = gps
        for point in points:
            toplam_mesafe += math.sqrt((prev[0] - point[0]) ** 2 + (prev[1] - point[1]) ** 2 + (prev[2] - point[2]) ** 2)
            prev = point
        return self._enerji_tahmin_et(rov, toplam_mesafe)

    def _mevcut_hedef_koordinati(self, rov) -> tuple[float, float, float] | None:
        rov_id = int(getattr(rov, "id", -1))
        route = getattr(self.filo, "_git_nokta_listesi", {}).get(rov_id)
        idx = int(getattr(self.filo, "_git_mevcut_nokta_indeksi", {}).get(rov_id, 0) or 0)
        z_store = getattr(self.filo, "_git_hedef_derinligi", {})
        if route and idx < len(route):
            z = float(z_store.get(rov_id, self._gps_al(rov)[2])) if isinstance(z_store, dict) else self._gps_al(rov)[2]
            return float(route[idx][0]), float(route[idx][1]), z
        hedef = getattr(self.filo, "_rov_hedefleri", {}).get(rov_id)
        if hedef and len(hedef) >= 3:
            return float(hedef[0]), float(hedef[1]), float(hedef[2])
        gnc_hedef = getattr(getattr(rov, "gnc", None), "gorev_hedef", None)
        if gnc_hedef:
            try:
                return self._hedef_koordinat_coz(gorev_hedefi_coz(gnc_hedef))
            except Exception:
                return None
        return None

    def _hedef_ayni_mi(self, mevcut_hedef, hedef: GorevHedefi) -> bool:
        if not mevcut_hedef:
            return False
        try:
            mevcut = gorev_hedefi_coz(mevcut_hedef)
            return mevcut.gorev_adi == hedef.gorev_adi and self._hedef_koordinat_coz(mevcut) == self._hedef_koordinat_coz(hedef)
        except Exception:
            return False


def _koordinat_tuple(value) -> tuple[float, float, float] | None:
    if value is None:
        return None
    vals = list(value)
    if len(vals) < 2:
        return None
    z = float(vals[2]) if len(vals) >= 3 else -20.0
    return float(vals[0]), float(vals[1]), z


def rov_deger_havuzu(filo, gorev_hedefi) -> list[RovDegerOnerisi]:
    return RovDegerOnerici(filo).deger_havuzu(gorev_hedefi)


def en_iyi_rovlari_sec(filo, gorev_hedefi, gereken_rov_sayisi: int | None = None) -> list[Any]:
    return RovDegerOnerici(filo).en_iyi_rovlari_sec(gorev_hedefi, gereken_rov_sayisi=gereken_rov_sayisi)
