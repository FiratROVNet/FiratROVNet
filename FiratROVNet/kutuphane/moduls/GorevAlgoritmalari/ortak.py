from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable

from FiratROVNet.config import GATLimitleri


@dataclass(frozen=True)
class TaramaAlani:
    """Sim koordinat duzleminde dikdortgen alan: x/y yatay, z derinlik."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float
    z: float = -20.0

    @property
    def width(self) -> float:
        return max(0.0, self.x_max - self.x_min)

    @property
    def height(self) -> float:
        return max(0.0, self.y_max - self.y_min)

    @property
    def center(self) -> tuple[float, float, float]:
        return ((self.x_min + self.x_max) * 0.5, (self.y_min + self.y_max) * 0.5, self.z)

    def sirala(self) -> "TaramaAlani":
        return TaramaAlani(
            min(self.x_min, self.x_max),
            min(self.y_min, self.y_max),
            max(self.x_min, self.x_max),
            max(self.y_min, self.y_max),
            self.z,
        )


def alan_normalize(alan: TaramaAlani | dict[str, Any] | Iterable[float], z: float | None = None) -> TaramaAlani:
    if isinstance(alan, TaramaAlani):
        if z is None:
            return alan
        return TaramaAlani(alan.x_min, alan.y_min, alan.x_max, alan.y_max, float(z))

    if isinstance(alan, dict):
        if {"x_min", "y_min", "x_max", "y_max"}.issubset(alan):
            dz = float(alan.get("z", z if z is not None else -20.0))
            return TaramaAlani(
                float(alan["x_min"]),
                float(alan["y_min"]),
                float(alan["x_max"]),
                float(alan["y_max"]),
                dz,
            ).sirala()
        if {"baslangic", "genislik", "yukseklik"}.issubset(alan):
            bx, by = alan["baslangic"][:2]
            dz = float(alan.get("z", z if z is not None else -20.0))
            return TaramaAlani(float(bx), float(by), float(bx) + float(alan["genislik"]), float(by) + float(alan["yukseklik"]), dz).sirala()

    vals = list(alan)
    if len(vals) < 4:
        raise ValueError("Alan en az 4 sayi icermeli: (x_min, y_min, x_max, y_max)")
    dz = float(vals[4]) if len(vals) >= 5 else float(z if z is not None else -20.0)
    return TaramaAlani(float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3]), dz).sirala()


def aktif_grup_rovlari(filo, grup_id: int) -> list[Any]:
    grup = []
    try:
        grup = list(filo.g_rovs.get(grup_id, []) or [])
    except Exception:
        grup = []
    return [r for r in grup if r and not getattr(r, "is_destroyed", False)]


def rov_gorev_al(rov) -> str:
    gnc = getattr(rov, "gnc", None)
    return str(getattr(gnc, "gorev", "idle") or "idle")


def rov_idle_mi(rov) -> bool:
    return rov_gorev_al(rov) == "idle"


def idle_grup_rovlari(filo, grup_id: int, gereken_rov_sayisi: int | None = None) -> list[Any]:
    rovs = [rov for rov in aktif_grup_rovlari(filo, grup_id) if rov_idle_mi(rov)]
    rovs = sorted(rovs, key=lambda r: int(getattr(r, "id", 0)))
    if gereken_rov_sayisi is not None and gereken_rov_sayisi > 0:
        return rovs[: int(gereken_rov_sayisi)]
    return rovs


def lider_rov_bul(filo, grup_id: int, rovs: list[Any] | None = None) -> Any | None:
    rovs = rovs if rovs is not None else aktif_grup_rovlari(filo, grup_id)
    for rov in rovs:
        if int(getattr(rov, "role", 0)) == 1:
            return rov
    try:
        lider_id, _gps = filo.find_leader_info(g_id=grup_id)
        if lider_id is not None:
            return filo.find_rov_by_id(int(lider_id))
    except Exception:
        pass
    return rovs[0] if rovs else None


def rov_mod_ayarla(rov, mod: int) -> None:
    gnc = getattr(rov, "gnc", None)
    if gnc is not None:
        gnc.mod = int(mod)
        gnc.manuel_kontrol = False


def rov_gorev_ata(rov, gorev: str, mod: int | None = None, gorev_hedef=None) -> None:
    gnc = getattr(rov, "gnc", None)
    if gnc is None:
        return
    gnc.gorev = str(gorev or "idle")
    gnc.gorev_hedef = gorev_hedef
    gnc.manuel_kontrol = False
    if mod is not None:
        gnc.mod = int(mod)


def rov_gorev_bosalt(filo, rov_id: int, lideri_takip_et: bool = True) -> None:
    rov = filo.find_rov_by_id(rov_id) if hasattr(filo, "find_rov_by_id") else None
    if rov is None:
        return
    rov_gorev_ata(rov, "idle", mod=(1 if lideri_takip_et else None), gorev_hedef=None)
    for attr in ("_git_nokta_listesi", "_git_mevcut_nokta_indeksi", "_git_hedef_derinligi", "_rov_hedefleri"):
        store = getattr(filo, attr, None)
        if isinstance(store, dict):
            store.pop(rov_id, None)
    onceki_group = getattr(getattr(rov, "gnc", None), "onceki_group_id", None)
    if onceki_group is not None:
        rov.group_id = int(onceki_group)
        rov.gnc.onceki_group_id = None
    onceki_role = getattr(getattr(rov, "gnc", None), "onceki_role", None)
    if onceki_role is not None:
        try:
            rov.set("rol", int(onceki_role))
        except Exception:
            rov.role = int(onceki_role)
        rov.gnc.onceki_role = None


def gorev_grubu_olustur(filo, rovs: list[Any]) -> int | None:
    if not rovs:
        return None
    try:
        from UI.kopru import ilk_bos_grup_id
        mevcut_gruplar = [g for g in filo.g_rovs.keys() if g is not None]
        yeni_grup_id = ilk_bos_grup_id(mevcut_gruplar)
    except Exception:
        yeni_grup_id = 0
    for rov in rovs:
        gnc = getattr(rov, "gnc", None)
        if gnc is not None and getattr(gnc, "onceki_group_id", None) is None:
            gnc.onceki_group_id = int(getattr(rov, "group_id", 0))
        if gnc is not None and getattr(gnc, "onceki_role", None) is None:
            gnc.onceki_role = int(getattr(rov, "role", 0))
        rov.group_id = yeni_grup_id
    for idx, rov in enumerate(rovs):
        try:
            rov.set("rol", 1 if idx == 0 else 0)
        except Exception:
            rov.role = 1 if idx == 0 else 0
    return yeni_grup_id


def mesafe_2d(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def minimap_gorev_alanini_temizle(filo) -> None:
    """Görev bittiğinde minimap üzerindeki kalıcı alan çokgenini kaldırır."""
    ortam = getattr(filo, "ortam_ref", None)
    if ortam is None:
        return
    mm = getattr(ortam, "minimap", None)
    if mm is not None and hasattr(mm, "alan_gorev_temizle"):
        mm.alan_gorev_temizle()
    setattr(ortam, "_ui_minimap_gorev_poligon", None)


def kopma_menzili(default: float | None = None) -> float:
    return float(default if default is not None else getattr(GATLimitleri, "KOPMA", 50.0))
