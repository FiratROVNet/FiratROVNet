from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .ortak import (
    TaramaAlani,
    alan_normalize,
    gorev_grubu_olustur,
    idle_grup_rovlari,
    kopma_menzili,
    lider_rov_bul,
    rov_gorev_ata,
    rov_gorev_bosalt,
)
from ..rov_deger_onerisi import GorevHedefi, en_iyi_rovlari_sec
from ...config import AlanTaramaAyarlari


@dataclass
class AlanTaramaPlani:
    grup_id: int
    alan: TaramaAlani
    lider_id: int | None
    rota_by_rov: dict[int, list[tuple[float, float]]] = field(default_factory=dict)
    derinlik: float = field(default=AlanTaramaAyarlari.DERINLIK)
    serit_araligi: float = field(default=AlanTaramaAyarlari.SERIT_ARALIGI)
    iletisim_menzili: float = field(default=AlanTaramaAyarlari.ILETISIM_MENZILI)
    gorev_adi: str = field(default=AlanTaramaAyarlari.GOREV_ADI)
    kaynak_grup_id: int | None = None
    yaklasma_hedefi: tuple[float, float, float] | None = None
    asama: str = field(default=AlanTaramaAyarlari.ASAMA)


class AlanTaramaGorevi:
    """
    Boustrophedon/lawnmower alan tarama gorevi.

    Grup ROV'larini paralel hatlara dizer. Lider merkez hatta kalir; diger ROV'lar
    liderden kopma menzili icinde kalan offsetlerde tarama yapar. Alan genis ise
    ayni formasyon birden fazla bantta gezdirilir.
    """

    def __init__(self, filo_ref):
        self.filo = filo_ref
        self.aktif_planlar: dict[int, AlanTaramaPlani] = {}

    def plan_olustur(
        self,
        grup_id: int,
        alan,
        derinlik: float = -20.0,
        serit_araligi: float = 15.0,
        rov_basina_varsayilan_alan_m2: float = 400.0,
        iletisim_menzili: float | None = None,
        gereken_rov_sayisi: int | None = None,
        gorev_adi: str = "alan_tarama",
    ) -> AlanTaramaPlani:
        tarama_alani = alan_normalize(alan, z=derinlik)
        hedef_bilgisi = GorevHedefi(gorev_adi=gorev_adi, grup_id=grup_id, alan=tarama_alani)
        if gereken_rov_sayisi is None:
            gereken_rov_sayisi = len(idle_grup_rovlari(self.filo, grup_id))
        rovs = en_iyi_rovlari_sec(self.filo, hedef_bilgisi, gereken_rov_sayisi=gereken_rov_sayisi)
        if not rovs:
            raise ValueError(f"Grup-{grup_id} icin idle durumda ROV bulunamadi.")

        kaynak_grup_id = grup_id
        gorev_grup_id = gorev_grubu_olustur(self.filo, rovs)
        if gorev_grup_id is not None:
            grup_id = gorev_grup_id

        lider = lider_rov_bul(self.filo, grup_id, rovs)
        lider_id = int(getattr(lider, "id", rovs[0].id)) if lider else int(rovs[0].id)
        rovs = sorted(rovs, key=lambda r: (0 if getattr(r, "id", None) == lider_id else 1, int(getattr(r, "id", 0))))

        menzil = kopma_menzili(iletisim_menzili)
        min_swath = max(1.0, float(rov_basina_varsayilan_alan_m2) ** 0.5)
        swath = max(1.0, min(float(serit_araligi), min_swath, menzil * 0.45))
        offsets = self._offsetler(len(rovs), swath, menzil)
        band_genisligi = max(swath, (max(offsets) - min(offsets)) + swath)

        band_merkezleri = []
        x = tarama_alani.x_min + band_genisligi * 0.5
        while x <= tarama_alani.x_max + 1e-6:
            band_merkezleri.append(min(max(x, tarama_alani.x_min), tarama_alani.x_max))
            x += band_genisligi
        if not band_merkezleri:
            band_merkezleri = [tarama_alani.center[0]]

        rota_by_rov: dict[int, list[tuple[float, float]]] = {int(r.id): [] for r in rovs}
        ters = False
        for band_x in band_merkezleri:
            for rov, offset in zip(rovs, offsets):
                rx = min(max(band_x + offset, tarama_alani.x_min), tarama_alani.x_max)
                if ters:
                    rota_by_rov[int(rov.id)].extend([(rx, tarama_alani.y_max), (rx, tarama_alani.y_min)])
                else:
                    rota_by_rov[int(rov.id)].extend([(rx, tarama_alani.y_min), (rx, tarama_alani.y_max)])
            ters = not ters

        return AlanTaramaPlani(
            grup_id=grup_id,
            alan=tarama_alani,
            lider_id=lider_id,
            rota_by_rov=rota_by_rov,
            derinlik=float(derinlik),
            serit_araligi=swath,
            iletisim_menzili=menzil,
            gorev_adi=gorev_adi,
            kaynak_grup_id=kaynak_grup_id,
            yaklasma_hedefi=tarama_alani.center,
        )

    def baslat(
        self,
        grup_id: int,
        alan,
        derinlik: float = -20.0,
        serit_araligi: float = 15.0,
        rov_basina_varsayilan_alan_m2: float = 400.0,
        iletisim_menzili: float | None = None,
        gereken_rov_sayisi: int | None = None,
        gorev_adi: str = "alan_tarama",
        bagimsiz_mod: bool = True,
        sessiz: bool = True,
    ) -> AlanTaramaPlani:
        plan = self.plan_olustur(
            grup_id=grup_id,
            alan=alan,
            derinlik=derinlik,
            serit_araligi=serit_araligi,
            rov_basina_varsayilan_alan_m2=rov_basina_varsayilan_alan_m2,
            iletisim_menzili=iletisim_menzili,
            gereken_rov_sayisi=gereken_rov_sayisi,
            gorev_adi=gorev_adi,
        )
        self.aktif_planlar[grup_id] = plan
        self._yaklasma_baslat(plan, sessiz=sessiz)
        return plan

    def durdur(self, grup_id: int, lideri_takip_et: bool = True) -> None:
        plan = self.aktif_planlar.pop(grup_id, None)
        if not plan:
            return
        try:
            self.filo.aktif_formasyon[plan.grup_id] = None
        except Exception:
            pass
        for rov_id in plan.rota_by_rov:
            rov_gorev_bosalt(self.filo, rov_id, lideri_takip_et=lideri_takip_et)

    def guncelle(self, grup_id: int | None = None, lideri_takip_et: bool = True) -> list[int]:
        """Rotasi biten plan ROV'larini idle'a alir. Donus: biten grup idleri."""
        biten_gruplar = []
        planlar = list(self.aktif_planlar.items())
        for p_grup_id, plan in planlar:
            if grup_id is not None and p_grup_id != grup_id:
                continue
            tamamlandi = True
            if plan.asama == "yaklasma":
                self._plan_liderini_sabitle(plan)
                if self._yaklasma_tamamlandi_mi(plan):
                    self._tarama_baslat(plan, lideri_bagimsiz_yap=True, sessiz=False)
                continue
            for rov_id in plan.rota_by_rov:
                if not self._rov_tarama_bitti_mi(rov_id):
                    tamamlandi = False
                    break
            if tamamlandi:
                self.durdur(p_grup_id, lideri_takip_et=lideri_takip_et)
                biten_gruplar.append(p_grup_id)
        return biten_gruplar

    def _yaklasma_baslat(self, plan: AlanTaramaPlani, sessiz: bool) -> None:
        hedef_bilgisi = GorevHedefi(gorev_adi=plan.gorev_adi, grup_id=plan.grup_id, alan=plan.alan)
        for rov_id in plan.rota_by_rov:
            rov = self.filo.find_rov_by_id(rov_id)
            if not rov:
                continue
            eski_gorev = str(getattr(getattr(rov, "gnc", None), "gorev", "idle") or "idle")
            if eski_gorev != "idle" and eski_gorev != plan.gorev_adi:
                rov_gorev_bosalt(self.filo, rov_id, lideri_takip_et=False)
            rov_gorev_ata(rov, plan.gorev_adi, mod=1, gorev_hedef=hedef_bilgisi)

        try:
            self.filo.formasyon_sec(g_id=plan.grup_id, dinamik=True, sessiz=sessiz)
        except Exception as exc:
            if not sessiz:
                print(f"⚠️ [ALAN_TARAMA] Formasyon secilemedi: {exc}")

        if plan.lider_id is not None and plan.yaklasma_hedefi is not None:
            self._plan_liderini_sabitle(plan)
            self.filo.git_path(plan.lider_id, plan.yaklasma_hedefi, ai=True, isaret=True)
        plan.asama = "yaklasma"

    def _yaklasma_tamamlandi_mi(self, plan: AlanTaramaPlani) -> bool:
        if plan.lider_id is None:
            return True
        lider = self.filo.find_rov_by_id(plan.lider_id)
        if lider is None or plan.yaklasma_hedefi is None:
            return self._rov_tarama_bitti_mi(plan.lider_id)
        try:
            gps = self.filo.get(plan.lider_id, "gps")
            lider_vardi = self._lider_yaklasma_hedefine_vardi(plan, gps)
            if not lider_vardi:
                return False
            return self._takipciler_formasyona_yakin_mi(plan)
        except Exception:
            return self._rov_tarama_bitti_mi(plan.lider_id)

    def _lider_yaklasma_hedefine_vardi(self, plan: AlanTaramaPlani, gps) -> bool:
        rota = getattr(self.filo, "_git_nokta_listesi", {}).get(plan.lider_id) if plan.lider_id is not None else None
        indeks = getattr(self.filo, "_git_mevcut_nokta_indeksi", {}).get(plan.lider_id, 0) if plan.lider_id is not None else 0
        tolerans = max(10.0, min(25.0, plan.iletisim_menzili * 0.35))

        if rota:
            try:
                son_nokta = rota[-1]
                sdx = float(gps[0]) - float(son_nokta[0])
                sdy = float(gps[1]) - float(son_nokta[1])
                if (sdx * sdx + sdy * sdy) ** 0.5 <= tolerans:
                    return True
                hedef_nokta = son_nokta if indeks >= len(rota) - 1 else rota[int(indeks)]
                dx = float(gps[0]) - float(hedef_nokta[0])
                dy = float(gps[1]) - float(hedef_nokta[1])
            except Exception:
                return False
            return indeks >= len(rota) - 1 and (dx * dx + dy * dy) ** 0.5 <= tolerans

        dx = float(gps[0]) - float(plan.yaklasma_hedefi[0])
        dy = float(gps[1]) - float(plan.yaklasma_hedefi[1])
        return (dx * dx + dy * dy) ** 0.5 <= tolerans

    def _takipciler_formasyona_yakin_mi(self, plan: AlanTaramaPlani) -> bool:
        hedefler = getattr(self.filo, "_formasyon_hedefleri", {}) or {}
        tolerans = plan.iletisim_menzili
        lider_gps = None
        if plan.lider_id is not None:
            try:
                lider_gps = self.filo.get(plan.lider_id, "gps")
            except Exception:
                lider_gps = None
        lider_takip_toleransi = max(10.0, min(plan.iletisim_menzili * 0.8, 35.0))
        for rov_id in plan.rota_by_rov:
            if rov_id == plan.lider_id:
                continue
            rov = self.filo.find_rov_by_id(rov_id)
            if rov is None:
                continue
            hedef = hedefler.get(rov_id)
            if not hedef:
                if lider_gps is None:
                    return False
                try:
                    gps = self.filo.get(rov_id, "gps")
                    dx = float(gps[0]) - float(lider_gps[0])
                    dy = float(gps[1]) - float(lider_gps[1])
                except Exception:
                    return False
                if (dx * dx + dy * dy) ** 0.5 > lider_takip_toleransi:
                    return False
                continue
            try:
                hedef_pos = hedef.get("pozisyon") if isinstance(hedef, dict) else hedef
                gps = self.filo.get(rov_id, "gps")
                dx = float(gps[0]) - float(hedef_pos[0])
                dy = float(gps[1]) - float(hedef_pos[1])
            except Exception:
                return False
            formasyon_mesafesi = (dx * dx + dy * dy) ** 0.5
            if formasyon_mesafesi <= tolerans:
                continue
            if lider_gps is None:
                return False
            try:
                ldx = float(gps[0]) - float(lider_gps[0])
                ldy = float(gps[1]) - float(lider_gps[1])
            except Exception:
                return False
            if (ldx * ldx + ldy * ldy) ** 0.5 > lider_takip_toleransi:
                return False
        return True

    def _rov_tarama_bitti_mi(self, rov_id: int) -> bool:
        if bool(getattr(self.filo, "_git_nokta_listesi", {}).get(rov_id)):
            return False
        if bool(getattr(self.filo, "_git_mevcut_nokta_indeksi", {}).get(rov_id)):
            return False
        return True

    def _tarama_baslat(self, plan: AlanTaramaPlani, lideri_bagimsiz_yap: bool, sessiz: bool) -> None:
        plan.asama = "tarama"
        self._plan_liderini_sabitle(plan)
        try:
            self.filo.aktif_formasyon[plan.grup_id] = None
        except Exception:
            pass
        if not sessiz:
            print(f"✅ [ALAN_TARAMA] Grup-{plan.grup_id} yaklaşma tamamlandı, bağımsız tarama başlatılıyor.")
        for rov_id, rota in plan.rota_by_rov.items():
            if not rota:
                continue
            rov = self.filo.find_rov_by_id(rov_id)
            if not rov:
                continue
            hedef_bilgisi = GorevHedefi(gorev_adi=plan.gorev_adi, grup_id=plan.grup_id, alan=plan.alan)
            rov_gorev_ata(rov, plan.gorev_adi, mod=0, gorev_hedef=hedef_bilgisi)
            self.filo.git(rov_id, rota, z=plan.derinlik, ai=True, sessiz=sessiz)

    def _plan_liderini_sabitle(self, plan: AlanTaramaPlani) -> None:
        if plan.lider_id is None:
            return
        for rov_id in plan.rota_by_rov:
            rov = self.filo.find_rov_by_id(rov_id)
            if rov is None:
                continue
            try:
                rov.set("rol", 1 if rov_id == plan.lider_id else 0)
            except Exception:
                rov.role = 1 if rov_id == plan.lider_id else 0
        leader_manager = getattr(self.filo, "leader_manager", None)
        if leader_manager is not None and hasattr(leader_manager, "mevcut_lider_id"):
            try:
                leader_manager.mevcut_lider_id[plan.grup_id] = int(plan.lider_id)
            except Exception:
                pass

    def _offsetler(self, rov_sayisi: int, swath: float, menzil: float) -> list[float]:
        if rov_sayisi <= 1:
            return [0.0]
        raw = [(i - (rov_sayisi - 1) * 0.5) * swath for i in range(rov_sayisi)]
        max_abs = max(abs(v) for v in raw)
        limit = max(1.0, menzil * 0.75)
        if max_abs <= limit:
            return raw
        scale = limit / max_abs
        return [v * scale for v in raw]
